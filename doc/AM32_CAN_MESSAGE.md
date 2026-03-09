# AM32 CAN Message Reference Guide
## From Frame Reception to Motor Control

> **Target**: `VIMDRONES_L431_CAN` and any target with `DRONECAN_SUPPORT 1`.
> **Focus**: End-to-end pipeline for how a CAN frame arrives, what values it
> carries, how those values reach `setInput()`, and a step-by-step recipe for
> adding 4 new inputs that customize commutation speed — while guaranteeing
> that no other ESC on the bus reads those extra slots as throttle.

---

## Table of Contents

1. [Message Format — What the CAN Frame Contains](#1-message-format)
2. [Full Pipeline — Frame to Motor Control](#2-full-pipeline)
   - 2.1 [IRQ: Frame reception](#21-irq-frame-reception)
   - 2.2 [Reassembly: `canardHandleRxFrame`](#22-reassembly)
   - 2.3 [Dispatch: `onTransferReceived`](#23-dispatch)
   - 2.4 [Decode and map: `handle_RawCommand`](#24-decode-and-map)
   - 2.5 [Filter and write: `set_input`](#25-filter-and-write)
   - 2.6 [Direction and duty cycle: `setInput`](#26-direction-and-duty-cycle)
   - 2.7 [Commutation timing: `PeriodElapsedCallback`](#27-commutation-timing)
3. [Key Variables and Their Roles](#3-key-variables-and-their-roles)
4. [Accepted Values — Complete Reference](#4-accepted-values)
5. [Slot-Stride: Enforcing Input Count at Compile Time](#5-slot-stride)
   - 5.1 [The bus-collision problem](#51-the-bus-collision-problem)
   - 5.2 [Stride-based indexing — the solution](#52-stride-based-indexing)
   - 5.3 [Compile-time enforcement mechanisms](#53-compile-time-enforcement)
   - 5.4 [External-device compatibility rules](#54-external-device-compatibility)
6. [Step-by-Step: Adding 4 New CAN Inputs](#6-step-by-step-adding-4-new-can-inputs)
   - Step 1: Define `CAN_EXTRA_INPUTS_COUNT` in `targets.h`
   - Step 2: Derive stride and assert bounds in `DroneCAN.c`
   - Step 3: Declare `extern` variables in `signal.h`
   - Step 4: Define variables in `main.c`
   - Step 5: Extract from `RawCommand` in `DroneCAN.c`
   - Step 6: Consume in `setInput()` / `PeriodElapsedCallback()`
7. [Value Mapping Quick Reference](#7-value-mapping-quick-reference)

---

## 1. Message Format

### 1.1 DroneCAN message used for control

**Message type**: `uavcan.equipment.esc.RawCommand`
**Data type ID**: 1030
**DSDL definition**:

```
# uavcan.equipment.esc.RawCommand
# Throttle setpoint for a set of ESCs.
# Each value is in [-8191, +8191]. Negative = reverse.
int14[<=20] cmd
```

**On the wire**: 14-bit signed integers, packed. A single CAN transfer can hold
commands for up to **20 ESCs simultaneously**. Each element uses 14 bits, so:
- Minimum: −8191
- Maximum: +8191

The DSDL hard limit of 20 elements is enforced by the generated decoder
(`uavcan.equipment.esc.RawCommand.h:74`):

```c
if (msg->cmd.len > 20) { return true; /* invalid */ }
```

Any message with `len > 20` is rejected before `handle_RawCommand()` is called.
This ceiling is the primary constraint on how many ESCs and extra inputs can
share a single `RawCommand`.

### 1.2 Which slot belongs to which ESC

The EEPROM parameter `esc_index` (byte 177, CAN parameter `ESC_INDEX`) decides
which element of the `cmd` array this ESC reads.

```
cmd array layout (standard, no extra inputs):
  index 0  → ESC with esc_index = 0
  index 1  → ESC with esc_index = 1
  index N  → ESC with esc_index = N
  ...
  index 19 → ESC with esc_index = 19
```

**Currently consumed**: only `cmd[esc_index]` (throttle).
**Currently ignored**: `cmd[esc_index+1]` through `cmd[19]`.

These unused slots are the hook for new inputs — no new message type needed.
However, repurposing adjacent slots changes the array layout that all nodes on
the bus must agree on. This is the problem addressed in §5.

---

## 2. Full Pipeline

```
CAN bus wire
    │
    │  1.4–8 µs per bit @ 1 Mbps
    ▼
┌──────────────────────────────────────────────────────────────────────┐
│ STM32L431 bxCAN peripheral                                          │
│   CAN_RX = PA11   CAN_TX = PA12                                     │
│   FIFO0 (RX0) or FIFO1 (RX1) depending on filter match             │
└────────────────────────┬─────────────────────────────────────────────┘
                         │ hardware interrupt
                         ▼
    CAN1_RX0_IRQHandler / CAN1_RX1_IRQHandler       [sys_can_stm32.c]
         └─ DroneCAN_receiveFrame()
                         │
                         ▼ [2.2]
    canardHandleRxFrame()                              [libcanard/canard.c]
         ├─ shouldAcceptTransfer()  ← message filter
         └─ (on complete multi-frame transfer) onTransferReceived()
                         │
                         ▼ [2.3]
    onTransferReceived()                               [DroneCAN.c]
         ├─ UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID (1030) → handle_RawCommand()
         ├─ UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID   → handle_ArmingStatus()
         └─ (service IDs) → handle_GetNodeInfo / param_GetSet / etc.
                         │
                         ▼ [2.4]
    handle_RawCommand()                                [DroneCAN.c:609]
         ├─ decode  → uavcan_equipment_esc_RawCommand_decode()
         ├─ check   → cmd.cmd.len > esc_index (else discard)
         ├─ extract → input_can = cmd.cmd.data[esc_index]
         ├─ map     → this_input (0..2047)
         ├─ call    set_input(this_input)
         └─ [NEW]   extract extra slots esc_index+1..esc_index+N
                         │
                         ▼ [2.5]
    set_input(uint16_t input)                          [DroneCAN.c:582]
         ├─ arming gate  (require_arming && !dronecan_armed → force 0)
         ├─ 2-pole low-pass filter (Filter2P_apply, cutoff = filter_hz)
         ├─ newinput  ← filtered_input           ← READ by setInput()
         ├─ inputSet  ← 1
         ├─ dshot     ← eepromBuffer.bi_direction
         ├─ transfercomplete()
         └─ setInput()
                         │
                         ▼ [2.6]
    setInput()                                         [main.c:947]
         ├─ bidirectional mapping (servo or dshot path)
         │    newinput → adjusted_input
         ├─ sine-start ramp / speed PID / RPM control
         │    adjusted_input → input  (0..2047)
         └─ duty_cycle_setpoint = map(input, 47, 2047,
                                      minimum_duty_cycle, 2000)
                         │
                         ▼ [2.7]
    PeriodElapsedCallback()                            [main.c:885]
         ├─ commutate()
         ├─ commutation_interval ← running average of BEMF zero-crossing times
         ├─ advance = (commutation_interval * temp_advance) >> 6
         └─ waitTime = (commutation_interval >> 1) - advance
              └─ SET_AND_ENABLE_COM_INT(waitTime)
                    → this fires the next commutation step
```

### 2.1 IRQ: Frame reception

**File**: `Src/DroneCAN/sys_can_stm32.c`

The bxCAN peripheral delivers each received frame to FIFO0 or FIFO1. The IRQ
handler reads the frame out of the FIFO and hands it to libcanard:

```c
void CAN1_RX0_IRQHandler(void)
{
    CanardCANFrame frame;
    // ... read from FIFO0 ...
    DroneCAN_receiveFrame(&frame, timestamp_us);
}
```

IRQ priority is 5. FIFO overflow events are counted in `canstats.rx_errors`.

### 2.2 Reassembly

**File**: `Src/DroneCAN/libcanard/canard.c`

`canardHandleRxFrame()` accumulates CAN frames until a complete DroneCAN
*transfer* is assembled (DroneCAN uses multi-frame transfers for payloads >8 B).
Once complete, it calls back into `onTransferReceived()`.

`shouldAcceptTransfer()` is called first as a filter — it must return `true`
for the message to be processed. AM32 accepts:
- `UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID` (1030)
- `UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID` (1028)
- All service requests addressed to this node

### 2.3 Dispatch

**File**: `Src/DroneCAN/DroneCAN.c`

`onTransferReceived()` dispatches by `transfer->data_type_id`:

| Data type ID | Message | Handler |
|---|---|---|
| 1030 | `uavcan.equipment.esc.RawCommand` | `handle_RawCommand()` |
| 1028 | `uavcan.equipment.safety.ArmingStatus` | `handle_ArmingStatus()` |
| 1 | `uavcan.protocol.GetNodeInfo` | `handle_GetNodeInfo()` |
| 11 | `uavcan.protocol.param.GetSet` | `handle_param_GetSet()` |
| 10 | `uavcan.protocol.param.ExecuteOpcode` | `handle_param_ExecuteOpcode()` |
| 5 | `uavcan.protocol.RestartNode` | `handle_RestartNode()` |
| 40 | `uavcan.protocol.file.BeginFirmwareUpdate` | `handle_begin_firmware_update()` |
| 1 (anon) | DNA Allocation | `handle_DNA_Allocation()` |

### 2.4 Decode and map

**File**: `Src/DroneCAN/DroneCAN.c:609`

```c
static void handle_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer)
{
    struct uavcan_equipment_esc_RawCommand cmd;
    if (uavcan_equipment_esc_RawCommand_decode(transfer, &cmd)) {
        return;                             // decode error → discard
    }
    if (cmd.cmd.len <= eepromBuffer.can.esc_index) {
        return;                             // no slot for us → discard
    }

    const int16_t input_can = cmd.cmd.data[(unsigned)eepromBuffer.can.esc_index];

    uint16_t this_input = 0;
    if (input_can == 0) {
        this_input = 0;
    } else if (eepromBuffer.bi_direction) {
        const float scaled_value = input_can * (1000.0 / 8192);
        if (scaled_value >= 0) {
            this_input = (uint16_t)(1047 + scaled_value);    // forward: 1047..2047
        } else {
            this_input = (uint16_t)(47 + scaled_value * -1); // reverse: 47..1047
        }
    } else if (input_can > 0) {
        const float scaled_value = input_can * (2000.0 / 8192);
        this_input = (uint16_t)(47 + scaled_value);           // uni: 47..2047
    }

    canstats.num_commands++;
    canstats.total_commands++;
    canstats.last_raw_command_us = micros64();

    set_input(this_input);
}
```

**Mapping summary**:

| CAN wire value (`int16`) | `bi_direction=0` → `this_input` | `bi_direction=1` → `this_input` |
|---|---|---|
| 0 | 0 (off) | 0 (off) |
| +1 … +8191 | 47 … 2047 (forward only) | 1047 … 2047 (forward) |
| −1 … −8191 | 0 (ignored) | 47 … 1047 (reverse) |

### 2.5 Filter and write

**File**: `Src/DroneCAN/DroneCAN.c:582`

```c
static void set_input(uint16_t input)
{
    // allow mid-flight reconnect if ESC rebooted
    if (!armed && input != 0 && eepromBuffer.can.require_arming &&
        dronecan_armed && !eepromBuffer.can.require_zero_throttle) {
        armed = 1;
    }

    // arming gate: if require_arming and not armed → zero
    const uint16_t unfiltered_input =
        (dronecan_armed || !eepromBuffer.can.require_arming) ? input : 0;

    // 2-pole Butterworth low-pass filter (filter_hz = 0 → bypass)
    const uint16_t filtered_input =
        Filter2P_apply(unfiltered_input, eepromBuffer.can.filter_hz, 1000);

    newinput       = filtered_input;    // ← shared with setInput()
    last_can_input = unfiltered_input;  // ← replayed at 1 kHz by DroneCAN_update()
    inputSet       = 1;
    dshot          = eepromBuffer.bi_direction;

    transfercomplete();  // signal.c: marks that a new frame arrived
    setInput();          // main.c:  compute duty cycle

    canstats.num_input++;
}
```

The 1 kHz re-apply in `DroneCAN_update()` ensures the motor-control code
always sees a 1 kHz signal even when the CAN send rate is lower (e.g. 50 Hz):

```c
if (ts - canstats.last_raw_command_us > TARGET_PERIOD_US) {  // 1000 µs
    set_input(last_can_input);
    canstats.last_raw_command_us = ts;
}
```

250 ms watchdog (no RawCommand for 250 ms → motor off):

```c
if (canstats.last_raw_command_us != 0 &&
    ts - canstats.last_raw_command_us > 250000ULL) {
    canstats.last_raw_command_us = 0;
    set_input(0);
}
```

### 2.6 Direction and duty cycle

**File**: `Src/main.c:947`

`setInput()` translates `newinput` into `input` (the variable that feeds the
duty-cycle calculation):

```
newinput (0..2047)
    │
    ├─ bi_direction == 0:
    │     adjusted_input = newinput   (direct)
    │
    └─ bi_direction == 1:
          (servo path)  newinput vs 1000±deadband → fwd/rev adjusted_input
          (dshot path)  newinput > 1047 → forward
                        newinput 48..1047 → reverse
                        newinput < 48    → stop (adjusted_input = 0)
    │
    ▼
 adjusted_input
    │
    ├─ sine_start ramp → input = map(adjusted_input, 30, changeover*20, 47, 160)
    │                    then  = map(adjusted_input, changeover*20, 2047, 160, 2047)
    │
    ├─ speed PID (use_speed_control_loop) → input = input_override / 10000
    │
    └─ default: input = adjusted_input
    │
    ▼
 duty_cycle_setpoint = map(input, 47, 2047, minimum_duty_cycle, 2000)
```

### 2.7 Commutation timing

**File**: `Src/main.c:885`

This interrupt fires when the timer programmed by `SET_AND_ENABLE_COM_INT(waitTime)`
expires. It is the heart of the sensorless BLDC commutation:

```c
void PeriodElapsedCallback()
{
    DISABLE_COM_TIMER_INT();
    commutate();   // switch to next motor phase

    // running average of the last two BEMF zero-crossing intervals
    commutation_interval = ((commutation_interval) +
                            ((lastzctime + thiszctime) >> 1)) >> 1;

    // timing advance (degrees ahead of BEMF zero crossing)
    if (!eepromBuffer.auto_advance) {
        advance = (commutation_interval * temp_advance) >> 6;
        // temp_advance range 0..32 → advance = 0..commutation_interval/2
    } else {
        advance = (commutation_interval * auto_advance_level) >> 6;
    }

    // waitTime: how long to wait after BEMF ZC before commutating
    waitTime = (commutation_interval >> 1) - advance;

    enableCompInterrupts();
}
```

**Key variables for commutation speed customization**:

| Variable | Type | Role |
|---|---|---|
| `commutation_interval` | `uint32_t` | µs per electrical half-period (running average) |
| `temp_advance` | `uint8_t` | Timing advance multiplier; 0..32 → 0..~30 electrical degrees |
| `auto_advance_level` | `uint8_t` | Advance multiplier when `auto_advance=1` |
| `waitTime` | `uint16_t` | Timer count before next commutation fires |
| `advance` | computed | `(commutation_interval * temp_advance) >> 6` |
| `duty_cycle_setpoint` | `uint16_t` | PWM duty (0..2000 timer counts) |

---

## 3. Key Variables and Their Roles

| Variable | Defined in | Written by | Read by | Meaning |
|---|---|---|---|---|
| `newinput` | `main.c:538` | `set_input()` | `setInput()` | Filtered throttle demand, 0..2047 |
| `inputSet` | `main.c:539` | `set_input()` | main loop | 1 = at least one valid command received |
| `last_can_input` | `DroneCAN.c` | `set_input()` | `DroneCAN_update()` | Unfiltered last throttle, replayed at 1 kHz |
| `dshot` | `main.c:540` | `set_input()` | `setInput()` | 1 = bidirectional dshot encoding |
| `adjusted_input` | `main.c` | `setInput()` | `setInput()` | After direction logic, 0..2047 |
| `input` | `main.c:537` | `setInput()` | `PeriodElapsedCallback()` | Final demand after PID/ramp |
| `duty_cycle_setpoint` | `main.c` | `setInput()` | 10 kHz routine | PWM comparison value, 0..2000 |
| `commutation_interval` | `main.c:563` | `PeriodElapsedCallback()` | `PeriodElapsedCallback()` | µs per half-period |
| `temp_advance` | `main.c:323` | `loadEEpromSettings()` | `PeriodElapsedCallback()` | Timing advance index 0..32 |
| `armed` | `main.c:534` | main loop / `set_input()` | `setInput()` | Motor armed state |

---

## 4. Accepted Values

### 4.1 CAN wire format

| Field | Type | Range | Notes |
|---|---|---|---|
| `cmd.cmd.len` | `uint8_t` | 1..20 | Number of populated slots; hard limit from DSDL |
| `cmd.cmd.data[i]` | `int16_t` | −8191..+8191 | 14-bit signed, per DSDL |

The ESC reads only `cmd.cmd.data[eepromBuffer.can.esc_index]` for throttle.
Slots beyond that are unused by the existing firmware but are received intact.

### 4.2 AM32 internal range after mapping

| `this_input` value | Meaning |
|---|---|
| 0 | Motor off / brake |
| 1–46 | Reserved (special DShot telemetry codes when input type is DShot) |
| 47 | Minimum throttle (motor starts spinning) |
| 48–2046 | Throttle demand, linear |
| 2047 | Full throttle |

### 4.3 EEPROM parameters that affect decoding

| EEPROM field | Type | Default | Effect on pipeline |
|---|---|---|---|
| `can.esc_index` | uint8 | 0 | Which `cmd.data[]` slot to read for throttle |
| `can.require_arming` | bool | 1 | Gate: blocks throttle until `ArmingStatus=ARMED` |
| `can.require_zero_throttle` | bool | 1 | First arm requires throttle=0 |
| `can.filter_hz` | uint8 | 0 | 2-pole LPF cutoff on throttle (0 = bypass) |
| `bi_direction` | bool | 0 | Changes raw→this_input mapping |
| `advance_level` | uint8 | varies | Converted to `temp_advance` at boot |
| `auto_advance` | bool | 0 | If 1, ignores `temp_advance` and uses auto algorithm |

---

## 5. Slot-Stride: Enforcing Input Count at Compile Time

### 5.1 The bus-collision problem

Consider a bus with two AM32 ESCs, where ESC 0 (`esc_index=0`) is enhanced
with 4 extra inputs, and ESC 1 is a standard unit with `esc_index=1`:

```
cmd array as sent by the flight controller:
  [0] = throttle for ESC 0        ← ESC 0 reads this correctly
  [1] = can_timing_advance (0..8191)
  [2] = can_phase_offset   (0..8191)
  [3] = can_duty_offset    (−8191..+8191)
  [4] = can_commutation_scale (0..8191)
  ...
```

ESC 1 — whose firmware reads `cmd.data[1]` as throttle — will receive
`can_timing_advance` (a value up to 8191) as a throttle command, producing
uncontrolled full-power motor output. **This is a safety hazard.**

The root cause is that the extra slots are indistinguishable from throttle
slots at the wire level. The only protection is ensuring no other ESC
(AM32 or third-party) has its `esc_index` pointing at a slot used as extra data.

### 5.2 Stride-based indexing — the solution

Instead of assigning ESC indices sequentially (0, 1, 2, …), assign them in
**strides** of `CAN_ESC_SLOT_STRIDE = 1 + CAN_EXTRA_INPUTS_COUNT`:

```
With CAN_EXTRA_INPUTS_COUNT = 4  →  CAN_ESC_SLOT_STRIDE = 5

cmd array layout:
  [0]  throttle    → ESC position 0  (esc_index = 0)
  [1]  extra 1     ─┐
  [2]  extra 2      │ owned by ESC 0, no other ESC may have esc_index here
  [3]  extra 3      │
  [4]  extra 4     ─┘
  [5]  throttle    → ESC position 1  (esc_index = 5)
  [6]  extra 1     ─┐
  [7]  extra 2      │ owned by ESC 1
  [8]  extra 3      │
  [9]  extra 4     ─┘
  [10] throttle    → ESC position 2  (esc_index = 10)
  ...
  [15] throttle    → ESC position 3  (esc_index = 15)
  ...
```

Maximum ESC positions at stride 5: `floor(20 / 5) = 4`

The required `esc_index` for each physical ESC is:

```
esc_index = physical_position × CAN_ESC_SLOT_STRIDE
          = physical_position × (1 + CAN_EXTRA_INPUTS_COUNT)
```

| Physical position | `CAN_EXTRA_INPUTS_COUNT` | Required `esc_index` |
|---|---|---|
| 0 | 4 | 0 |
| 1 | 4 | 5 |
| 2 | 4 | 10 |
| 3 | 4 | 15 |

> The `esc_index` EEPROM parameter must be set to these stride-aligned values
> via the DroneCAN GUI Tool or `param.GetSet` before first use. The firmware
> exposes `ESC_INDEX` as a settable parameter; its allowed maximum is enforced
> at compile time (see §5.3).

### 5.3 Compile-time enforcement mechanisms

The following mechanisms are built into the implementation (see §6) to make
violations a **compile error** rather than a runtime surprise:

#### a) `CAN_EXTRA_INPUTS_COUNT` — the single source of truth

Rather than a bare flag (`USE_CAN_EXTRA_INPUTS`), the feature is enabled by
defining an **integer** in `targets.h`:

```c
#define CAN_EXTRA_INPUTS_COUNT 4
```

This value propagates everywhere:
- `#ifdef CAN_EXTRA_INPUTS_COUNT` gates compilation of all new code
- `CAN_ESC_SLOT_STRIDE` is derived from it, never written by hand
- `_Static_assert` uses it to verify array bounds at compile time
- The `ESC_INDEX` parameter maximum is computed from it, tightening the EEPROM
  settable range so an operator cannot configure an out-of-bounds index

#### b) `CAN_ESC_SLOT_STRIDE` — derived, never hardcoded

Defined once in `DroneCAN.c` (or a shared header) immediately after the
`_Static_assert` checks:

```c
#ifdef CAN_EXTRA_INPUTS_COUNT
#define CAN_ESC_SLOT_STRIDE  (1 + CAN_EXTRA_INPUTS_COUNT)
#endif
```

All code that needs the stride references this macro, not a literal `5`.

#### c) `_Static_assert` — array-bounds check at compile time

```c
// Placed at file scope in DroneCAN.c, inside the #ifdef block:

_Static_assert(CAN_EXTRA_INPUTS_COUNT >= 1,
    "CAN_EXTRA_INPUTS_COUNT must be at least 1");

_Static_assert(CAN_EXTRA_INPUTS_COUNT <= 19,
    "CAN_EXTRA_INPUTS_COUNT cannot exceed 19 (max 20 slots, 1 for throttle)");

_Static_assert(CAN_ESC_SLOT_STRIDE <= 20,
    "CAN_ESC_SLOT_STRIDE exceeds RawCommand array size (max 20 elements)");
```

If someone sets `CAN_EXTRA_INPUTS_COUNT 20`, the build fails with a readable
error message before any binary is produced.

#### d) `ESC_INDEX` parameter maximum — tightened at compile time

In the `parameters[]` array in `DroneCAN.c`, the existing entry is:

```c
{ "ESC_INDEX", T_UINT8, 0, 32, 0, &eepromBuffer.can.esc_index }
```

Replace the hardcoded max `32` with a compile-time expression:

```c
#ifdef CAN_EXTRA_INPUTS_COUNT
  // Maximum valid esc_index: last stride-aligned slot that still fits in 20 elements.
  // e.g. stride=5: max = (floor(20/5)-1)*5 = 3*5 = 15
  { "ESC_INDEX", T_UINT8, 0,
    (uint16_t)(((20 / CAN_ESC_SLOT_STRIDE) - 1) * CAN_ESC_SLOT_STRIDE),
    0, &eepromBuffer.can.esc_index },
#else
  { "ESC_INDEX", T_UINT8, 0, 32, 0, &eepromBuffer.can.esc_index },
#endif
```

With `CAN_EXTRA_INPUTS_COUNT=4` (stride=5), this sets max to **15**, making
it impossible for the operator to set `esc_index=16..32` via `param.GetSet`.
Values 16–19 would be extra-data slots of the ESC at position 3, and
values 20–32 would be out of the array entirely.

To change the number of extra inputs in the future, only one line in
`targets.h` changes:

```c
#define CAN_EXTRA_INPUTS_COUNT 2   // was 4; now stride=3, max 6 ESCs
```

All bounds checks, the parameter max, and the extraction code automatically
adapt without touching any other file.

### 5.4 External-device compatibility rules

The stride mechanism protects AM32 ESCs from each other. External devices
(third-party ESCs, flight controller output channels, other CAN nodes)
require operator-level configuration alignment:

| Scenario | Risk | Mitigation |
|---|---|---|
| Third-party ESC with `esc_index=1` on same bus as AM32 ESC at `esc_index=0` with 4 extra inputs | ESC reads extra slot as throttle → runaway | Set third-party ESC's index to 5 (or the next stride-aligned slot) |
| Flight controller sending standard `RawCommand` with `len=4` | Slots 1–3 contain real throttle for other ESCs, not extra data | Reconfigure FC to use stride-based assignment |
| Pure AM32 bus (all ESCs use this firmware) | No conflict if all `esc_index` are stride-aligned | Set `esc_index` to `position × CAN_ESC_SLOT_STRIDE` on every ESC |
| Mixed bus: some ESCs have `CAN_EXTRA_INPUTS_COUNT`, some do not | ESCs without extra inputs will ignore extra slots if their `esc_index` is not in those slots | Assign non-enhanced ESCs to stride-aligned positions; they read only their throttle slot |

**Bottom line**: The compile-time mechanisms prevent firmware misconfiguration.
Bus-level safety depends on the sender (flight controller or test tool)
populating the `RawCommand` array with the stride-aware layout, and every node
on the bus having its `esc_index` set to a stride-aligned value.

---

## 6. Step-by-Step: Adding 4 New CAN Inputs

This section is the complete implementation recipe. Follow every step in order.

### Slot assignment with `CAN_EXTRA_INPUTS_COUNT = 4`

```
cmd array layout after this implementation:
  [esc_index + 0]  → throttle           (existing, unchanged)
  [esc_index + 1]  → can_timing_advance  (0..8191 → 0..32)
  [esc_index + 2]  → can_phase_offset    (0..8191 → 0..360 degrees)
  [esc_index + 3]  → can_duty_offset     (−8191..+8191 → ±2000 counts)
  [esc_index + 4]  → can_commutation_scale (0..8191 → 50..200 %)
```

---

### Step 1 — Define `CAN_EXTRA_INPUTS_COUNT` in `Inc/targets.h`

Replace the previous boolean flag with a numeric count. This is the **only**
line that needs to change to add or remove extra inputs.

```c
// Inc/targets.h

#ifdef VIMDRONES_L431_CAN
#define FIRMWARE_NAME        "VimdroneL431"
#define FILE_NAME            "VIMDRONES_L431_CAN"
#define DRONECAN_SUPPORT     1
#define DRONECAN_NODE_NAME   "com.vimdrones.esc_dev"
#define DEAD_TIME            45
#define HARDWARE_GROUP_L4_B
#define TARGET_VOLTAGE_DIVIDER 94
#define MILLIVOLT_PER_AMP    30
#define CURRENT_OFFSET       110
#define USE_SERIAL_TELEMETRY

#define CAN_EXTRA_INPUTS_COUNT  4   // ← ADD THIS LINE (change 4 to adjust)
#endif
```

To disable all extra inputs for a build, remove or comment out that one line.
To use 2 extra inputs instead of 4, change `4` to `2` — everything else
(bounds checks, parameter max, extraction loop) updates automatically.

---

### Step 2 — Derive stride and assert bounds in `Src/DroneCAN/DroneCAN.c`

Add these definitions at file scope, near the top of `DroneCAN.c`, after the
`#include` block:

```c
// Src/DroneCAN/DroneCAN.c — file-scope, inside #if DRONECAN_SUPPORT

#ifdef CAN_EXTRA_INPUTS_COUNT

/* Stride: slots consumed per ESC = 1 throttle + N extra.
 * Every ESC on the bus must have esc_index = position × CAN_ESC_SLOT_STRIDE. */
#define CAN_ESC_SLOT_STRIDE  (1 + CAN_EXTRA_INPUTS_COUNT)

/* Compile-time bounds checks — build fails here with a readable message
 * if the configuration is geometrically impossible. */
_Static_assert(CAN_EXTRA_INPUTS_COUNT >= 1,
    "CAN_EXTRA_INPUTS_COUNT must be >= 1");
_Static_assert(CAN_EXTRA_INPUTS_COUNT <= 19,
    "CAN_EXTRA_INPUTS_COUNT too large: max 19 (20 slots total, 1 for throttle)");
_Static_assert(CAN_ESC_SLOT_STRIDE <= 20,
    "CAN_ESC_SLOT_STRIDE exceeds RawCommand hard limit of 20 elements");

#endif // CAN_EXTRA_INPUTS_COUNT
```

Also replace the `ESC_INDEX` entry in the `parameters[]` array:

```c
// Src/DroneCAN/DroneCAN.c — inside parameters[]

// Replace:
{ "ESC_INDEX", T_UINT8, 0, 32, 0, &eepromBuffer.can.esc_index },

// With:
#ifdef CAN_EXTRA_INPUTS_COUNT
    { "ESC_INDEX", T_UINT8, 0,
      (uint16_t)(((20 / CAN_ESC_SLOT_STRIDE) - 1) * CAN_ESC_SLOT_STRIDE),
      0, &eepromBuffer.can.esc_index },
#else
    { "ESC_INDEX", T_UINT8, 0, 32, 0, &eepromBuffer.can.esc_index },
#endif
```

With `CAN_EXTRA_INPUTS_COUNT=4` this evaluates to max = **15**, meaning
`esc_index` can only be set to 0, 1, …, 15 via `param.GetSet`. This does not
prevent an invalid `esc_index` from being written directly to EEPROM via other
means, but it prevents it through the normal configuration interface.

---

### Step 3 — Declare `extern` variables in `Inc/signal.h`

`signal.h` is included by both `DroneCAN.c` (writer) and `main.c` (reader).
Add at the bottom of the file:

```c
// Inc/signal.h

#ifdef CAN_EXTRA_INPUTS_COUNT
/*
 * Extra CAN input channels, extracted from RawCommand slots
 * esc_index+1 … esc_index+CAN_EXTRA_INPUTS_COUNT.
 * Written from handle_RawCommand() (ISR context).
 * Read from setInput() and PeriodElapsedCallback() (interrupt context).
 * Declared volatile because they cross interrupt boundaries.
 *
 * can_timing_advance   : timing advance override, 0..32 (same units as temp_advance)
 * can_phase_offset     : additive electrical phase trim, 0..360 degrees
 * can_duty_offset      : signed duty-cycle trim, −2000..+2000 counts
 * can_commutation_scale: commutation interval scale for advance calc, 50..200 %
 */
extern volatile uint16_t can_timing_advance;      // 0..32
extern volatile uint16_t can_phase_offset;        // 0..360 (degrees)
extern volatile int16_t  can_duty_offset;         // −2000..+2000
extern volatile uint16_t can_commutation_scale;   // 50..200 (percent)
#endif
```

---

### Step 4 — Define variables in `Src/main.c`

Add definitions near the other `newinput*` globals (around line 548):

```c
// Src/main.c — after the newinput_anglePwm block (~line 548)

#ifdef CAN_EXTRA_INPUTS_COUNT
volatile uint16_t can_timing_advance    = 16;   // default: mid-range advance
volatile uint16_t can_phase_offset      = 0;    // 0 degrees
volatile int16_t  can_duty_offset       = 0;    // no trim
volatile uint16_t can_commutation_scale = 100;  // 100% = no scaling
#endif
```

The defaults apply from boot until the first RawCommand arrives carrying
populated extra slots. Choose values that are safe for uncontrolled startup.

---

### Step 5 — Extract from `RawCommand` in `Src/DroneCAN/DroneCAN.c`

Add the extraction block immediately after `set_input(this_input)` inside
`handle_RawCommand()` (around line 650). The `#if CAN_EXTRA_INPUTS_COUNT >= N`
guards mean that if the count is later reduced (e.g., to 2), slots 3 and 4 are
automatically removed from the extraction without touching this code.

```c
// Src/DroneCAN/DroneCAN.c — inside handle_RawCommand(), after set_input():

    set_input(this_input);   // ← existing line, do not modify

#ifdef CAN_EXTRA_INPUTS_COUNT
    {
        const uint8_t idx = eepromBuffer.can.esc_index;

        /*
         * Each guard checks two things:
         *   1. cmd.cmd.len > idx+N  : sender populated this slot
         *   2. data[idx+N] >= 0     : slot carries a valid non-negative value
         *                            (omitted for signed can_duty_offset)
         * If the sender does not populate the slot, the variable retains its
         * previous value — giving backward-compatible behaviour.
         */

#if CAN_EXTRA_INPUTS_COUNT >= 1
        /* Slot +1: timing advance.  Wire 0..8191 → internal 0..32 */
        if (cmd.cmd.len > (uint8_t)(idx + 1) &&
            cmd.cmd.data[idx + 1] >= 0) {
            can_timing_advance =
                (uint16_t)((cmd.cmd.data[idx + 1] * 32UL) / 8191UL);
        }
#endif

#if CAN_EXTRA_INPUTS_COUNT >= 2
        /* Slot +2: phase offset.  Wire 0..8191 → 0..360 degrees */
        if (cmd.cmd.len > (uint8_t)(idx + 2) &&
            cmd.cmd.data[idx + 2] >= 0) {
            can_phase_offset =
                (uint16_t)((cmd.cmd.data[idx + 2] * 360UL) / 8191UL);
        }
#endif

#if CAN_EXTRA_INPUTS_COUNT >= 3
        /* Slot +3: duty offset (signed).  Wire −8191..+8191 → −2000..+2000 */
        if (cmd.cmd.len > (uint8_t)(idx + 3)) {
            can_duty_offset =
                (int16_t)((cmd.cmd.data[idx + 3] * 2000L) / 8191L);
        }
#endif

#if CAN_EXTRA_INPUTS_COUNT >= 4
        /* Slot +4: commutation scale.  Wire 0..8191 → 50..200 percent */
        if (cmd.cmd.len > (uint8_t)(idx + 4) &&
            cmd.cmd.data[idx + 4] >= 0) {
            can_commutation_scale =
                (uint16_t)(50UL + (cmd.cmd.data[idx + 4] * 150UL) / 8191UL);
        }
#endif
    }
#endif // CAN_EXTRA_INPUTS_COUNT
```

**Notes on safety**:
- Casts to `uint8_t` on the length comparison prevent wrap-around if `idx`
  is large (in practice `idx` ≤ 15 thanks to the parameter max, but explicit
  is safer).
- `can_duty_offset` omits the `>= 0` guard because negative values are valid
  for that channel.

---

### Step 6 — Consume in `setInput()` and `PeriodElapsedCallback()`

#### 6a. `setInput()` — duty-cycle trim

In `Src/main.c`, after the `duty_cycle_setpoint = map(...)` line (~line 1142):

```c
// Src/main.c — inside setInput(), after duty_cycle_setpoint is computed

#if defined(CAN_EXTRA_INPUTS_COUNT) && CAN_EXTRA_INPUTS_COUNT >= 3
    if (armed) {
        /* Apply signed duty trim.  Clamp to valid PWM range. */
        int32_t trimmed = (int32_t)duty_cycle_setpoint + can_duty_offset;
        if (trimmed < minimum_duty_cycle) trimmed = minimum_duty_cycle;
        if (trimmed > 2000)              trimmed = 2000;
        duty_cycle_setpoint = (uint16_t)trimmed;
    }
#endif
```

The guard `CAN_EXTRA_INPUTS_COUNT >= 3` ensures this block is only compiled
when `can_duty_offset` is actually extracted (slot +3).

#### 6b. `PeriodElapsedCallback()` — advance and commutation scale

In `Src/main.c`, replace the `advance` calculation block (~line 890):

```c
// Src/main.c — inside PeriodElapsedCallback()

    commutation_interval = ((commutation_interval) +
                            ((lastzctime + thiszctime) >> 1)) >> 1;

#ifdef CAN_EXTRA_INPUTS_COUNT
    /*
     * Scale commutation_interval for advance calculation only.
     * can_commutation_scale 50..200 %:
     *   100 → no change
     *    50 → acts as if motor runs twice as fast (less advance margin)
     *   200 → acts as if motor runs half as fast (more advance margin)
     * The BEMF-tracking interval itself is unaffected.
     */
#if CAN_EXTRA_INPUTS_COUNT >= 4
    const uint32_t scaled_interval =
        (commutation_interval * (uint32_t)can_commutation_scale) / 100UL;
#else
    const uint32_t scaled_interval = commutation_interval;
#endif

    /* Additive phase trim: convert degrees to timer counts */
#if CAN_EXTRA_INPUTS_COUNT >= 2
    const uint32_t phase_offset_counts =
        (scaled_interval * (uint32_t)can_phase_offset) / 360UL;
#else
    const uint32_t phase_offset_counts = 0;
#endif

    /* Timing advance */
#if CAN_EXTRA_INPUTS_COUNT >= 1
    if (!eepromBuffer.auto_advance) {
        advance = (scaled_interval * can_timing_advance) >> 6;
    } else {
        advance = (scaled_interval * auto_advance_level) >> 6;
    }
#else
    if (!eepromBuffer.auto_advance) {
        advance = (commutation_interval * temp_advance) >> 6;
    } else {
        advance = (commutation_interval * auto_advance_level) >> 6;
    }
#endif
    advance += phase_offset_counts;

#else  // no CAN_EXTRA_INPUTS_COUNT — original behaviour unchanged
    if (!eepromBuffer.auto_advance) {
        advance = (commutation_interval * temp_advance) >> 6;
    } else {
        advance = (commutation_interval * auto_advance_level) >> 6;
    }
#endif // CAN_EXTRA_INPUTS_COUNT

    waitTime = (commutation_interval >> 1) - advance;
    /* Clamp: advance must not exceed half-period (unsigned wrap protection) */
    if (waitTime > (commutation_interval >> 1)) {
        waitTime = 1;
    }
```

---

### Complete diff summary

| File | Change |
|---|---|
| `Inc/targets.h` | Add `#define CAN_EXTRA_INPUTS_COUNT 4` |
| `Inc/signal.h` | Add `extern` declarations under `#ifdef CAN_EXTRA_INPUTS_COUNT` |
| `Src/main.c` | Add variable definitions; modify `setInput()` and `PeriodElapsedCallback()` |
| `Src/DroneCAN/DroneCAN.c` | Add `CAN_ESC_SLOT_STRIDE`, `_Static_assert`s, tighten `ESC_INDEX` max, add extraction block |

---

## 7. Value Mapping Quick Reference

### RawCommand slot layout with `CAN_EXTRA_INPUTS_COUNT = 4`, `CAN_ESC_SLOT_STRIDE = 5`

```
Sender must populate cmd[] with this layout:

  cmd[0]   throttle ESC pos 0  (esc_index must be 0)
  cmd[1]   can_timing_advance
  cmd[2]   can_phase_offset
  cmd[3]   can_duty_offset
  cmd[4]   can_commutation_scale
  cmd[5]   throttle ESC pos 1  (esc_index must be 5)
  cmd[6]   can_timing_advance
  ...
  cmd[15]  throttle ESC pos 3  (esc_index must be 15)
  cmd[16]  can_timing_advance
  cmd[17]  can_phase_offset
  cmd[18]  can_duty_offset
  cmd[19]  can_commutation_scale
```

Maximum 4 ESCs per `RawCommand` at this stride.

### Receive path (wire → internal variable)

| Slot | CAN wire (`int16`) | Internal variable | Internal range | Formula |
|---|---|---|---|---|
| `esc_index+0` | −8191..+8191 | `newinput` (via `this_input`) | 0..2047 | See §2.4 |
| `esc_index+1` | 0..+8191 | `can_timing_advance` | 0..32 | `raw × 32 / 8191` |
| `esc_index+2` | 0..+8191 | `can_phase_offset` | 0..360 | `raw × 360 / 8191` |
| `esc_index+3` | −8191..+8191 | `can_duty_offset` | −2000..+2000 | `raw × 2000 / 8191` |
| `esc_index+4` | 0..+8191 | `can_commutation_scale` | 50..200 | `50 + raw × 150 / 8191` |

### Motor-control path (internal variable → hardware effect)

| Variable | Range | Used in | Hardware effect |
|---|---|---|---|
| `newinput` | 0..2047 | `setInput()` | Base throttle demand |
| `can_duty_offset` | −2000..+2000 | `setInput()` | Trims `duty_cycle_setpoint` |
| `can_timing_advance` | 0..32 | `PeriodElapsedCallback()` | Advance angle ≈ 0..30 electrical degrees |
| `can_phase_offset` | 0..360 | `PeriodElapsedCallback()` | Additive phase trim (timer counts) |
| `can_commutation_scale` | 50..200 | `PeriodElapsedCallback()` | Scales interval for advance calculation |

### Key commutation equations

```
scaled_interval = commutation_interval × can_commutation_scale / 100

advance = (scaled_interval × can_timing_advance) >> 6
        + (scaled_interval × can_phase_offset / 360)

waitTime = (commutation_interval >> 1) − advance
           [clamped to 1 if advance ≥ commutation_interval / 2]
```

The next commutation fires `waitTime` timer counts after the BEMF zero crossing.
A larger `can_timing_advance` or `can_phase_offset` commutes earlier.
`can_commutation_scale` > 100 widens the available advance window.

### `ESC_INDEX` parameter maximum as a function of `CAN_EXTRA_INPUTS_COUNT`

| `CAN_EXTRA_INPUTS_COUNT` | `CAN_ESC_SLOT_STRIDE` | Max `esc_index` (from param) | Max ESCs on bus |
|---|---|---|---|
| 0 (not defined) | — | 32 (original) | up to 20 |
| 1 | 2 | 18 | 10 |
| 2 | 3 | 18 | 7 |
| 3 | 4 | 16 | 5 |
| **4** | **5** | **15** | **4** |
| 9 | 10 | 10 | 2 |
| 19 | 20 | 0 | 1 |

---

*Generated from code review of `Src/DroneCAN/DroneCAN.c`, `Src/main.c`,
`Inc/eeprom.h`, `Inc/signal.h`, `Inc/targets.h`, and
`Src/DroneCAN/dsdl_generated/include/uavcan.equipment.esc.RawCommand.h` —
AM32 branch `AS5047_Read`.*