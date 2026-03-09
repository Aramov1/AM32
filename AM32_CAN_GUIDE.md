# AM32 DroneCAN — Complete Developer Guide

> **Scope**: This guide covers the `VIMDRONES_L431_CAN` target (and any target that defines
> `DRONECAN_SUPPORT 1`).  All file paths are relative to the AM32 repository root.

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Hardware & Compile-time Configuration](#2-hardware--compile-time-configuration)
3. [CAN Bus Initialisation](#3-can-bus-initialisation)
4. [Node Identity & Dynamic Allocation (DNA)](#4-node-identity--dynamic-allocation-dna)
5. [Received Messages (Inputs)](#5-received-messages-inputs)
   - 5.1 [RawCommand — throttle path (full pipeline)](#51-rawcommand--throttle-path-full-pipeline)
   - 5.2 [ArmingStatus](#52-armingstatus)
   - 5.3 [Service requests](#53-service-requests)
6. [Transmitted Messages (Telemetry & Status)](#6-transmitted-messages-telemetry--status)
   - 6.1 [NodeStatus (1 Hz)](#61-nodestatus-1-hz)
   - 6.2 [ESCStatus (configurable rate)](#62-escstatus-configurable-rate)
   - 6.3 [FlexDebug (configurable rate)](#63-flexdebug-configurable-rate)
   - 6.4 [LogMessage (printf over CAN)](#64-logmessage-printf-over-can)
7. [EEPROM Parameters (CAN-specific)](#7-eeprom-parameters-can-specific)
8. [Signal Timeout & Watchdog](#8-signal-timeout--watchdog)
9. [How to Introduce Code Changes](#9-how-to-introduce-code-changes)
10. [Adding New Input Variables to the CAN Message](#10-adding-new-input-variables-to-the-can-message)
    - 10.1 [Step 1 — Target-guarded declarations in `signal.h` / `targets.h`](#101-step-1--target-guarded-declarations)
    - 10.2 [Step 2 — Define the variables in `main.c`](#102-step-2--define-the-variables-in-mainc)
    - 10.3 [Step 3 — Extract from `RawCommand` in `DroneCAN.c`](#103-step-3--extract-from-rawcommand-in-dronecanc)
    - 10.4 [Step 4 — Consume in `setInput()` / motor control](#104-step-4--consume-in-setinput--motor-control)
    - 10.5 [Full worked example — `newinput_phaseoffset` & `newinput_amplitude`](#105-full-worked-example)
11. [Sending New Messages Back (Telemetry Output)](#11-sending-new-messages-back)
    - 11.1 [Using the existing FlexDebug channel](#111-using-the-existing-flexdebug-channel)
    - 11.2 [Adding a brand-new broadcast message](#112-adding-a-brand-new-broadcast-message)
    - 11.3 [Full worked example — custom status packet](#113-full-worked-example--custom-status-packet)
12. [DroneCAN GUI App — Interaction Guide](#12-dronecan-gui-app--interaction-guide)

---

## 1. Architecture Overview

```
┌────────────────────────────────────────────────────────────────┐
│                        STM32L431 (ESC)                         │
│                                                                │
│  CAN peripheral (bxCAN)                                        │
│   PA11 = CAN_RX    PA12 = CAN_TX   @1 Mbps                    │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  CAN1_RX0_IRQHandler / CAN1_RX1_IRQHandler              │  │
│  │    └─ DroneCAN_receiveFrame()                            │  │
│  │         └─ canardHandleRxFrame()                         │  │
│  │              ├─ shouldAcceptTransfer()  ← filter         │  │
│  │              └─ onTransferReceived()   ← dispatch        │  │
│  │                   ├─ handle_RawCommand()                 │  │
│  │                   ├─ handle_ArmingStatus()               │  │
│  │                   ├─ handle_GetNodeInfo()                │  │
│  │                   ├─ handle_param_GetSet()               │  │
│  │                   ├─ handle_param_ExecuteOpcode()        │  │
│  │                   ├─ handle_RestartNode()                │  │
│  │                   ├─ handle_begin_firmware_update()      │  │
│  │                   └─ handle_DNA_Allocation()             │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                │
│  Main loop (every iteration)                                   │
│    DroneCAN_update()                                           │
│      ├─ DroneCAN_processTxQueue()  ← flush TX frames          │
│      ├─ request_DNA()              ← if node ID not yet set   │
│      ├─ process1HzTasks()          ← NodeStatus + CAN term    │
│      ├─ send_ESCStatus()           ← at telem_rate Hz         │
│      ├─ send_FlexDebug()           ← at debug_rate Hz         │
│      └─ input watchdog / 1 kHz re-apply                       │
│                                                                │
│  Shared variables (written by CAN, read by motor control)      │
│    newinput          ← throttle (0..2047)                      │
│    inputSet          ← 1 once first valid command received     │
│    armed             ← set by arming logic                     │
│    dshot             ← forced = bi_direction for CAN mode      │
└────────────────────────────────────────────────────────────────┘
```

The CAN stack is built on **libcanard** (a lightweight UAVCAN/DroneCAN library).
The ESC implements the standard `uavcan.equipment.esc` namespace and a subset of
`uavcan.protocol` services, making it compatible with any DroneCAN ground-station
tool (Mission Planner, DroneCAN GUI Tool, custom Python scripts, the ESC_CAN_APP).

---

## 2. Hardware & Compile-time Configuration

### 2.1 Enabling DroneCAN for a target

In `Inc/targets.h`, add the following to your target block:

```c
#ifdef MY_TARGET
#define FIRMWARE_NAME        "MyTarget"
#define FILE_NAME            "MY_TARGET"
#define DRONECAN_SUPPORT     1                       // ← enables the entire stack
#define DRONECAN_NODE_NAME   "com.vendor.my_esc"    // ← shown in GUI tools
#define DEAD_TIME            45
#define HARDWARE_GROUP_L4_B
#define TARGET_VOLTAGE_DIVIDER 94
#define MILLIVOLT_PER_AMP      30
#define CURRENT_OFFSET         110
#define USE_SERIAL_TELEMETRY
#endif
```

`DRONECAN_SUPPORT 1` is the single compile-time gate that enables:
- `Src/DroneCAN/DroneCAN.c`
- `Src/DroneCAN/sys_can_stm32.c` (for `MCU_L431`)
- The `#if DRONECAN_SUPPORT` blocks in `Src/main.c` and `Src/dshot.c`

### 2.2 Current-sensor calibration

| Define | Effect |
|---|---|
| `MILLIVOLT_PER_AMP` | Converts raw ADC counts to milliamps. Lower value = more sensitive sensor. |
| `CURRENT_OFFSET` | ADC zero-current offset (subtracted before conversion). |

These directly affect the `pkt.current` field in `send_ESCStatus()`.

### 2.3 CAN physical layer

File: `Src/DroneCAN/sys_can_stm32.c`

- Pins: **PA11** (CAN_RX) and **PA12** (CAN_TX), Alternate Function 9.
- Speed: **1 Mbps** (hardcoded for 80 MHz PCLK1: prescaler=8, BS1=8, BS2=1, SJW=1).
- IRQ priority: **5** for RX0, RX1, and TX.
- FIFO overflow is counted in `canstats.rx_errors`.

---

## 3. CAN Bus Initialisation

Call order at first entry to `DroneCAN_update()` (guarded by `done_startup`):

```
DroneCAN_update()
  └─ DroneCAN_Startup()             [DroneCAN.c:1123]
       ├─ load_settings()            ← validate EEPROM CAN params
       ├─ canardInit()               ← init libcanard instance
       ├─ canardSetLocalNodeID()     ← if can_node != 0 in EEPROM
       ├─ sys_can_init()             ← configure bxCAN peripheral + IRQs
       └─ if input_type == DRONECAN_IN:
            NVIC_DisableIRQ(DMA1_Channel5_IRQn)   ← kills DShot DMA
            NVIC_DisableIRQ(EXTI15_10_IRQn)        ← kills signal-pin EXTI
            EXTI->IMR1 &= ~(1U << 15)
```

> **Important**: When `eepromBuffer.input_type` is `DRONECAN_IN` (=5), the physical
> signal pin is permanently disconnected at firmware level.  The ESC only accepts
> throttle from the CAN bus.  When `input_type` is anything else, both the physical
> pin and CAN bus are active simultaneously; whichever writes `newinput` last wins.

---

## 4. Node Identity & Dynamic Allocation (DNA)

| Parameter | EEPROM field | Default |
|---|---|---|
| Static node ID | `eepromBuffer.can.can_node` | 0 (= use DNA) |

If `can_node == 0` the firmware runs the **UAVCAN Dynamic Node-ID Allocation**
protocol until a node ID is assigned by an allocator on the bus.

DNA request cycle (`request_DNA`, `DroneCAN.c:796`):
1. Broadcasts `uavcan.protocol.dynamic_node_id.Allocation` with a random back-off.
2. Allocator replies with a node-ID offer.
3. Firmware stores it: `canardSetLocalNodeID()`.

Until the node ID is resolved, `DroneCAN_update()` returns early — no telemetry
is sent and no throttle commands are processed.

---

## 5. Received Messages (Inputs)

### 5.1 RawCommand — throttle path (full pipeline)

**Message**: `uavcan.equipment.esc.RawCommand` (ID 1030)
**File**: `Src/DroneCAN/dsdl_generated/include/uavcan.equipment.esc.RawCommand.h`

```
struct uavcan_equipment_esc_RawCommand {
    struct { uint8_t len; int16_t data[20]; } cmd;
};
```

Each `data[i]` is a **signed 14-bit value** encoded on the wire (`-8191` … `+8191`).
A single message can carry commands for up to **20 ESCs simultaneously**.

#### Stage-by-stage pipeline

```
CAN bus frame(s)
  ↓ (interrupt)
canardHandleRxFrame()      ← re-assembles multi-frame transfers
  ↓
onTransferReceived()       ← called when complete message assembled
  ↓
handle_RawCommand()        [DroneCAN.c:609]
  │
  │  1. Decode:
  │       uavcan_equipment_esc_RawCommand_decode(transfer, &cmd)
  │
  │  2. Index check:
  │       if cmd.cmd.len <= esc_index → discard (not for us)
  │
  │  3. Extract this ESC's value:
  │       input_can = cmd.cmd.data[eepromBuffer.can.esc_index]
  │            range: -8191 … +8191
  │
  │  4. Map to AM32 internal range (0 … 2047):
  │
  │       Unidirectional (bi_direction == 0):
  │         input_can == 0  → this_input = 0
  │         input_can  > 0  → this_input = 47 + (input_can * 2000.0 / 8192)
  │                           range: 47 … 2047
  │
  │       Bidirectional (bi_direction == 1):
  │         input_can == 0  → this_input = 0
  │         input_can  > 0  → this_input = 1047 + (input_can * 1000.0 / 8192)
  │                           range: 1047 … 2047  (forward)
  │         input_can  < 0  → this_input = 47 + (|input_can| * 1000.0 / 8192)
  │                           range: 47 … 1047    (reverse)
  │
  │  5. Update statistics:
  │       canstats.num_commands++, canstats.total_commands++
  │       canstats.last_raw_command_us = micros64()
  │
  └─ set_input(this_input)    [DroneCAN.c:582]
       │
       │  6. Arming gate:
       │       if require_arming && !dronecan_armed → force input = 0
       │       (but allow restart if ESC rebooted mid-flight)
       │
       │  7. Low-pass filter (2-pole Butterworth):
       │       filtered = Filter2P_apply(unfiltered, filter_hz, 1000)
       │       filter_hz = 0 → no filtering (pass-through)
       │
       │  8. Write shared state:
       │       newinput  = filtered_input        ← consumed by setInput()
       │       inputSet  = 1                     ← tells main loop input is live
       │       dshot     = bi_direction           ← needed for bi-dir logic
       │
       │  9. Trigger motor-control path:
       │       transfercomplete()   ← same function as DShot/PWM DMA completion
       │       setInput()           [main.c:947]
       │         │
       │         │  10. Direction/bi-dir mapping → adjusted_input
       │         │       Unidirectional: adjusted_input = newinput
       │         │       Bidirectional (servo):
       │         │         newinput > 1000+deadband → forward, map to 47…2047
       │         │         newinput < 1000-deadband → reverse, map to 2047…47
       │         │       Bidirectional (dshot):
       │         │         newinput > 1047 → forward, map to 47…2047
       │         │         newinput 47…1047 → reverse, map to 47…2047
       │         │         newinput < 48 → stop (adjusted_input = 0)
       │         │
       │         │  11. Throttle to duty cycle:
       │         │       input = adjusted_input   (if no speed control loop)
       │         │       OR speed/current PID overrides
       │         │       Range: 0 (off) … 2047 (full)
       │         │
       │         └─ duty_cycle_setpoint = map(input, 47, 2047,
       │                                      minimum_duty_cycle, 2000)
       │
       └─ canstats.num_input++
```

#### AM32 internal value encoding

| `input` value | Meaning |
|---|---|
| 0 | Motor off / brake |
| 1–46 | Reserved / special DShot codes |
| 47 | Minimum throttle (motor starts) |
| 48–2047 | Throttle demand, linear |
| 2047 | Full throttle |

### 5.2 ArmingStatus

**Message**: `uavcan.equipment.safety.ArmingStatus` (broadcast)

```c
// handle_ArmingStatus() [DroneCAN.c:656]
dronecan_armed = (cmd.status == STATUS_FULLY_ARMED);  // 255 = armed, 0 = disarmed
if (!dronecan_armed && require_arming)
    set_input(0);   // immediate zero on disarm
```

The `dronecan_armed` flag is the master arming gate. Without it (and with
`require_arming=1` in EEPROM), all throttle commands produce zero output.

### 5.3 Service requests

| Service | Handler | Effect |
|---|---|---|
| `GetNodeInfo` | `handle_GetNodeInfo` | Returns firmware version, hardware version, unique ID, node name |
| `param.GetSet` | `handle_param_GetSet` | Read or write any of the 24 exposed EEPROM parameters |
| `param.ExecuteOpcode` | `handle_param_ExecuteOpcode` | ERASE (factory reset) or SAVE (write to flash) |
| `RestartNode` | `handle_RestartNode` | Hard reboot via `NVIC_SystemReset()` |
| `BeginFirmwareUpdate` | `handle_begin_firmware_update` | Stores update request in RTC backup registers, reboots to bootloader |

---

## 6. Transmitted Messages (Telemetry & Status)

### 6.1 NodeStatus (1 Hz)

**Message**: `uavcan.protocol.NodeStatus` (ID 341)

Sent every second from `process1HzTasks()`. Fields:

| Field | Value |
|---|---|
| `uptime_sec` | Seconds since boot (`micros64() / 1e6`) |
| `health` | `HEALTH_OK` (0) — no error reporting currently |
| `mode` | `MODE_OPERATIONAL` (0) |
| `vendor_specific_status_code` | Number of `RawCommand` messages received in the last second (command rate) |

### 6.2 ESCStatus (configurable rate)

**Message**: `uavcan.equipment.esc.Status` (ID 1034)

Sent at `eepromBuffer.can.telem_rate` Hz (default 25 Hz, 0 = disabled).

```c
// send_ESCStatus() [DroneCAN.c:1013]
pkt.voltage          = battery_voltage * 0.01;       // V  (ADC → voltage divider)
pkt.current          = avg_current * 0.01;           // A  (averaged over interval)
pkt.temperature      = C_TO_KELVIN(degrees_celsius); // K
pkt.rpm              = (e_rpm * 200) / motor_poles;  // electrical → mechanical RPM
pkt.power_rating_pct = 0;                            // not implemented
pkt.esc_index        = eepromBuffer.can.esc_index;   // identifies which ESC sent it
```

Current is accumulated in `current.sum / current.count` since the last
`send_ESCStatus()` call, then reset — giving a true average over the interval.

### 6.3 FlexDebug (configurable rate)

**Message**: `dronecan.protocol.FlexDebug` (ID 16371)

Sent at `eepromBuffer.can.debug_rate` Hz (default 0 = disabled). The payload is
an arbitrary byte blob identified by a `uint16_t id` field.

AM32 uses `id = DRONECAN_PROTOCOL_FLEXDEBUG_AM32_RESERVE_START + 0` (= 100):

```c
// The debug1 structure [DroneCAN.c:84]
static struct PACKED {
    uint8_t  version;               // = 1
    uint32_t commutation_interval;  // µs, current electrical period
    uint16_t num_commands;          // RawCommand count since last FlexDebug
    uint16_t num_input;             // set_input() count since last FlexDebug
    uint16_t rx_errors;             // CAN RX error count
    uint16_t rxframe_error;         // libcanard frame error count
    int32_t  rx_ecode;              // last libcanard error code
    uint8_t  auto_advance_level;    // auto timing advance level
} debug1;
```

### 6.4 LogMessage (printf over CAN)

**Message**: `uavcan.protocol.debug.LogMessage`

Used internally by `can_printf()` [DroneCAN.c:283]. Any `can_printf("...")` call
broadcasts a human-readable string visible in any DroneCAN GUI tool's log panel.
This is low-priority and should not be used in time-critical paths.

---

## 7. EEPROM Parameters (CAN-specific)

All CAN parameters live at the end of the EEPROM map (`eepromBuffer.can`, bytes 176–191):

| Parameter name (CAN) | EEPROM field | Type | Default | Range | Description |
|---|---|---|---|---|---|
| `CAN_NODE` | `can.can_node` | uint8 | 0 | 0–127 | Static node ID. 0 = use DNA. |
| `ESC_INDEX` | `can.esc_index` | uint8 | 0 | 0–32 | Which slot in RawCommand this ESC reads |
| `TELEM_RATE` | `can.telem_rate` | uint8 | 25 | 0–200 | ESCStatus TX rate in Hz |
| `DEBUG_RATE` | `can.debug_rate` | uint8 | 0 | 0–200 | FlexDebug TX rate in Hz |
| `REQUIRE_ARMING` | `can.require_arming` | bool | 1 | 0–1 | Block throttle until ArmingStatus=ARMED |
| `REQUIRE_ZERO_THROTTLE` | `can.require_zero_throttle` | bool | 1 | 0–1 | Require zero throttle for first arm |
| `INPUT_FILTER_HZ` | `can.filter_hz` | uint8 | 0 | 0–100 | 2-pole low-pass cutoff on throttle input |
| `CAN_TERM_ENABLE` | `can.term_enable` | bool | 0 | 0–1 | Enable CAN termination resistor (if `CAN_TERM_PIN` defined) |

General ESC parameters that also affect CAN operation:

| Parameter name | EEPROM field | Description |
|---|---|---|
| `INPUT_SIGNAL_TYPE` | `input_type` | Set to 5 (`DRONECAN_IN`) to disable physical pin |
| `BI_DIRECTIONAL` | `bi_direction` | Enables bidirectional mode; changes RawCommand mapping |
| `DIR_REVERSED` | `dir_reversed` | Reverses motor direction |
| `MOTOR_KV` | `motor_kv` | Used for low-RPM power protection thresholds |
| `MOTOR_POLES` | `motor_poles` | Used for RPM calculation in ESCStatus |
| `CURRENT_LIMIT` | `limits.current` | Activates current PID loop |
| `BRAKE_ON_STOP` | `brake_on_stop` | Active braking when input = 0 |

Parameters are exposed over the `uavcan.protocol.param.GetSet` service and can be
read/written by any DroneCAN tool without a firmware reflash.

---

## 8. Signal Timeout & Watchdog

Two independent timeout mechanisms protect against a silent CAN sender:

```c
// [DroneCAN.c:1207–1218]

// 1. Hard timeout — 250 ms with no RawCommand → force zero
if (last_raw_command_us != 0 &&
    ts - last_raw_command_us > 250000ULL) {
    last_raw_command_us = 0;
    set_input(0);
}

// 2. 1 kHz re-apply — ensures motor control sees 1 kHz update rate
//    even when CAN rate is lower (e.g. 50 Hz)
if (ts - last_raw_command_us > TARGET_PERIOD_US) {  // 1000 µs
    set_input(last_can_input);
    last_raw_command_us = ts;
}
```

Additionally, `signaltimeout` in `main.c` is reset to 0 on every call to
`onTransferReceived()`, preventing the main-loop half-second disarm from firing
while CAN messages are flowing.

---

## 9. How to Introduce Code Changes

### 9.1 Files and their roles

| File | Role |
|---|---|
| `Inc/targets.h` | Target macros. Add `#define` guards here first. |
| `Inc/signal.h` | `extern` declarations for globals shared between `signal.c`/`main.c` and DroneCAN. |
| `Src/main.c` | Global variable definitions; `setInput()`; `tenKhzRoutine()`; main loop. |
| `Src/signal.c` | `transfercomplete()`, `detectInput()`, servo/DShot compute functions. |
| `Src/DroneCAN/DroneCAN.c` | The entire DroneCAN stack. All CAN-specific logic lives here. |
| `Src/DroneCAN/DroneCAN.h` | Public interface: `DroneCAN_Init`, `DroneCAN_update`, `DroneCAN_active`. |
| `Src/DroneCAN/sys_can_stm32.c` | Low-level bxCAN driver for STM32L431. Do not modify unless changing hardware. |
| `Src/DroneCAN/dsdl_generated/` | Auto-generated message codecs. Do not edit by hand. |
| `Inc/eeprom.h` | EEPROM map. Add new persistent fields here (bytes 184–191 are reserved). |

### 9.2 Guard pattern

Always wrap DroneCAN-only code in the same guard used throughout the codebase:

```c
#if DRONECAN_SUPPORT
// ... CAN-only code ...
#endif
```

In `targets.h` use the target `#ifdef` to add target-specific defines that then
control features in `DroneCAN.c` or `main.c`.

### 9.3 Adding a new EEPROM parameter

1. Add the field in the `can` struct inside `Inc/eeprom.h` (use the 8 reserved bytes at 184–191).
2. Add a `{ "PARAM_NAME", T_UINT8, min, max, default, &eepromBuffer.can.new_field }` entry
   to the `parameters[]` array in `DroneCAN.c`.
3. The field is immediately accessible from any DroneCAN tool via param.GetSet.

---

## 10. Adding New Input Variables to the CAN Message

### 10.1 Background — the RawCommand array

The `RawCommand` message carries up to **20 signed values** (`int16_t data[20]`),
each in the range −8191 … +8191.  Currently only `data[esc_index]` (throttle) is
consumed.  Slots at `esc_index+1`, `esc_index+2`, … are received but silently
discarded.

This means you can repurpose adjacent slots as additional control channels at
**zero wire-protocol cost** — no new message type, no DSDL changes, no signature
changes.  The sender (flight controller or GUI app) simply needs to populate more
slots in the same `RawCommand`.

### 10.2 Step 1 — Target-guarded declarations

Add a `#define` to your target block in `Inc/targets.h` to enable the new inputs
only for the CAN targets that need them:

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
#define MILLIVOLT_PER_AMP      30
#define CURRENT_OFFSET         110
#define USE_SERIAL_TELEMETRY

// ── NEW: enable additional CAN input channels ─────────────────────
#define USE_CAN_EXTRA_INPUTS   // gates the new variables everywhere
// ─────────────────────────────────────────────────────────────────
#endif
```

Then add the `extern` declarations to `Inc/signal.h` inside the same guard:

```c
// Inc/signal.h  (at the bottom, before the final #endif if present)

#ifdef USE_CAN_EXTRA_INPUTS
/*
 * Additional input channels received from CAN RawCommand.
 * newinput_phaseoffset : electrical phase offset in degrees, range 0..360.
 *                        Encoded in RawCommand as 0..8191 → 0..360 degrees.
 * newinput_amplitude   : sinusoidal drive amplitude, range 0..100 percent.
 *                        Encoded in RawCommand as 0..8191 → 0..100 %.
 */
extern volatile uint16_t newinput_phaseoffset;   // 0..360  (degrees)
extern volatile uint16_t newinput_amplitude;     // 0..100  (percent × 10 for precision)
#endif
```

### 10.3 Step 2 — Define the variables in `main.c`

```c
// Src/main.c  — near the other newinput* declarations (around line 538)

#ifdef USE_CAN_EXTRA_INPUTS
volatile uint16_t newinput_phaseoffset = 0;   // degrees, 0..360
volatile uint16_t newinput_amplitude   = 0;   // percent×10, 0..1000
#endif
```

### 10.4 Step 3 — Extract from `RawCommand` in `DroneCAN.c`

Modify `handle_RawCommand()` to read the extra slots.  The convention used here
is:

| Slot | Variable | Sender range | Meaning |
|---|---|---|---|
| `esc_index + 0` | throttle (existing) | −8191 … +8191 | Motor throttle |
| `esc_index + 1` | `newinput_phaseoffset` | 0 … +8191 | Phase offset 0°–360° |
| `esc_index + 2` | `newinput_amplitude` | 0 … +8191 | Amplitude 0–100 % |

```c
// Src/DroneCAN/DroneCAN.c — inside handle_RawCommand(), after set_input():

#ifdef USE_CAN_EXTRA_INPUTS
    {
        const uint8_t idx = eepromBuffer.can.esc_index;

        // Phase offset: slot esc_index+1, 0..8191 → 0..360 degrees
        if (cmd.cmd.len > idx + 1 && cmd.cmd.data[idx + 1] >= 0) {
            newinput_phaseoffset =
                (uint16_t)((cmd.cmd.data[idx + 1] * 360UL) / 8191UL);
        }

        // Amplitude: slot esc_index+2, 0..8191 → 0..1000 (= 0..100.0 %)
        if (cmd.cmd.len > idx + 2 && cmd.cmd.data[idx + 2] >= 0) {
            newinput_amplitude =
                (uint16_t)((cmd.cmd.data[idx + 2] * 1000UL) / 8191UL);
        }
    }
#endif
```

> **Bounds check**: The `cmd.cmd.len > idx + N` guard ensures that if the sender
> only fills the throttle slot (backward-compatible), the extra variables simply
> retain their previous value (defaulting to 0 on startup).

### 10.5 Step 4 — Consume in `setInput()` / motor control

In `Src/main.c`, inside `setInput()` or `tenKhzRoutine()`, use the variables
under the same guard so they have no effect on non-CAN builds:

```c
// Example usage in setInput() or tenKhzRoutine()

#ifdef USE_CAN_EXTRA_INPUTS
    if (armed && running) {
        // newinput_phaseoffset and newinput_amplitude are now valid.
        // Pass to whatever motor algorithm needs them.
        // Example: call a hypothetical sine-drive routine:
        // setSineDriveParameters(newinput_phaseoffset, newinput_amplitude);
    }
#endif
```

### 10.5 Full worked example

Below is the complete, minimal diff needed to add `newinput_phaseoffset` and
`newinput_amplitude` to the `VIMDRONES_L431_CAN` target.

#### `Inc/targets.h` — add `USE_CAN_EXTRA_INPUTS`

```c
#ifdef VIMDRONES_L431_CAN
#define FIRMWARE_NAME          "VimdroneL431"
#define FILE_NAME              "VIMDRONES_L431_CAN"
#define DRONECAN_SUPPORT       1
#define DRONECAN_NODE_NAME     "com.vimdrones.esc_dev"
#define DEAD_TIME              45
#define HARDWARE_GROUP_L4_B
#define TARGET_VOLTAGE_DIVIDER 94
#define MILLIVOLT_PER_AMP      30
#define CURRENT_OFFSET         110
#define USE_SERIAL_TELEMETRY
#define USE_CAN_EXTRA_INPUTS          // ← ADD THIS
#endif
```

#### `Inc/signal.h` — extern declarations

```c
// Add at the bottom of Inc/signal.h

#ifdef USE_CAN_EXTRA_INPUTS
extern volatile uint16_t newinput_phaseoffset;  // 0..360 degrees
extern volatile uint16_t newinput_amplitude;    // 0..1000 (= 0..100.0 %)
#endif
```

#### `Src/main.c` — variable definitions

```c
// Add near line 548, alongside other newinput* variables:

#ifdef USE_CAN_EXTRA_INPUTS
volatile uint16_t newinput_phaseoffset = 0;
volatile uint16_t newinput_amplitude   = 0;
#endif
```

#### `Src/DroneCAN/DroneCAN.c` — extract from RawCommand

In `handle_RawCommand()`, after the existing `set_input(this_input);` call:

```c
    set_input(this_input);   // ← existing line

#ifdef USE_CAN_EXTRA_INPUTS
    {
        const uint8_t idx = eepromBuffer.can.esc_index;

        if (cmd.cmd.len > idx + 1 && cmd.cmd.data[idx + 1] >= 0) {
            newinput_phaseoffset =
                (uint16_t)((cmd.cmd.data[idx + 1] * 360UL) / 8191UL);
        }
        if (cmd.cmd.len > idx + 2 && cmd.cmd.data[idx + 2] >= 0) {
            newinput_amplitude =
                (uint16_t)((cmd.cmd.data[idx + 2] * 1000UL) / 8191UL);
        }
    }
#endif
```

#### Sender side (Python / ESC_CAN_APP)

```python
import dronecan

node = dronecan.make_node('/dev/ttyUSB0', node_id=127, bitrate=1_000_000)

ESC_INDEX       = 0          # this ESC's index
PHASE_RAW       = 2048       # ≈ 90° (2048/8191 × 360 ≈ 90°)
AMPLITUDE_RAW   = 4096       # ≈ 50 % (4096/8191 × 100 ≈ 50 %)
THROTTLE_RAW    = 1000       # throttle

# The array must have at least ESC_INDEX+3 elements
cmd = [0] * (ESC_INDEX + 3)
cmd[ESC_INDEX + 0] = THROTTLE_RAW
cmd[ESC_INDEX + 1] = PHASE_RAW
cmd[ESC_INDEX + 2] = AMPLITUDE_RAW

node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=cmd))
node.spin(timeout=0.1)
```

---

## 11. Sending New Messages Back

### 11.1 Using the existing FlexDebug channel

The easiest way to add new telemetry is to extend the existing `debug1` struct and
increase its `id` counter.  `FlexDebug` supports multiple independent packets, each
identified by a unique `id` value.  AM32 reserves IDs 100–109 (`AM32_RESERVE_START`).

```c
// Src/DroneCAN/DroneCAN.c

// 1. Define a second packed struct (after the existing debug1):
#ifdef USE_CAN_EXTRA_INPUTS
static struct PACKED {
    uint8_t  version;
    uint16_t phaseoffset;   // current phase offset in degrees
    uint16_t amplitude;     // current amplitude × 10
} debug2;
#endif

// 2. Add a send function:
#ifdef USE_CAN_EXTRA_INPUTS
static void send_FlexDebug2(void)
{
    debug2.version     = 1;
    debug2.phaseoffset = newinput_phaseoffset;
    debug2.amplitude   = newinput_amplitude;

    struct dronecan_protocol_FlexDebug pkt;
    uint8_t buffer[DRONECAN_PROTOCOL_FLEXDEBUG_MAX_SIZE];

    pkt.id      = DRONECAN_PROTOCOL_FLEXDEBUG_AM32_RESERVE_START + 1; // id=101
    pkt.u8.len  = sizeof(debug2);
    memcpy(pkt.u8.data, (const uint8_t *)&debug2, sizeof(debug2));
    uint32_t len = dronecan_protocol_FlexDebug_encode(&pkt, buffer);

    static uint8_t transfer_id;
    canardBroadcast(&canard,
                    DRONECAN_PROTOCOL_FLEXDEBUG_SIGNATURE,
                    DRONECAN_PROTOCOL_FLEXDEBUG_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer, len);
}
#endif

// 3. Call it from DroneCAN_update() alongside send_FlexDebug():
#ifdef USE_CAN_EXTRA_INPUTS
    if (eepromBuffer.can.debug_rate > 0 && ts >= next_flexdebug_at) {
        send_FlexDebug2();
    }
#endif
```

**Receiver side (Python)**:

```python
def on_flexdebug(event):
    msg = event.message
    if msg.id == 101:           # AM32_RESERVE_START + 1
        data = bytes(msg.u8)    # raw bytes
        version, phaseoffset, amplitude = struct.unpack_from('<BHH', data)
        print(f"Phase: {phaseoffset}°  Amplitude: {amplitude/10:.1f}%")

node.add_handler(dronecan.dronecan.protocol.FlexDebug, on_flexdebug)
```

### 11.2 Adding a brand-new broadcast message

For more structured telemetry, define a new custom DroneCAN message:

1. **Write a DSDL file** (e.g. `20000.CustomESCStatus.uavcan`):
   ```
   # com.vimdrones.esc_dev.CustomESCStatus
   # Data type ID: 20000 (in the vendor-specific range 20000–21999)
   uint16 phase_offset_deg   # 0..360
   uint16 amplitude_pct10    # 0..1000  (= 0..100.0 %)
   uint8  esc_index
   ```

2. **Generate C codec** using `dronecan_dsdlc` (or the existing generator script):
   ```bash
   python3 dronecan_dsdlc \
       --output-dir Src/DroneCAN/dsdl_generated/ \
       20000.CustomESCStatus.uavcan
   ```
   This creates `include/com.vimdrones.esc_dev.CustomESCStatus.h` and
   `src/com.vimdrones.esc_dev.CustomESCStatus.c`.

3. **Add to `dronecan_msgs.h`**:
   ```c
   #include "com.vimdrones.esc_dev.CustomESCStatus.h"
   ```

4. **Broadcast in `DroneCAN.c`**:
   ```c
   static void send_CustomESCStatus(void)
   {
       struct com_vimdrones_esc_dev_CustomESCStatus pkt;
       uint8_t buffer[COM_VIMDRONES_ESC_DEV_CUSTOMESCSTATUS_MAX_SIZE];

       pkt.phase_offset_deg = newinput_phaseoffset;
       pkt.amplitude_pct10  = newinput_amplitude;
       pkt.esc_index        = eepromBuffer.can.esc_index;

       uint32_t len = com_vimdrones_esc_dev_CustomESCStatus_encode(&pkt, buffer);
       static uint8_t tid;
       canardBroadcast(&canard,
                       COM_VIMDRONES_ESC_DEV_CUSTOMESCSTATUS_SIGNATURE,
                       COM_VIMDRONES_ESC_DEV_CUSTOMESCSTATUS_ID,
                       &tid,
                       CANARD_TRANSFER_PRIORITY_LOW,
                       buffer, len);
   }
   ```

5. **Subscribe in Python**:
   ```python
   import dronecan
   # Point dronecan at your DSDL directory:
   dronecan.load_dsdl('path/to/your/dsdl')

   def on_custom(event):
       m = event.message
       print(f"Phase: {m.phase_offset_deg}°  "
             f"Amplitude: {m.amplitude_pct10/10:.1f}%  "
             f"ESC: {m.esc_index}")

   node.add_handler(dronecan.com.vimdrones.esc_dev.CustomESCStatus, on_custom)
   ```

### 11.3 Full worked example — custom status packet

This extends the full worked example from §10 with a matching telemetry output.

Add to `Src/DroneCAN/DroneCAN.c`:

```c
// ── Custom telemetry for newinput_phaseoffset / newinput_amplitude ──

#ifdef USE_CAN_EXTRA_INPUTS

static struct PACKED {
    uint8_t  version;        // always 1
    uint16_t phaseoffset;    // degrees, 0..360
    uint16_t amplitude;      // percent×10, 0..1000
    uint8_t  esc_index;
} custom_status;

static void send_CustomStatus(void)
{
    custom_status.version    = 1;
    custom_status.phaseoffset = newinput_phaseoffset;
    custom_status.amplitude   = newinput_amplitude;
    custom_status.esc_index   = eepromBuffer.can.esc_index;

    struct dronecan_protocol_FlexDebug pkt;
    uint8_t buffer[DRONECAN_PROTOCOL_FLEXDEBUG_MAX_SIZE];
    pkt.id     = DRONECAN_PROTOCOL_FLEXDEBUG_AM32_RESERVE_START + 1;
    pkt.u8.len = sizeof(custom_status);
    memcpy(pkt.u8.data, (const uint8_t *)&custom_status, sizeof(custom_status));

    uint32_t len = dronecan_protocol_FlexDebug_encode(&pkt, buffer);
    static uint8_t tid;
    canardBroadcast(&canard,
                    DRONECAN_PROTOCOL_FLEXDEBUG_SIGNATURE,
                    DRONECAN_PROTOCOL_FLEXDEBUG_ID,
                    &tid,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer, len);
}

#endif // USE_CAN_EXTRA_INPUTS
```

In `DroneCAN_update()`, call `send_CustomStatus()` at the debug rate:

```c
    if (eepromBuffer.can.debug_rate > 0 && ts >= next_flexdebug_at) {
        next_flexdebug_at += 1000000ULL / eepromBuffer.can.debug_rate;
        send_FlexDebug();
#ifdef USE_CAN_EXTRA_INPUTS
        send_CustomStatus();
#endif
    }
```

---

## 12. DroneCAN GUI App — Interaction Guide

The `ESC_CAN_APP/` directory contains a PyQt6 application for controlling and
monitoring the `VIMDRONES_L431_CAN` ESC.  It connects via a VIMDRONES USB-to-CAN
adapter (SLCAN over serial) and communicates using the standard `dronecan` Python
library.

### 12.1 Hardware connection

```
PC ─── USB ─── VIMDRONES USB-to-CAN adapter ─── CAN bus ─── ESC(s)
                     (SLCAN protocol, /dev/ttyUSB0)          120Ω termination at each end
```

The adapter presents as a standard serial port.  The `dronecan` library
auto-detects SLCAN when the path matches a serial device pattern.

### 12.2 Starting the application

```bash
cd ESC_CAN_APP
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 main.py
```

### 12.3 Config Tab — defining control channels

Before connecting, define one or more *entities* (control channels).  Each entity
maps a user-facing display value to a raw CAN integer in the RawCommand array.

| Field | Meaning |
|---|---|
| **Name** | Human-readable label (e.g. "Throttle", "Phase Offset") |
| **Signal Type** | Preset: Throttle, Servo (±), Angle, RPM, or Custom |
| **ESC Index** | Which slot in the RawCommand `data[]` array this entity writes to |
| **Target Node ID** | 255 = broadcast to all nodes |
| **Display Min/Max/Step/Default** | The range shown in the slider/spinbox |
| **Raw Min/Raw Max** | The raw values (0–8191) that map to display Min and Max |

**Example — three-channel setup** for the extended inputs:

| Name | ESC Index | Display range | Raw range |
|---|---|---|---|
| Throttle | 0 | 0–100 % | 0–8191 |
| Phase Offset | 1 | 0–360 ° | 0–8191 |
| Amplitude | 2 | 0–100 % | 0–8191 |

The linear transform applied by the app:

```
raw = raw_min + (display - display_min) / (display_max - display_min)
              × (raw_max - raw_min)
```

The live **Preview** line at the bottom of the form shows you the mapping for
the min, mid, and max display values before you save.

### 12.4 Connect Tab — runtime operation

```
┌─ CAN Connection ────────────┐   ┌─ Live ESC Telemetry ──────┐
│ Interface Port: /dev/ttyUSB0│   │ ESC idx 0                 │
│ Bitrate:        1 000 000   │   │ Voltage : 12.34 V  ████   │
│ Own Node ID:    127         │   │ Current :  2.10 A  ██     │
│ Send Rate:      10 Hz       │   │ Temp    : 35.0 °C         │
│ [Connect] [Disconnect]      │   │ RPM     :  4200           │
│ Status: ● Connected         │   │ Power   :  42 %           │
└─────────────────────────────┘   └───────────────────────────┘

┌─ Controls ───────────────────────────────────────────────────┐
│ [Arm (FULLY_ARMED)]  [Disarm]    ● ARMED   [⛔ Emergency]   │
│                                                              │
│ Throttle  idx 0 · node 255  [═══════════════]  50.0 %  raw: 4096 │
│ Phase Off idx 1 · node 255  [═══════════]      90.0 °  raw: 2048 │
│ Amplitude idx 2 · node 255  [══════════════]   50.0 %  raw: 4096 │
└──────────────────────────────────────────────────────────────┘

┌─ Log ────────────────────────────────────────────────────────┐
│ [SAFETY] Boot sequence — zeroing all channels               │
│ [TX]     RawCommand [0, 0, 0]  (3/s)                        │
│ [SAFETY] Boot sequence complete — motor armed               │
│ [TX]     RawCommand [4096, 2048, 4096]  (10/s)              │
│ [RX]     ESC Status  idx=0  12.34V  2.10A  4200rpm  42%pwr  │
└──────────────────────────────────────────────────────────────┘
```

#### Safety sequence on connect (automatic)

1. All entity values are zeroed.
2. Three zero `RawCommand` frames are sent (satisfies `REQUIRE_ZERO_THROTTLE`).
3. `ArmingStatus = FULLY_ARMED` is sent (satisfies `REQUIRE_ARMING`).
4. The control panel is enabled.

#### Safety sequence on disconnect (automatic)

1. All entity values are zeroed.
2. Three zero `RawCommand` frames are sent.
3. `ArmingStatus = DISARMED` is sent.
4. The CAN node is closed.

#### Emergency Stop button

Immediately zeroes all channels, sends a zero `RawCommand`, and sends
`ArmingStatus = DISARMED`.  The motor cuts immediately.  The GUI remains
connected so you can re-arm when safe.

#### Send rate

The `RawCommand` is sent at the configured rate (5–50 Hz).  **The minimum safe
rate is 5 Hz** — the ESC's 250 ms watchdog will trigger if more than 250 ms
pass without a command.  10 Hz is a reasonable default; use 50 Hz for
responsive control.

### 12.5 Telemetry display

The telemetry panel automatically creates one card per unique ESC index seen
on the bus.  Each card shows:

- Numeric values: Voltage (V), Current (A), Temperature (°C), RPM,
  Power (%), Error Count.
- Sparkline plots (last 60 samples) for Voltage, Current, and RPM.
- A **"● Stale"** indicator if no data is received for more than 2 seconds.

Telemetry requires `TELEM_RATE > 0` in the ESC's EEPROM (set via param.GetSet
or the DroneCAN GUI Tool before connecting the app).

### 12.6 Interacting with ESC parameters via a DroneCAN GUI Tool

For full parameter access (read, write, save to flash, factory reset) use the
official **DroneCAN GUI Tool** (separate application, available from
`dronecan.github.io`):

1. Connect the USB-to-CAN adapter.
2. Open DroneCAN GUI Tool, set the SLCAN interface and 1 Mbps bitrate.
3. The ESC will appear in the *Online Nodes* panel identified by its node name
   (`com.vimdrones.esc_dev`).
4. Double-click the node to open the parameter panel.
5. Edit any parameter from the list in §7. Changed values are effective
   immediately. Use the **Save** (ExecuteOpcode) button to persist to flash.
6. Use **Restart** to reboot the ESC after changing `CAN_NODE` or
   `INPUT_SIGNAL_TYPE`.

### 12.7 Firmware update over CAN (OTA)

The firmware supports `BeginFirmwareUpdate` requests:

1. Place the new `.bin` file on the bus server (Mission Planner or the
   DroneCAN GUI Tool's firmware server).
2. Issue a `BeginFirmwareUpdate` service request to the ESC's node ID.
3. The ESC stores the file server information in RTC backup registers and
   reboots to the bootloader, which downloads and flashes the new firmware
   over CAN.

---

## Appendix — Quick Reference: Value Ranges

| Stage | Variable | Range | Notes |
|---|---|---|---|
| CAN wire | `cmd.data[i]` (int16) | −8191 … +8191 | 14 bits signed, per DSDL |
| After mapping (uni) | `this_input` | 0, 47…2047 | 0 = off |
| After mapping (bi fwd) | `this_input` | 1047…2047 | |
| After mapping (bi rev) | `this_input` | 47…1047 | |
| After filter | `newinput` | 0…2047 | |
| After setInput (uni) | `adjusted_input` | 0, 47…2047 | |
| Duty cycle map | `input` | 0, 47…2047 | |
| PWM compare | `duty_cycle_setpoint` | `minimum_duty_cycle`…2000 | |
| Extra: phase offset | `newinput_phaseoffset` | 0…360 | degrees |
| Extra: amplitude | `newinput_amplitude` | 0…1000 | = 0.0–100.0 % |
| Telemetry: voltage | `pkt.voltage` | float, V | `battery_voltage × 0.01` |
| Telemetry: current | `pkt.current` | float, A | averaged ADC, × 0.01 |
| Telemetry: RPM | `pkt.rpm` | int32 | `(e_rpm × 200) / motor_poles` |
| Telemetry: temp | `pkt.temperature` | float, K | `degrees_celsius + 273.15` |
