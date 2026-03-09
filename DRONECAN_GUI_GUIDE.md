# ESC CAN Controller — Complete GUI Guide

**Application**: `ESC_CAN_APP/`
**Hardware required**: AM32 ESC (VIMDRONES_L431_CAN firmware), VIMDRONES USB-to-CAN adapter, computer
**Protocol**: DroneCAN (UAVCAN v0) at 1 Mbps

---

## Table of Contents

1. [Installation & Launch](#1-installation--launch)
2. [Application Layout](#2-application-layout)
3. [Tab 1 — Config: Defining Control Variables](#3-tab-1--config-defining-control-variables)
   - 3.1 [What is an Entity?](#31-what-is-an-entity)
   - 3.2 [Signal Types and Default Presets](#32-signal-types-and-default-presets)
   - 3.3 [The Display → Raw Transform](#33-the-display--raw-transform)
   - 3.4 [All Form Fields Explained](#34-all-form-fields-explained)
   - 3.5 [Live Preview](#35-live-preview)
   - 3.6 [Creating, Editing, and Deleting Entities](#36-creating-editing-and-deleting-entities)
   - 3.7 [Config Persistence (entities.json)](#37-config-persistence-entitiesjson)
   - 3.8 [Adding Variables for Extended CAN Inputs](#38-adding-variables-for-extended-can-inputs)
4. [Tab 2 — Connect: Runtime Operation](#4-tab-2--connect-runtime-operation)
   - 4.1 [CAN Connection Form](#41-can-connection-form)
   - 4.2 [Safety Boot Sequence](#42-safety-boot-sequence)
   - 4.3 [Arming & Disarming](#43-arming--disarming)
   - 4.4 [Emergency Stop](#44-emergency-stop)
   - 4.5 [Controls Section — Sliders](#45-controls-section--sliders)
   - 4.6 [Disconnecting Safely](#46-disconnecting-safely)
5. [Live ESC Telemetry Panel](#5-live-esc-telemetry-panel)
   - 5.1 [Telemetry Cards](#51-telemetry-cards)
   - 5.2 [Sparkline Plots](#52-sparkline-plots)
   - 5.3 [Stale Detection](#53-stale-detection)
   - 5.4 [Enabling Telemetry on the ESC](#54-enabling-telemetry-on-the-esc)
6. [Log Panel](#6-log-panel)
7. [How Data Flows: End-to-End](#7-how-data-flows-end-to-end)
8. [ESC Parameter Interaction](#8-esc-parameter-interaction)
9. [**Driving the Motor Without a Flight Controller**](#9-driving-the-motor-without-a-flight-controller)
   - 9.1 [Prerequisites and Safety](#91-prerequisites-and-safety)
   - 9.2 [One-time ESC Configuration](#92-one-time-esc-configuration)
   - 9.3 [Step-by-Step: First Motor Run](#93-step-by-step-first-motor-run)
   - 9.4 [What to Expect in the Log and Telemetry](#94-what-to-expect-in-the-log-and-telemetry)
   - 9.5 [Typical Test Session Checklist](#95-typical-test-session-checklist)
   - 9.6 [Safe Shutdown Procedure](#96-safe-shutdown-procedure)
10. [Troubleshooting](#10-troubleshooting)
11. [Quick Reference — Entity Field Summary](#11-quick-reference--entity-field-summary)

---

## 1. Installation & Launch

### 1.1 Requirements

| Dependency | Minimum version |
|---|---|
| Python | 3.10 |
| PyQt6 | 6.6 |
| dronecan | 1.0.25 |
| pyqtgraph | 0.13 |

### 1.2 Installation (first time only)

```bash
# From the ESC_CAN_APP directory:
cd ESC_CAN_APP

# Activate the pre-created virtual environment
source .venv/bin/activate          # Linux / macOS
# .venv\Scripts\activate           # Windows (PowerShell)

# Install dependencies
pip install -r requirements.txt

# Linux only — grant serial port access without sudo
sudo usermod -aG dialout $USER
# Log out and back in for this to take effect.
# Verify with:  groups | grep dialout
```

### 1.3 Launching

```bash
# With the venv active, from ESC_CAN_APP:
python main.py
```

The window title reads **"ESC CAN Controller — AM32 DroneCAN"**.
Minimum window size: 960 × 700 px.  Default: 1200 × 820 px.
The application uses a **dark Fusion theme** throughout.

---

## 2. Application Layout

The application has exactly **two tabs**:

```
┌────────────────────────────────────────────────────────────────┐
│  ESC CAN Controller — AM32 DroneCAN                            │
├──────────┬─────────────────────────────────────────────────────┤
│  Config  │  Connect                                            │
├──────────┴─────────────────────────────────────────────────────┤
│                                                                │
│   Tab 1 (Config): define what you want to send                 │
│   Tab 2 (Connect): connect, arm, send values, view telemetry   │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

**Important rule**: The **Config tab is locked** (read-only) while a CAN connection is
active. You must disconnect first to edit or add entities.

---

## 3. Tab 1 — Config: Defining Control Variables

### 3.1 What is an Entity?

An **entity** is a named control channel.  It describes:
- **What** you want to control (e.g. "Throttle", "Phase Offset", "Amplitude")
- **Which slot** in the DroneCAN `RawCommand` array carries the value (index 0–19)
- **How to display it** in human-friendly units
- **How to map** between the displayed value and the raw integer sent over CAN

Each entity occupies exactly **one slot** (`esc_index`) in the
`uavcan.equipment.esc.RawCommand` message.  The same message carries all entities
simultaneously in one broadcast frame (or multi-frame transfer).

Entities are stored persistently in `config/entities.json` and reloaded every time
the application starts.

### 3.2 Signal Types and Default Presets

Selecting a signal type from the dropdown fills the form with sensible defaults.
You can override any field after selecting a type.

| Signal Type | Display range | Unit | Raw range | Notes |
|---|---|---|---|---|
| **Raw** | 0 – 8191 | *(none)* | 0 – 8191 | Direct CAN integer, 1:1 passthrough. Use when you want full control over the wire value. |
| **Throttle (%)** | 0 – 100 | % | 0 – 8191 | 0 % = raw 0, 100 % = raw 8191. Most common for motor throttle. |
| **PWM (μs)** | 1000 – 2000 | μs | 0 – 8191 | Familiar RC servo range. 1000 μs = raw 0, 2000 μs = raw 8191. |
| **RPM** | 0 – *max_rpm* | RPM | 0 – 8191 | Requires you to enter Max RPM (mandatory field appears). The ESC must have a speed control loop enabled to use this. |
| **Custom** | user-defined | user-defined | user-defined | Completely free range and unit. Use for any variable that does not fit the above. |

### 3.3 The Display → Raw Transform

The conversion from slider/spinbox value to the integer sent on CAN is a
**linear clamp-and-round**:

```
raw = raw_min + (display - display_min) / (display_max - display_min)
              × (raw_max - raw_min)

Result is rounded to nearest integer and clamped to [raw_min, raw_max].
```

**Inverse** (raw → display, used for the preview):
```
display = display_min + (raw - raw_min) / (raw_max - raw_min)
                       × (display_max - display_min)
```

**Examples**:

| Signal Type | display=0 | display=50 | display=100 |
|---|---|---|---|
| Throttle (%) | raw 0 | raw 4096 | raw 8191 |
| PWM (μs) @ 1000–2000 | raw 0 | raw 4096 | raw 8191 |
| Custom 0–360° → 0–8191 | raw 0 | raw 4096 | raw 8191 |

The `raw: XXXX` readout next to each slider in the Connect tab shows the
exact integer being transmitted in real time.

### 3.4 All Form Fields Explained

```
┌─ Edit / Create Entity ─────────────────────────────────────────┐
│ Name:           [ Throttle              ]                       │
│ Signal Type:    [ Throttle (%)     ▼   ]                       │
│ Unit Label:     [ %                    ]                       │
│ ESC Index:      [ 0  ▲▼ ]  ← slot in RawCommand array         │
│ Target Node ID: [ 255▲▼ ]  ← 1-127 or 255 for broadcast       │
│                                                                │
│ ┌─ Display Range (slider domain) ──────────────────────────┐  │
│ │ Min:     [   0.0000 ]   Max:   [ 100.0000 ]              │  │
│ │ Step:    [   0.1000 ]   Default: [  0.0000 ]             │  │
│ └──────────────────────────────────────────────────────────┘  │
│                                                                │
│ ┌─ Raw CAN Range (sent value 0–8191) ──────────────────────┐  │
│ │ Raw Min: [   0 ▲▼ ]   Raw Max:  [ 8191 ▲▼ ]             │  │
│ └──────────────────────────────────────────────────────────┘  │
│                                                                │
│ Preview:  50.0 % → raw 4095  |  0.0 % → raw 0  |  100.0% → 8191│
│                                                                │
│                          [ Save Entity ]                       │
└────────────────────────────────────────────────────────────────┘
```

| Field | Type | Constraints | Description |
|---|---|---|---|
| **Name** | text | non-empty | Displayed in the control panel slider row. Should be unique and descriptive. |
| **Signal Type** | dropdown | one of 5 types | Loads default preset; you can modify any field afterwards. |
| **Unit Label** | text | optional | Suffix after the spinbox value (e.g. `%`, `°`, `μs`, `RPM`). |
| **ESC Index** | integer | 0 – 19 | Which slot in `RawCommand.cmd.data[]` this entity writes. Must be unique per entity. Corresponds to `eepromBuffer.can.esc_index` on the ESC for the throttle channel; use adjacent indices for extra channels. |
| **Target Node ID** | integer | 1–127 or 255 | `255` = broadcast to all nodes (recommended unless you have multiple ESCs and need to target one). Use the ESC's `CAN_NODE` EEPROM value for unicast. |
| **Display Min** | float | < Display Max | Leftmost slider position value in display units. |
| **Display Max** | float | > Display Min | Rightmost slider position value in display units. |
| **Step** | float | > 0 | Increment per slider notch and spinbox arrow click. Sets the number of decimal places in the spinbox automatically. |
| **Default** | float | in [Min,Max] | Value loaded into the slider/spinbox when the connection opens. For throttle, always set this to `0` (or display_min) to prevent accidental startup at non-zero throttle. |
| **Raw Min** | integer | 0 – Raw Max − 1 | Raw CAN integer that corresponds to display_min. Normally `0`. Can be raised to add a dead zone or minimum threshold. |
| **Raw Max** | integer | Raw Min + 1 – 8191 | Raw CAN integer that corresponds to display_max. Normally `8191`. **Reducing this is an effective software safety ceiling** — e.g. `raw_max = 4096` limits the motor to ≈50 % of full command. |
| **Max RPM** | integer | > 0 | Visible only for RPM signal type. Sets display_max = this value. |

### 3.5 Live Preview

The **Preview** line updates instantly as you change any field:

```
Preview:  50.0 % → raw 4095   |   0.0 % → raw 0   |   100.0 % → raw 8191
```

It shows the mapping at three reference points: min, midpoint, and max.
If the range is invalid (e.g. min ≥ max) it shows `⚠ Invalid range`.
Always verify this before clicking Save.

### 3.6 Creating, Editing, and Deleting Entities

**Create**:
1. Click **+ New Entity** (top of the Config tab) — clears the form.
2. Fill in all fields.
3. Check the Preview.
4. Click **Save Entity**.

**Edit**:
1. Click any row in the entity table — the form is populated automatically.
2. Modify any field.
3. Click **Save Entity** — the entity at the same ESC Index is replaced.

**Delete**:
1. Click the row to select it.
2. Click **Delete Selected**.

> The ESC Index must be unique across all entities.  Attempting to save an entity
> with an index already in use by a different entity shows a validation error.

### 3.7 Config Persistence (entities.json)

All entities are saved automatically to `config/entities.json` every time
you click **Save Entity** or **Delete Selected**.  The file is reloaded on every
application start.  You can inspect or back it up directly:

```json
// config/entities.json — example with three channels
[
  {
    "name": "Throttle",
    "esc_index": 0,
    "node_id": 255,
    "signal_type": "Throttle (%)",
    "unit_label": "%",
    "display_min": 0.0,
    "display_max": 100.0,
    "display_step": 0.1,
    "display_default": 0.0,
    "raw_min": 0,
    "raw_max": 8191,
    "max_rpm": 0
  },
  {
    "name": "Phase Offset",
    "esc_index": 1,
    "node_id": 255,
    "signal_type": "Custom",
    "unit_label": "°",
    "display_min": 0.0,
    "display_max": 360.0,
    "display_step": 1.0,
    "display_default": 0.0,
    "raw_min": 0,
    "raw_max": 8191,
    "max_rpm": 0
  }
]
```

You can also **edit this file directly** in a text editor while the application is
closed, using the schema above.  Invalid JSON is silently ignored on load
(resulting in an empty entity list).

### 3.8 Adding Variables for Extended CAN Inputs

If you have added custom input channels to the firmware (e.g. `newinput_phaseoffset`
at `esc_index + 1` and `newinput_amplitude` at `esc_index + 2` as described in
`AM32_CAN_GUIDE.md`), you simply create entities with those indices:

| Entity Name | ESC Index | Signal Type | Display range | Unit | Raw range |
|---|---|---|---|---|---|
| Throttle | 0 | Throttle (%) | 0 – 100 | % | 0 – 8191 |
| Phase Offset | 1 | Custom | 0 – 360 | ° | 0 – 8191 |
| Amplitude | 2 | Custom | 0 – 100 | % | 0 – 8191 |

The app packs all three into a single `RawCommand` array:
`[throttle_raw, phase_raw, amplitude_raw]` and broadcasts it at the configured
send rate.

The firmware reads:
- `data[0]` → throttle (standard path)
- `data[1]` → `newinput_phaseoffset` (if `USE_CAN_EXTRA_INPUTS` is defined)
- `data[2]` → `newinput_amplitude` (if `USE_CAN_EXTRA_INPUTS` is defined)

No firmware change is required on the **sending** side — only the receiving side
(firmware) needs to be updated to interpret the extra slots.

---

## 4. Tab 2 — Connect: Runtime Operation

### 4.1 CAN Connection Form

```
┌─ CAN Connection ────────────────────────────────┐
│ Interface Port:  [ /dev/ttyUSB0 ▼ ] [ ↺ ]      │
│ Bitrate:         [ 1000000       ] bps           │
│ Own Node ID:     [ 127           ]               │
│ Send Rate:       [ 10            ] Hz            │
│                                                  │
│  [ Connect ]        [ Disconnect ]               │
│ Status:  ● Disconnected                          │
└──────────────────────────────────────────────────┘
```

| Field | Default | Notes |
|---|---|---|
| **Interface Port** | `/dev/ttyUSB0` | Serial port of the VIMDRONES USB-to-CAN adapter. Click **↺** to rescan `/dev/ttyUSB*` and `/dev/ttyACM*`. The dropdown is also editable — type any path. |
| **Bitrate** | 1 000 000 | Must match the ESC firmware (AM32 uses 1 Mbps). Do not change unless you know what you are doing. |
| **Own Node ID** | 127 | The CAN node ID this application will use on the bus. Must not collide with the ESC's node ID. Any free value 1–127 works; 127 is conventional for ground-station tools. |
| **Send Rate** | 10 Hz | How many `RawCommand` frames per second are sent. Range: 5–50 Hz. **Minimum 5 Hz** — the ESC firmware will zero the throttle if no command is received for 250 ms. |

**Status indicator** states:
- `● Disconnected` (grey) — idle
- `● Connecting…` (yellow) — opening the port and running boot sequence
- `● Connected` (green) — active session

### 4.2 Safety Boot Sequence

When you click **Connect**, the application runs an automatic safety sequence
**before** enabling the control sliders.  This sequence is mandatory and cannot
be skipped:

```
Step 1  All entity values are set to 0 (regardless of what the sliders show).
Step 2  Three all-zero RawCommand messages are broadcast.
        → Satisfies the ESC "require_zero_throttle" arming condition.
Step 3  ArmingStatus = FULLY_ARMED (status = 255) is broadcast.
        → Satisfies the ESC "require_arming" condition.
Step 4  "Boot sequence complete — motor armed" is logged.
Step 5  Control sliders are enabled.
```

You will see all steps logged in orange (`SAFETY` level) in the Log panel.

If any step fails (e.g. CAN port opens but the spin call throws an exception),
the application runs the safety disconnect sequence and emits `disconnected`.

### 4.3 Arming & Disarming

```
[ Arm (FULLY_ARMED) ]    [ Disarm ]    ● ARMED    [ ⛔ Emergency Stop ]
```

- **Arm** button: re-sends `ArmingStatus = FULLY_ARMED` at any time. Use this if
  the ESC disarmed unexpectedly (e.g. after a brief CAN interruption) and you want
  to resume without disconnecting.

- **Disarm** button: sends `ArmingStatus = DISARMED` (status = 0). The ESC will
  immediately zero the throttle output (if `require_arming` is set in its EEPROM).
  The sliders remain active and the connection stays open, but the motor will not
  respond to throttle commands until re-armed.

- **● ARMED / ● DISARMED** label: reflects the last arming action taken by the
  application. It does not confirm the ESC's internal arming state directly — it
  reflects what was last sent.

### 4.4 Emergency Stop

```
[ ⛔ Emergency Stop ]   ← large red button, always accessible while connected
```

Clicking Emergency Stop does, **in this order**, in the CAN worker thread:

1. All entity values are set to 0 in the internal state.
2. A zero `RawCommand` is sent immediately.
3. `ArmingStatus = DISARMED` is sent.
4. The **● DISARMED** label is shown.
5. The **connection stays open** — you can re-arm and resume.

Use Emergency Stop any time you need the motor to cut immediately.  It is faster
than clicking Disconnect (which also runs zero + disarm but then closes the port).

### 4.5 Controls Section — Sliders

Each entity defined in the Config tab appears as one slider row:

```
Throttle   idx 0 · node 255   [━━━━━━━━━●━━━━━━━━━━━━━━━]   50.0 %   raw: 4095
                                ^slider                        ^spinbox  ^raw readout
```

**Interaction methods**:
- **Drag the slider** — slider position maps to display value via step increments.
- **Click on the slider track** — jumps to the clicked position.
- **Scroll the mouse wheel** on the slider — increments/decrements by one step.
- **Type a value in the spinbox** directly, or click the ▲▼ arrows.
- **Slider and spinbox are always synchronized** — changing one updates the other
  immediately (with a feedback-loop guard to prevent recursive updates).

**Raw readout** (`raw: XXXX`): shows the exact integer that will be placed in
`RawCommand.cmd.data[esc_index]` at the next send cycle. Updated live as you move
the slider.

**Default value**: when the connection opens, all sliders are reset to their
configured `display_default`. For throttle, this is always `0`.  The sliders are
also reset to default when you click Disconnect or Emergency Stop.

**Rows are sorted** by ESC index (ascending) when the connection opens.

### 4.6 Disconnecting Safely

Click **Disconnect**.  The sequence is:

```
Step 1  All entity values are set to 0.
Step 2  Three all-zero RawCommand messages are broadcast.
Step 3  ArmingStatus = DISARMED is broadcast.
Step 4  The CAN node is closed.
Step 5  All telemetry cards are cleared.
Step 6  The Config tab is unlocked.
```

Even if the CAN bus is already dead (cable pulled), the application will attempt
the zero/disarm sequence and then close cleanly.

---

## 5. Live ESC Telemetry Panel

Located in the **top-right** of the Connect tab, to the right of the connection
form.  The two panels are separated by a **resizable horizontal splitter** — drag
the divider to give more space to telemetry or to the controls.

### 5.1 Telemetry Cards

A **TelemetryCard** is created automatically the first time an
`uavcan.equipment.esc.Status` message is received from a given ESC index.  One
card = one ESC.  If you have multiple ESCs on the bus, you get multiple cards.

```
┌─ ESC  idx 0 ─────────────────────────────────────────────────┐
│ Voltage:  12.34 V    │  V  ▁▂▃▄▅▆▆▅▄▃▂▁▁▂▃▄▅▆▇▇▆▅ (60 pts)  │
│ Current:   2.10 A    │  A  ▁▁▁▂▂▃▃▃▄▄▅▅▅▄▄▄▃▃▂▂▁▁           │
│ Temp:     35.0 °C    │ RPM ▁▂▃▄▅▆▆▅▄▃▂▁▁▂▃▄▅▆▇▇▇▇▇           │
│ RPM:       4200      │                                        │
│ Power:       42 %    │                                        │
│ Errors:         0    │                                        │
│ ● Live  (last: 12:34:01.234)                                  │
└────────────────────────────────────────────────────────────────┘
```

**Displayed fields** (sourced from `uavcan.equipment.esc.Status`, sent by the ESC):

| Label | Source field | Units | Notes |
|---|---|---|---|
| Voltage | `pkt.voltage` | V | Battery voltage from ADC + voltage divider |
| Current | `pkt.current` | A | Averaged motor current since last ESC Status packet |
| Temp | `pkt.temperature` (Kelvin − 273.15) | °C | NTC thermistor on the ESC PCB |
| RPM | `pkt.rpm` | RPM | Mechanical RPM = `(e_rpm × 200) / motor_poles` |
| Power | `pkt.power_rating_pct` | % | Not currently implemented in AM32 (always 0) |
| Errors | `pkt.error_count` | count | Cumulative DroneCAN error counter |

### 5.2 Sparkline Plots

Each card has three **sparkline** plots showing the last **60 samples**:
- **Voltage** (green, `#4CAF50`)
- **Current** (blue, `#2196F3`)
- **RPM** (orange, `#FF9800`)

The x-axis is sample index (time flows left to right).  Y-axis auto-scales to the
data range.  No pan or zoom (mouse interaction is disabled to prevent accidental
modifications during motor operation).  Plots are updated every time a new ESC
Status packet arrives.

### 5.3 Stale Detection

Each card has an internal 2-second timer.  If no telemetry is received within that
window:
- The status line changes from `● Live  (last: HH:MM:SS.mmm)` to `● Stale`
  (orange, `#FF9800`).

The timer resets automatically on each received packet.  Stale status does **not**
affect the CAN connection or the control sliders — it is purely informational.

### 5.4 Enabling Telemetry on the ESC

The ESC only sends `ESCStatus` if `TELEM_RATE > 0` in its EEPROM.  The default
after flashing is **25 Hz**.  If you see no cards:

1. Use the DroneCAN GUI Tool (separate application) to check/set `TELEM_RATE`.
2. Or use the `can_printf` debug log (if `DEBUG_RATE > 0`) to verify the ESC is
   alive on the bus.
3. Confirm with `ls /dev/ttyUSB*` that the adapter is recognized.

---

## 6. Log Panel

Located at the **bottom** of the Connect tab.  Always active — messages appear
even when disconnected.

```
┌─ Log ───────────────────────────────────────────────── [Clear] ─┐
│ [12:34:00.012] SAFETY  Boot sequence — zeroing all channels      │
│ [12:34:00.112] TX      RawCommand [0, 0, 0]  (3/s)              │
│ [12:34:00.412] SAFETY  Boot sequence complete — motor armed      │
│ [12:34:01.012] TX      RawCommand [0, 0, 0]  (10/s)             │
│ [12:34:01.212] RX      ESC Status  idx=0  12.34V  2.10A  4200rpm │
│ [12:34:02.012] WARN    No entities defined — go to Config tab    │
└─────────────────────────────────────────────────────────────────┘
```

**Log levels** and their meanings:

| Level | Colour | Events |
|---|---|---|
| `SAFETY` | Orange `#FF9800` | Safety sequence steps: zero all channels, arm, disarm, emergency stop. Always shown. |
| `INFO` | White `#FFFFFF` | Node active message with interface, node ID, and send rate. |
| `TX` | Cyan `#00BCD4` | Outgoing `RawCommand` (rate-limited to 1 log entry/second; the entry shows how many frames were sent in that second). Also `ArmingStatus` sends. |
| `RX` | Green `#4CAF50` | Incoming `ESCStatus` messages (voltage, current, RPM, etc.) and `NodeStatus` (node health). |
| `WARN` | Yellow `#FFEB3B` | Non-fatal warnings (e.g. no entities configured). |
| `ERROR` | Red `#F44336` | Connection failures, CAN bus errors, RawCommand send failures. |

**TX rate-limiting**: `RawCommand` is sent at e.g. 10 Hz, but the log only shows
one entry per second to avoid flooding the panel.  The entry reads:
`RawCommand [val0, val1, ...]  (10/s)` — the number in parentheses is the actual
frames sent in the last second.

**Rolling buffer**: the log is capped at **1000 lines**.  When full, the oldest
line is removed before each new entry.

**Clear button**: empties the log immediately.  Does not affect the CAN connection.

---

## 7. How Data Flows: End-to-End

```
User moves slider                           ESC receives
(Connect tab)                               (DroneCAN.c)
      │                                           │
      ▼                                           │
EntityRow._emit_raw()                             │
  display_to_raw(display_val, ...)                │
  → raw integer (0–8191)                          │
  → CANWorker.set_entity_value(esc_idx, raw)       │
      │                                           │
      ▼                                           │
CANWorker._entity_values dict                     │
  { 0: throttle_raw,                              │
    1: phase_raw,                                 │
    2: amplitude_raw }                            │
      │                                           │
      ▼  (every 1/send_rate seconds)              │
CANWorker._send_raw_command()                     │
  builds cmd_array = [0,...,0,                    │
                      data[0]=throttle_raw,       │
                      data[1]=phase_raw,          │
                      data[2]=amplitude_raw, ...]  │
  dronecan.uavcan.equipment.esc.RawCommand(       │
      cmd=cmd_array)                              │
  node.broadcast(msg)                             │
      │                                           │
      ▼  (CAN bus, 1 Mbps)                        │
USB-to-CAN adapter ──────────────────────────────►│
                                                  ▼
                                    handle_RawCommand()
                                      data[0] → newinput (throttle)
                                      data[1] → newinput_phaseoffset
                                      data[2] → newinput_amplitude
                                      set_input() → setInput()
                                      → duty_cycle_setpoint
                                      → motor spins

ESC sends (at telem_rate Hz)          App receives
(DroneCAN.c)                          (CANWorker)
      │                                     │
      ▼                                     │
send_ESCStatus()                            │
  uavcan.equipment.esc.Status               │
  { voltage, current, temp,                 │
    rpm, power_pct, esc_index }             │
      │                                     ▼
      └──────────────────────► _handle_esc_status()
                                 → esc_status_received signal
                                 → TelemetryPanel.update_esc()
                                 → TelemetryCard.update_data()
                                 → live numbers + sparklines updated
```

---

## 8. ESC Parameter Interaction

The ESC CAN Controller app **does not** expose a parameter editor panel directly.
Parameter read/write (the `uavcan.protocol.param.GetSet` service) must be done
with the **DroneCAN GUI Tool** (a separate, standalone application available from
`dronecan.github.io`) or with Mission Planner's DroneCAN panel.

**To check and configure ESC parameters** before using this app:

1. Connect the USB-to-CAN adapter to the computer.
2. Open the DroneCAN GUI Tool.
3. Set the interface to your serial port at 1 Mbps.
4. The ESC appears in the *Online Nodes* panel (identified as
   `com.vimdrones.esc_dev` with its node ID).
5. Double-click the ESC node to open its parameter list.

**Key parameters to verify** before driving a motor:

| Parameter | Recommended value | Why |
|---|---|---|
| `ESC_INDEX` | `0` | Must match the ESC Index of your Throttle entity in the app |
| `TELEM_RATE` | `25` (or higher) | Enable live telemetry cards |
| `REQUIRE_ARMING` | `1` (default) | The app handles this automatically |
| `REQUIRE_ZERO_THROTTLE` | `1` (default) | The app handles this automatically |
| `INPUT_SIGNAL_TYPE` | `5` (DRONECAN_IN) | Disable the physical signal pin so only CAN controls the motor |
| `INPUT_FILTER_HZ` | `0` or `5–20` | `0` = no filtering; increase for smoother throttle at low rates |
| `BRAKE_ON_STOP` | `1` | Active braking when throttle = 0 (recommended) |
| `CURRENT_LIMIT` | `0` (disabled) or your value | Protect the motor during testing |

After changing parameters, click **Save** (ExecuteOpcode SAVE) to persist them to
flash, then **Restart** the ESC.

---

## 9. Driving the Motor Without a Flight Controller

This section is the **complete step-by-step guide** for running the motor using
only the ESC, the VIMDRONES USB-to-CAN adapter, and this application.  No flight
controller or RC receiver is involved.

### 9.1 Prerequisites and Safety

> ⚠ **SAFETY — read before proceeding**
>
> - **Secure the motor and propeller (if any)**. Always remove the propeller or
>   physically restrain the motor for bench testing.  A spinning motor can cause
>   injury.
> - **Fuse your power supply** or set a current limit on your bench PSU to
>   protect the ESC from catastrophic failure.
> - **Keep the Emergency Stop button accessible** at all times during operation.
>   Position the app window so the red button is visible and reachable.
> - **Start at 0 % throttle** and increase slowly.  Never snap the throttle slider
>   to a high value immediately.
> - **Know your motor's KV rating** before applying voltage.  Ensure the battery/PSU
>   voltage is within the ESC's rating.

Hardware checklist:
- [ ] ESC flashed with `VIMDRONES_L431_CAN` firmware
- [ ] Motor wired to ESC (A/B/C phase connections)
- [ ] Power supply or battery connected to ESC (correct polarity, within voltage range)
- [ ] VIMDRONES USB-to-CAN adapter connected to the CAN bus (CAN_H / CAN_L)
- [ ] 120 Ω CAN termination resistors at both ends of the bus (if applicable)
- [ ] USB adapter connected to computer
- [ ] Application launched and venv active

### 9.2 One-time ESC Configuration

Do this once using the DroneCAN GUI Tool before the first run.  You do not need
to repeat it unless you change hardware.

```
Parameter         Set to     Reason
─────────────────────────────────────────────────────────────────
ESC_INDEX         0          Slot 0 in RawCommand = throttle
TELEM_RATE        25         Show live voltage/current/RPM in app
REQUIRE_ARMING    1          App handles arming automatically
INPUT_SIGNAL_TYPE 5          Use CAN only (disable physical pin)
BRAKE_ON_STOP     1          Motor stops cleanly at 0 throttle
CAN_NODE          1          (or 0 for DNA — either works)
```

After setting parameters:
1. Click **Save** in the DroneCAN GUI Tool.
2. Click **Restart** (or power-cycle the ESC).
3. Verify the ESC reappears on the bus with its node ID.

### 9.3 Step-by-Step: First Motor Run

#### Step 1 — Create the Throttle entity (one-time, Config tab)

1. Launch the application.
2. Go to the **Config** tab.
3. Click **+ New Entity**.
4. Fill in:

   | Field | Value |
   |---|---|
   | Name | `Throttle` |
   | Signal Type | `Throttle (%)` |
   | Unit Label | `%` |
   | ESC Index | `0` |
   | Target Node ID | `255` |
   | Display Min | `0.0` |
   | Display Max | `100.0` |
   | Step | `0.1` |
   | Default | `0.0` ← **must be 0** |
   | Raw Min | `0` |
   | Raw Max | `8191` |

5. Verify the Preview: `0.0 % → raw 0  |  50.0 % → raw 4095  |  100.0 % → raw 8191`
6. Click **Save Entity**.

> **Optional: add a safety ceiling** — set Raw Max to `2000` to limit the maximum
> command to ≈24 % full-range during initial testing.  You can raise it later.

#### Step 2 — Connect to the ESC (Connect tab)

1. Go to the **Connect** tab.
2. Click **↺** to refresh the port list.
3. Select the adapter port (e.g. `/dev/ttyUSB0`).
4. Set Bitrate to `1000000`.
5. Set Own Node ID to `127` (or any ID not used by the ESC).
6. Set Send Rate to `10` Hz (sufficient for motor control).
7. Click **Connect**.

#### Step 3 — Wait for the safety boot sequence

Watch the Log panel.  You should see:

```
[HH:MM:SS] SAFETY  Boot sequence — zeroing all channels
[HH:MM:SS] TX      RawCommand [0]  (3/s)
[HH:MM:SS] TX      RawCommand [0]  (3/s)
[HH:MM:SS] TX      RawCommand [0]  (3/s)
[HH:MM:SS] TX      ArmingStatus → FULLY_ARMED
[HH:MM:SS] SAFETY  Boot sequence complete — motor armed
[HH:MM:SS] INFO    DroneCAN node active — ID 127, interface /dev/ttyUSB0, send rate 10 Hz
```

The status changes to `● Connected` (green).  The Throttle slider appears,
set to `0.0 %`.  The arming label shows `● ARMED`.

#### Step 4 — Verify telemetry

Look at the **Live ESC Telemetry** panel (top-right).  A card for `ESC idx 0`
should appear within 1–2 seconds showing:

```
● Live  (last: HH:MM:SS.mmm)
Voltage:  XX.XX V      ← should match your power supply voltage
Current:   0.00 A      ← idle, no motor current
Temp:      XX.X °C
RPM:          0
Power:         0 %
Errors:        0
```

If no card appears, check that `TELEM_RATE` is set above 0 on the ESC.

#### Step 5 — Spin the motor

> ⚠ Keep your hands clear of the motor and propeller before this step.

1. Confirm the motor is secured and/or propeller is removed.
2. Slowly drag the Throttle slider to the **right**, or type a small value in the
   spinbox, e.g. `5.0 %`.
3. Watch the Log for TX frames with a non-zero value:
   ```
   [HH:MM:SS] TX  RawCommand [409]  (10/s)
   ```
4. Watch the Telemetry card — RPM should climb, Current should increase.
5. The motor should **start spinning** at a low RPM.  If it does not start at 5 %,
   increase to 10 % or 15 %.  AM32 has a minimum startup throttle threshold;
   very low values below this threshold produce no movement.

> **Typical first-spin throttle**: 5–15 % depending on motor KV and load.  The
> motor will jerk once on the first startup (AM32 startup sequence) then
> stabilize.

6. Once spinning, you can increase the slider smoothly to explore the RPM range.
7. Keep one finger on the mouse over the **⛔ Emergency Stop** button at all times.

#### Step 6 — Monitor the data

While the motor runs, watch:

| Telemetry field | What it tells you |
|---|---|
| **Voltage** | Should sag slightly under load vs. idle — indicates battery state |
| **Current** | Proportional to load and throttle — verify it stays within safe limits |
| **Temp** | Should rise slowly — cut if it approaches 70–80 °C |
| **RPM** | Should track throttle changes smoothly |
| **Errors** | Should remain 0 — any non-zero value indicates a CAN communication problem |

### 9.4 What to Expect in the Log and Telemetry

**Typical log during a motor run session:**

```
[12:34:00.012] SAFETY  Boot sequence — zeroing all channels
[12:34:00.112] TX      RawCommand [0]  (3/s)
[12:34:00.412] SAFETY  Boot sequence complete — motor armed
[12:34:01.012] INFO    DroneCAN node active — ID 127, ...
[12:34:01.012] TX      RawCommand [0]  (10/s)
[12:34:01.212] RX      ESC Status  idx=0  12.30V  0.00A  0rpm  0%pwr  errs=0
[12:34:05.012] TX      RawCommand [819]  (10/s)     ← 10% throttle
[12:34:05.212] RX      ESC Status  idx=0  11.80V  1.20A  2100rpm  10%pwr  errs=0
[12:34:10.012] TX      RawCommand [2457]  (10/s)    ← 30% throttle
[12:34:10.212] RX      ESC Status  idx=0  11.50V  3.40A  5800rpm  30%pwr  errs=0
```

**What the raw values mean** (with default Throttle (%) entity, `raw_max = 8191`):

| Slider value | Raw sent | AM32 internal (approx) | Effect |
|---|---|---|---|
| 0 % | 0 | 0 | Motor off |
| 1–5 % | 82–409 | Below startup threshold | Motor silent (may jerk) |
| 5–15 % | 409–1228 | ≈47–170 | Motor starts to spin |
| 25 % | 2048 | ≈560 | Medium-low throttle |
| 50 % | 4095 | ≈1047 | Mid throttle |
| 100 % | 8191 | 2047 | Full throttle |

> Note: AM32 maps CAN values 0–8191 to its internal range 47–2047 for unidirectional
> mode.  Values below the minimum startup threshold simply keep the motor off.

### 9.5 Typical Test Session Checklist

```
Before starting:
  ☐ Motor secured (no propeller, or propeller firmly attached and workspace clear)
  ☐ Power supply current limit set (or battery fused)
  ☐ App launched, entities configured
  ☐ ESC parameters confirmed (especially ESC_INDEX=0, INPUT_SIGNAL_TYPE=5)
  ☐ CAN adapter plugged in and port detected (↺ refresh)

On connect:
  ☐ Boot sequence logged as SAFETY (orange)
  ☐ "Boot sequence complete — motor armed" seen in log
  ☐ Telemetry card appeared with correct voltage
  ☐ Slider at 0 %

During test:
  ☐ Increase throttle slowly in steps of 2–5 %
  ☐ Monitor Current — stays within motor/ESC rating
  ☐ Monitor Temp — stays below 70 °C
  ☐ Emergency Stop accessible at all times

After test:
  ☐ Reduce throttle to 0 % before disconnecting
  ☐ Click Disconnect (automatic zero + disarm)
  ☐ Power down ESC / disconnect power supply
```

### 9.6 Safe Shutdown Procedure

1. Reduce the Throttle slider to **0 %** first (wait for motor to stop).
2. Click **Disconnect** — the app sends zero commands and disarms automatically.
3. Watch the log confirm the disconnect sequence:
   ```
   [HH:MM:SS] SAFETY  Disconnect — zeroing all channels
   [HH:MM:SS] TX      RawCommand [0]  (1/s)
   [HH:MM:SS] TX      ArmingStatus → DISARMED
   ```
4. Only after the log shows disconnect is complete, **power down the ESC**.
5. The Config tab unlocks.

---

## 10. Troubleshooting

| Symptom | Likely cause | Resolution |
|---|---|---|
| Port not listed after clicking ↺ | Adapter not plugged in, or no `dialout` group membership | Plug in adapter. Run `sudo usermod -aG dialout $USER` and log out/in. Verify with `ls /dev/ttyUSB*`. |
| "Failed to open CAN interface" | Wrong port, driver not loaded, or port in use by another application | Try unplugging and re-plugging. Check `dmesg | tail -20` for USB errors. Close other serial terminal applications. |
| "CAN runtime error" during operation | Bitrate mismatch, CAN bus fault, or missing termination resistors | Verify both ESC and app use 1 Mbps. Check CAN wiring and add 120 Ω termination at each end. Check `canstats.rx_errors` via FlexDebug if `DEBUG_RATE > 0`. |
| Motor does not spin at any throttle | `require_arming` not satisfied, or `ESC_INDEX` mismatch | Check log for SAFETY messages — arming should be confirmed. Verify `ESC_INDEX` in ESC parameters matches entity's ESC Index field (both should be `0` for a single ESC). |
| Motor starts, then cuts out after 250 ms | Send rate too low (< 5 Hz), or CAN bus intermittent | Increase Send Rate to 10–20 Hz. Check CAN wiring. |
| No telemetry cards appear | `TELEM_RATE = 0` in ESC EEPROM | Set `TELEM_RATE = 25` via DroneCAN GUI Tool and save/restart ESC. |
| Telemetry card shows `● Stale` immediately | ESC not on bus, or node ID conflict | Verify ESC is powered and shows a heartbeat in DroneCAN GUI Tool. Check for node ID collisions. |
| Motor runs at unexpected speed / won't stop | `REQUIRE_ARMING = 0` and `INPUT_SIGNAL_TYPE ≠ 5` | If the physical signal pin is still active, a PWM signal on it can override CAN. Set `INPUT_SIGNAL_TYPE = 5`. |
| Config tab is greyed out | CAN connection is active | Click Disconnect first, then edit entities. |
| Slider values reset to 0 on connect | Expected behavior | All sliders use `display_default` (which is 0 for throttle) when a connection opens. This is the safety-first design. |
| Sparklines not updating | `TELEM_RATE` set but telemetry not arriving | Check `rx_errors` in FlexDebug. Verify CAN termination. |
| Application crashes on start | PyQt6 or dronecan not installed correctly | Re-run `pip install -r requirements.txt` with the venv active. Check Python version ≥ 3.10. |

---

## 11. Quick Reference — Entity Field Summary

| Field | Range | Saved in JSON | Effect when changed |
|---|---|---|---|
| `name` | non-empty string | yes | Changes the label in the control panel |
| `esc_index` | 0–19 | yes | Changes which `RawCommand` slot this entity writes |
| `node_id` | 1–127 or 255 | yes | `255` = broadcast. Match to ESC `CAN_NODE` for unicast |
| `signal_type` | one of 5 types | yes | Changes default presets; does not affect saved values |
| `unit_label` | any string | yes | Suffix after spinbox value; cosmetic only |
| `display_min` | < display_max | yes | Left end of the slider |
| `display_max` | > display_min | yes | Right end of the slider |
| `display_step` | > 0 | yes | Slider resolution and spinbox increment |
| `display_default` | in [min,max] | yes | Value loaded on connect; use 0 for throttle |
| `raw_min` | 0–raw_max−1 | yes | Raw value sent at display_min; raise to create a dead zone |
| `raw_max` | raw_min+1–8191 | yes | Raw value sent at display_max; **lower to create a safety ceiling** |
| `max_rpm` | >0 for RPM type | yes | Sets display_max = max_rpm for RPM entities |

---

## Appendix — entities.json Schema

```json
[
  {
    "name":            "string (required, non-empty)",
    "esc_index":       "integer 0–19 (required, unique)",
    "node_id":         "integer 1–127 or 255",
    "signal_type":     "Raw | Throttle (%) | PWM (μs) | RPM | Custom",
    "unit_label":      "string (optional suffix, e.g. '%')",
    "display_min":     "float",
    "display_max":     "float > display_min",
    "display_step":    "float > 0",
    "display_default": "float in [display_min, display_max]",
    "raw_min":         "integer 0–8190",
    "raw_max":         "integer raw_min+1–8191",
    "max_rpm":         "integer > 0 (only required when signal_type == 'RPM')"
  }
]
```
