# AM32 — Flashing Bootloader + Main Firmware (VIMDRONES L431 CAN)

This guide covers flashing the AM32 CAN bootloader and the AM32 main CAN firmware
to the VIMDRONES board (STM32L431), enabling Dynamic Node Allocation (DNA) over
DroneCAN. It includes a deep analysis of why the bootloader fails to jump to the
main firmware and how to fix every root cause.

---

## Table of Contents

1. [Memory Map Reference](#1-memory-map-reference)
2. [Root-Cause Analysis: Why the Bootloader Does Not Jump](#2-root-cause-analysis)
   - 2.1 [Critical Bug: Wrong Flash Address in st-flash command](#21-critical-bug-wrong-flash-address)
   - 2.2 [App Signature Not Filled In (CRC Mismatch)](#22-app-signature-crc-mismatch)
   - 2.3 [Wrong Bootloader Variant](#23-wrong-bootloader-variant)
   - 2.4 [VTOR Not Relocated After Jump](#24-vtor-not-relocated-after-jump)
   - 2.5 [RTC Backup Register Left in Firmware-Update State](#25-rtc-backup-register-in-firmware-update-state)
   - 2.6 [Non-CAN Binary Flashed at CAN Address](#26-non-can-binary-flashed-at-can-address)
3. [Prerequisites](#3-prerequisites)
4. [Build the Firmware Correctly](#4-build-the-firmware-correctly)
5. [Step-by-Step Flash Procedure](#5-step-by-step-flash-procedure)
6. [Verifying the Jump Succeeded](#6-verifying-the-jump-succeeded)
7. [Troubleshooting Checklist](#7-troubleshooting-checklist)

---

## 1. Memory Map Reference

The following layout applies when using the AM32 CAN bootloader with the
`VIMDRONES_L431_TUDELFT_CAN` (or any `*_L431_*CAN*`) target.

```
STM32L431 internal flash (256 K, 0x08000000 – 0x0803FFFF)
┌──────────────────────────────────────────────────────────────┐
│  0x08000000  Bootloader (up to 16 K)                         │
│              ├─ Vector table                                 │
│              ├─ DNA allocator                                │
│              ├─ CAN OTA update logic                         │
│              └─ App-signature validator                      │
├──────────────────────────────────────────────────────────────┤
│  0x08004000  AM32 main firmware  ← flash here (16384 bytes   │
│              ├─ FLASH1 (512 B)     offset from flash start)  │
│              │   ├─ ISR vector table  (placed first)         │
│              │   └─ app_signature    (44 bytes, CRC'd)       │
│              ├─ FILE_NAME (32 B, at 0x08004200)              │
│              └─ FLASH  — .text, .rodata, .data init, ...     │
├──────────────────────────────────────────────────────────────┤
│  0x0801F800  EEPROM (2 K)                                    │
└──────────────────────────────────────────────────────────────┘

RAM (64 K, 0x20000000 – 0x2000FFFF)
```

These regions are defined in `Mcu/l431/ldscript_CAN.ld` and are automatically
selected by the Makefile for any target whose name contains `_CAN`.

> **Key address**: the AM32 main firmware starts at **`0x08004000`**.
> This is a 33-bit address beginning with `0x08`, not `0x80`.

---

## 2. Root-Cause Analysis

### 2.1 Critical Bug: Wrong Flash Address

**This is the most likely reason the bootloader never jumps.**

The address `0x80004000` is wrong. The correct address is `0x08004000`.

| Address       | What it is |
|---------------|-----------|
| `0x08000000`  | Start of STM32 internal flash — bootloader goes here |
| `0x08004000`  | Offset 16 K into flash — AM32 main firmware goes here |
| `0x80004000`  | **NOT flash.** This is in the external memory region of the Cortex-M4 address space. Any `st-flash write` command targeting this address will either fail with an error or write to a completely wrong memory region. The bootloader finds nothing valid at `0x08004000` and stays in bootloader mode forever. |

The correct `st-flash` command for the main firmware is:

```bash
# CORRECT
st-flash write AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin 0x8004000

# WRONG  (the leading 0x80 instead of 0x08 corrupts the whole procedure)
st-flash write AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin 0x80004000
```

Both `0x8004000` and `0x08004000` are accepted by `st-flash` and refer to the same
address. `0x80004000` is a completely different 32-bit value.

---

### 2.2 App Signature CRC Mismatch

The AM32 CAN bootloader validates the application before jumping by checking the
`app_signature` struct embedded in the firmware at the start of `FLASH1`
(`0x08004000 + sizeof(isr_vector)`). The struct is:

```c
// Src/DroneCAN/DroneCAN.c
const struct {
    uint32_t magic1;   // 0x68f058e6
    uint32_t magic2;   // 0xafcee5a0
    uint32_t fwlen;    // total binary length in bytes
    uint32_t crc1;     // CRC32 of binary bytes BEFORE the descriptor
    uint32_t crc2;     // CRC32 of binary bytes AFTER the descriptor
    char     mcu[16];  // e.g. "L431"
    uint32_t unused[2];
} app_signature __attribute__((section(".app_signature")));
```

**The `fwlen`, `crc1`, and `crc2` fields start as 0** in source code. They are
filled in post-build by `Src/DroneCAN/set_app_signature.py`, which is called
automatically by the Makefile:

```makefile
$(QUIET)python3 Src/DroneCAN/set_app_signature.py $$@ $$(<)
```

**If the script fails** (wrong Python version, missing file, script error) the
fields remain 0, the bootloader computes mismatched CRCs, and it refuses to jump.

**Check**: after building, run the script manually and look for the success line:

```bash
cd /home/andre/ESC_DEV/AM32
python3 Src/DroneCAN/set_app_signature.py \
    obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin \
    obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.elf
# Expected output:
# Applied APP_DESCRIPTOR 3f8a21c4b0d17e92 for obj/AM32_...bin
```

If it prints `No APP_DESCRIPTOR found in ...`, the binary was built without
`DRONECAN_SUPPORT=1` — see §2.6.

> **Note**: `set_app_signature.py` silently exits if the filename does not contain
> `_CAN_`. The target name must include those characters; the VIMDRONES CAN targets
> all do (`VIMDRONES_L431_TUDELFT_CAN`, `VIMDRONES_L431_CAN`, etc.).

---

### 2.3 Wrong Bootloader Variant

AM32 has two entirely different bootloader families:

| Bootloader | App start address | Protocol |
|---|---|---|
| Standard AM32 serial bootloader | `0x08001000` (4 K offset) | 1-wire UART / BLHeli passthrough |
| AM32 CAN bootloader | `0x08004000` (16 K offset) | DroneCAN DNA + OTA firmware update |

**Mixing them causes a permanent failure to boot:**

- CAN bootloader + firmware linked for `0x08001000` → bootloader looks at
  `0x08004000`, finds either garbage or the middle of the firmware, never jumps.
- Serial bootloader + firmware linked for `0x08004000` → bootloader looks at
  `0x08001000`, may find a valid stack pointer but then jumps into the firmware
  at the wrong offset, instantly HardFaults.

To use Dynamic Node Allocation, you **must** use the AM32 CAN bootloader. The
VIMDRONES CAN target firmware is already linked for `0x08004000` via
`ldscript_CAN.ld`. Make sure the bootloader binary you are flashing is also the
CAN variant (it will occupy addresses `0x08000000`–`0x08003FFF`).

You can verify by checking the bootloader binary size:

```bash
# CAN bootloader should be ≤ 16384 bytes (16 K)
ls -l bootloader.bin
wc -c < bootloader.bin
```

If the bootloader is larger than 16 K (16384 bytes) it will overlap with the
firmware region and corrupt the app's vector table at `0x08004000`.

---

### 2.4 VTOR Not Relocated After Jump

On a Cortex-M4, the **Vector Table Offset Register** (SCB->VTOR) must point to
the active firmware's ISR vector table. After reset, it points to `0x08000000`
(the bootloader's vectors). Before jumping to the app, the CAN bootloader must
relocate VTOR to `0x08004000`.

If the bootloader forgets this step, the chip runs the app's `Reset_Handler` but
all interrupts (SysTick, CAN RX/TX, timers, DMA) still dispatch through the
**bootloader's** vector table. The app's ISR handlers are never called. From the
app's perspective, nothing works — no CAN traffic is processed, no motor control
fires, and the app appears "stuck" even though it technically started.

**Fix in the bootloader**: the CAN bootloader should perform these steps before
the jump:

```c
// Pseudocode — inside the CAN bootloader, just before jumping
uint32_t app_base = 0x08004000;
SCB->VTOR = app_base;           // relocate interrupt vectors
__DSB();                        // data sync barrier
__ISB();                        // instruction sync barrier

// read stack pointer and reset handler from the app's vector table
uint32_t app_sp  = *(volatile uint32_t *)(app_base + 0);
uint32_t app_pc  = *(volatile uint32_t *)(app_base + 4);

// set stack pointer and jump
__set_MSP(app_sp);
((void(*)(void))app_pc)();
```

**Defensive fix in the app** (`Mcu/l431/Src/peripherals.c:initAfterJump`):

The app can re-set VTOR itself as its very first action, before any peripheral
initialisation. Edit `initAfterJump()`:

```c
// Mcu/l431/Src/peripherals.c
void initAfterJump(void)
{
    // Re-set VTOR in case the bootloader did not do it.
    // The linker places .isr_vector at 0x08004000 (FLASH1 origin in ldscript_CAN.ld).
    SCB->VTOR = 0x08004000;
    __DSB();
    __ISB();
    __enable_irq();
}
```

This is safe even if the bootloader already set VTOR correctly — writing the same
value is a no-op. This one-line fix eliminates VTOR as a failure mode entirely.

> You must also ensure `USER_VECT_TAB_ADDRESS` is **not** defined for the L431
> target (it is not, by default), since `SystemInit()` only updates VTOR when that
> macro is defined. The `initAfterJump()` fix above is therefore the reliable path.

---

### 2.5 RTC Backup Register Left in Firmware-Update State

The firmware uses RTC backup registers to communicate state across reboots between
the app and the bootloader (`Src/DroneCAN/sys_can.h`):

```c
#define RTC_BKUP0_FWUPDATE 0x42c7   // bootloader: "please do a CAN OTA update"
#define RTC_BKUP0_BOOTED   0x8c42c8 // app: "I started successfully"
#define RTC_BKUP0_SIGNAL   0x8c42c9 // app: "I processed 5 CAN frames (fully working)"
```

**Scenario that locks you out:**
1. A previous session triggered `handle_begin_firmware_update()` (OTA update
   request). The app wrote `RTC_BKUP0_FWUPDATE` to backup register 0 and rebooted.
2. The OTA update was never completed (e.g. file server not running).
3. The CAN bootloader reads backup register 0, sees `FWUPDATE`, and enters OTA
   update mode — it does **not** boot the app.
4. You power-cycle the board; the backup register survives because it is powered
   by VBAT (or the main supply keeps RTC domain alive). The loop repeats.

**Fix**: clear the RTC backup registers by flashing both images with the full
erase sequence below. The `st-flash` `--reset` flag resets the chip but does not
clear RTC backup registers. You need to either:

- Flash a small program that clears backup registers and then use the normal
  procedure, **or**
- Use `st-flash erase` to fully erase the chip (this cuts power to RTC domain
  long enough to reset backup registers on most boards), then re-flash both images.

```bash
# Nuclear option: full chip erase clears everything including RTC backup registers
st-flash erase
# then re-flash bootloader and firmware (see §5)
```

---

### 2.6 Non-CAN Binary Flashed at CAN Address

The Makefile produces two distinct binary types for the L431:

| Target | Linker script | App start | Has app_signature? |
|---|---|---|---|
| `VIMDRONES_L431` | `ldscript.ld` | `0x08001000` | No |
| `VIMDRONES_L431_TUDELFT` | `ldscript.ld` | `0x08001000` | No |
| `VIMDRONES_L431_CAN` | `ldscript_CAN.ld` | `0x08004000` | Yes |
| `VIMDRONES_L431_TUDELFT_CAN` | `ldscript_CAN.ld` | `0x08004000` | Yes |

Only a binary built with `ldscript_CAN.ld` (i.e. target name contains `_CAN`) has:
- Its vector table at `0x08004000`
- The `.app_signature` section with magic values the bootloader checks
- The `.file_name` section padding to 512 bytes

Flashing a non-CAN binary at `0x08004000` means the bootloader reads an ISR vector
table that was designed for `0x08001000`. The stack pointer and reset handler
values still refer to the correct RAM/flash ranges, so the bootloader's basic
sanity check may pass, but the app immediately crashes because every relative
offset in the binary is wrong by `0x3000` bytes.

**Always use the `_CAN_` suffixed binary when flashing with the CAN bootloader.**

---

## 3. Prerequisites

### Hardware

- VIMDRONES board (STM32L431)
- ST-Link V2 (or compatible) programmer connected via SWD (SWDIO, SWDCLK, GND, 3.3 V)
- CAN bus termination (120 Ω) at each end if testing DNA
- USB-to-CAN adapter (SLCAN) for DroneCAN GUI verification

### Software

```bash
# st-flash (part of stlink tools)
sudo apt install stlink-tools       # Ubuntu/Debian
# or build from source: https://github.com/stlink-org/stlink

# Python 3 (for app-signature tool)
python3 --version   # must be 3.6+

# Verify st-flash can see the board
st-flash --version
st-info --probe
```

Expected output of `st-info --probe`:

```
Found 1 stlink programmers
  version:    V2J...
  serial:     ...
  flash:      262144 (pagesize: 2048)   ← 256 K, 2 K pages
  sram:       65536
  chipid:     0x435
  descr:      L43x/L44x
```

### Binary Files Required

| File | Source | Target address |
|---|---|---|
| `AM32_CAN_BOOTLOADER_L431.bin` | AM32 CAN bootloader repository | `0x08000000` |
| `AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin` | Built in this repo (`obj/`) | `0x08004000` |

---

## 4. Build the Firmware Correctly

Always do a clean build to avoid stale object files mixing between CAN and non-CAN
targets.

```bash
cd /home/andre/ESC_DEV/AM32

# Clean previous build artifacts
make clean

# Build only the VIMDRONES CAN target (fast)
make AM32_VIMDRONES_L431_TUDELFT_CAN

# Verify the output files exist
ls -lh obj/AM32_VIMDRONES_L431_TUDELFT_CAN_*.bin
ls -lh obj/AM32_VIMDRONES_L431_TUDELFT_CAN_*.elf
```

**Verify the app signature was applied:**

```bash
python3 Src/DroneCAN/set_app_signature.py \
    obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin \
    obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.elf
```

Expected output (the hex values will differ):

```
Applied APP_DESCRIPTOR 3f8a21c4b0d17e92 for obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin
```

> If the script prints `No APP_DESCRIPTOR found`, the build used the wrong linker
> script. Confirm the target contains `_CAN` in its name and re-run `make clean`
> before building again.

**Verify the binary starts at the right address:**

```bash
arm-none-eabi-objdump -h obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.elf | head -20
```

The `.isr_vector` section must show `VMA = 0x08004000`:

```
Sections:
Idx Name          Size      VMA       LMA       ...
  0 .isr_vector   00000200  08004000  08004000  ...
  1 .file_name    ...       08004200  08004200  ...
  2 .text         ...       08004220  08004220  ...
```

If `.isr_vector` shows `VMA = 0x08001000`, a non-CAN linker script was used.

---

## 5. Step-by-Step Flash Procedure

### Step 1 — Full chip erase (recommended first time or when stuck)

A full erase clears all flash pages **and** resets the RTC backup registers,
eliminating the firmware-update state problem (§2.5).

```bash
st-flash erase
```

Expected output:

```
st-flash 1.7.0
2024-xx-xx ... INFO common.c: Erasing chip
2024-xx-xx ... INFO common.c: Flash mass erase done.
```

### Step 2 — Flash the CAN bootloader

```bash
st-flash write /path/to/AM32_CAN_BOOTLOADER_L431.bin 0x8000000
```

Expected output:

```
st-flash 1.7.0
2024-xx-xx ... INFO common.c: Loading device parameters....
2024-xx-xx ... INFO common.c: Device connected is: L43x/L44x device, id 0x10016435
2024-xx-xx ... INFO common.c: SRAM size: 0x10000 bytes (64 KiB), Flash: 0x40000 bytes (256 KiB)
2024-xx-xx ... INFO common.c: Attempting to write 14820 (0x39e4) bytes to stm32 address: 134217728 (0x8000000)
FlashLoader: Done. Bytes written: 14820
2024-xx-xx ... INFO common.c: Finished erasing 8 pages of 2048 (0x800) bytes
2024-xx-xx ... INFO common.c: Starting Flash write for VL/F0/F3/F1_XL core id
2024-xx-xx ... INFO common.c: Flash written and verified! jolly good!
```

The bootloader size must be less than 16384 bytes (8 flash pages of 2048 bytes).
If the write report shows more than 8 pages, the bootloader is too large and will
overwrite the firmware region.

### Step 3 — Flash the AM32 CAN firmware

```bash
# NOTE: address is 0x8004000 (= 0x08004000), NOT 0x80004000
st-flash write obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin 0x8004000
```

Expected output:

```
2024-xx-xx ... INFO common.c: Attempting to write 65432 (0xff98) bytes to stm32 address: 134234112 (0x8004000)
FlashLoader: Done. Bytes written: 65432
2024-xx-xx ... INFO common.c: Flash written and verified! jolly good!
```

Address `134234112` in decimal is `0x08004000` in hex — confirm this matches.

### Step 4 — Reset the board

```bash
st-flash reset
```

The bootloader runs first, validates the app signature at `0x08004000`, and jumps
to the AM32 firmware. The board is now running the main firmware with DNA enabled.

---

## 6. Verifying the Jump Succeeded

### Method A — DroneCAN GUI Tool

1. Connect the USB-to-CAN (SLCAN) adapter between PC and CAN bus.
2. Ensure 120 Ω termination at both ends of the bus.
3. Open the DroneCAN GUI Tool (or `dronecan_gui_tool`).
4. Set interface to the SLCAN serial port and 1 Mbps.
5. The ESC will appear in **Online Nodes** within a few seconds, named
   `com.vimdrones.esc_tudelft` (or `com.vimdrones.esc_dev` depending on target).
6. The presence of the node confirms the bootloader jumped, DNA resolved, and the
   app is running.

The firmware sets RTC backup register 0 on startup:
- `RTC_BKUP0_BOOTED (0x8c42c8)` — set immediately on first `DroneCAN_update()`.
- `RTC_BKUP0_SIGNAL (0x8c42c9)` — set after 5 CAN frames are processed.

If the GUI tool sees the node and `SIGNAL` state is set, everything is working.

### Method B — SWD Debugger Live Watch

Connect via OpenOCD/GDB and check:

```
(gdb) x/1xw 0x40002850    # RTC->BKP0R
```

Expected value after successful DNA: `0x8c42c9` (`RTC_BKUP0_SIGNAL`).

If the value is `0x42c7xxxx` (upper bits matching `RTC_BKUP0_FWUPDATE`), the
board is stuck in OTA update mode — run `st-flash erase` and redo §5.

### Method C — Binary Inspection Before Flashing

Verify the app signature in the binary with Python before flashing:

```python
#!/usr/bin/env python3
import struct

MAGIC1 = 0x68f058e6
MAGIC2 = 0xafcee5a0

with open("obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin", "rb") as f:
    data = f.read()

magic = struct.pack("<II", MAGIC1, MAGIC2)
offset = data.find(magic)
if offset == -1:
    print("ERROR: app_signature not found — wrong binary or missing _CAN_ build")
else:
    magic1, magic2, fwlen, crc1, crc2 = struct.unpack_from("<IIIII", data, offset)
    print(f"app_signature found at offset 0x{offset:04x}")
    print(f"  fwlen = {fwlen}  (0 = signature script did not run)")
    print(f"  crc1  = 0x{crc1:08x}")
    print(f"  crc2  = 0x{crc2:08x}")
    if fwlen == 0:
        print("ERROR: CRC fields are zero — re-run set_app_signature.py")
    else:
        print("OK: signature looks valid")
```

---

## 7. Troubleshooting Checklist

Work through this list in order. Each item corresponds to a root cause in §2.

| # | Symptom | Check | Fix |
|---|---|---|---|
| 1 | `st-flash write` returns an error about invalid address or target reset | The address `0x80004000` is wrong | Use `0x8004000` (= `0x08004000`) |
| 2 | Bootloader stays alive after power-cycle; no node appears on CAN bus | App signature CRC fields are 0 | Re-run `set_app_signature.py` manually; confirm it prints `Applied APP_DESCRIPTOR ...` |
| 3 | Node never appears but board seems to run (e.g. PWM output works) | You may have flashed a non-CAN build | Check `arm-none-eabi-objdump -h` — `.isr_vector` VMA must be `0x08004000` |
| 4 | Node appears briefly then disappears; CAN RX/TX ISR never fires | VTOR not set to `0x08004000` | Add `SCB->VTOR = 0x08004000` at the top of `initAfterJump()` in `Mcu/l431/Src/peripherals.c` |
| 5 | Bootloader loops forever after OTA attempt; erase and re-flash does not help | RTC backup register stuck at `FWUPDATE` | Run `st-flash erase` (full chip erase resets RTC domain) then flash both images |
| 6 | Bootloader overlaps firmware; vector table at `0x08004000` is corrupt | Bootloader binary > 16384 bytes | Use the correct CAN bootloader variant; verify its size with `wc -c` |
| 7 | `st-info --probe` shows no device | SWD wiring or BOOT0 issue | Check SWDIO/SWDCLK/GND; set BOOT0=0 (normal boot from flash) |
| 8 | `Flash written and verified` but board does not run | Option bytes read-protect set | Use STM32CubeProgrammer to clear RDP level back to 0 before flashing |

### Quick Command Reference

```bash
# Full erase (also clears RTC backup registers)
st-flash erase

# Flash bootloader (CAN variant, ≤ 16 K)
st-flash write <bootloader>.bin 0x8000000

# Flash AM32 CAN firmware — CORRECT address
st-flash write obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin 0x8004000

# Reset without reflashing
st-flash reset

# Verify app signature in binary
python3 Src/DroneCAN/set_app_signature.py \
    obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.bin \
    obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.elf

# Confirm linker placed vector table at 0x08004000
arm-none-eabi-objdump -h obj/AM32_VIMDRONES_L431_TUDELFT_CAN_2.19.elf | grep isr_vector

# Check board is detected
st-info --probe
```

---

## Appendix — Why `0x80004000` vs `0x08004000` Matters

Both numbers look similar but are completely different 32-bit values:

```
0x08004000  =  0000 1000 0000 0000 0100 0000 0000 0000  ← internal flash
0x80004000  =  1000 0000 0000 0000 0100 0000 0000 0000  ← external RAM/AHB region
```

The Cortex-M4 address map for the STM32L431 is:

| Region | Address range |
|---|---|
| Code / internal flash | `0x00000000` – `0x1FFFFFFF` |
| SRAM | `0x20000000` – `0x3FFFFFFF` |
| Peripherals | `0x40000000` – `0x5FFFFFFF` |
| External RAM | `0x60000000` – `0x9FFFFFFF`  ← `0x80004000` falls here |
| External device | `0xA0000000` – `0xDFFFFFFF` |
| System region | `0xE0000000` – `0xFFFFFFFF` |

Writing to `0x80004000` via `st-flash` targets an external memory region that
does not exist on this board. `st-flash` may silently report success (some versions
do not validate the target region against the known memory map), but the internal
flash at `0x08004000` remains either blank or untouched, so the bootloader finds
no valid application and loops forever.

Always double-check the address before issuing the write command.
