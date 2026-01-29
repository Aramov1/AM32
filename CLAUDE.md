# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AM32 is open-source firmware for ARM-based brushless motor ESCs (Electronic Speed Controllers). It supports multiple MCU families: STM32F051, STM32F031, STM32G071, STM32G031, STM32G431, STM32L431, GD32E230, AT32F415, AT32F421, and CH32V203.

## Build Commands

### First-time setup (installs ARM toolchain)
```bash
make arm_sdk_install
```

### Build specific target
```bash
make <TARGET_NAME>    # e.g., make FD6288
```

### Build all targets for a specific MCU
```bash
make f051             # Build all F051 targets
make g071             # Build all G071 targets
make l431             # Build all L431 targets
```

### Build all targets
```bash
make all
make -j8              # Parallel build (Linux/macOS)
```

### List available targets
```bash
make targets
```

### Clean build artifacts
```bash
make clean
```

Output files (.bin, .hex, .elf) are placed in the `obj/` directory.

## Architecture

### Directory Structure
- `Src/` - Core firmware source (main.c, motor control, protocols)
- `Inc/` - Core headers including targets.h (all ESC target definitions)
- `Mcu/<mcu>/` - MCU-specific HAL code, startup files, linker scripts, and peripherals
- `Src/DroneCAN/` - DroneCAN protocol support (CAN bus communication)

### MCU Abstraction
Each MCU family has:
- `<mcu>makefile.mk` - Build configuration at repo root
- `Mcu/<mcu>/Src/` - MCU-specific implementations (ADC.c, IO.c, comparator.c, peripherals.c, phaseouts.c)
- `Mcu/<mcu>/Inc/` - MCU-specific headers

### Target System
Targets are defined in `Inc/targets.h`. Each target (e.g., `FD6288_F051`) defines:
- `FILE_NAME` - Build identifier
- `FIRMWARE_NAME` - Display name (max 12 chars)
- `DEAD_TIME` - PWM dead time
- `HARDWARE_GROUP_*` - Pin/peripheral mapping
- ADC channels, voltage dividers, current sensing config

Targets with `_CAN` suffix enable DroneCAN support and use separate linker scripts.

### Motor Control Flow
- `main.c` - Main control loop, commutation timing, 10kHz control loop
- `dshot.c`/`signal.c` - Input protocol handling (DShot, Servo PWM, Serial)
- `functions.c` - Commutation, sinusoidal startup, speed/current PID loops
- `sounds.c` - Startup tones and beeps

### Input Protocols
Supports AUTO detection between: DShot (300/600), Servo PWM, Serial, DroneCAN

### Configuration
Settings stored in EEPROM structure defined in `Inc/eeprom.h`. Configurable via AM32 Configurator or BetaFlight passthrough.

## Adding New Targets

1. Add target definition block in `Inc/targets.h` with appropriate `#ifdef TARGET_NAME` guard
2. Define `FILE_NAME`, `FIRMWARE_NAME`, `DEAD_TIME`, `HARDWARE_GROUP_*`
3. The Makefile automatically discovers targets via `FILE_NAME` patterns

## Debugging

Building copies `debug.elf` and `debug.svd` to `obj/` for VSCode Cortex-Debug integration.
