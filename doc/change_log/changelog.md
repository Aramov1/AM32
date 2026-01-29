# AM32 – Multi-Purpose Brushless Controller Firmware (STM32F051)

## Changelog

### Version 1.54
- Added firmware name to targets and firmware version to `main`
- Added two more DShot beacons (1–3 currently working)
- Added KV option to firmware; low-RPM power protection is now KV-based
- Start power now controls minimum idle power as well as startup strength
- Changed default timing to 22.5°
- Lowered default minimum idle setting to **1.5% duty cycle** (slider range 1–2)
- Added DShot commands to save settings and reset ESC

---

### Version 1.56
- Added check to stall protection to wait until after **40 zero-crosses** to fix high-startup throttle hiccup
- Added **TIMER1 update interrupt**; PWM changes now occur once per PWM period
- Reduced commutation interval averaging length
- Reduced false-positive filter level to 2 and eliminated threshold where filter stops
- Disabled interrupt before sounds
- Disabled TIM1 interrupt during **stepper sinusoidal mode**
- Added 28 µs delay for DShot300
- Report 0 RPM until the first 10 successful steps
- Moved serial ADC telemetry calculations and desync check to **10 kHz interrupt**

---

### Version 1.57
- Removed spurious commutations and RPM data at startup by polling for a longer interval

---

### Version 1.58
- Moved signal timeout to 10 kHz routine
- Set armed timeout to **250 ms** (2500 / 10000)

---

### Version 1.59
- Moved commutation order definitions to `target.h`
- Fixed update version number if older than new version
- Major cleanup:
  - Moved all input/output to `IO.c`
  - Moved comparator functions to `comparator.c`
  - Removed many unused variables
- Added **Siskin** target
- Moved PWM changes to 10 kHz routine
- Moved basic functions to `functions.c`
- Moved peripheral setup to `peripherals.c`
- Added crawler mode settings

---

### Version 1.60
- Added sine mode hysteresis
- Increased power in stall protection and lowered start RPM for crawlers
- Removed OneShot125 from crawler mode
- Reduced maximum startup power from 400 to 350
- Changed minimum duty cycle to `DEAD_TIME`
- Moved version and name to permanent FLASH location (thanks Mikeller)

---

### Version 1.61
- Moved duty-cycle calculation to 10 kHz and added max-change option
- Decreased maximum interval change to 25%
- Reduced wait time on fast acceleration
- Added check for early zero-cross in interrupt

---

### Version 1.62
- Moved control to 10 kHz loop
- Changed low-RPM filter condition from `||` to `&&`
- Introduced max deceleration (20 ms from 100 → 0)
- Added configurable servo throttle ranges

---

### Version 1.63
- Increased time for zero-cross error detection below 250 µs commutation interval
- Increased max change at low RPM by ×10
- Lowered throttle ramp low limit and increased upper range
- Changed desync event from full restart to throttle reduction

---

### Version 1.64
- Added startup check for continuous high signal (reboot to bootloader)
- Added brake-on-stop from EEPROM
- Added stall protection from EEPROM
- Added motor pole divider for sinusoidal and low-RPM power protection
- Fixed DShot commands; added confirmation beeps; removed blocking behavior

---

### Version 1.65
- Added 32 ms telemetry output
- Added low-voltage cutoff (divider and cutoff voltage stored in EEPROM)
- Added cell-count beep when low-voltage active
- Added current sensing on PA3 (conversion factor in EEPROM)
- Fixed servo input capture to read only positive pulse
- Disabled OneShot125
- Extended servo range to match receiver output
- Added RC-car style reverse and proportional braking
- Added brushed motor control mode
- Added EEPROM settings version 1
- Added gimbal control option

---

### Version 1.66
- Moved IWDG init to after input tuning
- Removed reset after save command (DShot)
- Added **Wraith32** target
- Added average pulse check for signal detection

---

### Version 1.67
- Reworked file structure for multiple MCU support
- Added **G071** MCU

---

### Version 1.68
- Increased allowed average pulse length to avoid double startup

---

### Version 1.69
- Removed line re-enabling comparator after disabling

---

### Version 1.70
- Fixed DShot for KISS FC

---

### Version 1.71
- Fixed DShot for ArduPilot / PX4 FC

---

### Version 1.72
- Fixed telemetry output
- Added 1-second arming delay

---

### Version 1.73
- Fixed false arming when no signal
- Removed low-RPM throttle protection below 300 KV

---

### Version 1.74
- Added sine mode range
- Added brake strength adjustment

---

### Version 1.75
- Disabled brake-on-stop for `PWM_ENABLE_BRIDGE`
- Removed automatic brake-on-stop on neutral for RC-car proportional brake
- Adjusted sine speed and stall-protection speed
- Applied Makefile fixes from Cruwaller
- Removed GD32 build until firmware is functional

---

### Version 1.76
- Adjusted G071 PWM frequency and startup power to match F051
- Reduced polling back-EMF checks for G071

---

### Version 1.77
- Increased PWM frequency range to **8–48 kHz**

---

### Version 1.78
- Fixed Bluejay tune frequency and speed
- Fixed G071 dead time
- Incremented EEPROM version

---

### Version 1.79
- Added stick throttle calibration routine
- Added variable telemetry interval

---

### Version 1.80
- Enabled comparator blanking for G071 on TIM1 CH4
- Added hardware group F for Iflight Blitz
- Adjusted PWM frequency parameters
- Added sine-mode power variable and EEPROM setting
- Fixed telemetry RPM during sine mode
- Fixed sounds for extended PWM range
- Added adjustable braking strength while driving

---

### Version 1.81
- Added current-limiting PID loop
- Fixed current sense scaling
- Increased brake power on maximum reverse (car mode)
- Added HK and Blpwr targets
- Changed low-KV motor throttle limit
- Added reverse speed threshold based on motor KV
- Doubled filter length for motors under 900 KV

---

### Version 1.82
- Added speed-control PID loop

---

### Version 1.83
- Added stall-protection PID loop
- Improved sine-mode transition
- Decreased speed step when re-entering sine mode
- Added fixed duty-cycle and fixed speed build options
- Added RPM-controlled input signal mode

---

### Version 1.84
- Changed PID values to `int` for faster calculations
- Enabled dual-channel brushed motor control
- Added current-limit max duty cycle

---

### Version 1.85
- Fixed current limit not allowing full RPM on G071 or low PWM frequency
- Removed unused brake-on-stop conditional

---

### Version 1.86
- Created do-once logic in sine mode instead of resetting PWM mode each time

---

### Version 1.87
- Fixed fixed-mode max RPM limits

---

### Version 1.88
- Fixed sine-mode re-entry stutter due to position reset

---

### Version 1.89
- Fixed drive-by-RPM mode scaling
- Fixed DShot PX4 timings

---

### Version 1.90
- Disabled comparator interrupts for brushed mode
- Re-enter polling mode after prop strike or desync
- Added G071 “N” variant
- Added preliminary Extended DShot

---

### Version 1.91
- Reset average interval time on desync only after 100 zero-crosses

---

### Version 1.92
- Moved G071 comparator blanking to TIM1 OC5
- Increased ADC read frequency and current-sense filtering
- Added addressable LED strip for G071 targets

---

### Version 1.93
- Build-process optimizations
- Added firmware file name to each target HEX
- Fixed extended telemetry not activating DShot600
- Fixed low-voltage cutoff timeout

---

### Version 1.94
- Added selectable input types

---

### Version 1.95
- Reduced armed timeout to 0.5 s

---

### Version 1.96
- Improved eRPM accuracy (DShot and serial telemetry)
- Fixed PID loop integral
- Added over-current low-voltage cutoff to brushed mode

---

### Version 1.97
- Enabled input pull-up

---

### Version 1.98
- DShot eRPM rounding compensation

---

### Version 1.99
- Added per-target max duty-cycle change option
- Fixed DShot telemetry delay on F4 and E230 MCUs

---

### Version 2.00
- Cleanup of target structure

---

### Version 2.01
- Increased 10 kHz timer to 20 kHz
- Increased max duty-cycle change

---

### Version 2.02
- Increased startup power for inverted-output targets

---

### Version 2.03
- Moved chime from DShot direction-change commands to save command

---

### Version 2.04
- Fixed current protection (max duty-cycle not increasing)
- Fixed double startup chime
- Changed current averaging method for higher precision
- Fixed startup ramp speed adjustment

---

### Version 2.05
- Fixed ramp tied to input frequency

---

### Version 2.06
- Fixed input pull-ups
- Removed half-transfer interrupt from servo routine
- Updated running brake and brake-on-stop behavior

---

### Version 2.07
- Dead-time change for F4A

---

### Version 2.08
- Moved zero-cross timing

---

### Version 2.09
- Filtered out short zero-crosses

---

### Version 2.10
- Polling only below commutation interval of 1500–2000 µs
- Fixed tune frequency again

---

### Version 2.11
- RC-car mode fix

---

### Version 2.12
- Reduced advance on hard braking

---

### Version 2.13
- Removed input capture filter for DShot2400
- Changed DShot300 speed detection threshold

---

### Version 2.14
- Reduced G071 zero-cross checks
- Assigned all MCUs duty-cycle resolution of 2000 steps

---

### Version 2.15
- Enforced minimum ½ commutation interval for G071
- Reverted timing change on braking
- Added per-target override for max duty-cycle change
- **TODO:** Fix signal detection

---

### Version 2.16
- Added **L431** MCU
- Added variable auto-timing
- Added DroneCAN support
