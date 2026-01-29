#define DEAD_TIME 10


/*===========================================================================
 * Library Includes
 *===========================================================================*/

#include "main.h"
#include "ADC.h"
#include "IO.h"
#include "common.h"
#include "comparator.h"
#include "dshot.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "serial_telemetry.h"
#include "kiss_telemetry.h"
#include "signal.h"
#include "sounds.h"
#include "targets.h"
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <version.h>

#ifdef USE_LED_STRIP
#include "WS2812.h"
#endif

#ifdef USE_CRSF_INPUT
#include "crsf.h"
#endif

#if DRONECAN_SUPPORT
#include "DroneCAN/DroneCAN.h"
#endif

void zcfoundroutine(void);

/*===========================================================================
 * Firmware Build Options
 *===========================================================================*/

// firmware build options !! fixed speed and duty cycle modes are not to be used
// with sinusoidal startup !!

//#define FIXED_DUTY_MODE             // bypasses signal input and arming, uses a set duty cycle. For pumps, slot cars etc 
//#define FIXED_DUTY_MODE_POWER 100   //0-100 percent not used in fixed speed mode

// #define FIXED_SPEED_MODE  // bypasses input signal and runs at a fixed rpm using the speed control loop PID 
// #define FIXED_SPEED_MODE_RPM  1000  // intended final rpm , ensure pole pair numbers are entered correctly in config tool.

#define USE_MULTIPLE_INPUT      // Enables multiple input references (Average ("newinput"), Amplitude("newinput_amplitude"), Phase_offset ("newinput_phase_offset")) for brushless motor control"
                                // Incompatible with speed only control modes
#define FIXED_SPEED_SINE_MODE   // bypasses input signal and runs at a fixed (AVG_RPM + AMP_RPM * sin(AVG_RPM * dt + PHASE_OFFSET_DEG))
#define FIXED_SINE_MODE_AMP 200              // amplitude of sine modulation
#define FIXED_SINE_MODE_PHASE_OFFSET_DEG 0   // phase offset in degrees
#define FIXED_SINE_MODE_AVERAGE 1000         // Average Throttle


// #define BRUSHED_MODE   // overrides all brushless config settings and enables two channels for brushed control 
//#define GIMBAL_MODE     // sinusoidal_startup needs to be on, maps input to sinusoidal angle.



/*===========================================================================
 * Configuration & Defaults
 *===========================================================================*/

/***************************/
/*** Hardware PARAMETERS ***/
/***************************/
EEprom_t eepromBuffer;
uint32_t eeprom_address = EEPROM_START_ADD; 

// Firmware metadata
const char filename[30] __attribute__((section(".file_name"))) = FILE_NAME;
_Static_assert(sizeof(FIRMWARE_NAME) <=13,"Firmware name too long");   // max 12 character firmware name plus NULL


#define TEMP30_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFFF7B8))  // current auto reset value
#define TEMP110_CAL_VALUE ((uint16_t*)((uint32_t)0x1FFFF7C2)) // maximum auto reset register value

// Timers
tim1_arr = TIM1_AUTORELOAD;       // Current auto-reload value, PWM FREQUENCY
timer1_max_arr = TIM1_AUTORELOAD; // Maximum auto-reload value
/*
The firmware uses two different "worlds":
  ┌──────────┬─────────────────────┬──────────────┬─────────────────────────────────────┐
  │  World   │      Variable       │    Range     │               Purpose               │
  ├──────────┼─────────────────────┼──────────────┼─────────────────────────────────────┤
  │ Internal │ duty_cycle          │ 0 - 2000     │ Fixed scale for calculations        │
  ├──────────┼─────────────────────┼──────────────┼─────────────────────────────────────┤
  │ Hardware │ adjusted_duty_cycle │ 0 - tim1_arr │ Actual timer compare register value │
  └──────────┴─────────────────────┴──────────────┴─────────────────────────────────────┘
  The tim1_arr (Timer 1 Auto-Reload Register) varies based on PWM frequency settings, but the internal logic 
  always works with a consistent 0-2000 scale.
*/


typedef enum { GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET } GPIO_PinState;

/**********************/
/*** PID PARAMETERS ***/
/**********************/
typedef struct {
    fastPID angle;     // for Mechanical Angle Control
    fastPID speed;     // for Mechanical Speed Control
    fastPID current;   // for Current Control
    fastPID stall;     // for Stall Protection
} PID_Config_t;

PID_Config_t pid_cfg = {
    .angle =   { .Kp = 10,   .Ki = 5,  .Kd = 0,    .integral_limit = 10000, .output_limit = 50000 },
    .speed =   { .Kp = 10,   .Ki = 0,  .Kd = 100,  .integral_limit = 10000, .output_limit = 50000 },
    .current = { .Kp = 400,  .Ki = 0,  .Kd = 1000, .integral_limit = 20000, .output_limit = 100000},
    .stall =   { .Kp = 1,    .Ki = 0,  .Kd = 50,   .integral_limit = 10000, .output_limit = 50000 }
};

// IMPORTANT NOTE: Implemented control loop diagram at doc/implementation/control_diagram.png



/***************************************/
/*** Motor Specifications PARAMETERS ***/
/***************************************/
char forward = 1;
char prop_brake_active;


/******************************/
/*** Motor State PARAMETERS ***/
/******************************/
// Status flags
char crawler_mode; 
char armed;
char running;          // Flag to indicate if motor is/has started to move
uint8_t step = 0;      // Eletrical Cycle is made of 6 steps
char stepper_sine;     // Flag for Open Loop Motor Control without BEMF Sensing
char old_routine = 1;  // BEMF sampling mode: 0: Polling Mode, 1: Interruption Mode
/*
Interrupt mode vs Polling mode:
  ┌───────────────────────────┬──────────────────────────────────────────────┬───────────────────────────────────────────┐
  │           Mode            │             Zero-cross detection             │            Commutation trigger            │
  ├───────────────────────────┼──────────────────────────────────────────────┼───────────────────────────────────────────┤
  │ Interrupt (old_routine=0) │ Comparator interrupt → interruptRoutine()    │ Timer interrupt → PeriodElapsedCallback() │
  ├───────────────────────────┼──────────────────────────────────────────────┼───────────────────────────────────────────┤
  │ Polling (old_routine=1)   │ Polled in tenKhzRoutine() via getBemfState() │ Directly via zcfoundroutine()             │
  └───────────────────────────┴──────────────────────────────────────────────┴───────────────────────────────────────────┘

    ---
  Mode Transitions

                      STARTUP
                         │
                         ▼
          ┌──────────────────────────┐
          │  use_sine_start enabled? │
          └──────────────────────────┘
                │              │
               YES            NO
                │              │
                ▼              ▼
       ┌────────────┐   ┌────────────┐
       │stepper_sine│   │old_routine │
       │    = 1     │   │    = 1     │
       │ (open loop)│   │ (polling)  │
       └────────────┘   └────────────┘
                │              │
                │   ┌──────────┘
                ▼   ▼
       ┌─────────────────────────────┐
       │       old_routine = 1       │
       │    (polling BEMF mode)      │
       │  Transition when:           │
       │  - phase_A_position == 0    │
       │    (sine mode complete)     │
       └─────────────────────────────┘
                      │
                      │ Transition when:
                      │ - commutation_interval < threshold
                      │ - OR zero_crosses >= 20 (with stall prot)
                      ▼
       ┌─────────────────────────────┐
       │       old_routine = 0       │
       │   (interrupt BEMF mode)     │
       │                             │
       │  Falls back to old_routine=1│
       │  on desync or direction     │
       │  change                     │
       └─────────────────────────────┘

  ---
  When to Use Each Mode
  Mode: stepper_sine
  Use Case: Heavy props, high-pole motors, crawlers
  Why: Motor may not generate enough BEMF at very low speed; open-loop ensures rotation starts
  ────────────────────────────────────────
  Mode: old_routine=1 (polling)
  Use Case: Startup, low RPM, recovery
  Why: More robust filtering, works with weak BEMF signal
  ────────────────────────────────────────
  Mode: old_routine=0 (interrupt)
  Use Case: Normal operation, high RPM
  Why: Lower CPU overhead, precise timing
  ---


  */
// Motor Protection related
char desync_check;    // Set to 1 in commutate() every full eletrical cycle, to to detect if the   motor has lost synchronization with the BEMF signals 
                      // In main(),  a desync is detected when ALL conditions are meet:
                      //    1. desync_check == 1 (end of electrical revolution)
                      //    2. zero_crosses > 10 (motor has started running) (Used over running = 1 ensure stable commutaton and prevent trigger false desync detections during normal startup)p )
                      //    3. Interval changed by more than 50%: getAbsDif(last_average_interval, average_interval) > average_interval >> 1
                      //    4. average_interval < 2000 (motor is spinning reasonably fast)
uint8_t desync_happened; // For Debug Purposes
uint8_t stuckcounter;    // For Debug Purposes

char cell_count = 0;      // Number of battery cells detected at arming time (used for low voltage cutoff)
uint16_t battery_voltage; // Smoothed battery voltage (x100) reading. Updated in main(). Used for low voltyage cutoff protection, Telemetry Reporting & Voltage Based Ramp.
int32_t consumed_current; // Total Consummed current in mAh units 
int16_t actual_current;   // Measured current in 10mA units
uint8_t degrees_celsius;  // 
uint16_t e_rpm;     // electrical RPM / 10
uint16_t k_erpm;    // eletrical RPM / 100


uint16_t signaltimeout;   // Failsafe watchdog, incremented every tenkhzRoutine(), used to detect loss of the input signal. Triggers a safety shutdown in main() and is reseted to 0 when valied input is received.

/******************************/
/*** Commutation PARAMETERS ***/
/******************************/
char rising = 1;      // indicates whether the firmware is looking for a rising edge or falling edge zero-crossing on the BEMF signal.
                      // In a 3-phase BLDC motor, the undriven (floating) phase produces a BEMF signal that alternates between rising and falling through the neutral point.  
                      // The firmware must know which edge to look for to correctly identify the zero-crossing and time the next commutation
uint32_t zero_crosses;// Number of zero crosses detected -> Ensure motor is running smoothly
uint16_t thiszctime;  // Capture time that will happen until next ZC
uint16_t lastzctime;  // Capture previous time between ZCs
uint32_t commutation_interval = 12500; // Time between last two commutations
uint16_t commutation_intervals[6];     // List with previous 6 commutation intervals
uint16_t advance;            // Expected hardware execution delay time   .
uint8_t temp_advance;        // Used to compute "advance" to estimate next waitTime accounting for hardware delays
uint8_t auto_advance_level;  // Used to compute "advance" to estimate next waitTime accounting for hardware delays
uint16_t waitTime;           // Delay from next ZC to next commutation. Equals to ("commutation_interval/2" - "advance")

/***********************/
/*** Bemf PARAMETERS ***/
/***********************/
uint8_t filter_level = 5;  // Number of consecutive readings required to validate zero crossing detection in Interrupt Mode
uint8_t bemfcounter;       // Updated in getBemfState, only used in polling mode. Number of consecutive comparator samples that match the expected BEMF state
                           // when bemfcounter > min_bemf_counts, a zero crossing is detected (zcfound = 1)
uint8_t bad_count;         // Updated in getBemfState, only used in polling mode. Number of consecutive comparator samples that DON'T MATCH the expected BEMF state
                           // when bad_count > bad_count_threshold, reject BEMF Counter ans start counting again
uint8_t zcfound;           // Only used in polling mode. Set to 1 when valid zc_found and jum to zc_foundroutine()
uint8_t min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;    // Indicates minimum number of coherent BEMF detections to accept ZC in polling mode 
uint8_t min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;  // Indicates minimum number of coherent BEMF detections to accept ZC in polling mode 
uint8_t bad_count_threshold = CPU_FREQUENCY_MHZ / 24;

uint8_t bemf_timeout_happened;  // Counter for consecutive BEMF timeout events (0-255 (set to 102 when triggered))
char bemf_timeout = 10;         // Threshold before rotor protection activates (10 (normal) or 100 (low throttle))



/****************************/
/*** Telemetry PARAMETERS ***/
/****************************/

char send_telemetry;
uint16_t telem_ms_count; // 
uint8_t telemetry_interval_ms = 30;

/************************/
/*** Input PARAMETERS ***/
/******************+*****/
char dshot;     // Indicates if DShot protocol is detected. 1 = DShot, 0 = PWM
                // NOTE: AM32 supports two types of telemetry:                                                                
                //    1. Serial Telemetry - Periodic data packets sent over a separate wire (KISS/BLHeli protocol)       
                //    2. DShot Bidirectional Telemetry - eRPM sent back on the same signal wire
char servoPwm;  // Indicates if input protocol is analog servo PWM (vs DShot). 1 = PWM, 0 = DShot
uint8_t process_adc_flag; // Flag to trigger ADC reading at lower priority. Set every 1kHz loop in tenkhzRoutine(), and read in main().

uint16_t armed_timeout_count; // Counts how long throttle has been at zero (in 20kHz loop ticks). Arming requires count > LOOP_FREQUENCY_HZ (~1 second of zero throttle)
uint16_t zero_input_count;    // Counts consecutive zero-input readings for noise filtering. Used at arming, as arming requires > 30 consecutive zero readings
char inputSet;                // Indicates if a valid input signal has been received from receiver/FC. 1 = valid input, 0 = no input

uint16_t newinput;        // Raw decoded throttle value   (DShot(0-2027 = 2^11 bits), Servo PWM (0-2000))
uint16_t adjusted_input;  // Throtlle value after correction for direction handling and dead band (0-2047)
uint16_t input;           // Final throttle for duty cycle calculation (Sine Mode curve Mapping, Speed Control PID, Stuck Rotor Protection)

// Inputs for Angle Control Mode Enabled
uint16_t newinput_amplitude;         // Raw decoded Sine Amplitude value   (????????????????????????????????)
uint16_t newinput_phase_offset;      // Raw decoded Sine Phase Offset value  (???????????????????????????????)
uint16_t adjusted_input_amplitude;   // Throtlle value after correction for direction handling and dead band (0-2047)
uint16_t adjusted_input_phase_offset;// Throtlle value after correction for direction handling and dead band (0-2047)



/********************************/
/*** Motor Control PARAMETERS ***/
/********************************/
 
char use_speed_control_loop; // Indicates if new_input should be interpreted as Throttle (0) or Speed (1)
char use_angle_control_loop = 0; // Indicates if external angle control loop is enabled (1) or disabled (0)

int32_t e_com_time; // time for one complete electrical revolution in microseconds


// NOTE: Angle Control Only allowed with DShot input protocol
int32_t minimum_angle_control_threshold = 500; // ~25% throttle
int32_t angle_controller_output = 0;  // Output from fast anglePID
int32_t target_m_angle = 0;
int32_t m_angle = 0;
int32_t m_angle_base = 0;
int8_t m_step = 0;



uint16_t m_angle_inc;  // Mechanical Angle increment between commutation steps (deg), in multiples of 0.1 deg  




/******************************/
/*** Motor Input PARAMETERS ***/
/******************************/
uint16_t duty_cycle_setpoint; // target/desired duty cycle calculated from the input signal. Updated in SetInput()
uint16_t duty_cycle;          // actual duty cycle applied to the motor after ramping. 
                              // It gradually approaches duty_cycle_setpoint to ensure smooth throttle response with rate limiting.
uint16_t last_duty_cycle;
uint16_t adjusted_duty_cycle; // ???????
uint8_t max_duty_cycle_change = 2;
volatile uint8_t max_ramp_startup =  RAMP_SPEED_STARTUP;  // Default 2
volatile uint8_t max_ramp_low_rpm =  RAMP_SPEED_LOW_RPM;  // Default 6
volatile uint8_t max_ramp_high_rpm = RAMP_SPEED_HIGH_RPM; // Default 16
volatile uint8_t ramp_divider;  // Controls frequency of the ramp limiter logic in tenKhzRoutine() (Normal Operation 1 -> every cycle). Defined in loadEEpromSettings(). 
uint32_t average_interval;      // average time per commutation step in timer counts, derived from the e_com_time in main()
uint32_t last_average_interval; // previous average time per commutation step in timer counts. Updated in main()
/*  
  ┌────────────────────────┬──────────────────┬──────────────────────┐
  │ average_interval Value │ Approximate eRPM │     Motor State      │
  ├────────────────────────┼──────────────────┼──────────────────────┤
  │ > 2500                 │ < 400            │ Very slow / starting │
  ├────────────────────────┼──────────────────┼──────────────────────┤
  │ > 500                  │ < 2000           │ Low RPM              │
  ├────────────────────────┼──────────────────┼──────────────────────┤
  │ 200 - 500              │ 2000 - 5000      │ Medium RPM           │
  ├────────────────────────┼──────────────────┼──────────────────────┤
  │ 100 - 200              │ 5000 - 10000     │ High RPM             │
  ├────────────────────────┼──────────────────┼──────────────────────┤
  │ < 100                  │ > 10000          │ Very high RPM        │
  └────────────────────────┴──────────────────┴──────────────────────┘
  */

/*********************************/
/*** Loop Structure PARAMETERS ***/
/*********************************/
uint16_t tenkhz_counter;
uint16_t one_khz_loop_counter;
uint16_t led_counter;

/*
  Complete Signal Flow: Input → PWM

  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                           1. INPUT PROTOCOL LAYER                               │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  DShot (300/600)  │  Servo PWM  │  Serial  │  DroneCAN  │  ADC (analog)         │
  │       ↓                 ↓            ↓           ↓              ↓               │
  │  computeDshotDMA()  signalinterrupt()   (various handlers)  ADC_smoothed_input  │
  │       ↓                 ↓            ↓           ↓              ↓               │
  │                    newinput (0-2000 raw value)                                  │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                     2. setInput() - DIRECTION & MAPPING                         │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  newinput (0-2000)                                                              │
  │       ↓                                                                         │
  │  Bi-directional mode:                                                           │
  │    • Center = 1000, dead band applied (servo_dead_band)                         │
  │    • >1000+dead_band → forward, map to 47-2047                                  │
  │    • <1000-dead_band → reverse, map to 47-2047                                  │
  │    • Direction change only allowed at low speed (commutation_interval > threshold)│
  │  Uni-directional mode:                                                          │
  │    • adjusted_input = newinput                                                  │
  │       ↓                                                                         │
  │  adjusted_input (0-2047)                                                        │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                     3. SPEED CONTROL PID (optional)                             │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  If use_speed_control_loop enabled (1kHz):                                      │
  │       ↓                                                                         │
  │  target_e_com_time = map(adjusted_input, 47, 2047, MIN_RPM, MAX_RPM)            │
  │       ↓                                                                         │
  │  speedPid: error = e_com_time - target_e_com_time                               │
  │       ↓                                                                         │
  │  input_override += doPidCalculations(&speedPid, e_com_time, target_e_com_time)  │
  │       ↓                                                                         │
  │  input = input_override / 10000  (replaces adjusted_input)                      │
  │                                                                                 │
  │  Else: input = adjusted_input                                                   │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                  4. INPUT → DUTY_CYCLE_SETPOINT MAPPING                         │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  input (47-2047)                                                                │
  │       ↓                                                                         │
  │  duty_cycle_setpoint = map(input, 47, 2047, minimum_duty_cycle, 2000)           │
  │       ↓                                                                         │
  │  With sine startup: map(input, 137, 2047, minimum_duty_cycle+40, 2000)          │
  │       ↓                                                                         │
  │  duty_cycle_setpoint (minimum_duty_cycle to 2000)                               │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                    5. STARTUP CONSTRAINTS (zero_crosses < 30)                   │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  During startup:                                                                │
  │    • MIN: duty_cycle_setpoint >= min_startup_duty                               │
  │    • MAX: duty_cycle_setpoint <= startup_max_duty_cycle (minimum + 400)         │
  │                                                                                 │
  │  min_startup_duty = minimum_duty_cycle + eepromBuffer.startup_power             │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                    6. DUTY_CYCLE_MAXIMUM LIMIT                                  │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  duty_cycle_maximum dynamically adjusted by:                                    │
  │                                                                                 │
  │  Low RPM Throttle Protection:                                                   │
  │    duty_cycle_maximum = map(k_erpm, low_rpm_level, high_rpm_level,              │
  │                             throttle_max_at_low_rpm, 2000)                      │
  │                                                                                 │
  │  Temperature Protection:                                                        │
  │    duty_cycle_maximum = map(degrees_celsius, temp-10, temp+10, 2000, 0)         │
  │       ↓                                                                         │
  │  if duty_cycle_setpoint > duty_cycle_maximum:                                   │
  │      duty_cycle_setpoint = duty_cycle_maximum                                   │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                    7. CURRENT LIMIT PID (1kHz loop)                             │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  currentPid: error = actual_current - (eepromBuffer.limits.current * 200)       │
  │       ↓                                                                         │
  │  use_current_limit_adjust -= doPidCalculations(&currentPid, ...) / 10000        │
  │       ↓                                                                         │
  │  Clamped: minimum_duty_cycle <= use_current_limit_adjust <= 2000                │
  │       ↓                                                                         │
  │  if duty_cycle_setpoint > use_current_limit_adjust:                             │
  │      duty_cycle_setpoint = use_current_limit_adjust                             │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                    8. STALL PROTECTION PID (1kHz loop)                          │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  stallPid: error = commutation_interval - stall_protect_target_interval         │
  │       ↓                                                                         │
  │  stall_protection_adjust += doPidCalculations(&stallPid, ...)                   │
  │       ↓                                                                         │
  │  Clamped: 0 <= stall_protection_adjust <= 150 * 10000                           │
  │       ↓                                                                         │
  │  duty_cycle_setpoint += stall_protection_adjust / 10000                         │
  │  (BOOSTS throttle when RPM drops - for crawlers/RC cars only)                   │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │              9. tenKhzRoutine() - DUTY CYCLE RAMPING (20kHz)                    │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  duty_cycle = duty_cycle_setpoint                                               │
  │       ↓                                                                         │
  │  max_duty_cycle_change selected based on conditions:                            │
  │    • Startup (zero_crosses < 150): max_ramp_startup                             │
  │    • Low RPM (average_interval > 500): max_ramp_low_rpm                         │
  │    • High RPM: max_ramp_high_rpm                                                │
  │       ↓                                                                         │
  │  Rate limiting (every ramp_divider cycles):                                     │
  │    if (duty_cycle - last_duty_cycle) > max_duty_cycle_change:                   │
  │        duty_cycle = last_duty_cycle + max_duty_cycle_change                     │
  │    if (last_duty_cycle - duty_cycle) > max_duty_cycle_change:                   │
  │        duty_cycle = last_duty_cycle - max_duty_cycle_change                     │
  │       ↓                                                                         │
  │  last_duty_cycle = duty_cycle                                                   │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                    10. PWM TIMER SCALING                                        │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  duty_cycle (0-2000 internal scale)                                             │
  │       ↓                                                                         │
  │  adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000) + 1                     │
  │       ↓                                                                         │
  │  tim1_arr = TIMER1_MAX_ARR (varies with PWM frequency setting)                  │
  │       ↓                                                                         │
  │  adjusted_duty_cycle (0 to tim1_arr timer counts)                               │
  └─────────────────────────────────────────────────────────────────────────────────┘
                                        ↓
  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │                    11. PWM OUTPUT                                               │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  SET_AUTO_RELOAD_PWM(tim1_arr)     // Set PWM period                            │
  │  SET_DUTY_CYCLE_ALL(adjusted_duty_cycle)  // Apply to all 3 phases              │
  │       ↓                                                                         │
  │  TIM1->CCR1, CCR2, CCR3 = adjusted_duty_cycle                                   │
  │       ↓                                                                         │
  │  PWM signals to motor phases (active phase selected by commutation step)        │
  └─────────────────────────────────────────────────────────────────────────────────┘

  ---
  Key Variables Summary
  ┌─────────────────────┬─────────────────────────┬───────────────────────────────────────────┐
  │      Variable       │          Range          │                  Purpose                  │
  ├─────────────────────┼─────────────────────────┼───────────────────────────────────────────┤
  │ newinput            │ 0-2000                  │ Raw input from protocol handler           │
  ├─────────────────────┼─────────────────────────┼───────────────────────────────────────────┤
  │ adjusted_input      │ 0-2047                  │ After direction/dead band processing      │
  ├─────────────────────┼─────────────────────────┼───────────────────────────────────────────┤
  │ input               │ 0-2047                  │ After speed control override (if enabled) │
  ├─────────────────────┼─────────────────────────┼───────────────────────────────────────────┤
  │ duty_cycle_setpoint │ minimum_duty_cycle-2000 │ Target duty cycle                         │
  ├─────────────────────┼─────────────────────────┼───────────────────────────────────────────┤
  │ duty_cycle          │ minimum_duty_cycle-2000 │ Ramped duty cycle                         │
  ├─────────────────────┼─────────────────────────┼───────────────────────────────────────────┤
  │ adjusted_duty_cycle │ 0-tim1_arr              │ Timer counts for PWM                      │
  └─────────────────────┴─────────────────────────┴───────────────────────────────────────────┘
  PID Controllers Summary
  ┌────────────┬──────────────────────┬──────────────────────┬───────────────────────────────────────┐
  │    PID     │    Input (actual)    │        Target        │           Effect on Output            │
  ├────────────┼──────────────────────┼──────────────────────┼───────────────────────────────────────┤
  │ speedPid   │ e_com_time           │ target_e_com_time    │ Overrides input for closed-loop speed │
  ├────────────┼──────────────────────┼──────────────────────┼───────────────────────────────────────┤
  │ currentPid │ actual_current       │ current_limit*200    │ Caps duty_cycle_setpoint              │
  ├────────────┼──────────────────────┼──────────────────────┼───────────────────────────────────────┤
  │ stallPid   │ commutation_interval │ stall_protect_target │ Boosts duty_cycle_setpoint            │
  └────────────┴──────────────────────┴──────────────────────┴───────────────────────────────────────┘
*/


//prop_brake_active;
//prop_brake_duty_cycle;
//min_startup_duty;


uint8_t changeover_step = 5;
char do_once_sinemode;
int16_t phase_A_position;
int16_t phase_B_position;
int16_t phase_C_position;
uint16_t enter_sine_angle = 180;
uint16_t step_delay = 100;

uint16_t min_startup_duty = 120;
uint16_t startup_max_duty_cycle = 200;
uint16_t sin_mode_min_s_d = 120;
char startup_boost = 50;
uint16_t minimum_duty_cycle = DEAD_TIME;
uint16_t duty_cycle_maximum = 2000;
uint16_t throttle_max_at_low_rpm = 400;
uint16_t throttle_max_at_high_rpm = 2000;
char low_rpm_throttle_limit = 1;
uint16_t low_rpm_level = 20;
uint16_t high_rpm_level = 70;
char fast_accel = 1;
char fast_deccel = 0;
char maximum_throttle_change_ramp=1;

uint16_t low_threshold = 1100;
uint16_t high_threshold = 1900;
uint16_t neutral = 1500;
uint8_t dead_band = 100;

uint16_t voltage_divider = TARGET_VOLTAGE_DIVIDER;
char low_voltage_cutoff_enabled = 0;
uint16_t low_cell_volt_cutoff = 330;
uint16_t low_voltage_count;
uint16_t prop_brake_duty_cycle = 0;
char reversing_dead_band=1;
uint16_t reverse_speed_threshold=1500;
char brushed_direction_set=0;

uint32_t mcu_id;
uint32_t rev_id;
uint32_t process_time;
uint32_t start_process;
char play_tone_flag;
uint16_t velocity_count;
uint16_t velocity_count_threshold = 75;
uint16_t low_pin_count;
uint8_t analog_watchdog_status = RESET;

int32_t input_override;
int16_t speed_pid_output;
uint16_t target_e_com_time;
uint16_t target_e_com_time_high;
uint16_t target_e_com_time_low;
uint8_t drive_by_rpm;
uint32_t maximum_rpm_speed_control = 10000;
uint32_t minimum_rpm_speed_control = 1000;

// Angle control
uint16_t current_angle = 90;
uint16_t desired_angle = 90;
char return_to_center;

// Current limiting
char use_current_limit;
int16_t use_current_limit_adjust = 2000;

// Stall protection
int32_t stall_protection_adjust;
uint16_t stall_protect_target_interval = TARGET_STALL_PROTECTION_INTERVAL;
uint16_t stall_protect_minimum_duty = DEAD_TIME;

int sin_divider = 2;
uint16_t motor_kv = 4300;
char low_kv;
char low_kv_filter_level = 20;
uint8_t filter_level = 5;
uint8_t advancedivisor;
uint8_t dead_time_override = DEAD_TIME;
uint16_t gate_drive_offset = DEAD_TIME;


uint16_t ADC_raw_temp;
uint16_t ADC_raw_volts;
uint16_t ADC_raw_current;
uint16_t ADC_raw_input;
uint16_t ADC_smoothed_input;
uint16_t ADC_CCR = 30;
int16_t converted_degrees;
uint8_t temperature_offset;
int32_t smoothed_raw_current;
uint16_t smoothed_current;
uint16_t readings[50];
uint8_t read_index = 0;
uint32_t total = 0;
const uint8_t num_readings = 50;
char telemetry_done;
char send_esc_info_flag;
uint16_t angle_input;
uint16_t new_angle_input;
uint16_t avg_speed_input;
uint16_t amplitude_input;
uint16_t phase_offset_input;

uint16_t armed_count_threshold;

uint8_t last_dshot_command;
uint8_t dshotcommand;
uint8_t compute_dshot_flag;
char dshot_telemetry;

uint8_t crsf_input_channel = 1;
uint8_t crsf_output_PWM_channel = 2;

#ifdef NEED_INPUT_READY
volatile char input_ready;
#endif


int32_t doPidCalculations(struct fastPID* pidnow, int actual, int target)
{
    // IMPORTANT NOTE:
    // AM32 does not use floating-point math for PID control, to ensure faster execution. 
    // For this reason, all PID constants (Kp, Ki, Kd) and limits are scaled by a factor of x10 000.
    pidnow->error = actual - target;
    pidnow->integral = pidnow->integral + pidnow->error * pidnow->Ki;
    if (pidnow->integral > pidnow->integral_limit) {
        pidnow->integral = pidnow->integral_limit;
    }
    if (pidnow->integral < -pidnow->integral_limit) {
        pidnow->integral = -pidnow->integral_limit;
    }

    pidnow->derivative = pidnow->Kd * (pidnow->error - pidnow->last_error);
    pidnow->last_error = pidnow->error;

    pidnow->pid_output = pidnow->error * pidnow->Kp + pidnow->integral + pidnow->derivative;

    if (pidnow->pid_output > pidnow->output_limit) {
        pidnow->pid_output = pidnow->output_limit;
    }
    if (pidnow->pid_output < -pidnow->output_limit) {
        pidnow->pid_output = -pidnow->output_limit;
    }
    return pidnow->pid_output;
}

void loadEEpromSettings()
{
    read_flash_bin(eepromBuffer.buffer, eeprom_address, sizeof(eepromBuffer.buffer));
    
    if(eepromBuffer.eeprom_version < EEPROM_VERSION){
      eepromBuffer.max_ramp = 160;    // 0.1% per ms to 25% per ms 
      eepromBuffer.minimum_duty_cycle = 1; // 0.2% to 51 percent
      eepromBuffer.disable_stick_calibration = 0; // 
      eepromBuffer.absolute_voltage_cutoff = 10;  // voltage level 1 to 100 in 0.5v increments
      eepromBuffer.current_P = 100; // 0-255
      eepromBuffer.current_I = 0; // 0-255
      eepromBuffer.current_D = 100; // 0-255
      eepromBuffer.active_brake_power = 0; // 1-5 percent duty cycle
      eepromBuffer.reserved_eeprom_3[0] = 0; //14-16  for crsf input
      eepromBuffer.reserved_eeprom_3[1] = 0;
      eepromBuffer.reserved_eeprom_3[2] = 0;
      eepromBuffer.reserved_eeprom_3[3] = 0;
    }
    // eepromBuffer.advance_level can either be set to 0-3 with config tools less than 1.90 or 10-42 with 1.90 or above 
    if (eepromBuffer.advance_level > 42 || (eepromBuffer.advance_level < 10 && eepromBuffer.advance_level > 3)){
        temp_advance = 16;
    }
    if (eepromBuffer.advance_level < 4) {         // old format needs to be converted to 0-32 range
        temp_advance = (eepromBuffer.advance_level<<3);
    }
    if (eepromBuffer.advance_level < 43 && eepromBuffer.advance_level > 9 ) { // new format subtract 10 from advance
        temp_advance = eepromBuffer.advance_level - 10;
    }

    if (eepromBuffer.pwm_frequency < 145 && eepromBuffer.pwm_frequency > 7) {
        if (eepromBuffer.pwm_frequency < 145 && eepromBuffer.pwm_frequency > 23) {
            TIMER1_MAX_ARR = map(eepromBuffer.pwm_frequency, 24, 144, TIM1_AUTORELOAD, TIM1_AUTORELOAD / 6);
        }
        if (eepromBuffer.pwm_frequency < 24 && eepromBuffer.pwm_frequency > 11) {
            TIMER1_MAX_ARR = map(eepromBuffer.pwm_frequency, 12, 24, TIM1_AUTORELOAD * 2, TIM1_AUTORELOAD);
        }
        if (eepromBuffer.pwm_frequency < 12 && eepromBuffer.pwm_frequency > 7) {
            TIMER1_MAX_ARR = map(eepromBuffer.pwm_frequency, 7, 16, TIM1_AUTORELOAD * 3,
                TIM1_AUTORELOAD / 2 * 3);
        }
        SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    } else {
        tim1_arr = TIM1_AUTORELOAD;
        SET_AUTO_RELOAD_PWM(tim1_arr);
    }
    if(eepromBuffer.minimum_duty_cycle < 51 && eepromBuffer.minimum_duty_cycle > 0){
    minimum_duty_cycle = eepromBuffer.minimum_duty_cycle * 10;
    }else{
    minimum_duty_cycle = 0;
    }
    if (eepromBuffer.startup_power < 151 && eepromBuffer.startup_power > 49) {
            min_startup_duty = minimum_duty_cycle + eepromBuffer.startup_power;
    } else {
        min_startup_duty = minimum_duty_cycle;
    }
    startup_max_duty_cycle = minimum_duty_cycle + 400;  

    motor_kv = (eepromBuffer.motor_kv * 40) + 20;
#ifdef THREE_CELL_MAX
		motor_kv =  motor_kv / 2;
#endif
    setVolume(2);
    if (eepromBuffer.eeprom_version > 0) { // these commands weren't introduced until eeprom version 1.
#ifdef CUSTOM_RAMP

#else
        if (eepromBuffer.beep_volume > 11) {
            setVolume(5);
        } else {
            setVolume(eepromBuffer.beep_volume);
        }
#endif
        servo_low_threshold = (eepromBuffer.servo.low_threshold * 2) + 750; // anything below this point considered 0
        servo_high_threshold = (eepromBuffer.servo.high_threshold * 2) + 1750; // anything above this point considered 2000 (max)
        servo_neutral = (eepromBuffer.servo.neutral) + 1374;
        servo_dead_band = eepromBuffer.servo.dead_band;

        low_cell_volt_cutoff = eepromBuffer.low_cell_volt_cutoff + 250; // 2.5 to 3.5 volts per cell range
        
        
#ifndef HAS_HALL_SENSORS
        eepromBuffer.use_hall_sensors = 0;
#endif

        if (eepromBuffer.sine_mode_changeover_thottle_level < 5 || eepromBuffer.sine_mode_changeover_thottle_level > 25) { // sine mode changeover 5-25 percent throttle
            eepromBuffer.sine_mode_changeover_thottle_level = 5;
        }
        if (eepromBuffer.drag_brake_strength == 0 || eepromBuffer.drag_brake_strength > 10) { // drag brake 1-10
            eepromBuffer.drag_brake_strength = 10;
        }

        if (eepromBuffer.driving_brake_strength == 0 || eepromBuffer.driving_brake_strength > 9) { // motor brake 1-9
            eepromBuffer.driving_brake_strength = 10;
        }

        if(eepromBuffer.driving_brake_strength < 10){
            dead_time_override = DEAD_TIME + (150 - (eepromBuffer.driving_brake_strength * 10));
            if (dead_time_override > 200) {
                dead_time_override = 200;
            }
        min_startup_duty = min_startup_duty + dead_time_override;
        minimum_duty_cycle = minimum_duty_cycle + dead_time_override;
        throttle_max_at_low_rpm = throttle_max_at_low_rpm + dead_time_override;
        startup_max_duty_cycle = startup_max_duty_cycle + dead_time_override;
#ifdef STMICRO
        TIM1->BDTR |= dead_time_override;
#endif
#ifdef ARTERY
        TMR1->brk |= dead_time_override;
#endif
#ifdef GIGADEVICES
        TIMER_CCHP(TIMER0) |= dead_time_override;
#endif
#ifdef WCH
            TIM1->BDTR |= dead_time_override;
#endif
        }
        if (eepromBuffer.limits.temperature < 70 || eepromBuffer.limits.temperature > 140) {
            eepromBuffer.limits.temperature = 255;
        }

        if (eepromBuffer.limits.current > 0 && eepromBuffer.limits.current < 100) {
            use_current_limit = 1;
        }
        
        currentPid.Kp = eepromBuffer.current_P*2;
        currentPid.Ki = eepromBuffer.current_I;
        currentPid.Kd = eepromBuffer.current_D*2;
        
        if (eepromBuffer.sine_mode_power == 0 || eepromBuffer.sine_mode_power > 10) {
            eepromBuffer.sine_mode_power = 5;
        }

        // unsinged int cant be less than 0
        if (eepromBuffer.input_type < 10) {
            switch (eepromBuffer.input_type) {
            case AUTO_IN:
                dshot = 0;
                servoPwm = 0;
                EDT_ARMED = 1;
                break;
            case DSHOT_IN:
                dshot = 1;
                EDT_ARMED = 1;
                break;
            case SERVO_IN:
                servoPwm = 1;
                break;
            case SERIAL_IN:
                break;
            case EDTARM_IN:
                EDT_ARM_ENABLE = 1;
                EDT_ARMED = 0;
                dshot = 1;
                break;
            };
        } else {
            dshot = 0;
            servoPwm = 0;
            EDT_ARMED = 1;
        }
        
        if(eepromBuffer.max_ramp < 10){
          ramp_divider = 10;
          max_ramp_startup = eepromBuffer.max_ramp;
          max_ramp_low_rpm = eepromBuffer.max_ramp;
          max_ramp_high_rpm = eepromBuffer.max_ramp;
        }else{
          ramp_divider = 1;
          if((eepromBuffer.max_ramp / 10) < max_ramp_startup){
            max_ramp_startup = eepromBuffer.max_ramp / 10;
          }
          if((eepromBuffer.max_ramp / 10) < max_ramp_low_rpm){
            max_ramp_low_rpm = eepromBuffer.max_ramp / 10;
          }
          if((eepromBuffer.max_ramp / 10) < max_ramp_high_rpm){
            max_ramp_high_rpm = eepromBuffer.max_ramp / 10;
          }
        }
        
        if (motor_kv < 300) {
            low_rpm_throttle_limit = 0;
        }
        low_rpm_level = motor_kv / 100 / (32 / eepromBuffer.motor_poles);
        high_rpm_level = motor_kv / 12 / (32 / eepromBuffer.motor_poles);	
        m_angle_inc = 3600 / (6 * eepromBuffer.motor_poles / 2);			
    }
    reverse_speed_threshold = map(motor_kv, 300, 3000, 1000, 500);
}

void saveEEpromSettings()
{
    save_flash_nolib(eepromBuffer.buffer, sizeof(eepromBuffer.buffer), eeprom_address);
}

static void checkDeviceInfo(void)
{
// IMPORTANT NOTE:
// checkDeviceInfo serves the purpose of transmitting information of the memory size from 
// the bootloader to the main firmware, so that the correct EEPROM address can be used, and ensuring 
// compatibility across different STM32 microcontroller variants.

#define DEVINFO_MAGIC1 0x5925e3da
#define DEVINFO_MAGIC2 0x4eb863d9

    const struct devinfo {
        uint32_t magic1;
        uint32_t magic2;
        const uint8_t deviceInfo[9];
    } *devinfo = (struct devinfo *)(0x1000 - 32);

    if (devinfo->magic1 != DEVINFO_MAGIC1 ||
        devinfo->magic2 != DEVINFO_MAGIC2) {
        // bootloader does not support this feature, nothing to do
        return;
    }
    // Update eeprom_address based on the code in the bootloaders device info
    switch (devinfo->deviceInfo[4]) { 
        case 0x1f: // 32KB chips
            eeprom_address = 0x08007c00;
            break;
        case 0x35: // 64KB chips
            eeprom_address = 0x0800f800;
            break;
        case 0x2b: // 128KB chips
            eeprom_address = 0x0801f800;
            break;
    }

    // TODO: check pin code and reboot to bootloader if incorrect
}

void startMotor()
{
    if (running == 0) {
        commutate();
        commutation_interval = 10000;
        SET_INTERVAL_TIMER_COUNT(5000); // set initial timer count to half commutation interval
                                        // Serves to "pre-load" a history that didn't happen to prevent the motor from crashing immediately.
        running = 1;
    }
    enableCompInterrupts();
}

void advanceincrement()
{
    // This function advances the sinusoidal phase position by one degree and updates the      
    // three-phase PWM outputs. It's used exclusively in open-loop sine mode (stepper_sine = 1)
    // and gimbal mode — before the motor has enough BEMF for closed-loop commutation. 
    // Called by main()      

    if (!forward) {
        phase_A_position++;
        if (phase_A_position > 359) {
            phase_A_position = 0;
        }
        phase_B_position++;
        if (phase_B_position > 359) {
            phase_B_position = 0;
        }
        phase_C_position++;
        if (phase_C_position > 359) {
            phase_C_position = 0;
        }
    } else {
        phase_A_position--;
        if (phase_A_position < 0) {
            phase_A_position = 359;
        }
        phase_B_position--;
        if (phase_B_position < 0) {
            phase_B_position = 359;
        }
        phase_C_position--;
        if (phase_C_position < 0) {
            phase_C_position = 359;
        }
    }
    // Set PWM duty cycles from sine table
#ifdef GIMBAL_MODE
    setPWMCompare1(((2 * pwmSin[phase_A_position]) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
    setPWMCompare2(((2 * pwmSin[phase_B_position]) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
    setPWMCompare3(((2 * pwmSin[phase_C_position]) + gate_drive_offset) * TIMER1_MAX_ARR / 2000);
#else
    setPWMCompare1((((2 * pwmSin[phase_A_position] / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) * eepromBuffer.sine_mode_power / 10);
    setPWMCompare2((((2 * pwmSin[phase_B_position] / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) * eepromBuffer.sine_mode_power / 10);
    setPWMCompare3((((2 * pwmSin[phase_C_position] / SINE_DIVIDER) + gate_drive_offset) * TIMER1_MAX_ARR / 2000) * eepromBuffer.sine_mode_power / 10);
#endif
}

uint16_t getSmoothedCurrent()
{
    // Low pass Filter implementation with averaging last numReadings current readings
    total = total - readings[readIndex];
    readings[readIndex] = ADC_raw_current;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) {
        readIndex = 0;
    }
    smoothedcurrent = total / numReadings;
    return smoothedcurrent;
}

void getBemfState()
{
    // IMPORTANT NOTE: This functions is called once per 10kHz loop when in Pooling mode!

    uint8_t current_state = 0;
#if defined(MCU_F031) || defined(MCU_G031)
    if (step == 1 || step == 4) {
        current_state = PHASE_C_EXTI_PORT->IDR & PHASE_C_EXTI_PIN;
    }
    if (step == 2 || step == 5) { //        in phase two or 5 read from phase A Pf1
        current_state = PHASE_A_EXTI_PORT->IDR & PHASE_A_EXTI_PIN;
    }
    if (step == 3 || step == 6) { // phase B pf0
        current_state = PHASE_B_EXTI_PORT->IDR & PHASE_B_EXTI_PIN;
    }
#else
    // Read BEMF in the non-driven motor phase and check if itsurprasses a threshold 
    current_state = !getCompOutputLevel(); // polarity reversed
#endif
    // Zero crossing Validation - Ensure a sufficient number of valid BEMF counts before accepting ZC
    if (rising) {
        if (current_state) {
            bemfcounter++;
        } else {
            bad_count++;
            if (bad_count > bad_count_threshold) {
                bemfcounter = 0;
            }
        }
    } else {
        if (!current_state) {
            bemfcounter++;
        } else {
            bad_count++;
            if (bad_count > bad_count_threshold) {
                bemfcounter = 0;
            }
        }
    }
}

void zcfoundroutine()
{ 
    // IMPORTANT: Polling-mode zero-crossing handler. When the ESC is in old_routine = 1 (polling mode), this function handles 
    //            everything that happens after a BEMF zero crossing is detected — timing measurement, wait, commutation, and transition to interrupt-driven mode.
    // Performs same functionalities as interruptRoutine() and periodElapsedCallback(), but by blocking the code

    thiszctime = INTERVAL_TIMER_COUNT; // Get Zero-crossing time
    SET_INTERVAL_TIMER_COUNT(0);       // Reset timer

    // Compute time for commutation (waitTime) and 
    commutation_interval = (thiszctime + (3 * commutation_interval)) / 4; // Low Pass Filter for Commutation Interval
    advance = (temp_advance * commutation_interval) >> 6; //7.5 degree increments, alwaws equal to fixed  EEPROM-configured fixed advance
    waitTime = commutation_interval / 2 - advance;
    while ((INTERVAL_TIMER_COUNT) < (waitTime)) { // Blocking wait. It busy-waits in the main loop/ISR context until the timer reaches waitTime.
        if (zero_crosses < 5) {  // During startup (zero_crosses < 5),  skip the wait to avoid getting stuck.
            break;
        }
    }
#ifdef MCU_GDE23
    TIMER_CAR(COM_TIMER) = waitTime;
#endif
#ifdef STMICRO
    COM_TIMER->ARR = waitTime;
#endif
#ifdef MCU_AT32
		COM_TIMER->pr = waitTime;
#endif
    // Perform Commutation
    commutate();
    bemfcounter = 0;
    bad_count = 0;
    zero_crosses++;

#ifdef NO_POLLING_START     // changes to interrupt mode after 2 zero crosses, does not re-enter
       if (zero_crosses > 2) {
            old_routine = 0;
            enableCompInterrupts(); // enable interrupt
        }
#else
    if (eepromBuffer.stall_protection || eepromBuffer.rc_car_reverse) {
        if (zero_crosses >= 20 && commutation_interval <= 2000) {
            old_routine = 0;
            enableCompInterrupts(); // enable interrupt
        }
    } else {
       if (commutation_interval < POLLING_MODE_THRESHOLD) { // Switch to interrupt mode when commutation interval sufficiently small
            old_routine = 0;                                // POLLING_MODE_THRESHOLD defined in targets.h
            enableCompInterrupts(); // enable interrupt
        }
    }
 #endif
}

void commutate()
{
    if (forward == 1) {
        step++;
        if (step > 6) {
            step = 1;
            desync_check = 1;
        }
        rising = step % 2;
    } else {
        step--;
        if (step < 1) {
            step = 6;
            desync_check = 1;
        }
        rising = !(step % 2);
    }

#ifdef USE_MULTIPLE_INPUT
    // Incremment mechanical step after each eletric cycle 
    if  (step == 1) {
        m_step++;
        if (m_step > eepromBuffer.motor_poles/2) {
            m_step = 1;
        }
    }
#endif

#ifdef INVERTED_EXTI
    rising = !rising;
#endif

    // Disable interrupts to ensure commutation happens without any interruption 
    __disable_irq(); // don't let dshot interrupt
    if (!prop_brake_active) {
        comStep(step); // Update which Phase should be Low/High/Float
    }
    __enable_irq();
    changeCompInput(); // Update comparator input to use the new floating phase

#ifndef NO_POLLING_START // if NO_POLLING_START is defined, the polling mode (oldroutine = 1) is no longer executed after transition to Interruption Mode  
	if (average_interval > 2500) { // average_interval = Average Eletrical Commutation time updated in main loop.
        old_routine = 1;           // If interval is too long interval, initiate startup routine
   }
#endif

    // Reset 
    bemfcounter = 0;  // Only used in polling mode
    zcfound = 0;      // Only used in polling mode
    commutation_intervals[step - 1] = commutation_interval; // Used to update average e_com_time in runSpeedControlLoop()
    
#ifdef USE_PULSE_OUT
		if(rising){
			GPIOB->scr = GPIO_PINS_8;
		}else{
			GPIOB->clr = GPIO_PINS_8;
		}
#endif
}

void PeriodElapsedCallback()
{
    // IMPORTANT: triggered by when COM_TIMER (commutation timer) interrupt, after a calculated delay following zero-crossing detection.
    // Only used in Interruption Mode (old_routine = 0), to perform commutation at the right time after ZC detection.
    /*
    Timing diagram:

       ZC detected          waitTime expires        next ZC detected
            │                      │                      │
            ▼                      ▼                      ▼ 
      ──────┼──────────────────────┼──────────────────────┼────►
            │           (*commutation_interval*)          │     
            │◄───────────────────────────────────────────►│
            │     (waitTime)       │
            │◄────────────────────►│
            │                      │
     interruptRoutine()    PeriodElapsedCallback()
     (record timing,        (commutate motor,
      set timer)             prepare for next)

    *commutation_interval* is the time between two consecutive previous zero-crossings (ZC), projected to the next ZC
    */

    DISABLE_COM_TIMER_INT();  // Disable COM_TIMER, preventing further interrupts until next ZC is detected
    commutate();

    // Update commutation interval (smoothed average - low pass filter is used to reduce noise)
    commutation_interval = ((commutation_interval)+((lastzctime + thiszctime) >> 1))>>1;
    
#ifdef USE_MULTIPLE_INPUT
    // Update base reference angle if angle control is enabled
    // If motor allignet
    if (step == 1 && m_step == 1){
        m_angle_base = 0;
    } else {
        m_angle_base = m_angle_base + m_angle_inc;
    }
    // Wrap m_angle_base to [0, 360] interval in multiples of 0.1 deg
    if (m_angle_base > 3600) {
        m_angle_base -= 3600;
    }
    if (m_angle_base < 0) {
        m_angle_base += 3600;
    }
#endif

    // Timing advance for optimal efficiency: 60° / 64 = 0.9375us per unit of temp_advance
    if (!eepromBuffer.auto_advance) {
	  advance = (commutation_interval * temp_advance) >> 6;
	} else {
	  advance = (commutation_interval * auto_advance_level) >> 6; 
    }
    /*
      Why timing advance??
      At high RPM, there are delays from:
        - Comparator response time
        - Current rise time in windings
        - Processing delays

        Advancing commutation compensates for these delays, keeping the magnetic field optimally positioned relative to the rotor for     
        maximum torque and efficiency.

        Without advance:          With advance:

            ZC    Commutate          ZC  Commutate
            │        │               │     │
            ▼        ▼               ▼     ▼
        ────┼────────┼────►      ────┼─────┼───────►
            │◄──────►│               │◄───►│
            waitTime = interval/2    waitTime = interval/2 - advance
    */

    // Calculate wait time for next commutation
    waitTime = (commutation_interval >> 1) - advance;

    if (!old_routine) {          // Re-enable comparator for next ZC
        enableCompInterrupts();  // Only if in Interrupt Mode, Otherwise, ZC is detected by sampling, no interrupt
    }
    if (zero_crosses < 10000) {
        zero_crosses++;  // Track number of successful commutations  
    }
}

void interruptRoutine()
{
    // IMPORTANT: triggered by the comparator interrupt when the BEMF signal crosses the neutral point (zero-crossing event (rising edge)
    
    // Low Pass filter: Ensure the zero-crossing is real (not noise)
    for (int i = 0; i < filter_level; i++) {
#if defined(MCU_F031) || defined(MCU_G031)
        if (((current_GPIO_PORT->IDR & current_GPIO_PIN) == !(rising))) {   
            return;
        }   
#else
        if (getCompOutputLevel() == rising) { // Compare 
            return;
        }
#endif
    }
    __disable_irq();       // Disable interrupts to timming measurements happen without any interruption      
    maskPhaseInterrupts(); // ZC Detected -> Disable comparator interrupts until commutation is done
    
    // Record ZC timing
    lastzctime = thiszctime;           // Capture previous time between ZC
    thiszctime = INTERVAL_TIMER_COUNT; // Capture time that will happen until next ZC
    SET_INTERVAL_TIMER_COUNT(0);       // Reset interval timer
    SET_AND_ENABLE_COM_INT(waitTime+1); // Schedule commutation after waitTime -> Zero-crossing occurs ~30° electrical after the previous commutation. The motor needs to commutate ~30° after the ZC (at the 60° point).
    
    __enable_irq();
}

void setInput()
{   
    // IMPORTANT: transforms the raw input signal (newinput) into the final throttle value (input)
    // Called every tenkhzRoutine() iteration
    /*
        ┌─────────────────────────────────────────────────────────────────┐
        │                      setInput() FLOW                            │
        ├─────────────────────────────────────────────────────────────────┤
        │                                                                 │
        │  newinput (0-2000 from protocol handler)                        │
        │         │                                                       │
        │         ▼                                                       │
        │  ┌─────────────────────────────────────┐                        │
        │  │     Bi-directional mode enabled?    │                        │
        │  └─────────────────────────────────────┘                        │
        │         │YES                     │NO                            │
        │         ▼                        ▼                              │
        │  ┌──────────────────┐    adjusted_input = newinput              │
        │  │ DShot protocol?  │           │                               │
        │  └──────────────────┘           │                               │
        │    │YES        │NO              │                               │
        │    ▼           ▼                │                               │
        │  DShot      Servo PWM           │                               │
        │  bidir      bidir               │                               │
        │  mapping    mapping             │                               │
        │    │           │                │                               │
        │    ├───────────┤                │                               │
        │    │           │                │                               │
        │    ▼           ▼                │                               │
        │  Direction change safety        │                               │
        │  (speed check, mode reset)      │                               │
        │    │                            │                               │
        │    └──────────┬─────────────────┘                               │
        │               │                                                 │
        │               ▼                                                 │
        │         adjusted_input (0-2047)                                 │
        │               │                                                 │
        │               ▼                                                 │
        │  ┌─────────────────────────────────────┐                        │
        │  │     Stuck rotor detected?           │                        │
        │  └─────────────────────────────────────┘                        │
        │         │YES                     │NO                            │
        │         ▼                        ▼                              │
        │    input = 0              ┌─────────────────┐                   │
        │    allOff()               │  Mode selection │                   │
        │                           └─────────────────┘                   │
        │                             │    │ *   │   │      *Angle Ctrl   │
        │                     Fixed   │Sine│Speed│ Normal     (with Fixed │
        │                     Duty    │Mode│Ctrl │               or Variable Parameters) │
        │                       │      │    │    │                        │
        │                       ▼      ▼    ▼    ▼                        │
        │                   Constant  Map  PID  Direct                    │
        │                   value    curve output pass                    │
        │                       │      │    │    │                        │
        │                       └──────┴────┴────┘                        │
        │                              │                                  │
        │                              ▼                                  │
        │                         input (0-2047)                          │
        │                              │                                  │
        │                              ▼                                  │
        │                    (used to calculate duty_cycle_setpoint)      │
        │                                                                 │
        └─────────────────────────────────────────────────────────────────┘
        */
    
    // 1. Input Parsing 
#ifdef USE_MULTIPLE_INPUT
    
    #ifdef FIXED_SPEED_SINE_MODE
        adjusted_input_amplitude     = FIXED_SINE_MODE_AMP;
        adjusted_input_phase_offset  = FIXED_SINE_MODE_PHASE_OFFSET_DEG;
        adjusted_input = FIXED_SINE_MODE_AVERAGE;
        m_angle = m_angle_base;
    #else
    // Angle Control Mode -> Three Inputs Received: Average Throttle, Sine Amplitude and Sine Phase_offset, 
    adjusted_input = newinput;
    adjusted_input_amplitude = newinput_amplitude;
    adjusted_input_phase_offset = newinput_phase_offset;
    m_angle = m_angle_input

    #endif
#else
    // 1. newinput -> adjusted_input (correct for direction and dead band)
    if (eepromBuffer.bi_direction) { 
        // Bi-directional mode (forward/reverse)
        if (dshot == 0) {
            if (eepromBuffer.rc_car_reverse) {
                if (newinput > (1000 + (servo_dead_band << 1))) {
                    if (forward == eepromBuffer.dir_reversed) {
                        adjusted_input = 0;
                        //               if (running) {
                        prop_brake_active = 1;
                        if (return_to_center) {
                            forward = 1 - eepromBuffer.dir_reversed;
                            prop_brake_active = 0;
                            return_to_center = 0;
                        }
                    }
                    if (prop_brake_active == 0) {
                        return_to_center = 0;
                        adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
                    }
                }
                if (newinput < (1000 - (servo_dead_band << 1))) {
                    if (forward == (1 - eepromBuffer.dir_reversed)) {
                        adjusted_input = 0;
                        prop_brake_active = 1;
                        if (return_to_center) {
                            forward = eepromBuffer.dir_reversed;
                            prop_brake_active = 0;
                            return_to_center = 0;
                        }
                    }
                    if (prop_brake_active == 0) {
                        return_to_center = 0;
                        adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
                    }
                }
                if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
                    adjusted_input = 0;
                    if (prop_brake_active) {
                        prop_brake_active = 0;
                        return_to_center = 1;
                    }
                }
            } else {
                if (newinput > (1000 + (servo_dead_band << 1))) {
                    if (forward == eepromBuffer.dir_reversed) {
                        if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                            forward = 1 - eepromBuffer.dir_reversed;
                            zero_crosses = 0;
                            old_routine = 1;
                            maskPhaseInterrupts();
                            brushed_direction_set = 0;
                        } else {
                            newinput = 1000;
                        }
                    }
                    adjusted_input = map(newinput, 1000 + (servo_dead_band << 1), 2000, 47, 2047);
                }
                if (newinput < (1000 - (servo_dead_band << 1))) {
                    if (forward == (1 - eepromBuffer.dir_reversed)) {
                        if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                            zero_crosses = 0;
                            old_routine = 1;
                            forward = eepromBuffer.dir_reversed;
                            maskPhaseInterrupts();
                            brushed_direction_set = 0;
                        } else {
                            newinput = 1000;
                        }
                    }
                    adjusted_input = map(newinput, 0, 1000 - (servo_dead_band << 1), 2047, 47);
                }

                if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band << 1))) {
                    adjusted_input = 0;
                    brushed_direction_set = 0;
                }
            }
        }

        if (dshot) {
            if (newinput > 1047) {

                if (forward == eepromBuffer.dir_reversed) {
                    if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                        forward = 1 - eepromBuffer.dir_reversed;
                        zero_crosses = 0;
                        old_routine = 1;
                        maskPhaseInterrupts();
                        brushed_direction_set = 0;
                    } else {
                        newinput = 0;
                    }
                }
                adjusted_input = ((newinput - 1048) * 2 + 47) - reversing_dead_band;
            }
            if (newinput <= 1047 && newinput > 47) {
                if (forward == (1 - eepromBuffer.dir_reversed)) {
                    if (((commutation_interval > reverse_speed_threshold) && (duty_cycle < 200)) || stepper_sine) {
                        zero_crosses = 0;
                        old_routine = 1;
                        forward = eepromBuffer.dir_reversed;
                        maskPhaseInterrupts();
                        brushed_direction_set = 0;
                    } else {
                        newinput = 0;
                    }
                }
                adjusted_input = ((newinput - 48) * 2 + 47) - reversing_dead_band;
            }
            if (newinput < 48) {
                adjusted_input = 0;
                brushed_direction_set = 0;
            }
        }
    } else { 
        //Uni-directional Mode
        adjusted_input = newinput;
    }
#endif

#ifndef BRUSHED_MODE
    // 2. Stuck Rotor Protection
    if ((bemf_timeout_happened > bemf_timeout) && eepromBuffer.stuck_rotor_protection) {
        allOff();                  // Disable outputs!
        maskPhaseInterrupts();     // Disable BEMF detection
        input = 0;                 // Force input to zero -> Motor Stopped      
        bemf_timeout_happened = 102; // bemf_timeout_happened will only reset after pilot sets throttle value to zero (reseted in main() when adjusted_input == 0).
                                     // Protection is continuously enforced until reset in main()
#ifdef USE_RGB_LED
        GPIOB->BRR = LL_GPIO_PIN_8; // on red
        GPIOB->BSRR = LL_GPIO_PIN_5; //
        GPIOB->BSRR = LL_GPIO_PIN_3;
#endif
    } else {
    // 3. Process Input for varios modes
#ifdef FIXED_DUTY_MODE 
        input = FIXED_DUTY_MODE_POWER * 20 + 47;
#else
        if (eepromBuffer.use_sine_start) { // Not Compatible with Speed or Angle Control
        /*    
        ┌─────────────────────────────────────────────────────────────────┐
        │                    SINE START MODE MAPPING                      │
        ├─────────────────────────────────────────────────────────────────┤
        │  adjusted_input    0    30                changeover      2047  │
        │                    │    │                     │            │    │
        │                    ▼    ▼                     ▼            ▼    │
        │  input             0    47 ──────────────► 160 ─────────► 2047  │
        │                         │                     │            │    │
        │                         └── Sine mode range ──└── Normal ──┘    │    
        └─────────────────────────────────────────────────────────────────┘
        */
            if (adjusted_input < 30) { // dead band ?
                input = 0;
            }
            if (adjusted_input > 30 && adjusted_input < (eepromBuffer.sine_mode_changeover_thottle_level * 20)) {
                input = map(adjusted_input, 30,
                    (eepromBuffer.sine_mode_changeover_thottle_level * 20), 47, 160);
            }
            if (adjusted_input >= (eepromBuffer.sine_mode_changeover_thottle_level * 20)) {
                input = map(adjusted_input, (eepromBuffer.sine_mode_changeover_thottle_level * 20),
                    2047, 160, 2047);
            }
        } else {
            /*
            ┌─────────────────────────────────────────────────────────────────────────────┐                                                                           │                    THREE CONTROL MODES                               │
            │                    THREE CONTROL MODES                                      │          
            ├─────────────────────────────────────────────────────────────────────────────┤                                                                         
            │                                                                             │
            │   1. Normal (both off):                                                     │
            │      Throttle stick → duty cycle directly                                   │
            │      input = adjusted_input                                                 │
            │                                                                             │
            │   2. use_speed_control_loop=1, drive_by_rpm=0 (FIXED_SPEED_MODE):           │
            │      Fixed RPM target, PID controls duty cycle                              │
            │      Throttle stick is IGNORED                                              │
            │                                                                             │
            │   3. use_speed_control_loop=1, drive_by_rpm=1, use_angle_control_loop = 0   │
            │      Throttle stick → RPM target, PID controls duty cycle                   │
            │      Stick maps to a speed, not a power level                               │
            │                                                                             │
            │                                                                             │
            |   4. use_speed_control_loop=1, drive_by_rpm=1, use_angle_control_loop = 1   │
            │      Throttle stick → RPM target, PID controls duty cycle                   │
            │      Mix Other two signals, giving additional Synosoid Amplitud and phase   │
            └─────────────────────────────────────────────────────────────────────────────┘
            */
            if (use_speed_control_loop) {         // Make use of closed loop inner speed control loop
                if (drive_by_rpm) {               // Allow Variable Speed Control

                    // Average speed component for target eletrical commutation time (us)
                    target_e_com_time = 60000000 / map(adjusted_input, 47, 2047, MINIMUM_RPM_SPEED_CONTROL, MAXIMUM_RPM_SPEED_CONTROL) / (eepromBuffer.motor_poles / 2);
                    
                    if (use_angle_control_loop) { // make use of closed mechanical angle control loop

                        if (adjusted_input < minimum_angle_control_threshold || zero_crosses < 100) { // Only allow angle control for high throttle values
                            adjusted_input_amplitude = 0; 
                        }

                        // TODO: 
                        // 1 - Compute m_angle -> Should Come From Sensor!
                        // 2 - Compute target_m_angle


                        target_m_angle = m_angle_base +  INTERVAL_TIMER_COUNT * (360/target_e_com_time + adjusted_input_amplitude * sin(target_m_angle - adjusted_input_phase_offset)); // Add factor tunned_speed/average_speed
                        // INSIDE SIN, SHALL IT BE target_m_angle or m_angle?
        
                        // Sinosoidal speed component for target eletrical commutation time (us) - From angle Control Loop
                        target_e_com_time += angle_controller_output;
                    } 

                    if (adjusted_input < 47) { // dead band ?
                        input = 0;
                        speedPid.error = 0;
                        input_override = 0;
                    } else {
                        input = (uint16_t)(input_override / 10000); // speed control pid override
                        if (input > 2047) {
                            input = 2047;
                        }
                        if (input < 48) {
                            input = 48;
                        }
                    }
                } else {
                    // Fixed RPM target, PID controls duty cycle
                    input = (uint16_t)(input_override / 10000); // speed control pid override
                    if (input > 2047) {
                        input = 2047;
                    }
                    if (input < 48) {
                        input = 48;
                    }
                }
            } else {
                // Normal Direct Control Mode
                input = adjusted_input;
            }
        }
#endif
    }
#endif
#ifndef BRUSHED_MODE
    if (!stepper_sine && armed) {  
        // 4. Input over Minimum Threshold (Motor Should Spin with Target duty_cycle_setpoint)
        if (input >= 47 + (80 * eepromBuffer.use_sine_start)) { 
            if (running == 0) {  // Motor StartUp Logic                          
                allOff();
                if (!old_routine) {
                    startMotor();
                }
                running = 1;
                last_duty_cycle = min_startup_duty;
            }
            // Target Duty Cycle Setpoint Calculation
            if (eepromBuffer.use_sine_start) {
                duty_cycle_setpoint = map(input, 137, 2047, minimum_duty_cycle+40, 2000);
            } else {
                duty_cycle_setpoint = map(input, 47, 2047, minimum_duty_cycle, 2000);
            }

            // Disable Prop Brake
            if (!eepromBuffer.rc_car_reverse) {
                prop_brake_active = 0;
            }
        }

        // 5. Input Below Threshold (Motor Should Stop/Idle)
        if (input < 47 + (80 * eepromBuffer.use_sine_start)) {
            
            if (play_tone_flag != 0) {// Play Pending Tones
                switch (play_tone_flag) {
                                    
                case 1:
                    playDefaultTone();
                    break;
                case 2:
                    playChangedTone();
                    break;
                case 3:
                    playBeaconTune3();
                    break;
                case 4:
                    playInputTune2();
                    break;
                case 5:
                    playDefaultTone();
                    break;
                }
                play_tone_flag = 0;
            }

            if (!eepromBuffer.comp_pwm) {
                duty_cycle_setpoint = 0;
                if (!running) {
                    old_routine = 1;
                    zero_crosses = 0;
                    if (eepromBuffer.brake_on_stop) {
                        fullBrake();
                    } else {
                        if (!prop_brake_active) {
                            allOff();
                        }
                    }
                }
                if (eepromBuffer.rc_car_reverse && prop_brake_active) {
    #ifndef PWM_ENABLE_BRIDGE
                    prop_brake_duty_cycle = (getAbsDif(1000, newinput) + 1000);
                    if (prop_brake_duty_cycle >= (1999)) {
                        fullBrake();
                    } else {
                        proportionalBrake();
                    }
    #endif
                }
            } else {
                if (!running) {

                    old_routine = 1;
                    zero_crosses = 0;
                    bad_count = 0;
                    if (eepromBuffer.brake_on_stop > 0) {
                        if (!eepromBuffer.use_sine_start) {
    #ifndef PWM_ENABLE_BRIDGE
                            if(eepromBuffer.brake_on_stop == 1){
                                prop_brake_duty_cycle =  eepromBuffer.drag_brake_strength * 200;
                                if (prop_brake_duty_cycle >= (1999)) {
                                fullBrake();
                                } else {
                                proportionalBrake();
                                prop_brake_active = 1;
                                }
                            }
    #else
                            // todo add proportional braking for pwm/enable style bridge.
    #endif
                        }
                    } else {
                        allOff();
                    }
                    duty_cycle_setpoint = 0;
                }

                phase_A_position = ((step - 1) * 60) + enter_sine_angle;
                if (phase_A_position > 359) {
                    phase_A_position -= 360;
                }
                phase_B_position = phase_A_position + 119;
                if (phase_B_position > 359) {
                    phase_B_position -= 360;
                }
                phase_C_position = phase_A_position + 239;
                if (phase_C_position > 359) {
                    phase_C_position -= 360;
                }

                if (eepromBuffer.use_sine_start == 1) {
                    stepper_sine = 1;
                }
                duty_cycle_setpoint = 0;
            }
        }

        // 6. Apply Constraints
        if (!prop_brake_active) {
            // Startup Constraints
            if (input >= 47 && (zero_crosses < (uint32_t)(30 >> eepromBuffer.stall_protection))) {
                if (duty_cycle_setpoint < min_startup_duty) {
                    duty_cycle_setpoint = min_startup_duty;
                }
                if (duty_cycle_setpoint > startup_max_duty_cycle) {
                    duty_cycle_setpoint = startup_max_duty_cycle;
                }
            }
            // Duty Cycle Constraints
            if (duty_cycle_setpoint > duty_cycle_maximum) {
                duty_cycle_setpoint = duty_cycle_maximum;
            }
            // Current Limit Constraints
            if (use_current_limit) {
                if (duty_cycle_setpoint > use_current_limit_adjust) {
                    duty_cycle_setpoint = use_current_limit_adjust;
                }
            }
            // Stall Protection Constraints
            if (stall_protection_adjust > 0 && input > 47) {
                duty_cycle_setpoint = duty_cycle_setpoint + (uint16_t)(stall_protection_adjust/10000);
            }
        }
    }
#endif
}

void tenKhzRoutine()
{ 
    // IMPORTANT: heart of the ESC's real-time control system. 
    // Triggered by timmer interrupt defined in the peripheral setup
    // Despite the name, runs at 20khz (as of 2.00 to be renamed)
    /* Summary table:  
      
    ┌──────────────────────┬──────────────────────┬────────────────────────────────────────────┐
    │         Task         │      Frequency       │                  Purpose                   │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ Input processing     │ 20kHz                │ Read and process throttle signal           │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ BEMF polling         │ 20kHz                │ Zero-cross detection in polling mode       │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ Duty cycle ramping   │ 20kHz / ramp_divider │ Smooth throttle response                   │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ PWM update           │ 20kHz                │ Set motor duty cycle                       │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ Current limit PID    │ 1kHz                 │ Protect motor/ESC from overcurrent         │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ Stall protection PID │ 1kHz                 │ Boost power at low RPM (crawlers)          │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ Speed control PID    │ 1kHz                 │ RPM-based throttle control                 │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ ADC trigger          │ 1kHz                 │ Start temperature/voltage/current readings │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ Telemetry            │ ~30ms                │ Send telemetry data                        │
    ├──────────────────────┼──────────────────────┼────────────────────────────────────────────┤
    │ Arming               │ 1 second             │ Wait for valid zero input                  │
    └──────────────────────┴──────────────────────┴────────────────────────────────────────────┘
    
    Implemented Control Loop Structure:

                    ┌─────────────────────────────────────────────────────────┐
                    │                    4kHz LOOP                            │
                    │  ┌──────────────┐     ┌──────────────┐                  │
   target_angle ───>│  │  Angle PID   │────>│ Speed Target │                  │
   (from ref gen)   │  │   anglePid   │     │ Calculation  │                  │
                    │  └──────────────┘     └──────────────┘                  │
                    │         ↑                    │                          │
                    │  current_mec_angle           │ target_e_com_time        │
                    │  (from commutate)            ↓                          │
                    └─────────────────────────────────────────────────────────┘
                                                   │
                    ┌──────────────────────────────────────────────────────────┐
                    │                    1kHz LOOP                             │
                    │         ↓                                                │
                    │  ┌──────────────┐     ┌──────────────┐                   │
                    │  │  Speed PID   │────>│input_override│                   │
                    │  │   speedPid   │     │              │                   │
                    │  └──────────────┘     └──────────────┘                   │
                    │         ↑                    │                           │
                    │    e_com_time                │                           │
                    │    (actual speed)            ↓                           │
                    └──────────────────────────────────────────────────────────┘
                                                   │
                    ┌──────────────────────────────────────────────────────────┐
                    │                   20kHz LOOP                             │
                    │         ↓                                                │
                    │  ┌──────────────┐     ┌──────────────┐     ┌──────────┐  │
                    │  │  setInput()  │────>│ Ramp Limiter │────>│   PWM    │  │
                    │  │              │     │              │     │          │  │
                    │  └──────────────┘     └──────────────┘     └──────────┘  │
                    └──────────────────────────────────────────────────────────┘
    */

    duty_cycle = duty_cycle_setpoint; // copy duty_cycle_setpoint to duty_cycle, and transform it rate limiting logic
    tenkhzcounter++;  
    one_khz_loop_counter++;
    ledcounter++;
    

    // 1. Arming Logic:
    // safety mechanism to prevents motor from spinning until the user has demonstrated a valid zero-throttle condition.
    // NOTE: The arming process resets if:
    //    - Non-zero input received
    //    - Timeout without enough zero readings
    //    - Low voltage cutoff triggered, preventing motor from rearming until power cycle
    if (!armed) {              // Check if motor is already armed. If so, skip arming logic.
        if (cell_count == 0) { // Check if Battery cell count is not yet determined (first boot) -> Arm once at boot, stay armed until power cycle or critical fault.
            if (inputSet) {    // Check if a valid input signal has been received from receiver/FC
                if (adjusted_input == 0) { //Check if Throttle stick is at zero position
                    armed_timeout_count++;
                    if (armed_timeout_count > LOOP_FREQUENCY_HZ) {  // Ensure Zero throttle held for ~1 second (20,000 loop iterations at 20kHz)
                        if (zero_input_count > 30) { // Check at least 30 consecutive zero-input readings confirmed
                                                      
                            //Procced with arm sequence
                            armed = 1;
#ifdef USE_LED_STRIP
                            //	send_LED_RGB(0,0,0);
                            delayMicros(1000);
                            send_LED_RGB(0, 255, 0);
#endif
#ifdef USE_RGB_LED
                            GPIOB->BRR = LL_GPIO_PIN_3; // turn on green
                            GPIOB->BSRR = LL_GPIO_PIN_8; // turn on green
                            GPIOB->BSRR = LL_GPIO_PIN_5;
#endif  
                            // Battery cell count is detected from voltage (battery_voltage / 370 = ~3.7V per cell)
                            if ((cell_count == 0) && eepromBuffer.low_voltage_cut_off == 1) { 
                                cell_count = battery_voltage / 370;
                                
                                // Startup tune plays (once per cell if low voltage cutoff enabled)
                                for (int i = 0; i < cell_count; i++) {
                                    playInputTune();
                                    delayMillis(100);
                                    RELOAD_WATCHDOG_COUNTER();
                                }
                            } else {
#ifdef MCU_AT415
								play_tone_flag = 4;
#else
								playInputTune();
#endif
                            }
                            if (!servoPwm) { // If input protocol is DShot/serial, reverse is controlled by the receiver
                                eepromBuffer.rc_car_reverse = 0;
                            }

                            if (use_angle_control_loop)

                        } else {
                            inputSet = 0;
                            armed_timeout_count = 0;
                        }
                    }
                } else {
                    armed_timeout_count = 0;
                }
            }
        }
    }

    // 2. For Serial Telemetry Send Timming (DShot Telemetry works on Demand!)
    // - telem_ms_count increments every 20kHz loop iteration (every 50µs)
    // - Default telemetry_interval_ms = 30 → triggers every ~30ms
    // - eepromBuffer.telemetry_on_interval adds slight offset per ESC to avoid collisions when multiple ESCs share one telemetry wire~
    //
    //  Interval = (30 - 1 + ESC_ID) × 20 = ~600 ticks at 20kHz ≈ 30ms
    //  Telemetry is sent in main when send_telemetry flag is set 1.    
    if (eepromBuffer.telemetry_on_interval) {
        telem_ms_count++;
        if (telem_ms_count > ((telemetry_interval_ms - 1 + eepromBuffer.telemetry_on_interval) * 20)) {
            send_telemetry = 1;
            telem_ms_count = 0;
        }
    }

#ifndef BRUSHED_MODE
    // 3. Process Input (only when NOT in sine startup mode -> The main loop handles sine mode input separately.)
    if (!stepper_sine) { // In sine mode, the motor is in open-loop control - input directly controls stepping speed, not throttle response, and so Sine mode does not rely on BEMF sensing!. 
        setInput();      
            
        // 3.1. BEMF polling (for old routine mode only)
#ifndef CUSTOM_RAMP
        if (old_routine && running) {
	        //send_LED_RGB(255, 0, 0);
            maskPhaseInterrupts();  // Disable comparator interrupts
            getBemfState();         // Poll comparator for BEMF state
            if (!zcfound) {
                if (rising) {
                    if (bemfcounter > min_bemf_counts_up) {
                        zcfound = 1;
                        zcfoundroutine(); // Handle zero-crossing in Polling mode
                    }
                } else {
                    if (bemfcounter > min_bemf_counts_down) {
                        zcfound = 1;
                        zcfoundroutine(); // Handle zero-crossing in Polling mode
                    }
                }
            }
        }
#endif
        // 3.2 PID Loops (at PID_LOOP_DIVIDER (~1kHz)) - (Effects will happen in next iteratioon of SetInput):  
        // 1. Angle Control PID 
        // 2. Speed Control PID
        // 3. Current Limit PID  -> Limit ESC by limiting maximum 
        // 4. Stall Protection PID
        //All loops take effect directly over the duty_cycle variable.        
        if (one_khz_loop_counter > PID_LOOP_DIVIDER) { // 1khz PID loop
            one_khz_loop_counter = 0;
            PROCESS_ADC_FLAG = 1; // set flag to do new adc read at lower priority
            
            if (use_current_limit && running) {
                use_current_limit_adjust -= (int16_t)(doPidCalculations(&currentPid, actual_current,
                                                          eepromBuffer.limits.current * 2 * 100) / 10000);

                // Clamp to valid range
                if (use_current_limit_adjust < minimum_duty_cycle) {
                    use_current_limit_adjust = minimum_duty_cycle;
                }
                if (use_current_limit_adjust > 2000) {
                    use_current_limit_adjust = 2000;
                }
            }
            if (eepromBuffer.stall_protection && running) { // this boosts throttle as the rpm gets lower, for crawlers
                                               // and rc cars only, do not use for multirotors.
                stall_protection_adjust += (doPidCalculations(&stallPid, commutation_interval,
                                               stall_protect_target_interval));
                if (stall_protection_adjust > 150 * 10000) {
                    stall_protection_adjust = 150 * 10000;
                }
                if (stall_protection_adjust <= 0) {
                    stall_protection_adjust = 0;
                }
            }
            
            if (use_speed_control_loop && running) { 
                input_override += doPidCalculations(&speedPid, e_com_time, target_e_com_time);
                if (input_override > 2047 * 10000) {
                    input_override = 2047 * 10000;
                }
                if (input_override < 0) {
                    input_override = 0;
                }
                if (zero_crosses < 100) {
                    speedPid.integral = 0;
                }
            }
            
            if (use_angle_control_loop && running && zero_crosses > 100) { // Ensure motor is running smoothly
                angle_controller_output += doPidCalculations(&anglePid, m_angle, target_m_angle);
                
                if (angle_controller_output > MAXIMUM_RPM_SPEED_CONTROL) {
                    angle_controller_output = MAXIMUM_RPM_SPEED_CONTROL;
                }
                if (angle_controller_output < - MAXIMUM_RPM_SPEED_CONTROL) {
                    angle_controller_output = - MAXIMUM_RPM_SPEED_CONTROL;
                }
                if (zero_crosses < 100) {
                    anglePid.integral = 0; // Reset integral
                }
            }
        }
        
        // 3.3. Duty cycle Ramp Limiter: prevents abrupt duty cycle changes, ensuring smooth transitions from the current duty cycle (last_duty_cycle)
        //                               to the target duty cycle (duty_cycle_setpoint) over a period of time. 
        /*
        ┌─────────────────────────────────────────────────────────────────┐
        │                  DUTY CYCLE RAMPING FLOW                        │
        ├─────────────────────────────────────────────────────────────────┤
        │                                                                 │
        │  duty_cycle_setpoint ◄── From setInput() with all constraints   │
        │         │                 (current limit, temp limit, etc.)     │
        │         │                                                       │
        │         ▼                                                       │
        │  ┌─────────────────────────────────────────┐                    │
        │  │   duty_cycle = duty_cycle_setpoint      │                    │
        │  └─────────────────────────────────────────┘                    │
        │         │                                                       │
        │         ▼                                                       │
        │  ┌─────────────────────────────────────────┐                    │
        │  │   tenkhzcounter % ramp_divider == 0?    │                    │
        │  └─────────────────────────────────────────┘                    │
        │         │YES                        │NO                         │
        │         ▼                           ▼                           │
        │  ┌──────────────────┐    ┌──────────────────────┐               │
        │  │ Select ramp rate │    │ duty_cycle =         │               │
        │  │ based on:        │    │   last_duty_cycle    │               │
        │  │ - Startup?       │    │ (hold previous)      │               │
        │  │ - Low RPM?       │    └──────────────────────┘               │
        │  │ - High RPM?      │               │                           │
        │  └────────┬─────────┘               │                           │
        │           │                         │                           │
        │           ▼                         │                           │
        │  ┌──────────────────────────┐       │                           │
        │  │ Rate limit duty_cycle:   │       │                           │
        │  │                          │       │                           │
        │  │ if increasing too fast:  │       │                           │
        │  │   duty_cycle = last +    │       │                           │
        │  │     max_duty_cycle_change│       │                           │
        │  │                          │       │                           │
        │  │ if decreasing too fast:  │       │                           │
        │  │   duty_cycle = last -    │       │                           │
        │  │     max_duty_cycle_change│       │                           │
        │  └────────┬─────────────────┘       │                           │
        │           │                         │                           │
        │           └────────────┬────────────┘                           │
        │                        │                                        │
        │                        ▼                                        │
        │  ┌─────────────────────────────────────────┐                    │
        │  │ last_duty_cycle = duty_cycle            │                    │
        │  │ (save for next iteration)               │                    │
        │  └─────────────────────────────────────────┘                    │
        │                        │                                        │
        │                        ▼                                        │
        │  ┌─────────────────────────────────────────┐                    │
        │  │ adjusted_duty_cycle =                   │                    │
        │  │   (duty_cycle × tim1_arr) / 2000 + 1    │                    │
        │  └─────────────────────────────────────────┘                    │
        │                        │                                        │
        │                        ▼                                        │
        │  ┌─────────────────────────────────────────┐                    │
        │  │ SET_DUTY_CYCLE_ALL(adjusted_duty_cycle) │                    │
        │  │ → TIM1->CCR1, CCR2, CCR3                │                    │
        │  └─────────────────────────────────────────┘                    │
        │                        │                                        │
        │                        ▼                                        │
        │                      PWM                                        │
        │                                                                 │
        └─────────────────────────────────────────────────────────────────┘
        */
        if (tenkhzcounter % ramp_divider == 0) { // Normal Operation, ramp divider = 1 -> Duty cycle updated every loop
#ifdef VOLTAGE_BASED_RAMP 
            // Voltage-Based Ramp
            uint16_t voltage_based_max_change = map(battery_voltage, 800, 2200, 10, 1);
            if (average_interval > 200) {
                max_duty_cycle_change = voltage_based_max_change;
            } else {
                max_duty_cycle_change = voltage_based_max_change * 3;
            }
#else 
            // RPM-Based Ramp (Default)
            if (zero_crosses < 150 || last_duty_cycle < 150) {  // Startup phase or  
                max_duty_cycle_change = max_ramp_startup;
            } else {
                if (average_interval > 500) {  // Low RPM
                    max_duty_cycle_change = max_ramp_low_rpm;
                } else { // High RPM
                    max_duty_cycle_change = max_ramp_high_rpm;
                }
            }
#endif
#ifdef CUSTOM_RAMP
   //         max_duty_cycle_change = eepromBuffer[30];
#endif      
            // Clamp duty-cycle change to max allowed change
            if ((duty_cycle - last_duty_cycle) > max_duty_cycle_change) {
                duty_cycle = last_duty_cycle + max_duty_cycle_change;

            }
            if ((last_duty_cycle - duty_cycle) > max_duty_cycle_change) {
                duty_cycle = last_duty_cycle - max_duty_cycle_change;
            }
            }else{
             duty_cycle = last_duty_cycle;
            }
        
        // 3.4 Output PWM update:  scaling conversion from an internal representation to actual PWM timer counts (HARDWARE DEPENDENT!)
        if ((armed && running) && input > 47) {
            // if (eepromBuffer.variable_pwm) {} TO BE IMPLEMENTED
            adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000) + 1;
        } else {
            if (prop_brake_active) {
              adjusted_duty_cycle =  tim1_arr - ((prop_brake_duty_cycle * tim1_arr) / 2000);
            } else {
              if((eepromBuffer.brake_on_stop == 2) && armed){  // require arming for active brake
                comStep(2);
                adjusted_duty_cycle = DEAD_TIME + ((eepromBuffer.active_brake_power * tim1_arr) / 2000)* 10;
                }else{
                    adjusted_duty_cycle = ((duty_cycle * tim1_arr) / 2000);
                }
            }
        }
        
        last_duty_cycle = duty_cycle;            // Save for next iteration
        SET_AUTO_RELOAD_PWM(tim1_arr);           // Reset PWM Frequency
        SET_DUTY_CYCLE_ALL(adjusted_duty_cycle); // Sets the compare value for all 3 PWM channels simultaneously
                                                 // NOTE: The comStep() function controls which phases are High (A, B, C) so in the end only one phase will inject power at adjusted_duty_cycle frequency
    }
#endif // def brushed_mode
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
    if (getInputPinState()) {
        signaltimeout++;
        if (signaltimeout > LOOP_FREQUENCY_HZ) {
            NVIC_SystemReset();
        }
    } else {
        signaltimeout = 0;
    }
    // Special Case: In fixed modes, the timeout is based on the input pin state rather than protocol decoding - if pin stays HIGH for 1 second, reset.
#else
    signaltimeout++; // The signaltimeout variable is a failsafe watchdog that detects loss of the input signal and triggers a safety shutdown in main().
                     // Reseted to 0 when valied input is received.      
#endif
}

#ifdef BRUSHED_MODE
void runBrushedLoop()
{

    uint16_t brushed_duty_cycle = 0;

    if (brushed_direction_set == 0 && adjusted_input > 48) {
        if (forward) {
            allOff();
            delayMicros(10);
            twoChannelForward();
        } else {
            allOff();
            delayMicros(10);
            twoChannelReverse();
        }
        brushed_direction_set = 1;
    }

    brushed_duty_cycle = map(adjusted_input, 48, 2047, 0,
        (TIMER1_MAX_ARR - (TIMER1_MAX_ARR / 20)));

    if (degrees_celsius > eepromBuffer.limits.temperature) {
        duty_cycle_maximum = map(degrees_celsius, eepromBuffer.limits.temperature,
            eepromBuffer.limits.temperature + 20, TIMER1_MAX_ARR / 2, 1);
    } else {
        duty_cycle_maximum = TIMER1_MAX_ARR - 50;
    }
    if (brushed_duty_cycle > duty_cycle_maximum) {
        brushed_duty_cycle = duty_cycle_maximum;
    }

    if (use_current_limit) {
        use_current_limit_adjust -= (int16_t)(doPidCalculations(&currentPid, actual_current,
                                                  CURRENT_LIMIT * 100)
            / 10000);
        if (use_current_limit_adjust < minimum_duty_cycle) {
            use_current_limit_adjust = minimum_duty_cycle;
        }

        if (brushed_duty_cycle > use_current_limit_adjust) {
            brushed_duty_cycle = use_current_limit_adjust;
        }
    }
    if ((brushed_duty_cycle > 0) && armed) {
        SET_DUTY_CYCLE_ALL(brushed_duty_cycle);
        //	  	TIM1->CCR1 = brushed_duty_cycle;
        //		TIM1->CCR2 = brushed_duty_cycle;
        //		TIM1->CCR3 = brushed_duty_cycle;

    } else {
        SET_DUTY_CYCLE_ALL(0);
        //		TIM1->CCR1 = 0;
        //// 		TIM1->CCR2 = 0; 		TIM1->CCR3 = 0;
        brushed_direction_set = 0;
    }
}
#endif

void processDshot()
{
    if (compute_dshot_flag == 1) {
        computeDshotDMA();
        compute_dshot_flag = 0;
    }
    if (compute_dshot_flag == 2) {
        make_dshot_package(e_com_time);
        compute_dshot_flag = 0;
        return;
    }
    setInput();
}

int main(void)
{
#ifdef USE_MULTIPLE_INPUT
    use_angle_control_loop = 1;
    use_speed_control_loop = 1;
    drive_by_rpm = 1;
#else
    use_angle_control_loop = 0;
#endif



    // 1. Initialization
    initAfterJump();          // MCU-specific post-reset setup
    checkDeviceInfo();        // Validate bootloader device info
    initCorePeripherals();    // Setup timers, ADC, comparator
    enableCorePeripherals();  // Start peripherals 
    loadEEpromSettings();     // Load configuration from flash

    
    if (VERSION_MAJOR != eepromBuffer.version.major || VERSION_MINOR != eepromBuffer.version.minor || EEPROM_VERSION > eepromBuffer.eeprom_version) { // Epprom version Check and Update
        eepromBuffer.version.major = VERSION_MAJOR;
        eepromBuffer.version.minor = VERSION_MINOR;
        eepromBuffer.eeprom_version = EEPROM_VERSION;
        saveEEpromSettings();
    }
    
    if (eepromBuffer.dir_reversed == 1) { // Set forward/reverse from EEPROM
        forward = 0;
    } else {
        forward = 1;
    }

    tim1_arr = TIMER1_MAX_ARR;

    if (!eepromBuffer.comp_pwm) {
        eepromBuffer.use_sine_start = 0; // sine start requires complementary pwm.
    }

    if (eepromBuffer.rc_car_reverse) { // overrides a whole lot of things!
        throttle_max_at_low_rpm = 1000;
        eepromBuffer.bi_direction = 1;
        eepromBuffer.use_sine_start = 0;
        low_rpm_throttle_limit = 1;
        eepromBuffer.variable_pwm = 0;
        // eepromBuffer.stall_protection = 1;
        eepromBuffer.comp_pwm = 0;
        eepromBuffer.stuck_rotor_protection = 0;
        minimum_duty_cycle = minimum_duty_cycle + 50;
        stall_protect_minimum_duty = stall_protect_minimum_duty + 50;
        min_startup_duty = min_startup_duty + 50;
    }

#ifdef MCU_F031
    GPIOF->BSRR = LL_GPIO_PIN_6; // uncomment to take bridge out of standby mode
                                 // and set oc level
    GPIOF->BRR = LL_GPIO_PIN_7; // out of standby mode
    GPIOA->BRR = LL_GPIO_PIN_11;
#endif
#ifdef MCU_G031
    GPIOA->BRR = LL_GPIO_PIN_11;
    GPIOA->BSRR = LL_GPIO_PIN_12;    // Pa12 attached to enable on dev board
#endif
#ifdef USE_LED_STRIP
    send_LED_RGB(125, 0, 0);
#endif

#ifdef USE_CRSF_INPUT
    inputSet = 1;
    playStartupTune();
    MX_IWDG_Init();
    LL_IWDG_ReloadCounter(IWDG);
#else
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
    MX_IWDG_Init();
    RELOAD_WATCHDOG_COUNTER();
    inputSet = 1;
    armed = 1;
    adjusted_input = 48;
    newinput = 48;
	comStep(2);
#ifdef FIXED_SPEED_MODE
    use_speed_control_loop = 1;
    eepromBuffer.use_sine_start = 0;
    target_e_com_time = 60000000 / FIXED_SPEED_MODE_RPM / (eepromBuffer.motor_poles / 2);
    input = 48;
#endif

#else
#ifdef BRUSHED_MODE
    // bi_direction = 1;
    commutation_interval = 5000;
    eepromBuffer.use_sine_start = 0;
    maskPhaseInterrupts();
    playBrushedStartupTune();
#else
 #ifdef MCU_AT415
    play_tone_flag = 5;
 #else
    playStartupTune();
	#endif
#endif
    zero_input_count = 0;
    MX_IWDG_Init();
    RELOAD_WATCHDOG_COUNTER();
#ifdef GIMBAL_MODE
    eepromBuffer.bi_direction = 1;
    eepromBuffer.use_sine_start = 1;
#endif

#ifdef USE_ADC_INPUT
    armed_count_threshold = 5000;
    inputSet = 1;

#else
    // checkForHighSignal();     // will reboot if signal line is high for 10ms
    receiveDshotDma();
    if (drive_by_rpm) {
        use_speed_control_loop = 1;
    }
#endif

#endif // end fixed duty mode ifdef
#endif // end crsf input

#ifdef MCU_F051
    MCU_Id = DBGMCU->IDCODE &= 0xFFF;
    REV_Id = DBGMCU->IDCODE >> 16;

    if (REV_Id >= 4096) {
        temperature_offset = 0;
    } else {
        temperature_offset = 230;
    }

#endif
#ifdef NEUTRONRC_G071
    setInputPullDown();
#else
    setInputPullUp();
#endif

#ifdef USE_STARTUP_BOOST
  min_startup_duty = min_startup_duty + 200 + ((eepromBuffer.pwm_frequency * 100)/24);
  minimum_duty_cycle = minimum_duty_cycle + 50 + ((eepromBuffer.pwm_frequency * 50 )/24);
  startup_max_duty_cycle = startup_max_duty_cycle + 400;
#endif

    while (1) {
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE) || defined(FIXED_SPEED_SINE_MODE)
    // Keep Motor Spinning From the first moment rather than wait for first timer interrupt  
    // Speed Is Constant so no problem in calling here and in tenKhzRoutine()
    setInput();
#endif

#ifdef NEED_INPUT_READY 
    // Active for F031 and F051 only, which have resource-constrained processors
 #ifdef MCU_F031
        if (input_ready) {
            setInput(); 
            input_ready = 0;
        }
#else
        if (input_ready) {
            processDshot();
            input_ready = 0;
        }
#endif
#endif
        // 2. Increcrease BEMF count threshould for low speeds (BEMF measure less reliable) 
        if(zero_crosses < 5){
            min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS * 2;
            min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS * 2;
        }else{
            min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;
            min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;
        }

        RELOAD_WATCHDOG_COUNTER(); // Resets the Independent Watchdog Timer (IWDG) to prevent a MCU reset
                                    // Watchdog prevents the firmware from getting stucked


        // 3. Adjusts the PWM frequency dynamically based on motor speed to improve efficiency (Determined by tim1_arr: PWM Frequency = CPU_FREQUENCY / (tim1_arr + 1))
        if (eepromBuffer.variable_pwm == 1) {      // uses range defined by pwm frequency setting
            // At high RPM, higher PWM frequency gives smoother current waveforms. At low RPM, lower frequency reduces switching losses
            tim1_arr = map(commutation_interval, 96, 200, TIMER1_MAX_ARR / 2, TIMER1_MAX_ARR);
        }

        if (eepromBuffer.variable_pwm == 2) {      // uses automatic range   
            // The PWM frequency automatically tracks motor speed, maintaining a consistent number of PWM cycles per electrical step, reduncing noise
            if(average_interval < 250 && average_interval > 100){
            tim1_arr = average_interval * (CPU_FREQUENCY_MHZ/9);
            }
            if(average_interval < 100 && average_interval > 0){
                tim1_arr = 100 * (CPU_FREQUENCY_MHZ/9);
            }
            if((average_interval >= 250) || (average_interval == 0)){
                tim1_arr = 250 * (CPU_FREQUENCY_MHZ/9);
            } 
        }

        // 4. Signal Timeout Handling: resets the ESC if the input signal is lost
        if (signaltimeout > (LOOP_FREQUENCY_HZ >> 1)) { // half second timeout when armed;
            if (armed) {
                allOff();                 // Turn off all motor phases
                armed = 0;                // Disarm
                input = 0;                // Clear input
                inputSet = 0;             // Mark input as not set
                zero_input_count = 0;     // Zero PWM duty cycle
                SET_DUTY_CYCLE_ALL(0);    // Zero PWM duty cycle
                resetInputCaptureTimer(); // Reset input capture peripheral
                for (int i = 0; i < 64; i++) {
                    dma_buffer[i] = 0;    // Clear DMA buffer (DShot data)
                }
                NVIC_SystemReset();       // MCU reset
            }
            if (signaltimeout > LOOP_FREQUENCY_HZ << 1) { // 2 second when not armed
                allOff();
                armed = 0;
                input = 0;
                inputSet = 0;
                zero_input_count = 0;
                SET_DUTY_CYCLE_ALL(0);
                resetInputCaptureTimer();
                for (int i = 0; i < 64; i++) {
                    dma_buffer[i] = 0;
                }
                NVIC_SystemReset();
            }
        }
#ifdef USE_CUSTOM_LED
        if ((input >= 47) && (input < 1947)) {
            if (ledcounter > (2000 >> forward)) {
                GPIOB->BSRR = LL_GPIO_PIN_3;
            } else {
                GPIOB->BRR = LL_GPIO_PIN_3;
            }
            if (ledcounter > (4000 >> forward)) {
                ledcounter = 0;
            }
        }
        if (input > 1947) {
            GPIOB->BSRR = LL_GPIO_PIN_3;
        }
        if (input < 47) {
            GPIOB->BRR = LL_GPIO_PIN_3;
        }
#endif

        // 5. Low Priority 1Hz Routine. tenkhzcounter incremented in tenKhzRoutine() at 20kHz
        if (tenkhzcounter > LOOP_FREQUENCY_HZ) { 
            // Current Measurement
            consumed_current += (actual_current << 16) / 360; // in mAh units
            // Extended D-shot Telemetry (Embedded into Dshot Frame Response)
            switch (dshot_extended_telemetry) {
                case 1:
                    send_extended_dshot = 0b0010 << 8 | degrees_celsius;
                    dshot_extended_telemetry = 2;
                    break;
                case 2:
                    send_extended_dshot = 0b0110 << 8 | (uint8_t)actual_current / 50;
                    dshot_extended_telemetry = 3;
                    break;
                case 3:
                    send_extended_dshot = 0b0100 << 8 | (uint8_t)(battery_voltage / 25);
                    dshot_extended_telemetry = 1;
                    break;
            }

            // Reset 1Hz Counter
            tenkhzcounter = 0;
        }

#ifndef BRUSHED_MODE
        // 6. BEMF Threshold update  
        if ((zero_crosses > 1000) || (adjusted_input == 0)) { // Mortor running well or Zero Input
            bemf_timeout_happened = 0;
        }
        if (zero_crosses > 100 && adjusted_input < 200) {  // Motor at low throttle
            bemf_timeout_happened = 0;
        }
        if (eepromBuffer.use_sine_start && adjusted_input < 160) { // Motor driven by open loop sine startup
            bemf_timeout_happened = 0;
        }
        if (crawler_mode) {
            if (adjusted_input < 400) {
                bemf_timeout_happened = 0;
            }
        } else {
            // Timeout Threshold Adjustment based on throttle setting -> low voltage on phases = weak BEMF signal, Need more time to detect zero crossings 
            if (adjusted_input < 150) { // startup duty cycle should be low enough to not burn motor. bemf_timeout used in stuck rotor protection section in main()
                bemf_timeout = 100;
            } else {
                bemf_timeout = 10;
            }
        }
#endif

        //  7. Desync Detection and Recovery (when ESC commutation timing is out of sync with actual rotor position)
        //  If Dessync Detected, the Motor restarts on the very next call to setInput(), which happens from tenKhzRoutine() at 20kHz        
        if (desync_check && zero_crosses > 10) {
            
            if ((getAbsDif(last_average_interval, average_interval) > average_interval >> 1) && (average_interval < 2000)) { // throttle resitricted before zc 20.
                // Desync detected is detected by checking if difference between average_interval and last_average_interval is greater than half of average_interval
                zero_crosses = 0;
                desync_happened++;
                if ((!eepromBuffer.bi_direction && (input > 47)) || commutation_interval > 1000) {
                    running = 0;
                }
                if (zero_crosses > 100) {
                    average_interval = 5000;
                }
                old_routine = 1;                         // Switch to polling mode 
                last_duty_cycle = min_startup_duty / 2;  // Reduce power for gentle restart
            }
            desync_check = 0;   // Reset desync check flag
            last_average_interval = average_interval;
        }

        // 8. Interrupt Priority Management
#if !defined(MCU_G031) && !defined(NEED_INPUT_READY)
        if (dshot_telemetry && (commutation_interval > DSHOT_PRIORITY_THRESHOLD)) {
            // At low RPM, timing is relaxed. Prioritize DShot           
  │         // telemetry response to meet protocol timing.    
            NVIC_SetPriority(IC_DMA_IRQ_NAME, 0);
            NVIC_SetPriority(COM_TIMER_IRQ, 1);
            NVIC_SetPriority(COMPARATOR_IRQ, 1);
         } else {
            //  At high RPM, precise commutation timing is critical.      
  │         // Missing a zero crossing = desync. DShot can wait.
            NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);// DShot = lower
            NVIC_SetPriority(COM_TIMER_IRQ, 0);  // Commutation = highest
            NVIC_SetPriority(COMPARATOR_IRQ, 0); // BEMF = highest
         }
#endif
        // 9. Telemetry Transmission
        if (send_telemetry) { // Send_telemetry set in tenKhzRoutine()
#ifdef USE_SERIAL_TELEMETRY
            makeTelemPackage((int8_t)degrees_celsius, battery_voltage, actual_current,
                (uint16_t)(consumed_current >> 16), e_rpm);
            send_telem_DMA(10);
            send_telemetry = 0;
#endif
        } else if(send_esc_info_flag ) {
           makeInfoPacket();
           send_telem_DMA(49); // send_telem_DMA() transmits via UART using DMA (non-blocking)
           send_esc_info_flag = 0;
        }

        // 10. ADC Input Process (PROCESS_ADC_FLAG set in tenKhzRoutine())
        if (PROCESS_ADC_FLAG == 1) { // for adc and telemetry set adc counter at 1khz loop rate
#if defined(STMICRO)
            ADC_DMA_Callback();
            LL_ADC_REG_StartConversion(ADC1);
            converted_degrees = __LL_ADC_CALC_TEMPERATURE(3300, ADC_raw_temp, LL_ADC_RESOLUTION_12B);
#endif
#ifdef MCU_GDE23
            ADC_DMA_Callback();
            // converted_degrees = (1.43 - ADC_raw_temp * 3.3 / 4096) * 1000 / 4.3 + 25;
            converted_degrees = ((int32_t)(357.5581395348837f * (1 << 16)) - ADC_raw_temp * (int32_t)(0.18736373546511628f * (1 << 16))) >> 16;
            adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
#endif
#ifdef ARTERY
            ADC_DMA_Callback();
            adc_ordinary_software_trigger_enable(ADC1, TRUE);
            converted_degrees = getConvertedDegrees(ADC_raw_temp);
#endif
#ifdef WCH
            startADCConversion( );
            converted_degrees = getConvertedDegrees(ADC_raw_temp);
#endif
            degrees_celsius = converted_degrees;
            battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100)) >> 3;
            smoothed_raw_current = getSmoothedCurrent();
            actual_current = ((smoothed_raw_current * 3300 / 41) - (CURRENT_OFFSET * 100)) / (MILLIVOLT_PER_AMP);
            if (actual_current < 0) {
                actual_current = 0;
            }             
            if (eepromBuffer.low_voltage_cut_off == 1) {  
                if (battery_voltage < (cell_count * low_cell_volt_cutoff)) {
                  low_voltage_count++;
                } else {
                  if(!LOW_VOLTAGE_CUTOFF){  // if set low cutoff has happened, require power cycle to reset
                    low_voltage_count = 0;
                  }
                }
            }
            if (eepromBuffer.low_voltage_cut_off == 2 ){   // absolute cut off
              if (battery_voltage <  eepromBuffer.absolute_voltage_cutoff) {
                low_voltage_count++;    
                } else {
                  if(!LOW_VOLTAGE_CUTOFF){
                    low_voltage_count = 0;
                  }
                }
            }
            if (low_voltage_count > (10000 - (stepper_sine * 9900))) {      // 10 second wait before cut-off for low voltage
              LOW_VOLTAGE_CUTOFF = 1;
              input = 0;
              allOff();
              maskPhaseInterrupts();
              running = 0;
              zero_input_count = 0;
              armed = 0;
             }
           
            PROCESS_ADC_FLAG = 0;
#ifdef USE_ADC_INPUT
            if (ADC_raw_input < 10) {
                zero_input_count++;
            } else {
                zero_input_count = 0;
            }
#endif
        }

#ifdef USE_ADC_INPUT
        signaltimeout = 0;
        ADC_smoothed_input = (((10 * ADC_smoothed_input) + ADC_raw_input) / 11);
        newinput = ADC_smoothed_input / 2;
        if (newinput > 2000) {
            newinput = 2000;
        }
#endif
        // stuckcounter = 0; TO BE IMPLEMENTED

        // 11. RPM Calculation
        e_com_time = ((commutation_intervals[0] + commutation_intervals[1] + commutation_intervals[2] + commutation_intervals[3] + commutation_intervals[4] + commutation_intervals[5]) + 4) >> 1; 
        average_interval = e_com_time / 3;

        if (stepper_sine == 0) { // For BEMF Pooling/Interrupt Mode

            e_rpm = running * (600000 / e_com_time); // in tens of rpm -> Mainly used for telemetry
                                                     // running ensures e_rpm = 0 when motor is stopped
            k_erpm = e_rpm / 10; // in hundreds of rpm 

            // Maximum Duty Cycle Protection 
            if (low_rpm_throttle_limit) { // some hardware doesn't need this, its on by default to keep hardware / motors
                                          // protected but can slow down the response in the very low end a little.
                duty_cycle_maximum = map(k_erpm, low_rpm_level, high_rpm_level, throttle_max_at_low_rpm, throttle_max_at_high_rpm);
                // for more performance lower the high_rpm_level, set to a consvervative number in source.
            }else{
				duty_cycle_maximum = 2000;
            }

            // Temperature Protection 
            if (degrees_celsius > eepromBuffer.limits.temperature) {
              duty_cycle_maximum = map(degrees_celsius, eepromBuffer.limits.temperature - 10, eepromBuffer.limits.temperature + 10,
                throttle_max_at_high_rpm / 2, 1);
            }

            // BEMF Filter Level Adjustment -> used in interruptRoutine() to validate zero crossings
            if (zero_crosses < 100 && commutation_interval > 500) { // For Low RPM or Start of rotation, keep max filter level to prevent noise
              filter_level = 12;
            } else {
              filter_level = map(average_interval, 100, 500, 3, 12);
            }

            if (commutation_interval < 50) {
              filter_level = 2;
            }

            // Auto Advance Level Adjustment -> Used in PeriodElapsedCallback()
            if (eepromBuffer.auto_advance) {
              auto_advance_level = map(duty_cycle, 100, 2000, 13, 23); // Defines how early to commutate before ideal ZC point 
            }

            /**************** old routine*********************/
#ifdef CUSTOM_RAMP
            if (old_routine && running) {
                maskPhaseInterrupts();
                getBemfState();
                if (!zcfound) {
                    if (rising) {
                        if (bemfcounter > min_bemf_counts_up) {
                            zcfound = 1;
                            zcfoundroutine();
                        }
                    } else {
                        if (bemfcounter > min_bemf_counts_down) {
                            zcfound = 1;
                            zcfoundroutine();
                        }
                    }
                }
            }
#endif
            // INTERVAL_TIMER_COUNT counts µs since last zero crossing. if > 45000µs (45ms) with no zero crossing detected, Motor is probably stalled
            // his prevents the motor from sitting with current flowing through one winding indefinitely (would overheat/burn) 
            if (INTERVAL_TIMER_COUNT > 45000 && running == 1) {
                bemf_timeout_happened++;

                maskPhaseInterrupts();
                old_routine = 1;
                if (input < 48) {
                    running = 0;
                    commutation_interval = 5000;
                }
                zero_crosses = 0;
                zcfoundroutine();
            }
        } else { // stepper sine Open Loop Driven Startup

#ifdef GIMBAL_MODE
            step_delay = 300;
            maskPhaseInterrupts();
            allpwm();
            if (newinput > 1000) {
                desired_angle = map(newinput, 1000, 2000, 180, 360);
            } else {
                desired_angle = map(newinput, 0, 1000, 0, 180);
            }
            if (current_angle > desired_angle) {
                forward = 1;
                advanceincrement();
                delayMicros(step_delay);
                current_angle--;
            }
            if (current_angle < desired_angle) {
                forward = 0;
                advanceincrement();
                delayMicros(step_delay);
                current_angle++;
            }
#else

            if (input > 48 && armed) {

                if (input > 48 && input < 137) { // sine wave stepper

                    if (do_once_sinemode) {
                        // disable commutation interrupt in case set
                        DISABLE_COM_TIMER_INT();
                        maskPhaseInterrupts();
                        SET_DUTY_CYCLE_ALL(0);
                        allpwm();
                        do_once_sinemode = 0;
                    }
                    advanceincrement();
                    step_delay = map(input, 48, 120, 7000 / eepromBuffer.motor_poles, 810 / eepromBuffer.motor_poles);
                    delayMicros(step_delay);
                    e_rpm = 600 / step_delay; // in hundreds so 33 e_rpm is 3300 actual erpm

                } else {
                    do_once_sinemode = 1;
                    advanceincrement();
                    if (input > 200) {
                        phase_A_position = 0;
                        step_delay = 80;
                    }

                    delayMicros(step_delay);
                    if (phase_A_position == 0) {
                        stepper_sine = 0;
                        running = 1;
                        old_routine = 1;
                        commutation_interval = 9000;
                        average_interval = 9000;
                        last_average_interval = average_interval;
                        SET_INTERVAL_TIMER_COUNT(9000);
                        zero_crosses = 20;
                        prop_brake_active = 0;
                        step = changeover_step;
                        // comStep(step);// rising bemf on a same as position 0.
                        if (eepromBuffer.stall_protection) {
                            last_duty_cycle = stall_protect_minimum_duty;
                        }
                        commutate();
                        generatePwmTimerEvent();
                    }
                }

            } else {
                do_once_sinemode = 1;
                if (eepromBuffer.brake_on_stop == 1) {
#ifndef PWM_ENABLE_BRIDGE
                    prop_brake_duty_cycle =  eepromBuffer.drag_brake_strength * 200;
                    adjusted_duty_cycle =  tim1_arr - ((prop_brake_duty_cycle * tim1_arr) / 2000);
                    if(adjusted_duty_cycle < 100){
                      fullBrake();
                    }else{
                      proportionalBrake();
                      SET_DUTY_CYCLE_ALL(adjusted_duty_cycle);
                      prop_brake_active = 1;
                    } 
#else
                    // todo add braking for PWM /enable style bridges.
#endif
                } else if (eepromBuffer.brake_on_stop == 2){
                  comStep(2);
                  SET_DUTY_CYCLE_ALL(DEAD_TIME + ((eepromBuffer.active_brake_power * tim1_arr) / 2000)* 10);
                }else{
                   SET_DUTY_CYCLE_ALL(0);
                   allOff();
                }
                e_rpm = 0;
            }

#endif // gimbal mode
        } // stepper/sine mode end

#ifdef BRUSHED_MODE
        runBrushedLoop();
#endif
        // 12. DroneCAN Communication
#if DRONECAN_SUPPORT
	DroneCAN_update();
#endif
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, tex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */













































































