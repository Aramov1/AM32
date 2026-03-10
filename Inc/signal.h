/*
 * IO.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "main.h"

extern char out_put;
extern char inputSet;
extern char dshot;
extern char servoPwm;
extern char send_telemetry;
extern uint8_t degrees_celsius;
extern char crawler_mode;

extern uint16_t ADC_raw_volts;
extern uint16_t servo_low_threshold; // anything below this point considered 0
extern uint16_t
    servo_high_threshold; // anything above this point considered 2000 (max)
extern uint16_t servo_neutral;
extern uint8_t servo_dead_band;
extern char inputSet;
extern char dshot;
extern char servoPwm;

void detectInput();

#ifdef USE_ANGLE_INPUT_INDEX
extern volatile uint8_t  as5047_index_flag;
extern volatile uint32_t as5047_revolution_count;
#endif

#ifdef CAN_EXTRA_INPUTS_COUNT
/*
 * Extra CAN input channels, extracted from RawCommand slots
 * esc_index+1 … esc_index+CAN_EXTRA_INPUTS_COUNT.
 * Written from handle_RawCommand() (ISR context).
 * Declared volatile because they cross interrupt boundaries.
 */
extern volatile uint16_t can_induced_sine_amplitude;      // 47..947 (0 to ~50% of max throttle)
extern volatile uint16_t can_induced_phase_offset;        // 0..3599 (degrees, 0.1 degree resolution)
extern volatile char m_step;
#endif