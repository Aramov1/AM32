#pragma once

#include <stdint.h>

// Backend identifiers
#define POSITION_ESTIMATOR_BACKEND_COMMUTATION  1
#define POSITION_ESTIMATOR_BACKEND_ABI_TIMER    2

// A target without an explicit selection falls back to commutation-based.
#ifndef POSITION_ESTIMATOR_BACKEND
#define POSITION_ESTIMATOR_BACKEND POSITION_ESTIMATOR_BACKEND_COMMUTATION
#endif

// Units:
//   theta_e_q16     : electrical angle, 1 electrical revolution = 65536 LSB
//   omega_e         : signed (electrical-revolutions / second) * 256, positive = forward
//   omega_nominal   : slow LPF (~1 s) of |omega_e|, same units
//
// All getters are safe to call from any context. The on_commutation hook must
// be called inside the commutation ISR after m_step has been updated, and the
// on_10khz_tick hook must be called once per tenKhzRoutine invocation.

void     position_estimator_init(void);
void     position_estimator_on_commutation(int dir);
void     position_estimator_on_10khz_tick(void);

uint32_t position_estimator_get_theta_e_q16(void);
int32_t  position_estimator_get_omega_e(void);
int32_t  position_estimator_get_omega_nominal(void);
