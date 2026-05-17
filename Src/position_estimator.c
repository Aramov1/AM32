#include "position_estimator.h"

#include <stdint.h>

#include "main.h"

extern volatile uint32_t commutation_interval; // 0.5 us units (INTERVAL_TIMER on L431)
extern int16_t           m_step;

#define PE_STEPS_PER_EREV 36
#define PE_STEP_Q16       (65536 / PE_STEPS_PER_EREV) // 1820 LSB per m_step

// Second-stage IIR on top of AM32's existing 75/25 filter in zcfoundroutine.
// alpha = 1 / (1 << POS_EST_OMEGA_IIR_SHIFT). User can override per-target.
#ifndef POS_EST_OMEGA_IIR_SHIFT
#define POS_EST_OMEGA_IIR_SHIFT 3
#endif

#ifndef POS_EST_OMEGA_NOM_IIR_SHIFT
#define POS_EST_OMEGA_NOM_IIR_SHIFT 10 // ~1 s time constant at typical commutation rates
#endif

#if POSITION_ESTIMATOR_BACKEND == POSITION_ESTIMATOR_BACKEND_COMMUTATION

static volatile uint32_t pe_tick_10k;
static volatile uint32_t pe_last_commutation_tick;
static volatile int32_t  pe_period_filt_us;
static volatile int32_t  pe_omega_e;
static volatile int32_t  pe_omega_nom;
static volatile int8_t   pe_direction;
static volatile uint8_t  pe_last_m_step;

void position_estimator_init(void)
{
    pe_tick_10k = 0;
    pe_last_commutation_tick = 0;
    pe_period_filt_us = 0;
    pe_omega_e = 0;
    pe_omega_nom = 0;
    pe_direction = 1;
    pe_last_m_step = 0;
}

void position_estimator_on_commutation(int dir)
{
    pe_direction = (dir >= 0) ? 1 : -1;
    pe_last_m_step = (uint8_t)((uint16_t)m_step);
    pe_last_commutation_tick = pe_tick_10k;

    int32_t raw_us = (int32_t)(commutation_interval >> 1); // 0.5us -> 1us
    if (raw_us < 1) {
        raw_us = 1;
    }

    if (pe_period_filt_us == 0) {
        pe_period_filt_us = raw_us;
    } else {
        const int32_t denom = (int32_t)1 << POS_EST_OMEGA_IIR_SHIFT;
        pe_period_filt_us = ((denom - 1) * pe_period_filt_us + raw_us) >> POS_EST_OMEGA_IIR_SHIFT;
    }
    if (pe_period_filt_us < 1) {
        pe_period_filt_us = 1;
    }

    // One m_step takes pe_period_filt_us microseconds; PE_STEPS_PER_EREV m_steps per e-rev.
    // omega (e-rev/s) = 1e6 / (PE_STEPS_PER_EREV * pe_period_filt_us)
    // Output in Q8: omega_q8 = 256 * 1e6 / (36 * period_us) = 7111111 / period_us.
    int32_t omega_mag = (int32_t)(7111111L / (long)pe_period_filt_us);
    pe_omega_e = (pe_direction > 0) ? omega_mag : -omega_mag;

    const int32_t ndenom = (int32_t)1 << POS_EST_OMEGA_NOM_IIR_SHIFT;
    pe_omega_nom = ((ndenom - 1) * pe_omega_nom + omega_mag) >> POS_EST_OMEGA_NOM_IIR_SHIFT;
}

void position_estimator_on_10khz_tick(void)
{
    pe_tick_10k++;
}

uint32_t position_estimator_get_theta_e_q16(void)
{
    uint32_t step_base = (uint32_t)pe_last_m_step * (uint32_t)PE_STEP_Q16;
    uint32_t elapsed_ticks = pe_tick_10k - pe_last_commutation_tick;
    int32_t elapsed_us = (int32_t)(elapsed_ticks * 100u); // 10 kHz -> 100 us per tick

    int32_t period = pe_period_filt_us;
    if (period < 1) {
        period = 1;
    }

    int32_t frac = (elapsed_us * PE_STEP_Q16) / period;
    if (frac > PE_STEP_Q16) {
        frac = PE_STEP_Q16; // clamp until next commutation arrives
    }
    if (frac < 0) {
        frac = 0;
    }

    int32_t signed_frac = (pe_direction > 0) ? frac : -frac;
    return (uint32_t)((int32_t)step_base + signed_frac) & 0xFFFFu;
}

int32_t position_estimator_get_omega_e(void)
{
    return pe_omega_e;
}

int32_t position_estimator_get_omega_nominal(void)
{
    return pe_omega_nom;
}

#elif POSITION_ESTIMATOR_BACKEND == POSITION_ESTIMATOR_BACKEND_ABI_TIMER

#error "POSITION_ESTIMATOR_BACKEND_ABI_TIMER not implemented yet"

#else

#error "Unknown POSITION_ESTIMATOR_BACKEND"

#endif
