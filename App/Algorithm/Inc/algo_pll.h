/*
 * Software PLL — Period Tracker / Phase-Locked Loop
 *
 * Two modes:
 *
 *   PERIOD_TRACKER  (mode 0, backward compatible):
 *     T_measured → [PI] → _period → _freq = 1/_period
 *     pll.step(&pll, T) returns tracked frequency Hz.
 *
 *   PHASE_LOCKED    (mode 1, true PLL):
 *     phase_err → [PI loop filter] → ω → [NCO] → φ → sin/cos
 *     pll.step_error(&pll, err_rad) returns tracked frequency Hz.
 *
 * Units:
 *   freq  = Hz
 *   omega = rad/s
 *   phase = rad  ([0, 2π) or [-π, π) depending on context)
 *   phase_error = rad  (accepted [-π, π), internally re-wrapped)
 *
 * sample_time_s must match the fixed calling period.
 * For ISR use, prefer step_error() which avoids sinf/cosf/atan2f.
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#ifndef ALGO_PLL_H
#define ALGO_PLL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ALGO_PLL_PI_F     3.14159265358979323846f
#define ALGO_PLL_TWO_PI_F (2.0f * ALGO_PLL_PI_F)

#ifndef ALGO_PLL_ENABLE_SINCOS_OUTPUT
#define ALGO_PLL_ENABLE_SINCOS_OUTPUT 1
#endif

#ifndef ALGO_PLL_ENABLE_ATAN2_INPUT
#define ALGO_PLL_ENABLE_ATAN2_INPUT 1
#endif

typedef enum {
    ALGO_PLL_MODE_PERIOD_TRACKER = 0,
    ALGO_PLL_MODE_PHASE_LOCKED   = 1,
} algo_pll_mode_t;

typedef struct algo_pll algo_pll_t;

typedef struct {
    float           kp;
    float           ki;
    float           period_nominal;
    float           period_min;
    float           period_max;
    algo_pll_mode_t mode;
    float           sample_time_s;
    float           freq_nominal_hz;
    float           freq_min_hz;
    float           freq_max_hz;
    float           phase_init_rad;
    float           phase_offset_rad;
} algo_pll_cfg_t;

typedef int   (*algo_pll_init_fn)(algo_pll_t *s, const algo_pll_cfg_t *c);
typedef float (*algo_pll_step_fn)(algo_pll_t *s, float T);
typedef void  (*algo_pll_reset_fn)(algo_pll_t *s);
typedef float (*algo_pll_get_freq_fn)(const algo_pll_t *s);
typedef float (*algo_pll_get_period_fn)(const algo_pll_t *s);
typedef float (*algo_pll_step_error_fn)(algo_pll_t *s, float phase_error_rad);
typedef float (*algo_pll_step_phase_fn)(algo_pll_t *s, float phase_meas_rad);
typedef float (*algo_pll_step_sincos_fn)(algo_pll_t *s, float sin_m, float cos_m);
typedef float (*algo_pll_get_phase_fn)(const algo_pll_t *s);
typedef float (*algo_pll_get_omega_fn)(const algo_pll_t *s);
typedef float (*algo_pll_get_sin_fn)(const algo_pll_t *s);
typedef float (*algo_pll_get_cos_fn)(const algo_pll_t *s);
typedef float (*algo_pll_get_phase_err_fn)(const algo_pll_t *s);
typedef int   (*algo_pll_set_phase_fn)(algo_pll_t *s, float phase_rad);
typedef int   (*algo_pll_set_freq_fn)(algo_pll_t *s, float freq_hz);

struct algo_pll {
    struct {
        algo_pll_init_fn           init;
        algo_pll_step_fn           step;
        algo_pll_reset_fn          reset;
        algo_pll_get_freq_fn       get_freq;
        algo_pll_get_period_fn     get_period;
        algo_pll_step_error_fn     step_error;
        algo_pll_step_phase_fn     step_phase;
        algo_pll_step_sincos_fn    step_sincos;
        algo_pll_get_phase_fn      get_phase;
        algo_pll_get_omega_fn      get_omega;
        algo_pll_get_sin_fn        get_sin;
        algo_pll_get_cos_fn        get_cos;
        algo_pll_get_phase_err_fn  get_phase_error;
        algo_pll_set_phase_fn      set_phase;
        algo_pll_set_freq_fn       set_freq;
    };

    algo_pll_mode_t _mode;
    float           _kp, _ki;
    float           _ts_s;
    float           _period_nominal, _period_min, _period_max;
    float           _period;
    float           _integral;
    float           _freq;

    float           _phase;
    float           _phase_offset;
    float           _phase_err;
    float           _omega;
    float           _omega_nominal, _omega_min, _omega_max;
    float           _loop_integral;

    float           _phase_init;
    float           _freq_nominal, _freq_min, _freq_max;

#if ALGO_PLL_ENABLE_SINCOS_OUTPUT
    float           _sin, _cos;
#endif

    bool            _inited;
};

void algo_pll_ctor(algo_pll_t *s);

static inline bool algo_pll_finite(float x)
{
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

#ifdef __cplusplus
}
#endif

#endif /* ALGO_PLL_H */
