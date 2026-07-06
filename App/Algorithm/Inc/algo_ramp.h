/*
 * Ramp Generator — Linear / Exponential / Trapezoidal / Smoothstep
 *
 * Linear mode:
 *   rate      = constant slope (output-units / s)
 *   per-step  = rate × step_time_s
 *
 * Exponential mode:
 *   rate      = time-constant τ (seconds)
 *   alpha     = 1 − exp(−step_time_s / τ)
 *   per-step  = current + alpha × (target − current)
 *
 * Trapezoidal mode (velocity + acceleration limits):
 *   rate      = max velocity v_max (output-units / s)
 *   accel_max = max acceleration a_max (output-units / s²)
 *   Phases:  ACCEL → CRUISE → DECEL → DONE
 *
 * Smoothstep mode (S-curve via duration):
 *   rate      = total duration (seconds), 0 = use rate-derived duration
 *   Perlin smoothstep:  s = 3t² − 2t³   where t ∈ [0, 1]
 *
 * step_time_s must match the fixed calling period.
 * ALGO_RAMP_EPSILON controls the snap-to-target threshold.
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#ifndef ALGO_RAMP_H
#define ALGO_RAMP_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ALGO_RAMP_EPSILON
#define ALGO_RAMP_EPSILON 1.0e-6f
#endif

typedef enum {
    ALGO_RAMP_MODE_LINEAR       = 0,
    ALGO_RAMP_MODE_EXPONENTIAL  = 1,
    ALGO_RAMP_MODE_TRAPEZOIDAL  = 2,
    ALGO_RAMP_MODE_SMOOTHSTEP   = 3,
} algo_ramp_mode_t;

typedef struct algo_ramp algo_ramp_t;

typedef struct {
    algo_ramp_mode_t mode;
    float            rate;
    float            step_time_s;
    float            target;
    float            initial;
    float            accel_max;
    float            duration_s;
} algo_ramp_cfg_t;

typedef int   (*algo_ramp_init_fn)(algo_ramp_t *s, const algo_ramp_cfg_t *c);
typedef float (*algo_ramp_step_fn)(algo_ramp_t *s);
typedef void  (*algo_ramp_reset_fn)(algo_ramp_t *s);
typedef int   (*algo_ramp_set_target_fn)(algo_ramp_t *s, float target);
typedef int   (*algo_ramp_set_current_fn)(algo_ramp_t *s, float current);
typedef float (*algo_ramp_get_current_fn)(const algo_ramp_t *s);
typedef float (*algo_ramp_get_target_fn)(const algo_ramp_t *s);
typedef bool  (*algo_ramp_is_done_fn)(const algo_ramp_t *s);

struct algo_ramp {
    struct {
        algo_ramp_init_fn        init;
        algo_ramp_step_fn        step;
        algo_ramp_reset_fn       reset;
        algo_ramp_set_target_fn  set_target;
        algo_ramp_set_current_fn set_current;
        algo_ramp_get_current_fn get_current;
        algo_ramp_get_target_fn  get_target;
        algo_ramp_is_done_fn     is_done;
    };

    algo_ramp_mode_t _mode;
    float            _inc;
    float            _alpha;
    float            _a_max;
    float            _v_max;
    float            _ts_s;
    float            _target;
    float            _initial;
    float            _current;
    float            _v;
    float            _elapsed;
    float            _duration;
    uint8_t          _phase;
    bool             _inited;
};

void algo_ramp_ctor(algo_ramp_t *s);

static inline bool algo_ramp_finite(float x)
{
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

#ifdef __cplusplus
}
#endif

#endif /* ALGO_RAMP_H */
