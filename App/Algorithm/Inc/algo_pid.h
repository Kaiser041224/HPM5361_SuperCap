/*
 * PID Controller
 *
 * Pure algorithm library — no hardware / SDK dependencies.
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#ifndef ALGO_PID_H
#define ALGO_PID_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── IEEE-754 finite check (memcpy-based, survives -ffast-math) ───────── */

static inline bool algo_pid_finite(float x)
{
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

/* ── Optional ILM deployment for hot control-loop paths ─────────────────
 * Set ALGO_ENABLE_ILM=0 (e.g. via build system) to keep algorithm code in
 * flash; default deploys hot paths to ILM (.fast) for deterministic 200kHz
 * inner-loop execution (AGENTS.md §1). */
#ifndef ALGO_ENABLE_ILM
#define ALGO_ENABLE_ILM 1
#endif
#if ALGO_ENABLE_ILM
#define ALGO_ATTR_RAMFUNC __attribute__((section(".fast")))
#else
#define ALGO_ATTR_RAMFUNC
#endif

typedef enum {
    ALGO_PID_MODE_POSITIONAL  = 0,
    ALGO_PID_MODE_INCREMENTAL = 1,
} algo_pid_mode_t;

typedef enum {
    ALGO_PID_ANTIWINDUP_NONE     = 0,
    ALGO_PID_ANTIWINDUP_CLAMP    = 1,
    ALGO_PID_ANTIWINDUP_BACKCALC = 2,
} algo_pid_antiwindup_t;

typedef struct {
    algo_pid_mode_t       mode;
    float                 kp;
    float                 ki;
    float                 kd;
    float                 sample_time_s;
    float                 out_min;
    float                 out_max;
    float                 integral_min;
    float                 integral_max;
    algo_pid_antiwindup_t antiwindup;
    float                 backcalc_coeff;
    float                 deriv_filter_coeff;
    bool                  deriv_on_measurement;
    float                 rate_limit;
    float                 setpoint_weight_p;
    float                 setpoint_weight_d;
} algo_pid_cfg_t;

typedef struct algo_pid algo_pid_t;

typedef int   (*algo_pid_init_fn)(algo_pid_t *self, const algo_pid_cfg_t *cfg);
typedef float (*algo_pid_step_fn)(algo_pid_t *self, float setpoint, float measurement);
typedef void  (*algo_pid_reset_fn)(algo_pid_t *self);
typedef void  (*algo_pid_set_gains_fn)(algo_pid_t *self, float kp, float ki, float kd);
typedef void  (*algo_pid_get_gains_fn)(const algo_pid_t *self, float *kp, float *ki, float *kd);

struct algo_pid {
    struct {
        algo_pid_init_fn       init;
        algo_pid_step_fn       step;
        algo_pid_reset_fn      reset;
        algo_pid_set_gains_fn  set_gains;
        algo_pid_get_gains_fn  get_gains;
    };

    float                 _kp, _ki, _kd;
    float                 _ki_T;
    float                 _kd_invT;
    float                 _inv_T;
    float                 _rate_limit_T;
    float                 _spw_p, _spw_d;
    float                 _out_min, _out_max;
    float                 _integral_min, _integral_max;
    float                 _backcalc_coeff;
    float                 _deriv_fc;
    bool                  _deriv_on_pv;

    float                 _integral;
    float                 _prev_error;
    float                 _prev2_error;
    float                 _prev_setpoint;
    float                 _prev2_setpoint;
    float                 _prev_measurement;
    float                 _prev2_measurement;
    float                 _prev_output;
    float                 _prev_dout;
    float                 _deriv_state;
    bool                  _primed;

    algo_pid_antiwindup_t _aw_strategy;
    bool                  _has_rate_limit;
    bool                  _has_deriv_filter;
    bool                  _initialized;
};

extern const algo_pid_cfg_t ALGO_PID_CFG_DEFAULT;

void algo_pid_ctor(algo_pid_t *self);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_PID_H */
