/*
 * Software PLL Implementation
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#include "algo_pll.h"

#include <math.h>
#include <stddef.h>

static int   algo_pll_init_impl(algo_pll_t *s, const algo_pll_cfg_t *c);
static float algo_pll_step_impl(algo_pll_t *s, float T);
static void  algo_pll_reset_impl(algo_pll_t *s);
static float algo_pll_get_freq_impl(const algo_pll_t *s);
static float algo_pll_get_period_impl(const algo_pll_t *s);
static float algo_pll_step_error_impl(algo_pll_t *s, float e);
static float algo_pll_step_phase_impl(algo_pll_t *s, float pm);
static float algo_pll_step_sincos_impl(algo_pll_t *s, float sm, float cm);
static float algo_pll_get_phase_impl(const algo_pll_t *s);
static float algo_pll_get_omega_impl(const algo_pll_t *s);
static float algo_pll_get_sin_impl(const algo_pll_t *s);
static float algo_pll_get_cos_impl(const algo_pll_t *s);
static float algo_pll_get_phase_err_impl(const algo_pll_t *s);
static int   algo_pll_set_phase_impl(algo_pll_t *s, float rad);
static int   algo_pll_set_freq_impl(algo_pll_t *s, float hz);

static float algo_pll_wrap_pi(float x);
static float algo_pll_wrap_2pi(float x);
static void  algo_pll_update_sincos(algo_pll_t *s);
static float algo_pll_step_period_tracker(algo_pll_t *s, float T);
static float algo_pll_step_phase_pll(algo_pll_t *s, float phase_err);

/* ── Phase wrap ──────────────────────────────────────────────────────── */

static float algo_pll_wrap_pi(float x)
{
    while (x >=  ALGO_PLL_PI_F)  x -= ALGO_PLL_TWO_PI_F;
    while (x <  -ALGO_PLL_PI_F)  x += ALGO_PLL_TWO_PI_F;
    return x;
}

static float algo_pll_wrap_2pi(float x)
{
    while (x >=  ALGO_PLL_TWO_PI_F) x -= ALGO_PLL_TWO_PI_F;
    while (x <   0.0f)              x += ALGO_PLL_TWO_PI_F;
    return x;
}

/* ── sin/cos update ──────────────────────────────────────────────────── */

static void algo_pll_update_sincos(algo_pll_t *s)
{
#if ALGO_PLL_ENABLE_SINCOS_OUTPUT
    float ph = s->_phase + s->_phase_offset;
    s->_sin = sinf(ph);
    s->_cos = cosf(ph);
#else
    (void)s;
#endif
}

/* ── Init ─────────────────────────────────────────────────────────────── */

static int algo_pll_init_impl(algo_pll_t *s, const algo_pll_cfg_t *c)
{
    if (s != NULL) s->_inited = false;

    if (s == NULL)                     return -1;
    if (c == NULL)                     return -2;
    if (!algo_pll_finite(c->kp))        return -3;
    if (!algo_pll_finite(c->ki))        return -4;

    s->_kp = c->kp;
    s->_ki = c->ki;

    switch (c->mode) {

    case ALGO_PLL_MODE_PERIOD_TRACKER: {
        if (!algo_pll_finite(c->period_nominal)) return -5;
        if (!algo_pll_finite(c->period_min))     return -6;
        if (!algo_pll_finite(c->period_max))     return -7;
        if (c->period_nominal <= 0.0f)           return -5;
        if (c->period_min >= c->period_max)       return -8;
        if (c->period_nominal < c->period_min ||
            c->period_nominal > c->period_max)    return -9;

        s->_mode           = ALGO_PLL_MODE_PERIOD_TRACKER;
        s->_period_nominal = c->period_nominal;
        s->_period_min     = c->period_min;
        s->_period_max     = c->period_max;
        s->_period         = c->period_nominal;
        s->_integral       = 0.0f;
        s->_freq           = 1.0f / c->period_nominal;
        s->_ts_s           = 0.0f;
    } break;

    case ALGO_PLL_MODE_PHASE_LOCKED: {
        if (!algo_pll_finite(c->sample_time_s))  return -10;
        if (c->sample_time_s <= 0.0f)             return -10;
        if (!algo_pll_finite(c->freq_nominal_hz)) return -11;
        if (c->freq_nominal_hz <= 0.0f)           return -11;
        if (!algo_pll_finite(c->freq_min_hz))     return -12;
        if (c->freq_min_hz <= 0.0f)               return -12;
        if (!algo_pll_finite(c->freq_max_hz))     return -13;
        if (c->freq_max_hz <= c->freq_min_hz)     return -13;
        if (c->freq_nominal_hz < c->freq_min_hz ||
            c->freq_nominal_hz > c->freq_max_hz)  return -14;
        if (!algo_pll_finite(c->phase_init_rad))  return -15;
        if (!algo_pll_finite(c->phase_offset_rad)) return -16;
        if (c->kp < 0.0f) return -3;
        if (c->ki < 0.0f) return -4;

        s->_mode           = ALGO_PLL_MODE_PHASE_LOCKED;
        s->_ts_s           = c->sample_time_s;
        s->_freq_nominal   = c->freq_nominal_hz;
        s->_freq_min       = c->freq_min_hz;
        s->_freq_max       = c->freq_max_hz;
        s->_phase_init     = algo_pll_wrap_2pi(c->phase_init_rad);
        s->_phase_offset   = c->phase_offset_rad;

        s->_freq           = c->freq_nominal_hz;
        s->_period         = 1.0f / c->freq_nominal_hz;
        s->_omega_nominal  = ALGO_PLL_TWO_PI_F * c->freq_nominal_hz;
        s->_omega_min      = ALGO_PLL_TWO_PI_F * c->freq_min_hz;
        s->_omega_max      = ALGO_PLL_TWO_PI_F * c->freq_max_hz;
        s->_omega          = s->_omega_nominal;
        s->_phase          = s->_phase_init;
        s->_phase_err      = 0.0f;
        s->_loop_integral  = 0.0f;
        s->_period_nominal = 0.0f;
        s->_period_min     = 0.0f;
        s->_period_max     = 0.0f;
        s->_integral       = 0.0f;

        algo_pll_update_sincos(s);
    } break;

    default:
        return -17;
    }

    s->_inited = true;
    return 0;
}

/* ── Step: period tracker (backward compatible) ──────────────────────── */

static float algo_pll_step_period_tracker(algo_pll_t *s, float T)
{
    if (!algo_pll_finite(T) || T <= 0.0f) return s->_freq;

    float err = T - s->_period;
    s->_integral += s->_ki * err;

    if (!algo_pll_finite(s->_integral)) s->_integral = 0.0f;

    float p = s->_period_nominal + s->_kp * err + s->_integral;

    if (p < s->_period_min) {
        p = s->_period_min;
        s->_integral = p - s->_period_nominal - s->_kp * err;
    } else if (p > s->_period_max) {
        p = s->_period_max;
        s->_integral = p - s->_period_nominal - s->_kp * err;
    }

    s->_period = p;

    if (s->_period > 0.0f) {
        s->_freq   = 1.0f / s->_period;
        s->_omega  = ALGO_PLL_TWO_PI_F * s->_freq;
    }

    if (!algo_pll_finite(s->_freq)) s->_freq = 0.0f;

    return s->_freq;
}

/* ── Step: phase-locked PLL (NCO update) ─────────────────────────────── */

static float algo_pll_step_phase_pll(algo_pll_t *s, float phase_err)
{
    phase_err = algo_pll_wrap_pi(phase_err);

    if (!algo_pll_finite(phase_err)) return s->_freq;

    float Ts = s->_ts_s;
    s->_loop_integral += s->_ki * Ts * phase_err;
    if (!algo_pll_finite(s->_loop_integral)) s->_loop_integral = 0.0f;

    float omega_cmd = s->_omega_nominal + s->_kp * phase_err + s->_loop_integral;

    if (omega_cmd < s->_omega_min) {
        omega_cmd = s->_omega_min;
        s->_loop_integral = omega_cmd - s->_omega_nominal - s->_kp * phase_err;
    } else if (omega_cmd > s->_omega_max) {
        omega_cmd = s->_omega_max;
        s->_loop_integral = omega_cmd - s->_omega_nominal - s->_kp * phase_err;
    }

    s->_phase_err = phase_err;
    s->_omega     = omega_cmd;
    s->_freq      = omega_cmd / ALGO_PLL_TWO_PI_F;
    s->_period    = 1.0f / s->_freq;

    if (!algo_pll_finite(s->_freq)) {
        s->_freq = s->_freq_nominal;
        s->_period = 1.0f / s->_freq;
        return s->_freq;
    }

    s->_phase += omega_cmd * Ts;
    s->_phase  = algo_pll_wrap_2pi(s->_phase);

    algo_pll_update_sincos(s);

    return s->_freq;
}

/* ── Step dispatcher ─────────────────────────────────────────────────── */

static float algo_pll_step_impl(algo_pll_t *s, float T)
{
    if (s == NULL || !s->_inited) return 0.0f;

    if (s->_mode == ALGO_PLL_MODE_PHASE_LOCKED) {
        (void)T;
        return s->_freq;
    }

    return algo_pll_step_period_tracker(s, T);
}

/* ── step_error ──────────────────────────────────────────────────────── */

static float algo_pll_step_error_impl(algo_pll_t *s, float e)
{
    if (s == NULL || !s->_inited) return 0.0f;

    if (s->_mode == ALGO_PLL_MODE_PERIOD_TRACKER) {
        return algo_pll_step_period_tracker(s, e);
    }

    return algo_pll_step_phase_pll(s, e);
}

/* ── step_phase ──────────────────────────────────────────────────────── */

static float algo_pll_step_phase_impl(algo_pll_t *s, float pm)
{
    if (s == NULL || !s->_inited) return 0.0f;

    if (!algo_pll_finite(pm)) return s->_freq;

    float err = algo_pll_wrap_pi(pm - s->_phase);

    if (s->_mode == ALGO_PLL_MODE_PERIOD_TRACKER) {
        return algo_pll_step_period_tracker(s, err);
    }

    return algo_pll_step_phase_pll(s, err);
}

/* ── step_sincos ─────────────────────────────────────────────────────── */

static float algo_pll_step_sincos_impl(algo_pll_t *s, float sm, float cm)
{
    if (s == NULL || !s->_inited) return 0.0f;

    (void)sm; (void)cm;

#if ALGO_PLL_ENABLE_ATAN2_INPUT
    if (!algo_pll_finite(sm) || !algo_pll_finite(cm)) return s->_freq;

    float pm = atan2f(sm, cm);

    float err = algo_pll_wrap_pi(pm - s->_phase);

    if (s->_mode == ALGO_PLL_MODE_PERIOD_TRACKER) {
        return algo_pll_step_period_tracker(s, err);
    }

    return algo_pll_step_phase_pll(s, err);
#else
    return s->_freq;
#endif
}

/* ── Reset ────────────────────────────────────────────────────────────── */

static void algo_pll_reset_impl(algo_pll_t *s)
{
    if (s == NULL || !s->_inited) return;

    if (s->_mode == ALGO_PLL_MODE_PHASE_LOCKED) {
        s->_phase         = s->_phase_init;
        s->_freq          = s->_freq_nominal;
        s->_period        = 1.0f / s->_freq;
        s->_omega         = s->_omega_nominal;
        s->_loop_integral = 0.0f;
        s->_phase_err     = 0.0f;
        algo_pll_update_sincos(s);
    } else {
        s->_period   = s->_period_nominal;
        s->_integral = 0.0f;
        s->_freq     = 1.0f / s->_period_nominal;
    }
}

/* ── Getters ──────────────────────────────────────────────────────────── */

static float algo_pll_get_freq_impl(const algo_pll_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_freq;
}

static float algo_pll_get_period_impl(const algo_pll_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_period;
}

static float algo_pll_get_phase_impl(const algo_pll_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_phase;
}

static float algo_pll_get_omega_impl(const algo_pll_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_omega;
}

static float algo_pll_get_sin_impl(const algo_pll_t *s)
{
#if ALGO_PLL_ENABLE_SINCOS_OUTPUT
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_sin;
#else
    (void)s; return 0.0f;
#endif
}

static float algo_pll_get_cos_impl(const algo_pll_t *s)
{
#if ALGO_PLL_ENABLE_SINCOS_OUTPUT
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_cos;
#else
    (void)s; return 0.0f;
#endif
}

static float algo_pll_get_phase_err_impl(const algo_pll_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_phase_err;
}

/* ── Setters ──────────────────────────────────────────────────────────── */

static int algo_pll_set_phase_impl(algo_pll_t *s, float rad)
{
    if (s == NULL || !s->_inited) return -1;
    if (!algo_pll_finite(rad))    return -2;

    s->_phase = algo_pll_wrap_2pi(rad);
    algo_pll_update_sincos(s);
    return 0;
}

static int algo_pll_set_freq_impl(algo_pll_t *s, float hz)
{
    if (s == NULL || !s->_inited) return -1;
    if (!algo_pll_finite(hz))     return -2;

    if (s->_mode == ALGO_PLL_MODE_PHASE_LOCKED) {
        if (hz < s->_freq_min || hz > s->_freq_max) return -3;
    } else {
        float p = 1.0f / hz;
        if (p < s->_period_min || p > s->_period_max) return -3;
    }

    s->_freq   = hz;
    s->_period = 1.0f / hz;
    if (!algo_pll_finite(s->_period)) return -4;

    if (s->_mode == ALGO_PLL_MODE_PHASE_LOCKED) {
        s->_omega = ALGO_PLL_TWO_PI_F * hz;
    }

    return 0;
}

/* ── Constructor ──────────────────────────────────────────────────────── */

void algo_pll_ctor(algo_pll_t *s)
{
    if (s == NULL) return;

    s->init           = algo_pll_init_impl;
    s->step           = algo_pll_step_impl;
    s->reset          = algo_pll_reset_impl;
    s->get_freq       = algo_pll_get_freq_impl;
    s->get_period     = algo_pll_get_period_impl;
    s->step_error     = algo_pll_step_error_impl;
    s->step_phase     = algo_pll_step_phase_impl;
    s->step_sincos    = algo_pll_step_sincos_impl;
    s->get_phase      = algo_pll_get_phase_impl;
    s->get_omega      = algo_pll_get_omega_impl;
    s->get_sin        = algo_pll_get_sin_impl;
    s->get_cos        = algo_pll_get_cos_impl;
    s->get_phase_error = algo_pll_get_phase_err_impl;
    s->set_phase      = algo_pll_set_phase_impl;
    s->set_freq       = algo_pll_set_freq_impl;

    s->_mode           = ALGO_PLL_MODE_PERIOD_TRACKER;
    s->_kp             = 0.0f;
    s->_ki             = 0.0f;
    s->_ts_s           = 0.0f;
    s->_period_nominal = 0.0f;
    s->_period_min     = 0.0f;
    s->_period_max     = 0.0f;
    s->_period         = 0.0f;
    s->_integral       = 0.0f;
    s->_freq           = 0.0f;
    s->_phase          = 0.0f;
    s->_phase_offset   = 0.0f;
    s->_phase_err      = 0.0f;
    s->_omega          = 0.0f;
    s->_omega_nominal  = 0.0f;
    s->_omega_min      = 0.0f;
    s->_omega_max      = 0.0f;
    s->_loop_integral  = 0.0f;
    s->_phase_init     = 0.0f;
    s->_freq_nominal   = 0.0f;
    s->_freq_min       = 0.0f;
    s->_freq_max       = 0.0f;
#if ALGO_PLL_ENABLE_SINCOS_OUTPUT
    s->_sin = 0.0f;
    s->_cos = 0.0f;
#endif
    s->_inited = false;
}
