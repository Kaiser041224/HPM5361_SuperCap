/*
 * Ramp Generator Implementation
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#include "algo_ramp.h"

#include <math.h>
#include <stddef.h>

#define RAMP_PH_ACCEL  0
#define RAMP_PH_CRUISE 1
#define RAMP_PH_DECEL  2
#define RAMP_PH_DONE   3

static int   algo_ramp_init_impl(algo_ramp_t *s, const algo_ramp_cfg_t *c);
static float algo_ramp_step_impl(algo_ramp_t *s);
static void  algo_ramp_reset_impl(algo_ramp_t *s);
static int   algo_ramp_set_target_impl(algo_ramp_t *s, float target);
static int   algo_ramp_set_current_impl(algo_ramp_t *s, float current);
static float algo_ramp_get_current_impl(const algo_ramp_t *s);
static float algo_ramp_get_target_impl(const algo_ramp_t *s);
static bool  algo_ramp_is_done_impl(const algo_ramp_t *s);

static float algo_ramp_step_linear(algo_ramp_t *s);
static float algo_ramp_step_exponential(algo_ramp_t *s);
static float algo_ramp_step_trapezoidal(algo_ramp_t *s);
static float algo_ramp_step_smoothstep(algo_ramp_t *s);

/* ── Init ─────────────────────────────────────────────────────────────── */

static int algo_ramp_init_impl(algo_ramp_t *s, const algo_ramp_cfg_t *c)
{
    if (s == NULL)                                  return -1;
    if (c == NULL)                                  return -2;
    if (!algo_ramp_finite(c->step_time_s))           return -8;
    if (c->step_time_s <= 0.0f)                     return -3;
    if (!algo_ramp_finite(c->rate))                  return -7;
    if (c->rate <= 0.0f)                            return -4;
    if (!algo_ramp_finite(c->target))                return -5;
    if (!algo_ramp_finite(c->initial))               return -6;

    switch (c->mode) {
    case ALGO_RAMP_MODE_LINEAR:
    case ALGO_RAMP_MODE_EXPONENTIAL:
    case ALGO_RAMP_MODE_TRAPEZOIDAL:
    case ALGO_RAMP_MODE_SMOOTHSTEP:
        break;
    default:
        s->_inited = false; return -9;
    }

    s->_mode    = c->mode;
    s->_target  = c->target;
    s->_initial = c->initial;
    s->_current = c->initial;
    s->_ts_s    = c->step_time_s;
    s->_v       = 0.0f;
    s->_elapsed = 0.0f;
    s->_phase   = RAMP_PH_ACCEL;

    switch (c->mode) {

    case ALGO_RAMP_MODE_EXPONENTIAL: {
        float arg = -c->step_time_s / c->rate;
        if (!algo_ramp_finite(arg))          { s->_inited = false; return -10; }
        s->_alpha = 1.0f - expf(arg);
        if (!algo_ramp_finite(s->_alpha))    { s->_inited = false; return -10; }
        if (s->_alpha < 0.0f) s->_alpha = 0.0f;
        if (s->_alpha > 1.0f) s->_alpha = 1.0f;
        s->_inc      = 0.0f;
        s->_a_max    = 0.0f;
        s->_v_max    = 0.0f;
        s->_duration = 0.0f;
    } break;

    case ALGO_RAMP_MODE_TRAPEZOIDAL: {
        if (!algo_ramp_finite(c->accel_max)) { s->_inited = false; return -11; }
        if (c->accel_max <= 0.0f)            { s->_inited = false; return -11; }
        s->_v_max    = c->rate;
        s->_a_max    = c->accel_max;
        s->_inc      = 0.0f;
        s->_alpha    = 0.0f;
        s->_duration = 0.0f;
    } break;

    case ALGO_RAMP_MODE_SMOOTHSTEP: {
        if (c->duration_s > 0.0f) {
            if (!algo_ramp_finite(c->duration_s)) { s->_inited = false; return -12; }
            s->_duration = c->duration_s;
        } else {
            float dist = (c->target > c->initial)
                         ? (c->target - c->initial)
                         : (c->initial - c->target);
            s->_duration = dist / c->rate;
        }
        if (!algo_ramp_finite(s->_duration)) { s->_inited = false; return -12; }
        if (s->_duration <= 0.0f)            { s->_inited = false; return -12; }
        s->_inc      = 0.0f;
        s->_alpha    = 0.0f;
        s->_a_max    = 0.0f;
        s->_v_max    = 0.0f;
    } break;

    default: /* LINEAR */
        s->_inc = c->rate * c->step_time_s;
        if (!algo_ramp_finite(s->_inc))  { s->_inited = false; return -10; }
        if (s->_inc <= 0.0f)             { s->_inited = false; return -10; }
        s->_alpha    = 0.0f;
        s->_a_max    = 0.0f;
        s->_v_max    = 0.0f;
        s->_duration = 0.0f;
        break;
    }

    s->_inited = true;
    return 0;
}

/* ── Step dispatcher ─────────────────────────────────────────────────── */

static float algo_ramp_step_impl(algo_ramp_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;

    if (!algo_ramp_finite(s->_current) || !algo_ramp_finite(s->_target)) {
        if (algo_ramp_finite(s->_initial)) { s->_current = s->_initial; return s->_initial; }
        return 0.0f;
    }

    float err     = s->_target - s->_current;
    float abs_err = (err < 0.0f) ? -err : err;
    if (abs_err <= ALGO_RAMP_EPSILON) {
        s->_current = s->_target;
        s->_phase   = RAMP_PH_DONE;
        return s->_target;
    }

    switch (s->_mode) {
    case ALGO_RAMP_MODE_EXPONENTIAL:  return algo_ramp_step_exponential(s);
    case ALGO_RAMP_MODE_TRAPEZOIDAL:  return algo_ramp_step_trapezoidal(s);
    case ALGO_RAMP_MODE_SMOOTHSTEP:   return algo_ramp_step_smoothstep(s);
    default:                          return algo_ramp_step_linear(s);
    }
}

/* ── Linear ───────────────────────────────────────────────────────────── */

static float algo_ramp_step_linear(algo_ramp_t *s)
{
    if (s->_current < s->_target) {
        s->_current += s->_inc;
        if (s->_current > s->_target) s->_current = s->_target;
    } else {
        s->_current -= s->_inc;
        if (s->_current < s->_target) s->_current = s->_target;
    }
    return s->_current;
}

/* ── Exponential ──────────────────────────────────────────────────────── */

static float algo_ramp_step_exponential(algo_ramp_t *s)
{
    s->_current += s->_alpha * (s->_target - s->_current);
    return s->_current;
}

/* ── Trapezoidal ──────────────────────────────────────────────────────── */
/*
 * Phases:  ACCEL → CRUISE → DECEL → DONE
 * Decel triggered when remaining distance ≤ v² / (2 * a_max).
 */

static float algo_ramp_step_trapezoidal(algo_ramp_t *s)
{
    float Ts    = s->_ts_s;
    float a_max = s->_a_max;
    float v_max = s->_v_max;
    float dir   = (s->_target > s->_current) ? 1.0f : -1.0f;

    if (s->_phase == RAMP_PH_ACCEL) {
        s->_v += a_max * Ts;
        if (s->_v > v_max) s->_v = v_max;
    }

    float d_remain = (s->_target > s->_current)
                     ? (s->_target - s->_current)
                     : (s->_current - s->_target);
    float d_brake  = (s->_v * s->_v) / (2.0f * a_max + 1.0e-12f);

    if (d_remain <= d_brake && s->_phase != RAMP_PH_DECEL) {
        s->_phase = RAMP_PH_DECEL;
    }

    if (s->_v >= v_max && s->_phase == RAMP_PH_ACCEL) {
        s->_phase = RAMP_PH_CRUISE;
    }

    if (s->_phase == RAMP_PH_DECEL) {
        s->_v -= a_max * Ts;
        if (s->_v < 0.0f) s->_v = 0.0f;
    }

    s->_current += dir * s->_v * Ts;

    if (dir > 0.0f && s->_current > s->_target) { s->_current = s->_target; s->_v = 0.0f; s->_phase = RAMP_PH_DONE; }
    if (dir < 0.0f && s->_current < s->_target) { s->_current = s->_target; s->_v = 0.0f; s->_phase = RAMP_PH_DONE; }

    return s->_current;
}

/* ── Smoothstep ───────────────────────────────────────────────────────── */
/*
 * s(t) = 3t² − 2t³  (Perlin smoothstep), t = elapsed / duration ∈ [0, 1]
 */

static float algo_ramp_step_smoothstep(algo_ramp_t *s)
{
    s->_elapsed += s->_ts_s;
    if (s->_elapsed >= s->_duration) {
        s->_elapsed = s->_duration;
        s->_current = s->_target;
        return s->_target;
    }

    float t = s->_elapsed / s->_duration;
    float s_t = t * t * (3.0f - 2.0f * t);

    s->_current = s->_initial + (s->_target - s->_initial) * s_t;
    return s->_current;
}

/* ── Reset ────────────────────────────────────────────────────────────── */

static void algo_ramp_reset_impl(algo_ramp_t *s)
{
    if (s == NULL || !s->_inited) return;
    s->_current = s->_initial;
    s->_v       = 0.0f;
    s->_elapsed = 0.0f;
    s->_phase   = RAMP_PH_ACCEL;
}

/* ── set_target ───────────────────────────────────────────────────────── */

static int algo_ramp_set_target_impl(algo_ramp_t *s, float target)
{
    if (s == NULL || !s->_inited) return -1;
    if (!algo_ramp_finite(target)) return -2;
    s->_target = target;
    s->_phase  = RAMP_PH_ACCEL;
    if (s->_mode == ALGO_RAMP_MODE_SMOOTHSTEP) {
        s->_elapsed = 0.0f;
        s->_initial = s->_current;
    }
    return 0;
}

/* ── set_current ──────────────────────────────────────────────────────── */

static int algo_ramp_set_current_impl(algo_ramp_t *s, float current)
{
    if (s == NULL || !s->_inited) return -1;
    if (!algo_ramp_finite(current)) return -2;
    s->_current = current;
    s->_initial = current;
    s->_v       = 0.0f;
    s->_elapsed = 0.0f;
    s->_phase   = RAMP_PH_ACCEL;
    return 0;
}

/* ── getters ──────────────────────────────────────────────────────────── */

static float algo_ramp_get_current_impl(const algo_ramp_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_current;
}

static float algo_ramp_get_target_impl(const algo_ramp_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_target;
}

static bool algo_ramp_is_done_impl(const algo_ramp_t *s)
{
    if (s == NULL || !s->_inited) return false;
    float err = s->_target - s->_current;
    return (err <= ALGO_RAMP_EPSILON) && (err >= -ALGO_RAMP_EPSILON);
}

/* ── Constructor ──────────────────────────────────────────────────────── */

void algo_ramp_ctor(algo_ramp_t *s)
{
    if (s == NULL) return;

    s->init        = algo_ramp_init_impl;
    s->step        = algo_ramp_step_impl;
    s->reset       = algo_ramp_reset_impl;
    s->set_target  = algo_ramp_set_target_impl;
    s->set_current = algo_ramp_set_current_impl;
    s->get_current = algo_ramp_get_current_impl;
    s->get_target  = algo_ramp_get_target_impl;
    s->is_done     = algo_ramp_is_done_impl;
    s->_mode       = ALGO_RAMP_MODE_LINEAR;
    s->_inc        = 0.0f;
    s->_alpha      = 0.0f;
    s->_a_max      = 0.0f;
    s->_v_max      = 0.0f;
    s->_ts_s       = 0.0f;
    s->_target     = 0.0f;
    s->_initial    = 0.0f;
    s->_current    = 0.0f;
    s->_v          = 0.0f;
    s->_elapsed    = 0.0f;
    s->_duration   = 0.0f;
    s->_phase      = 0;
    s->_inited     = false;
}
