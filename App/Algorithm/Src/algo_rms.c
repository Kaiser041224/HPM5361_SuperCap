/*
 * RMS Implementation
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#include "algo_rms.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

static int   algo_rms_init_impl(algo_rms_t *s, const algo_rms_cfg_t *c);
static float algo_rms_step_impl(algo_rms_t *s, float x);
static void  algo_rms_reset_impl(algo_rms_t *s);
static float algo_rms_get_rms_impl(const algo_rms_t *s);
static float algo_rms_get_mean_sq_impl(const algo_rms_t *s);
static float algo_rms_update_sq_impl(algo_rms_t *s, float x);
static void  algo_rms_rebuild_sum_sq(algo_rms_t *s);
static float algo_rms_calc_mean_sq(const algo_rms_t *s);

static float algo_rms_calc_mean_sq(const algo_rms_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;

    float denom = s->_filled ? (float)s->_size : (float)((s->_count > 0U) ? s->_count : 1U);
    float mean_sq = s->_sum_sq / denom;

    if (s->_remove_dc) {
        float mean = s->_sum / denom;
        mean_sq -= mean * mean;
        if (mean_sq < 0.0f) mean_sq = 0.0f;
    }

    return mean_sq;
}

/* ── Init ─────────────────────────────────────────────────────────────── */

static int algo_rms_init_impl(algo_rms_t *s, const algo_rms_cfg_t *c)
{
    if (s != NULL) s->_inited = false;

    if (s == NULL)           return -1;
    if (c == NULL)           return -2;
    if (c->buffer == NULL)   return -3;
    if (c->window_size == 0) return -4;

    s->_buf       = c->buffer;
    s->_size      = c->window_size;
    s->_inv_size  = 1.0f / (float)c->window_size;
    s->_idx       = 0;
    s->_count     = 0;
    s->_sum_sq    = 0.0f;
    s->_sum       = 0.0f;
    s->_y         = 0.0f;
    s->_remove_dc = c->remove_dc;
    s->_filled    = false;
    s->_inited    = true;

    for (uint16_t i = 0; i < s->_size; i++) {
        s->_buf[i] = 0.0f;
    }

    return 0;
}

/* ── Rebuild _sum_sq / _sum from buffer (robustness) ─────────────────── */

static void algo_rms_rebuild_sum_sq(algo_rms_t *s)
{
    s->_sum_sq = 0.0f;
    s->_sum    = 0.0f;

    uint16_t n = s->_filled ? s->_size : s->_count;

    for (uint16_t i = 0; i < n; i++) {
        float v = s->_buf[i];
        if (!algo_rms_finite(v)) {
            s->_buf[i] = 0.0f;
            v = 0.0f;
        }
        float v2 = v * v;
        s->_sum_sq += v2;
        s->_sum    += v;
    }

    if (!algo_rms_finite(s->_sum_sq)) s->_sum_sq = 0.0f;
    if (!algo_rms_finite(s->_sum))    s->_sum    = 0.0f;
}

/* ── update_sq: update buffer & _sum_sq, return mean_sq, no sqrtf ────── */

static float algo_rms_update_sq_impl(algo_rms_t *s, float x)
{
    if (s == NULL || !s->_inited) return 0.0f;

    if (!algo_rms_finite(x)) return algo_rms_calc_mean_sq(s);

    float x2 = x * x;
    if (!algo_rms_finite(x2)) return algo_rms_calc_mean_sq(s);

    float old = s->_buf[s->_idx];
    s->_buf[s->_idx] = x;

    s->_idx++;
    if (s->_idx >= s->_size) s->_idx = 0;

    if (s->_count < s->_size) s->_count++;

    if (s->_filled) {
        float old2 = old * old;
        if (!algo_rms_finite(old2)) old2 = 0.0f;
        s->_sum_sq += x2 - old2;
        s->_sum    += x - old;
    } else {
        s->_sum_sq += x2;
        s->_sum    += x;
        if (s->_count >= s->_size) s->_filled = true;
    }

    if (!algo_rms_finite(s->_sum_sq)) algo_rms_rebuild_sum_sq(s);
    if (s->_sum_sq < 0.0f) s->_sum_sq = 0.0f;

    return algo_rms_calc_mean_sq(s);
}

/* ── step: update + sqrtf, return RMS ────────────────────────────────── */

static float algo_rms_step_impl(algo_rms_t *s, float x)
{
    if (s == NULL || !s->_inited) return 0.0f;

    float mean_sq = algo_rms_update_sq_impl(s, x);
    s->_y = sqrtf(mean_sq);
    return s->_y;
}

/* ── Reset ────────────────────────────────────────────────────────────── */

static void algo_rms_reset_impl(algo_rms_t *s)
{
    if (s == NULL || !s->_inited) return;

    s->_idx    = 0;
    s->_count  = 0;
    s->_sum_sq = 0.0f;
    s->_sum    = 0.0f;
    s->_y      = 0.0f;
    s->_filled = false;

    for (uint16_t i = 0; i < s->_size; i++) {
        s->_buf[i] = 0.0f;
    }
}

/* ── get_rms ──────────────────────────────────────────────────────────── */

static float algo_rms_get_rms_impl(const algo_rms_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;
    return s->_y;
}

/* ── get_mean_sq ──────────────────────────────────────────────────────── */

static float algo_rms_get_mean_sq_impl(const algo_rms_t *s)
{
    if (s == NULL || !s->_inited) return 0.0f;

    return algo_rms_calc_mean_sq(s);
}

/* ── Constructor ──────────────────────────────────────────────────────── */

void algo_rms_ctor(algo_rms_t *s)
{
    if (s == NULL) return;

    s->init        = algo_rms_init_impl;
    s->step        = algo_rms_step_impl;
    s->reset       = algo_rms_reset_impl;
    s->get_rms     = algo_rms_get_rms_impl;
    s->get_mean_sq = algo_rms_get_mean_sq_impl;
    s->update_sq   = algo_rms_update_sq_impl;
    s->_buf        = NULL;
    s->_size       = 0;
    s->_count      = 0;
    s->_inv_size   = 0.0f;
    s->_idx        = 0;
    s->_sum_sq     = 0.0f;
    s->_sum        = 0.0f;
    s->_y          = 0.0f;
    s->_remove_dc  = false;
    s->_filled     = false;
    s->_inited     = false;
}
