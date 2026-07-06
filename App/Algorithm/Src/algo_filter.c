/*
 * Filter Library Implementation
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#include "algo_filter.h"

#include <stddef.h>

/* ILM deployment for hot ISR paths */
#ifndef ALGO_FILTER_RAMFUNC
#define ALGO_FILTER_RAMFUNC __attribute__((section(".fast")))
#endif

/* ═══════════════════════════════════════════════════════════════════════
 *  Moving Average
 * ═══════════════════════════════════════════════════════════════════════ */

static int algo_ma_init_impl(algo_ma_t *s, const algo_ma_cfg_t *c)
{
    if (s == NULL)           return -1;
    if (c == NULL)           return -2;
    if (c->buffer == NULL)   return -3;
    if (c->window_size == 0) return -4;

    s->_buf      = c->buffer;
    s->_size     = c->window_size;
    s->_inv_size = 1.0f / (float)c->window_size;
    s->_idx      = 0;
    s->_sum      = 0.0f;
    s->_filled   = false;
    s->_inited   = true;

    for (uint16_t i = 0; i < s->_size; i++) {
        s->_buf[i] = 0.0f;
    }

    return 0;
}

ALGO_FILTER_RAMFUNC
static float algo_ma_step_impl(algo_ma_t *s, float x)
{
    return algo_ma_step_fast(s, x);
}

static void algo_ma_reset_impl(algo_ma_t *s)
{
    if (!s->_inited) return;

    s->_idx    = 0;
    s->_sum    = 0.0f;
    s->_filled = false;

    for (uint16_t i = 0; i < s->_size; i++) {
        s->_buf[i] = 0.0f;
    }
}

void algo_ma_ctor(algo_ma_t *s)
{
    if (s == NULL) return;

    s->init      = algo_ma_init_impl;
    s->step      = algo_ma_step_impl;
    s->reset     = algo_ma_reset_impl;
    s->_buf      = NULL;
    s->_size     = 0;
    s->_inv_size = 0.0f;
    s->_idx      = 0;
    s->_sum      = 0.0f;
    s->_filled   = false;
    s->_inited   = false;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  1st-Order Low-Pass
 * ═══════════════════════════════════════════════════════════════════════ */

static int algo_lpf_init_impl(algo_lpf_t *s, const algo_lpf_cfg_t *c)
{
    if (s == NULL)         return -1;
    if (c == NULL)         return -2;

    float fs = c->sample_rate_hz;
    float fc = c->cutoff_hz;

    if (fc <= 0.0f)        return -3;
    if (fs <= 0.0f)        return -4;
    if (fc >= fs * 0.5f)   return -5;
    if (!algo_flt_finite(fc)) return -6;
    if (!algo_flt_finite(fs)) return -7;

    float ts    = 1.0f / fs;
    float rc    = 1.0f / (2.0f * ALGO_PI_F * fc);
    s->_alpha   = ts / (ts + rc);
    s->_y       = 0.0f;
    s->_primed  = false;
    s->_inited  = true;

    return 0;
}

static float algo_lpf_step_impl(algo_lpf_t *s, float x)
{
    return algo_lpf_step_fast(s, x);
}

static void algo_lpf_reset_impl(algo_lpf_t *s)
{
    if (!s->_inited) return;

    s->_y      = 0.0f;
    s->_primed = false;
}

void algo_lpf_ctor(algo_lpf_t *s)
{
    if (s == NULL) return;

    s->init    = algo_lpf_init_impl;
    s->step    = algo_lpf_step_impl;
    s->reset   = algo_lpf_reset_impl;
    s->_alpha  = 0.0f;
    s->_y      = 0.0f;
    s->_primed = false;
    s->_inited = false;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  FIR
 * ═══════════════════════════════════════════════════════════════════════ */

static int algo_fir_init_impl(algo_fir_t *s, const algo_fir_cfg_t *c)
{
    if (s == NULL)          return -1;
    if (c == NULL)          return -2;
    if (c->coeffs == NULL)  return -3;
    if (c->buffer == NULL)  return -4;
    if (c->num_taps == 0)   return -5;

    for (uint16_t i = 0; i < c->num_taps; i++) {
        if (!algo_flt_finite(c->coeffs[i])) return -6;
    }

    s->_coeffs = c->coeffs;
    s->_buf    = c->buffer;
    s->_taps   = c->num_taps;
    s->_idx    = 0;
    s->_y      = 0.0f;
    s->_inited = true;

    for (uint16_t i = 0; i < s->_taps; i++) {
        s->_buf[i] = 0.0f;
    }

    return 0;
}

static float algo_fir_step_impl(algo_fir_t *s, float x)
{
    if (!s->_inited) return 0.0f;

    if (!algo_flt_finite(x)) return s->_y;

    s->_buf[s->_idx] = x;

    float    sum = 0.0f;
    uint16_t r   = s->_idx;

    for (uint16_t k = 0; k < s->_taps; k++) {
        sum += s->_coeffs[k] * s->_buf[r];
        if (r == 0) {
            r = s->_taps - 1U;
        } else {
            r--;
        }
    }

    s->_idx++;
    if (s->_idx >= s->_taps) s->_idx = 0;

    s->_y = sum;
    return sum;
}

static void algo_fir_reset_impl(algo_fir_t *s)
{
    if (!s->_inited) return;

    s->_idx = 0;
    s->_y   = 0.0f;

    for (uint16_t i = 0; i < s->_taps; i++) {
        s->_buf[i] = 0.0f;
    }
}

void algo_fir_ctor(algo_fir_t *s)
{
    if (s == NULL) return;

    s->init    = algo_fir_init_impl;
    s->step    = algo_fir_step_impl;
    s->reset   = algo_fir_reset_impl;
    s->_coeffs = NULL;
    s->_buf    = NULL;
    s->_taps   = 0;
    s->_idx    = 0;
    s->_y      = 0.0f;
    s->_inited = false;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Biquad  (Direct Form II Transposed, a0 = 1)
 * ═══════════════════════════════════════════════════════════════════════ */

static int algo_biquad_init_impl(algo_biquad_t *s, const algo_biquad_cfg_t *c)
{
    if (s == NULL) return -1;
    if (c == NULL) return -2;

    float b0 = c->coeffs.b0, b1 = c->coeffs.b1, b2 = c->coeffs.b2;
    float a1 = c->coeffs.a1, a2 = c->coeffs.a2;

    if (!algo_flt_finite(b0)) return -3;
    if (!algo_flt_finite(b1)) return -4;
    if (!algo_flt_finite(b2)) return -5;
    if (!algo_flt_finite(a1)) return -6;
    if (!algo_flt_finite(a2)) return -7;

    s->_b0     = b0;
    s->_b1     = b1;
    s->_b2     = b2;
    s->_a1     = a1;
    s->_a2     = a2;
    s->_z1     = 0.0f;
    s->_z2     = 0.0f;
    s->_y      = 0.0f;
    s->_inited = true;

    return 0;
}

static float algo_biquad_step_impl(algo_biquad_t *s, float x)
{
    return algo_biquad_step_fast(s, x);
}

static void algo_biquad_reset_impl(algo_biquad_t *s)
{
    if (!s->_inited) return;

    s->_z1 = 0.0f;
    s->_z2 = 0.0f;
    s->_y  = 0.0f;
}

void algo_biquad_ctor(algo_biquad_t *s)
{
    if (s == NULL) return;

    s->init    = algo_biquad_init_impl;
    s->step    = algo_biquad_step_impl;
    s->reset   = algo_biquad_reset_impl;
    s->_b0     = 0.0f;
    s->_b1     = 0.0f;
    s->_b2     = 0.0f;
    s->_a1     = 0.0f;
    s->_a2     = 0.0f;
    s->_z1     = 0.0f;
    s->_z2     = 0.0f;
    s->_y      = 0.0f;
    s->_inited = false;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  Median
 * ═══════════════════════════════════════════════════════════════════════ */

static int algo_med_init_impl(algo_med_t *s, const algo_med_cfg_t *c)
{
    if (s != NULL) s->_inited = false;

    if (s == NULL)           return -1;
    if (c == NULL)           return -2;
    if (c->buffer == NULL)   return -3;
    if (c->sort_buf == NULL) return -4;
    if (c->window_size == 0) return -5;
    if (c->window_size < 3)  return -6;

    s->_buf   = c->buffer;
    s->_sort  = c->sort_buf;
    s->_size  = c->window_size;
    s->_idx   = 0;
    s->_count = 0;
    s->_y     = 0.0f;
    s->_inited = true;

    for (uint16_t i = 0; i < s->_size; i++) {
        s->_buf[i]  = 0.0f;
        s->_sort[i] = 0.0f;
    }

    return 0;
}

static float algo_med_step_impl(algo_med_t *s, float x)
{
    if (s == NULL || !s->_inited) return 0.0f;

    if (!algo_flt_finite(x)) return s->_y;

    s->_buf[s->_idx] = x;

    s->_idx++;
    if (s->_idx >= s->_size) s->_idx = 0;

    if (s->_count < s->_size) s->_count++;

    uint16_t n = s->_count;

    for (uint16_t i = 0; i < n; i++) {
        s->_sort[i] = s->_buf[i];
    }

    for (uint16_t i = 1; i < n; i++) {
        float key = s->_sort[i];
        uint16_t j = i;
        while (j > 0 && s->_sort[j - 1] > key) {
            s->_sort[j] = s->_sort[j - 1];
            j--;
        }
        s->_sort[j] = key;
    }

    if (n & 1U) {
        s->_y = s->_sort[n / 2U];
    } else {
        s->_y = 0.5f * (s->_sort[n / 2U] + s->_sort[n / 2U - 1U]);
    }

    return s->_y;
}

static void algo_med_reset_impl(algo_med_t *s)
{
    if (s == NULL || !s->_inited) return;

    s->_idx   = 0;
    s->_count = 0;
    s->_y     = 0.0f;

    for (uint16_t i = 0; i < s->_size; i++) {
        s->_buf[i]  = 0.0f;
        s->_sort[i] = 0.0f;
    }
}

void algo_med_ctor(algo_med_t *s)
{
    if (s == NULL) return;

    s->init   = algo_med_init_impl;
    s->step   = algo_med_step_impl;
    s->reset  = algo_med_reset_impl;
    s->_buf   = NULL;
    s->_sort  = NULL;
    s->_size  = 0;
    s->_idx   = 0;
    s->_count = 0;
    s->_y     = 0.0f;
    s->_inited = false;
}

