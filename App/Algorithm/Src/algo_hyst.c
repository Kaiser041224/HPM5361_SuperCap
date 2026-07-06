/*
 * Schmitt Trigger Implementation
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#include "algo_hyst.h"

#include <stddef.h>

static int  algo_hyst_init_impl(algo_hyst_t *s, const algo_hyst_cfg_t *c);
static bool algo_hyst_step_impl(algo_hyst_t *s, float x);
static void algo_hyst_reset_impl(algo_hyst_t *s);
static bool algo_hyst_get_state_impl(const algo_hyst_t *s);
static void algo_hyst_set_state_impl(algo_hyst_t *s, bool state);
static int  algo_hyst_set_thresholds_impl(algo_hyst_t *s, float lower, float upper);

/* ── Init ─────────────────────────────────────────────────────────────── */

static int algo_hyst_init_impl(algo_hyst_t *s, const algo_hyst_cfg_t *c)
{
    if (s != NULL) s->_inited = false;

    if (s == NULL)            return -1;
    if (c == NULL)            return -2;
    if (!algo_hyst_finite(c->upper)) return -3;
    if (!algo_hyst_finite(c->lower)) return -4;
    if (c->upper <= c->lower)        return -5;

    s->_upper      = c->upper;
    s->_lower      = c->lower;
    s->_state      = c->init_state;
    s->_init_state = c->init_state;
    s->_inited     = true;

    return 0;
}

/* ── Step ─────────────────────────────────────────────────────────────── */

static bool algo_hyst_step_impl(algo_hyst_t *s, float x)
{
    if (s == NULL || !s->_inited) return false;

    if (!algo_hyst_finite(x)) return s->_state;

    if (x > s->_upper)       s->_state = true;
    else if (x < s->_lower)  s->_state = false;

    return s->_state;
}

/* ── Reset ────────────────────────────────────────────────────────────── */

static void algo_hyst_reset_impl(algo_hyst_t *s)
{
    if (s == NULL || !s->_inited) return;
    s->_state = s->_init_state;
}

/* ── get_state ────────────────────────────────────────────────────────── */

static bool algo_hyst_get_state_impl(const algo_hyst_t *s)
{
    if (s == NULL || !s->_inited) return false;
    return s->_state;
}

/* ── set_state ────────────────────────────────────────────────────────── */

static void algo_hyst_set_state_impl(algo_hyst_t *s, bool state)
{
    if (s == NULL || !s->_inited) return;
    s->_state = state;
}

/* ── set_thresholds ───────────────────────────────────────────────────── */

static int algo_hyst_set_thresholds_impl(algo_hyst_t *s, float lower, float upper)
{
    if (s == NULL || !s->_inited) return -1;
    if (!algo_hyst_finite(lower)) return -2;
    if (!algo_hyst_finite(upper)) return -3;
    if (upper <= lower)           return -4;

    s->_lower = lower;
    s->_upper = upper;
    return 0;
}

/* ── Constructor ──────────────────────────────────────────────────────── */

void algo_hyst_ctor(algo_hyst_t *s)
{
    if (s == NULL) return;

    s->init           = algo_hyst_init_impl;
    s->step           = algo_hyst_step_impl;
    s->reset          = algo_hyst_reset_impl;
    s->get_state      = algo_hyst_get_state_impl;
    s->set_state      = algo_hyst_set_state_impl;
    s->set_thresholds = algo_hyst_set_thresholds_impl;
    s->_upper         = 0.0f;
    s->_lower         = 0.0f;
    s->_state         = false;
    s->_init_state    = false;
    s->_inited        = false;
}
