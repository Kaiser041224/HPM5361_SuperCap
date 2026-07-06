/*
 * Feedforward Implementation
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#include "algo_ffd.h"

#include <stddef.h>
#include <stdint.h>

static int   algo_ffd_init_impl(algo_ffd_t *s, const algo_ffd_cfg_t *c);
static float algo_ffd_step_impl(algo_ffd_t *s, float sp, float dv);
static void  algo_ffd_reset_impl(algo_ffd_t *s);

static float algo_ffd_interp1d(const float *x_tbl, const float *y_tbl,
                               uint16_t n, float x);

/* ── Init ─────────────────────────────────────────────────────────────── */

static int algo_ffd_init_impl(algo_ffd_t *s, const algo_ffd_cfg_t *c)
{
    if (s != NULL) s->_inited = false;

    if (s == NULL)  return -1;
    if (c == NULL)  return -2;

    /* shared checks — both modes use all gains */
    if (!algo_ffd_finite(c->gain_sp)) return -3;
    if (!algo_ffd_finite(c->gain_dv)) return -4;
    if (!algo_ffd_finite(c->offset))  return -5;

    switch (c->mode) {
    case ALGO_FFD_MODE_LINEAR:
        break;

    case ALGO_FFD_MODE_TABLE:
        if (c->x_tbl == NULL || c->y_tbl == NULL) return -6;
        if (c->n_pts < 2)                         return -7;
        for (uint16_t i = 0; i < c->n_pts; i++) {
            if (!algo_ffd_finite(c->x_tbl[i])) return -8;
            if (!algo_ffd_finite(c->y_tbl[i])) return -9;
        }
        for (uint16_t i = 1; i < c->n_pts; i++) {
            if (c->x_tbl[i] <= c->x_tbl[i - 1]) return -10;
        }
        break;

    default:
        return -11;
    }

    s->_mode    = c->mode;
    s->_gain_sp = c->gain_sp;
    s->_gain_dv = c->gain_dv;
    s->_offset  = c->offset;
    s->_x_tbl   = c->x_tbl;
    s->_y_tbl   = c->y_tbl;
    s->_n_pts   = c->n_pts;
    s->_y       = 0.0f;
    s->_inited  = true;

    return 0;
}

/* ── Step ─────────────────────────────────────────────────────────────── */

static float algo_ffd_step_impl(algo_ffd_t *s, float sp, float dv)
{
    if (s == NULL || !s->_inited) return 0.0f;

    if (!algo_ffd_finite(sp)) sp = 0.0f;
    if (!algo_ffd_finite(dv)) dv = 0.0f;

    float u = s->_offset + s->_gain_dv * dv;

    if (s->_mode == ALGO_FFD_MODE_TABLE) {
        u += algo_ffd_interp1d(s->_x_tbl, s->_y_tbl, s->_n_pts, sp);
    } else {
        u += s->_gain_sp * sp;
    }

    if (!algo_ffd_finite(u)) return s->_y;

    s->_y = u;
    return u;
}

/* ── Reset ────────────────────────────────────────────────────────────── */

static void algo_ffd_reset_impl(algo_ffd_t *s)
{
    if (s == NULL || !s->_inited) return;
    s->_y = 0.0f;
}

/* ── interp1d (binary search) ────────────────────────────────────────── */

static float algo_ffd_interp1d(const float *x_tbl, const float *y_tbl,
                               uint16_t n, float x)
{
    if (x_tbl == NULL || y_tbl == NULL || n < 2 || !algo_ffd_finite(x)) {
        return 0.0f;
    }

    if (x <= x_tbl[0])   return y_tbl[0];
    if (x >= x_tbl[n - 1]) return y_tbl[n - 1];

    uint16_t lo = 0;
    uint16_t hi = n - 1;

    while (hi - lo > 1) {
        uint16_t mid = (lo + hi) >> 1;
        if (x < x_tbl[mid]) {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    float t = (x - x_tbl[lo]) / (x_tbl[hi] - x_tbl[lo]);
    return y_tbl[lo] + t * (y_tbl[hi] - y_tbl[lo]);
}

/* ── Constructor ──────────────────────────────────────────────────────── */

void algo_ffd_ctor(algo_ffd_t *s)
{
    if (s == NULL) return;

    s->init     = algo_ffd_init_impl;
    s->step     = algo_ffd_step_impl;
    s->reset    = algo_ffd_reset_impl;
    s->_mode    = ALGO_FFD_MODE_LINEAR;
    s->_gain_sp = 0.0f;
    s->_gain_dv = 0.0f;
    s->_offset  = 0.0f;
    s->_x_tbl   = NULL;
    s->_y_tbl   = NULL;
    s->_n_pts   = 0;
    s->_y       = 0.0f;
    s->_inited  = false;
}

/* ═══════════════════════════════════════════════════════════════════════
 *  PID + Feedforward  —  Unified Controller
 * ═══════════════════════════════════════════════════════════════════════ */

static int   algo_pid_ffd_init_impl(algo_pid_ffd_t *s, const algo_pid_ffd_cfg_t *c);
static float algo_pid_ffd_step_impl(algo_pid_ffd_t *s, float sp, float pv, float dv);
static void  algo_pid_ffd_reset_impl(algo_pid_ffd_t *s);

static int algo_pid_ffd_init_impl(algo_pid_ffd_t *s, const algo_pid_ffd_cfg_t *c)
{
    if (s != NULL) s->_inited = false;
    if (s == NULL) return -1;
    if (c == NULL) return -2;

    int r = s->_ffd.init(&s->_ffd, &c->ffd);
    if (r < 0) return r;

    r = s->_pid.init(&s->_pid, &c->pid);
    if (r < 0) return r - 100;

    s->_inited = true;
    return 0;
}

static float algo_pid_ffd_step_impl(algo_pid_ffd_t *s, float sp, float pv, float dv)
{
    if (s == NULL || !s->_inited) return 0.0f;

    float u_pid = s->_pid.step(&s->_pid, sp, pv);
    float u_ffd = s->_ffd.step(&s->_ffd, sp, dv);

    if (!algo_ffd_finite(u_pid)) u_pid = 0.0f;
    if (!algo_ffd_finite(u_ffd)) u_ffd = 0.0f;

    float u = u_pid + u_ffd;
    if (!algo_ffd_finite(u)) return u_pid;
    return u;
}

static void algo_pid_ffd_reset_impl(algo_pid_ffd_t *s)
{
    if (s == NULL || !s->_inited) return;

    s->_pid.reset(&s->_pid);
    s->_ffd.reset(&s->_ffd);
}

void algo_pid_ffd_ctor(algo_pid_ffd_t *s)
{
    if (s == NULL) return;

    s->init    = algo_pid_ffd_init_impl;
    s->step    = algo_pid_ffd_step_impl;
    s->reset   = algo_pid_ffd_reset_impl;
    s->_inited = false;

    algo_pid_ctor(&s->_pid);
    algo_ffd_ctor(&s->_ffd);
}

algo_pid_t *algo_pid_ffd_get_pid(algo_pid_ffd_t *s)
{
    return s ? &s->_pid : NULL;
}

algo_ffd_t *algo_pid_ffd_get_ffd(algo_pid_ffd_t *s)
{
    return s ? &s->_ffd : NULL;
}
