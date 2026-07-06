/*
 * Feedforward — Linear / Lookup-Table
 *
 * Linear mode:   u = gain_sp × sp + gain_dv × dv + offset
 * Table mode:    u = interp1d(x_tbl, y_tbl, sp) + gain_dv × dv + offset
 *
 * MATLAB table generation:
 *   x_tbl = [10, 15, 20, 25, 30];           % breakpoints
 *   y_tbl = [0.48, 0.32, 0.24, 0.19, 0.16]; % measured feedforward
 *   → const float x[] = {10,15,20,25,30};
 *   → const float y[] = {0.48,0.32,0.24,0.19,0.16};
 *   → algo_ffd_cfg_t .x_tbl=x .y_tbl=y .n_pts=5
 *
 * Usage with PID:
 *   duty_ff = ffd.step(&ffd, v_in_meas, 0);
 *   duty_pid = pid.step(&pid, v_link_target, v_link_meas);
 *   duty = duty_pid + duty_ff;
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#ifndef ALGO_FFD_H
#define ALGO_FFD_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "algo_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ALGO_FFD_MODE_LINEAR = 0,
    ALGO_FFD_MODE_TABLE  = 1,
} algo_ffd_mode_t;

typedef struct algo_ffd algo_ffd_t;

typedef struct {
    algo_ffd_mode_t mode;
    float           gain_sp;
    float           gain_dv;
    float           offset;
    const float    *x_tbl;
    const float    *y_tbl;
    uint16_t        n_pts;
} algo_ffd_cfg_t;

typedef int   (*algo_ffd_init_fn)(algo_ffd_t *s, const algo_ffd_cfg_t *c);
typedef float (*algo_ffd_step_fn)(algo_ffd_t *s, float sp, float dv);
typedef void  (*algo_ffd_reset_fn)(algo_ffd_t *s);

struct algo_ffd {
    struct {
        algo_ffd_init_fn  init;
        algo_ffd_step_fn  step;
        algo_ffd_reset_fn reset;
    };

    algo_ffd_mode_t _mode;
    float           _gain_sp;
    float           _gain_dv;
    float           _offset;
    const float    *_x_tbl;
    const float    *_y_tbl;
    uint16_t        _n_pts;
    float           _y;
    bool            _inited;
};

void algo_ffd_ctor(algo_ffd_t *s);

static inline bool algo_ffd_finite(float x)
{
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

/* ════════════════════════════════════════════════════════════════════════
 *  PID + Feedforward  —  Unified Controller
 *
 *     u = pid(sp, pv) + ffd(sp, dv)
 *
 *  ctor / init / step / reset 一站式调用，内部持有 pid + ffd 两个对象。
 *  通过 get_pid() / get_ffd() 可访问各自方法（set_gains, set_target 等）。
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct algo_pid_ffd algo_pid_ffd_t;

typedef struct {
    algo_ffd_cfg_t ffd;
    algo_pid_cfg_t pid;
} algo_pid_ffd_cfg_t;

typedef int   (*algo_pid_ffd_init_fn)(algo_pid_ffd_t *s, const algo_pid_ffd_cfg_t *c);
typedef float (*algo_pid_ffd_step_fn)(algo_pid_ffd_t *s, float sp, float pv, float dv);
typedef void  (*algo_pid_ffd_reset_fn)(algo_pid_ffd_t *s);

struct algo_pid_ffd {
    struct {
        algo_pid_ffd_init_fn  init;
        algo_pid_ffd_step_fn  step;
        algo_pid_ffd_reset_fn reset;
    };

    algo_pid_t _pid;
    algo_ffd_t _ffd;
    bool       _inited;
};

void algo_pid_ffd_ctor(algo_pid_ffd_t *s);

algo_pid_t *algo_pid_ffd_get_pid(algo_pid_ffd_t *s);
algo_ffd_t *algo_pid_ffd_get_ffd(algo_pid_ffd_t *s);

#ifdef __cplusplus
}
#endif

#endif /* ALGO_FFD_H */
