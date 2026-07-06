/*
 * RMS — Sliding-Window True RMS / AC RMS
 *
 * True RMS:  y = sqrt( mean(x²) )
 * AC RMS:    y = sqrt( mean(x²) − mean(x)² )
 *
 * O(1) running sum-of-squares per sample.
 *
 * MATLAB:
 *   true_rms = rms(x)
 *   ac_rms   = rms(x - mean(x))
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#ifndef ALGO_RMS_H
#define ALGO_RMS_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct algo_rms algo_rms_t;

typedef struct {
    uint16_t window_size;
    float   *buffer;
    bool     remove_dc;
} algo_rms_cfg_t;

typedef int   (*algo_rms_init_fn)(algo_rms_t *s, const algo_rms_cfg_t *c);
typedef float (*algo_rms_step_fn)(algo_rms_t *s, float x);
typedef void  (*algo_rms_reset_fn)(algo_rms_t *s);
typedef float (*algo_rms_get_rms_fn)(const algo_rms_t *s);
typedef float (*algo_rms_get_mean_sq_fn)(const algo_rms_t *s);
typedef float (*algo_rms_update_sq_fn)(algo_rms_t *s, float x);

struct algo_rms {
    struct {
        algo_rms_init_fn        init;
        algo_rms_step_fn        step;
        algo_rms_reset_fn       reset;
        algo_rms_get_rms_fn     get_rms;
        algo_rms_get_mean_sq_fn get_mean_sq;
        algo_rms_update_sq_fn   update_sq;
    };

    float    *_buf;
    uint16_t  _size;
    uint16_t  _count;
    float     _inv_size;
    uint16_t  _idx;
    float     _sum_sq;
    float     _sum;
    float     _y;
    bool      _remove_dc;
    bool      _filled;
    bool      _inited;
};

void algo_rms_ctor(algo_rms_t *s);

static inline bool algo_rms_finite(float x)
{
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

#ifdef __cplusplus
}
#endif

#endif /* ALGO_RMS_H */
