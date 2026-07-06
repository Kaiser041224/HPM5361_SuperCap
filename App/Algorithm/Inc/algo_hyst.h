/*
 * Schmitt Trigger — Hysteresis Comparator
 *
 * Boundary semantics (strict):
 *     x >  upper  →  state = true
 *     x <  lower  →  state = false
 *     x == upper  →  state unchanged
 *     x == lower  →  state unchanged
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#ifndef ALGO_HYST_H
#define ALGO_HYST_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct algo_hyst algo_hyst_t;

typedef struct {
    float upper;
    float lower;
    bool  init_state;
} algo_hyst_cfg_t;

typedef int  (*algo_hyst_init_fn)(algo_hyst_t *s, const algo_hyst_cfg_t *c);
typedef bool (*algo_hyst_step_fn)(algo_hyst_t *s, float x);
typedef void (*algo_hyst_reset_fn)(algo_hyst_t *s);
typedef bool (*algo_hyst_get_state_fn)(const algo_hyst_t *s);
typedef void (*algo_hyst_set_state_fn)(algo_hyst_t *s, bool state);
typedef int  (*algo_hyst_set_thresholds_fn)(algo_hyst_t *s, float lower, float upper);

struct algo_hyst {
    struct {
        algo_hyst_init_fn           init;
        algo_hyst_step_fn           step;
        algo_hyst_reset_fn          reset;
        algo_hyst_get_state_fn      get_state;
        algo_hyst_set_state_fn      set_state;
        algo_hyst_set_thresholds_fn set_thresholds;
    };

    float _upper;
    float _lower;
    bool  _state;
    bool  _init_state;
    bool  _inited;
};

void algo_hyst_ctor(algo_hyst_t *s);

static inline bool algo_hyst_finite(float x)
{
    uint32_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (bits & 0x7F800000u) != 0x7F800000u;
}

#ifdef __cplusplus
}
#endif

#endif /* ALGO_HYST_H */
