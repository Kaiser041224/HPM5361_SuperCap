/*
 * GPTMR Interface - Unified General Purpose Timer contract
 *
 * Supports per-channel configuration:
 *   - PWM output (duty, frequency)
 *   - Periodic timer interrupt (callback)
 *   - PWM output + interrupt (combined)
 *   - Input capture (edge detection)
 *   - SYNCI synchronization (SYNT via TRGM)
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef INTF_GPTMR_H
#define INTF_GPTMR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t intf_gptmr_ch_t;

typedef void (*intf_gptmr_irq_callback_t)(void);

typedef enum {
    INTF_GPTMR_MODE_PWM        = 0,  /* PWM output only */
    INTF_GPTMR_MODE_TIMER      = 1,  /* Periodic timer interrupt only */
    INTF_GPTMR_MODE_PWM_TIMER  = 2,  /* PWM output + interrupt */
    INTF_GPTMR_MODE_CAPTURE    = 3,  /* Input capture */
} intf_gptmr_mode_t;

typedef enum {
    INTF_GPTMR_CAPTURE_EDGE_RISING  = 0,
    INTF_GPTMR_CAPTURE_EDGE_FALLING = 1,
    INTF_GPTMR_CAPTURE_EDGE_BOTH    = 2,
} intf_gptmr_capture_edge_t;

typedef struct {
    intf_gptmr_mode_t mode;
    uint32_t frequency_hz;
    float duty;                            /* PWM modes: [0.0, 1.0] */
    bool invert_output;                    /* PWM modes */
    intf_gptmr_capture_edge_t capture_edge; /* CAPTURE mode */
    intf_gptmr_irq_callback_t callback;    /* TIMER / PWM_TIMER modes */
    bool enable_sync;                      /* enable SYNCI (SYNT sync) */
} intf_gptmr_cfg_t;

typedef struct {
    bool captured;
    uint32_t count;
    uint32_t period_ticks;
} intf_gptmr_capture_t;

typedef struct {
    uint8_t instance_id;
    struct {
        int (*init)(intf_gptmr_ch_t ch, const intf_gptmr_cfg_t *cfg);
        int (*start)(intf_gptmr_ch_t ch);
        int (*stop)(intf_gptmr_ch_t ch);
        int (*set_duty)(intf_gptmr_ch_t ch, float duty);
        int (*set_frequency)(intf_gptmr_ch_t ch, uint32_t frequency_hz);
        int (*force_low)(intf_gptmr_ch_t ch);
        int (*force_release)(intf_gptmr_ch_t ch);
        int (*capture_poll)(intf_gptmr_ch_t ch, intf_gptmr_capture_t *capture);
    };
} intf_gptmr_t;

int  intf_gptmr_register(const intf_gptmr_t *ops);
int  intf_gptmr_init(intf_gptmr_ch_t ch, const intf_gptmr_cfg_t *cfg);
int  intf_gptmr_start(intf_gptmr_ch_t ch);
int  intf_gptmr_stop(intf_gptmr_ch_t ch);
int  intf_gptmr_set_duty(intf_gptmr_ch_t ch, float duty);
int  intf_gptmr_set_frequency(intf_gptmr_ch_t ch, uint32_t frequency_hz);
int  intf_gptmr_force_low(intf_gptmr_ch_t ch);
int  intf_gptmr_force_release(intf_gptmr_ch_t ch);
int  intf_gptmr_capture_poll(intf_gptmr_ch_t ch, intf_gptmr_capture_t *capture);

#ifdef __cplusplus
}
#endif

#endif /* INTF_GPTMR_H */
