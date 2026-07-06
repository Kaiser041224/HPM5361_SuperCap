/*
 * TRGM Interface - C17 Abstract Interface (multi-instance)
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef INTF_TRGM_H
#define INTF_TRGM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Trigger Source Enumeration
 *
 * HPM5361 supports TRGM0/1 instances. Current usage routes PWM compare
 * reference outputs to ADC preemption trigger inputs (PTRGIxA/xB/xC).
 * ============================================================================ */

typedef enum {
    INTF_TRGM_SRC_PWM0_CH8REF  = 0,
    INTF_TRGM_SRC_PWM0_CH9REF,
    INTF_TRGM_SRC_PWM0_CH10REF,
    INTF_TRGM_SRC_PWM0_CH11REF,
    INTF_TRGM_SRC_PWM1_CH8REF,
    INTF_TRGM_SRC_PWM1_CH9REF,
    INTF_TRGM_SRC_PWM1_CH10REF,
    INTF_TRGM_SRC_PWM1_CH11REF,
    INTF_TRGM_SRC_SYNT_CH0,
    INTF_TRGM_SRC_SYNT_CH1,
    INTF_TRGM_SRC_SYNT_CH2,
    INTF_TRGM_SRC_SYNT_CH3,
} intf_trgm_src_t;

/* ============================================================================
 * Trigger Destination Enumeration
 * ============================================================================ */

typedef enum {
    INTF_TRGM_DST_ADC_PTRGI0A = 0,   /* → ADC TRG0A (pmt_trig_ch=0) */
    INTF_TRGM_DST_ADC_PTRGI0B,       /* → ADC TRG0B (pmt_trig_ch=1) */
    INTF_TRGM_DST_ADC_PTRGI0C,       /* → ADC TRG0C (pmt_trig_ch=2) */
    INTF_TRGM_DST_ADC_PTRGI1A,       /* → ADC TRG1A (pmt_trig_ch=3) */
    INTF_TRGM_DST_ADC_PTRGI1B,       /* → ADC TRG1B (pmt_trig_ch=4) */
    INTF_TRGM_DST_ADC_PTRGI1C,       /* → ADC TRG1C (pmt_trig_ch=5) */
    INTF_TRGM_DST_GPTMR0_SYNCI,      /* → GPTMR0 counter sync input */
    INTF_TRGM_DST_GPTMR1_SYNCI,      /* → GPTMR1 counter sync input */
    INTF_TRGM_DST_GPTMR2_SYNCI,      /* → GPTMR2 counter sync input */
    INTF_TRGM_DST_GPTMR3_SYNCI,      /* → GPTMR3 counter sync input */
} intf_trgm_dst_t;

/* ============================================================================
 * Interface Definition (Object-Oriented C17)
 * ============================================================================ */

typedef struct {
    uint8_t instance_id;
    struct {
        int (*connect)(intf_trgm_src_t src, intf_trgm_dst_t dst);
    };
} intf_trgm_t;

/* ============================================================================
 * Registration API
 * ============================================================================ */

int intf_trgm_register(const intf_trgm_t *ops);

/* ============================================================================
 * Functional API (wraps ops)
 * ============================================================================ */

int intf_trgm_connect(intf_trgm_src_t src, intf_trgm_dst_t dst);

#ifdef __cplusplus
}
#endif
#endif /* INTF_TRGM_H */
