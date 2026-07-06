/*
 * ADC Interface - C17 Abstract Interface (multi-instance)
 *
 * Copyright (c) 2024 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _INTF_ADC_H
#define _INTF_ADC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Channel Encoding
 *
 * HPM5361 has 2 ADC16 instances (ADC0 / ADC1), each with up to 16 channels.
 * Channel number encoding:
 *   bits [7:4] = instance (0 → ADC0, 1 → ADC1)
 *   bits [3:0] = physical channel index (0–15)
 *
 * Usage: INTF_ADC_CH(0, 3) → ADC0 channel 3
 *        INTF_ADC_CH(1, 7) → ADC1 channel 7
 * ============================================================================ */

typedef uint8_t intf_adc_ch_t;

#define INTF_ADC_CH(inst, idx)  ((intf_adc_ch_t)(((uint8_t)(inst) << 4) | ((uint8_t)(idx) & 0x0FU)))
#define INTF_ADC_CH_INST(ch)    ((uint8_t)((ch) >> 4))
#define INTF_ADC_CH_IDX(ch)     ((uint8_t)((ch) & 0x0FU))
#define INTF_ADC_INSTANCE_COUNT (2U)

/* ============================================================================
 * Types
 * ============================================================================ */

/** @brief ADC resolution options */
typedef enum {
    INTF_ADC_RES_8_BITS = 8,
    INTF_ADC_RES_10_BITS = 10,
    INTF_ADC_RES_12_BITS = 12,
    INTF_ADC_RES_16_BITS = 16,
} intf_adc_resolution_t;

#define INTF_ADC_RES_DEFAULT INTF_ADC_RES_16_BITS

/* Configurable defaults (0 in cfg = use these values) */
#define INTF_ADC_DEFAULT_SAMPLE_CYCLE (25U)
#define INTF_ADC_DEFAULT_CLOCK_DIV    (2U) /* 120/3 = 40 MHz ≤ 50 MHz */
#define INTF_ADC_DEFAULT_VREF_MV      (3300.0f)

/** @brief ADC conversion mode */
typedef enum {
    INTF_ADC_MODE_ONESHOT = 0,
    INTF_ADC_MODE_PERIOD = 1,
    INTF_ADC_MODE_PMT = 2,
    INTF_ADC_MODE_SEQ = 3,
} intf_adc_mode_t;

/** @brief PMT trigger completion callback */
typedef void (*intf_adc_pmt_cb_t)(
    intf_adc_ch_t trig_ch, const uint16_t* values, uint8_t count, void* user_data);

/** @brief Sequence mode completion callback (DMA buffer ready) */
typedef void (*intf_adc_seq_cb_t)(intf_adc_ch_t trig_ch, void* user_data);

/** @brief Watchdog threshold violation callback */
typedef void (*intf_adc_wdog_cb_t)(intf_adc_ch_t ch, uint16_t value, void* user_data);

typedef struct {
    uint32_t irq_entry[INTF_ADC_INSTANCE_COUNT];
    uint32_t generic_entry[INTF_ADC_INSTANCE_COUNT];
    uint32_t pmt_complete[INTF_ADC_INSTANCE_COUNT];
    uint32_t pmt_startup_drop[INTF_ADC_INSTANCE_COUNT];
    uint32_t pmt_callback[INTF_ADC_INSTANCE_COUNT];
    uint32_t pmt_invalid[INTF_ADC_INSTANCE_COUNT];
    uint32_t pmt_invalid_cycle[INTF_ADC_INSTANCE_COUNT];
    uint32_t pmt_invalid_trig[INTF_ADC_INSTANCE_COUNT];
    uint32_t pmt_invalid_channel[INTF_ADC_INSTANCE_COUNT];
    uint32_t adc1_handled_in_adc0_irq;
    uint32_t isr_cycles_max[INTF_ADC_INSTANCE_COUNT];
} intf_adc_diag_snapshot_t;

/** @brief Per-instance ADC configuration */
typedef struct {
    intf_adc_resolution_t resolution;
    intf_adc_mode_t mode;
    uint32_t sample_rate_hz; /**< target sample rate (Period mode), or 0 for default */
    uint32_t sample_cycle;   /**< ADC sample cycles per channel (0 = default 20) */
    uint32_t clock_div;      /**< ADC clock divider 1–16 (0 = auto from sample_rate_hz) */
    float vref_mv;
    /* DMA (applicable to PMT and Sequence modes) */
    bool dma_en;
    uint32_t* dma_buff;
    uint32_t dma_buff_len;
    /* PMT mode */
    uint8_t pmt_trig_ch;
    uint8_t pmt_ch_count;
    uint8_t pmt_ch_list[4];
    intf_adc_pmt_cb_t pmt_cb;
    void* pmt_cb_user_data;
    /* Sequence mode */
    bool seq_hw_trig;
    uint8_t seq_ch_count;
    uint8_t seq_ch_list[16];
    intf_adc_seq_cb_t seq_cb;
    void* seq_cb_user_data;
    /* Watchdog */
    bool wdog_en;
    uint16_t wdog_thshd_high;
    uint16_t wdog_thshd_low;
    intf_adc_wdog_cb_t wdog_cb;
    void* wdog_cb_user_data;
} intf_adc_cfg_t;

/* ============================================================================
 * Interface Definition (Object-Oriented C17)
 * ============================================================================ */

typedef struct {
    uint8_t instance_id;
    struct {
        int (*init)(intf_adc_ch_t ch, const intf_adc_cfg_t* cfg);
        int (*read)(intf_adc_ch_t ch, uint16_t* value);
        int (*read_voltage)(intf_adc_ch_t ch, float* voltage_mv);
        int (*start)(intf_adc_ch_t ch);
        int (*stop)(intf_adc_ch_t ch);
        void (*set_vref)(float vref_mv);
        int (*calibrate)(void);
        void (*deinit)(intf_adc_ch_t ch);
    };
} intf_adc_t;

/* ============================================================================
 * Registration API
 * ============================================================================ */

int intf_adc_register(const intf_adc_t* ops);

/* ============================================================================
 * Functional API (wraps ops)
 * ============================================================================ */

int intf_adc_init(intf_adc_ch_t ch, const intf_adc_cfg_t* cfg);
int intf_adc_read(intf_adc_ch_t ch, uint16_t* value);
int intf_adc_read_voltage(intf_adc_ch_t ch, float* voltage_mv);
int intf_adc_start(intf_adc_ch_t ch);
int intf_adc_stop(intf_adc_ch_t ch);
void intf_adc_set_vref(intf_adc_ch_t ch, float vref_mv);
int intf_adc_calibrate(intf_adc_ch_t ch); /* re-trigger ADC offset calibration */
int intf_adc_get_diag_snapshot(intf_adc_diag_snapshot_t* snapshot);
void intf_adc_reset_diag_max(void);

/* WDOG re-arm: re-enable interrupt for a channel after wdog_cb fired.
 * Required because the ISR auto-disables the channel's WDOG interrupt
 * to avoid flooding. */
void intf_adc_wdog_reenable(intf_adc_ch_t ch);

#ifdef __cplusplus
}
#endif

#endif /* _INTF_ADC_H */
