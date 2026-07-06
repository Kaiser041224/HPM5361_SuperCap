/*
 * GPTMR Driver - Unified General Purpose Timer implementation
 *
 * Supports GPTMR0~3, per-channel modes:
 *   PWM output, timer interrupt, PWM+interrupt, input capture.
 *   SYNCI support for hardware synchronization via SYNT + TRGM.
 *
 * Channel encoding: ch = inst * 4 + local_ch
 *   0-3   = GPTMR0 CH0-CH3
 *   4-7   = GPTMR1 CH0-CH3
 *   8-11  = GPTMR2 CH0-CH3
 *   12-15 = GPTMR3 CH0-CH3
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "intf_gptmr.h"

#include "hpm_gptmr_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_interrupt.h"
#include "hpm_soc.h"
#include "hpm_soc_irq.h"

#include <stddef.h>

#define GPTMR_INSTANCE_COUNT    4U
#define GPTMR_CHANNELS_PER_INST 4U
#define GPTMR_TOTAL_CHANNELS    (GPTMR_INSTANCE_COUNT * GPTMR_CHANNELS_PER_INST)

typedef struct {
    bool configured;
    intf_gptmr_mode_t mode;
    uint32_t frequency_hz;
    float duty;
    bool invert_output;
    uint32_t reload;
    intf_gptmr_irq_callback_t callback;
} gptmr_ch_state_t;

static GPTMR_Type * const gptmr_bases[GPTMR_INSTANCE_COUNT] = {
    HPM_GPTMR0,
    HPM_GPTMR1,
    HPM_GPTMR2,
    HPM_GPTMR3,
};

static const clock_name_t gptmr_clocks[GPTMR_INSTANCE_COUNT] = {
    clock_gptmr0,
    clock_gptmr1,
    clock_gptmr2,
    clock_gptmr3,
};

static const uint32_t gptmr_irq_nums[GPTMR_INSTANCE_COUNT] = {
    IRQn_GPTMR0,
    IRQn_GPTMR1,
    IRQn_GPTMR2,
    IRQn_GPTMR3,
};

ATTR_PLACE_AT_FAST_RAM_BSS static gptmr_ch_state_t gptmr_state[GPTMR_TOTAL_CHANNELS];

static bool gptmr_ch_is_valid(intf_gptmr_ch_t ch)
{
    return ch < GPTMR_TOTAL_CHANNELS;
}

static uint8_t gptmr_inst_from_ch(intf_gptmr_ch_t ch)
{
    return ch / GPTMR_CHANNELS_PER_INST;
}

static uint8_t gptmr_local_ch(intf_gptmr_ch_t ch)
{
    return ch % GPTMR_CHANNELS_PER_INST;
}

static GPTMR_Type *gptmr_get_base(intf_gptmr_ch_t ch)
{
    return gptmr_bases[gptmr_inst_from_ch(ch)];
}

static bool gptmr_is_valid_duty(float duty)
{
    return (duty == duty) && (duty >= 0.0f) && (duty <= 1.0f);
}

static int gptmr_apply_duty(intf_gptmr_ch_t ch, float duty)
{
    uint8_t lch = gptmr_local_ch(ch);
    GPTMR_Type *base = gptmr_get_base(ch);

    if (!gptmr_is_valid_duty(duty)) {
        return -1;
    }

    uint32_t cmp = (uint32_t)((float)gptmr_state[ch].reload * duty);
    gptmr_state[ch].duty = duty;

    gptmr_update_cmp(base, lch, 0, cmp);
    gptmr_update_cmp(base, lch, 1, gptmr_state[ch].reload);
    return 0;
}

static gptmr_work_mode_t gptmr_get_capture_mode(intf_gptmr_capture_edge_t edge)
{
    switch (edge) {
    case INTF_GPTMR_CAPTURE_EDGE_RISING:
        return gptmr_work_mode_capture_at_rising_edge;
    case INTF_GPTMR_CAPTURE_EDGE_FALLING:
        return gptmr_work_mode_capture_at_falling_edge;
    case INTF_GPTMR_CAPTURE_EDGE_BOTH:
        return gptmr_work_mode_capture_at_both_edge;
    default:
        return gptmr_work_mode_capture_at_rising_edge;
    }
}

static gptmr_counter_type_t gptmr_get_capture_counter_type(intf_gptmr_capture_edge_t edge)
{
    switch (edge) {
    case INTF_GPTMR_CAPTURE_EDGE_FALLING:
        return gptmr_counter_type_falling_edge;
    case INTF_GPTMR_CAPTURE_EDGE_RISING:
    case INTF_GPTMR_CAPTURE_EDGE_BOTH:
    default:
        return gptmr_counter_type_rising_edge;
    }
}

static uint32_t gptmr_calc_delta(uint32_t first, uint32_t next)
{
    return (next >= first) ? (next - first) : ((UINT32_MAX - first) + next + 1U);
}

static void gptmr_isr_dispatch_instance(uint8_t inst)
{
    GPTMR_Type *base = gptmr_bases[inst];
    uint8_t ch_base = inst * GPTMR_CHANNELS_PER_INST;

    for (uint8_t i = 0; i < GPTMR_CHANNELS_PER_INST; i++) {
        intf_gptmr_ch_t ch = ch_base + i;
        if (!gptmr_state[ch].configured) {
            continue;
        }
        if (gptmr_state[ch].mode != INTF_GPTMR_MODE_TIMER
            && gptmr_state[ch].mode != INTF_GPTMR_MODE_PWM_TIMER) {
            continue;
        }

        uint32_t rld_mask = GPTMR_CH_RLD_STAT_MASK(i);
        uint32_t status = gptmr_get_status(base);
        if (status & rld_mask) {
            gptmr_clear_status(base, rld_mask);
            /* Ensure W1C clear reaches the peripheral before ISR return. */
            (void)gptmr_get_status(base);
            if (gptmr_state[ch].callback != NULL) {
                gptmr_state[ch].callback();
            }
        }
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_GPTMR0, isr_gptmr0)
void isr_gptmr0(void) { gptmr_isr_dispatch_instance(0); }

SDK_DECLARE_EXT_ISR_M(IRQn_GPTMR1, isr_gptmr1)
void isr_gptmr1(void) { gptmr_isr_dispatch_instance(1); }

SDK_DECLARE_EXT_ISR_M(IRQn_GPTMR2, isr_gptmr2)
void isr_gptmr2(void) { gptmr_isr_dispatch_instance(2); }

SDK_DECLARE_EXT_ISR_M(IRQn_GPTMR3, isr_gptmr3)
void isr_gptmr3(void) { gptmr_isr_dispatch_instance(3); }

static int gptmr_drv_init(intf_gptmr_ch_t ch, const intf_gptmr_cfg_t *cfg)
{
    gptmr_channel_config_t hw_cfg;

    if (cfg == NULL || !gptmr_ch_is_valid(ch)) {
        return -1;
    }
    if (cfg->frequency_hz == 0U) {
        return -1;
    }

    uint8_t inst = gptmr_inst_from_ch(ch);
    uint8_t lch = gptmr_local_ch(ch);
    GPTMR_Type *base = gptmr_get_base(ch);

    clock_add_to_group(gptmr_clocks[inst], 0);
    uint32_t clock_hz = clock_get_frequency(gptmr_clocks[inst]);
    if (clock_hz <= cfg->frequency_hz) {
        return -1;
    }

    gptmr_stop_counter(base, lch);
    gptmr_disable_irq(base, GPTMR_CH_RLD_IRQ_MASK(lch));
    gptmr_disable_irq(base, GPTMR_CH_CMP_IRQ_MASK(lch, 0));
    gptmr_disable_irq(base, GPTMR_CH_CMP_IRQ_MASK(lch, 1));
    gptmr_clear_status(base, GPTMR_CH_RLD_STAT_MASK(lch));
    gptmr_clear_status(base, GPTMR_CH_CMP_STAT_MASK(lch, 0));
    gptmr_clear_status(base, GPTMR_CH_CMP_STAT_MASK(lch, 1));

    gptmr_channel_get_default_config(base, &hw_cfg);

    uint32_t reload = clock_hz / cfg->frequency_hz;
    hw_cfg.reload = reload;
    hw_cfg.enable_software_sync = cfg->enable_sync;
    hw_cfg.enable_sync_follow_previous_channel = false;
    hw_cfg.synci_edge = gptmr_synci_edge_none;

    switch (cfg->mode) {
    case INTF_GPTMR_MODE_PWM:
        if (!gptmr_is_valid_duty(cfg->duty)) {
            return -1;
        }
        hw_cfg.mode = gptmr_work_mode_no_capture;
        hw_cfg.enable_cmp_output = true;
        hw_cfg.cmp_initial_polarity_high = cfg->invert_output;
        if (gptmr_channel_config(base, lch, &hw_cfg, false) != status_success) {
            return -1;
        }
        gptmr_channel_reset_count(base, lch);
        gptmr_state[ch].duty = cfg->duty;
        gptmr_state[ch].invert_output = cfg->invert_output;
        gptmr_apply_duty(ch, cfg->duty);
        break;

    case INTF_GPTMR_MODE_TIMER:
        if (cfg->callback == NULL) {
            return -1;
        }
        hw_cfg.mode = gptmr_work_mode_no_capture;
        hw_cfg.enable_cmp_output = false;
        if (gptmr_channel_config(base, lch, &hw_cfg, false) != status_success) {
            return -1;
        }
        gptmr_channel_reset_count(base, lch);
        gptmr_enable_irq(base, GPTMR_CH_RLD_IRQ_MASK(lch));
        intc_m_enable_irq_with_priority(gptmr_irq_nums[inst], 3);
        break;

    case INTF_GPTMR_MODE_PWM_TIMER:
        if (!gptmr_is_valid_duty(cfg->duty) || cfg->callback == NULL) {
            return -1;
        }
        hw_cfg.mode = gptmr_work_mode_no_capture;
        hw_cfg.enable_cmp_output = true;
        hw_cfg.cmp_initial_polarity_high = cfg->invert_output;
        if (gptmr_channel_config(base, lch, &hw_cfg, false) != status_success) {
            return -1;
        }
        gptmr_channel_reset_count(base, lch);
        gptmr_state[ch].duty = cfg->duty;
        gptmr_state[ch].invert_output = cfg->invert_output;
        gptmr_apply_duty(ch, cfg->duty);
        gptmr_enable_irq(base, GPTMR_CH_RLD_IRQ_MASK(lch));
        intc_m_enable_irq_with_priority(gptmr_irq_nums[inst], 3);
        break;

    case INTF_GPTMR_MODE_CAPTURE:
        if (cfg->capture_edge > INTF_GPTMR_CAPTURE_EDGE_BOTH) {
            return -1;
        }
        hw_cfg.mode = gptmr_get_capture_mode(cfg->capture_edge);
        hw_cfg.enable_cmp_output = false;
        if (gptmr_channel_config(base, lch, &hw_cfg, false) != status_success) {
            return -1;
        }
        gptmr_channel_reset_count(base, lch);
        break;

    default:
        return -1;
    }

    gptmr_state[ch].configured = true;
    gptmr_state[ch].mode = cfg->mode;
    gptmr_state[ch].frequency_hz = cfg->frequency_hz;
    gptmr_state[ch].reload = reload;
    gptmr_state[ch].callback = cfg->callback;
    return 0;
}

static int gptmr_drv_start(intf_gptmr_ch_t ch)
{
    if (!gptmr_ch_is_valid(ch) || !gptmr_state[ch].configured) {
        return -1;
    }

    uint8_t lch = gptmr_local_ch(ch);
    GPTMR_Type *base = gptmr_get_base(ch);

    gptmr_stop_counter(base, lch);
    gptmr_channel_reset_count(base, lch);
    gptmr_clear_status(base, GPTMR_CH_RLD_STAT_MASK(lch));

    if (gptmr_state[ch].mode == INTF_GPTMR_MODE_PWM
        || gptmr_state[ch].mode == INTF_GPTMR_MODE_PWM_TIMER) {
        gptmr_enable_cmp_output(base, lch);
    }

    gptmr_start_counter(base, lch);
    return 0;
}

static int gptmr_drv_stop(intf_gptmr_ch_t ch)
{
    if (!gptmr_ch_is_valid(ch)) {
        return -1;
    }

    uint8_t lch = gptmr_local_ch(ch);
    GPTMR_Type *base = gptmr_get_base(ch);

    gptmr_disable_cmp_output(base, lch);
    gptmr_stop_counter(base, lch);
    return 0;
}

static int gptmr_drv_set_duty(intf_gptmr_ch_t ch, float duty)
{
    if (!gptmr_ch_is_valid(ch) || !gptmr_state[ch].configured) {
        return -1;
    }
    if (gptmr_state[ch].mode != INTF_GPTMR_MODE_PWM
        && gptmr_state[ch].mode != INTF_GPTMR_MODE_PWM_TIMER) {
        return -1;
    }
    return gptmr_apply_duty(ch, duty);
}

static int gptmr_drv_set_frequency(intf_gptmr_ch_t ch, uint32_t frequency_hz)
{
    if (!gptmr_ch_is_valid(ch) || !gptmr_state[ch].configured) {
        return -1;
    }
    if (frequency_hz == 0U) {
        return -1;
    }

    uint8_t inst = gptmr_inst_from_ch(ch);
    uint8_t lch = gptmr_local_ch(ch);
    GPTMR_Type *base = gptmr_get_base(ch);

    clock_add_to_group(gptmr_clocks[inst], 0);
    uint32_t clock_hz = clock_get_frequency(gptmr_clocks[inst]);
    if (clock_hz <= frequency_hz) {
        return -1;
    }

    gptmr_state[ch].reload = clock_hz / frequency_hz;
    gptmr_state[ch].frequency_hz = frequency_hz;

    gptmr_stop_counter(base, lch);
    gptmr_channel_config_update_reload(base, lch, gptmr_state[ch].reload);
    gptmr_channel_reset_count(base, lch);

    if (gptmr_state[ch].mode == INTF_GPTMR_MODE_PWM
        || gptmr_state[ch].mode == INTF_GPTMR_MODE_PWM_TIMER) {
        gptmr_apply_duty(ch, gptmr_state[ch].duty);
    }

    gptmr_start_counter(base, lch);
    return 0;
}

static int gptmr_drv_force_low(intf_gptmr_ch_t ch)
{
    if (!gptmr_ch_is_valid(ch) || !gptmr_state[ch].configured) {
        return -1;
    }

    uint8_t lch = gptmr_local_ch(ch);
    GPTMR_Type *base = gptmr_get_base(ch);

    gptmr_stop_counter(base, lch);
    gptmr_update_cmp(base, lch, 0, 0xFFFFFFFFU);
    gptmr_update_cmp(base, lch, 1, 0xFFFFFFFFU);
    gptmr_enable_cmp_output(base, lch);
    gptmr_channel_reset_count(base, lch);
    gptmr_start_counter(base, lch);
    return 0;
}

static int gptmr_drv_force_release(intf_gptmr_ch_t ch)
{
    if (!gptmr_ch_is_valid(ch) || !gptmr_state[ch].configured) {
        return -1;
    }

    uint8_t lch = gptmr_local_ch(ch);
    GPTMR_Type *base = gptmr_get_base(ch);

    gptmr_stop_counter(base, lch);
    gptmr_update_cmp(base, lch, 0,
        (uint32_t)((float)gptmr_state[ch].reload * gptmr_state[ch].duty));
    gptmr_update_cmp(base, lch, 1, gptmr_state[ch].reload);
    gptmr_enable_cmp_output(base, lch);
    gptmr_channel_reset_count(base, lch);
    gptmr_start_counter(base, lch);
    return 0;
}

static int gptmr_drv_capture_poll(intf_gptmr_ch_t ch, intf_gptmr_capture_t *capture)
{
    static bool has_first_edge[GPTMR_TOTAL_CHANNELS] = {false};
    static uint32_t first_count[GPTMR_TOTAL_CHANNELS] = {0};
    static uint32_t period_ticks[GPTMR_TOTAL_CHANNELS] = {0};

    if ((capture == NULL) || !gptmr_ch_is_valid(ch)
        || !gptmr_state[ch].configured
        || gptmr_state[ch].mode != INTF_GPTMR_MODE_CAPTURE) {
        return -1;
    }

    uint8_t lch = gptmr_local_ch(ch);
    GPTMR_Type *base = gptmr_get_base(ch);

    capture->captured = false;
    capture->count = 0;
    capture->period_ticks = period_ticks[ch];

    if (!gptmr_check_status(base, GPTMR_CH_CAP_STAT_MASK(lch))) {
        return 0;
    }

    gptmr_clear_status(base, GPTMR_CH_CAP_STAT_MASK(lch));
    uint32_t count = gptmr_channel_get_counter(base, lch,
        gptmr_get_capture_counter_type(INTF_GPTMR_CAPTURE_EDGE_RISING));
    capture->count = count;

    if (!has_first_edge[ch]) {
        first_count[ch] = count;
        has_first_edge[ch] = true;
        return 0;
    }

    period_ticks[ch] = gptmr_calc_delta(first_count[ch], count);
    first_count[ch] = count;
    capture->captured = true;
    capture->period_ticks = period_ticks[ch];
    return 0;
}

#define GPTMR_OPS_INITIALIZER(inst_id) { \
    .instance_id = (inst_id),            \
    .init = gptmr_drv_init,              \
    .start = gptmr_drv_start,            \
    .stop = gptmr_drv_stop,              \
    .set_duty = gptmr_drv_set_duty,      \
    .set_frequency = gptmr_drv_set_frequency, \
    .force_low = gptmr_drv_force_low,    \
    .force_release = gptmr_drv_force_release, \
    .capture_poll = gptmr_drv_capture_poll, \
}

static const intf_gptmr_t gptmr_ops[GPTMR_INSTANCE_COUNT] = {
    GPTMR_OPS_INITIALIZER(0),
    GPTMR_OPS_INITIALIZER(1),
    GPTMR_OPS_INITIALIZER(2),
    GPTMR_OPS_INITIALIZER(3),
};

void hpm_gptmr_driver_register(void)
{
    for (uint8_t i = 0; i < GPTMR_INSTANCE_COUNT; i++) {
        intf_gptmr_register(&gptmr_ops[i]);
    }
}
