/*
 * ADC App Layer - PMT Initialization
 *
 * Current hardware mapping:
 *   ADC0: PB14 / ADC0.6 = IL
 *   ADC1: PB11 / ADC1.3 = VOUT
 *         PB12 / ADC1.4 = VCAP
 *         PB13 / ADC1.5 = IIN
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_adc.h"
#include "app_hrpwm.h"
#include "irq_profiler.h"

#include "intf_adc.h"
#include "intf_attr.h"
#include "intf_hrpwm.h"
#include "intf_trgm.h"

#include <stddef.h>
#include <string.h>

static irq_prof_id_t g_prof_adc0;
static irq_prof_id_t g_prof_adc1;

typedef struct {
    uint8_t inst;
    uint8_t hw_ch;
} app_adc_map_t;

static const app_adc_map_t adc_map[ADC_CH_COUNT] = {
    [ADC_CH_VCAP] = {.inst = APP_ADC_INST_1, .hw_ch = 4},
    [ADC_CH_VOUT] = {.inst = APP_ADC_INST_1, .hw_ch = 3},
    [ADC_CH_I_IN] = {.inst = APP_ADC_INST_1, .hw_ch = 5},
    [ADC_CH_I_L]  = {.inst = APP_ADC_INST_0, .hw_ch = 6},
};

static app_adc_calibration_t adc_calibration[ADC_CH_COUNT] = {
    [ADC_CH_VCAP] = {.sense_gain = 1.0f, .sense_offset_mv = 0.0f, .physical_gain = 1.0f, .physical_offset = 0.0f},
    [ADC_CH_VOUT] = {.sense_gain = 1.0f, .sense_offset_mv = 0.0f, .physical_gain = 1.0f, .physical_offset = 0.0f},
    [ADC_CH_I_IN] = {.sense_gain = 1.0f, .sense_offset_mv = 0.0f, .physical_gain = 1.0f, .physical_offset = 0.0f},
    [ADC_CH_I_L]  = {.sense_gain = 1.0f, .sense_offset_mv = 0.0f, .physical_gain = 1.0f, .physical_offset = 0.0f},
};

static bool app_adc_channel_is_valid(adc_channel_t ch) { return ch < ADC_CH_COUNT; }

ATTR_RAMFUNC
static int app_adc_pmt_trigger_cmp_index(app_adc_inst_t inst, uint8_t *cmp_index)
{
    if (cmp_index == NULL) {
        return -1;
    }

    switch (inst) {
    case APP_ADC_INST_0:
        *cmp_index = APP_ADC_PMT_TRIGGER_CMP_INDEX_ADC0_PWM1;
        return 0;
    case APP_ADC_INST_1:
        *cmp_index = APP_ADC_PMT_TRIGGER_CMP_INDEX_ADC1_PWM1;
        return 0;
    default:
        return -1;
    }
}

static intf_adc_ch_t app_adc_encoded_channel(adc_channel_t ch)
{
    return INTF_ADC_CH(adc_map[ch].inst, adc_map[ch].hw_ch);
}

ATTR_RAMFUNC
int app_adc_set_pmt_trigger_position(app_adc_inst_t inst, float position_ratio)
{
    if (position_ratio < 0.0f || position_ratio > 1.0f || position_ratio != position_ratio) {
        return -1;
    }

    uint8_t cmp_index;
    if (app_adc_pmt_trigger_cmp_index(inst, &cmp_index) != 0) {
        return -1;
    }

    return intf_hrpwm_set_trigger_cmp_position(HRPWM_INST_BUCKBOOST, cmp_index, position_ratio);
}

ATTR_PLACE_AT_FAST_RAM_BSS static volatile uint16_t pmt_raw_cache[ADC_CH_COUNT];
ATTR_PLACE_AT_FAST_RAM_BSS static volatile bool pmt_raw_cache_valid[ADC_CH_COUNT];

static float s_adc_vref_mv[APP_ADC_INST_COUNT] = {
    [APP_ADC_INST_0] = INTF_ADC_DEFAULT_VREF_MV,
    [APP_ADC_INST_1] = INTF_ADC_DEFAULT_VREF_MV,
};

ATTR_PLACE_AT_FAST_RAM_BSS static app_adc_pmt_callback_t s_pmt_callback[APP_ADC_INST_COUNT];

int app_adc_register_pmt_callback(app_adc_inst_t inst, app_adc_pmt_callback_t cb)
{
    if (inst >= APP_ADC_INST_COUNT) {
        return -1;
    }
    s_pmt_callback[inst] = cb;
    return 0;
}

ATTR_RAMFUNC
int app_adc_get_pmt_raw(adc_channel_t ch, uint16_t *raw)
{
    if (!app_adc_channel_is_valid(ch) || raw == NULL) {
        return -1;
    }
    if (!pmt_raw_cache_valid[ch]) {
        return -2;
    }
    *raw = pmt_raw_cache[ch];
    return 0;
}

ATTR_PLACE_AT_FAST_RAM_BSS static uint32_t pmt_dma0[APP_ADC_PMT_DMA_BUFF_LEN]
    __attribute__((aligned(4)));
ATTR_PLACE_AT_FAST_RAM_BSS static uint32_t pmt_dma1[APP_ADC_PMT_DMA_BUFF_LEN]
    __attribute__((aligned(4)));

ATTR_RAMFUNC
static void app_adc_pmt_cb_adc0(
    intf_adc_ch_t trig, const uint16_t *values, uint8_t count, void *user_data)
{
    IRQ_PROF_ENTER(g_prof_adc0);
    (void)trig;
    (void)user_data;

    if (count > 1U) {
        pmt_raw_cache[ADC_CH_I_L] = values[1];
        pmt_raw_cache_valid[ADC_CH_I_L] = true;
    }

    if (s_pmt_callback[APP_ADC_INST_0] != NULL) {
        s_pmt_callback[APP_ADC_INST_0]();
    }

    IRQ_PROF_EXIT(g_prof_adc0);
}

ATTR_RAMFUNC
static void app_adc_pmt_cb_adc1(
    intf_adc_ch_t trig, const uint16_t *values, uint8_t count, void *user_data)
{
    IRQ_PROF_ENTER(g_prof_adc1);
    (void)trig;
    (void)user_data;

    static const adc_channel_t slot_to_logic[4] = {
        [0] = ADC_CH_COUNT,
        [1] = ADC_CH_I_IN,
        [2] = ADC_CH_VOUT,
        [3] = ADC_CH_VCAP,
    };

    for (uint8_t i = 1; i < count && i < 4U; i++) {
        if (slot_to_logic[i] < ADC_CH_COUNT) {
            pmt_raw_cache[slot_to_logic[i]] = values[i];
            pmt_raw_cache_valid[slot_to_logic[i]] = true;
        }
    }

    if (s_pmt_callback[APP_ADC_INST_1] != NULL) {
        s_pmt_callback[APP_ADC_INST_1]();
    }

    IRQ_PROF_EXIT(g_prof_adc1);
}

extern void hpm_adc_driver_register(void);
extern void hpm_trgm_driver_register(void);

void app_adc_init(void)
{
    g_prof_adc0 = irq_prof_register("ADC0_PMT");
    g_prof_adc1 = irq_prof_register("ADC1_PMT");
    irq_prof_measure_overhead();

    hpm_adc_driver_register();
    hpm_trgm_driver_register();

    memset((void *)pmt_raw_cache, 0, sizeof(pmt_raw_cache));
    memset((void *)pmt_raw_cache_valid, 0, sizeof(pmt_raw_cache_valid));

    /* ADC0 PMT: dummy + IL (PB14 / ADC0.6), triggered by PWM1 CMP10 at carrier center. */
    {
        memset(pmt_dma0, 0, sizeof(pmt_dma0));
        intf_adc_cfg_t cfg = {
            .resolution = INTF_ADC_RES_DEFAULT,
            .mode = INTF_ADC_MODE_PMT,
            .sample_cycle = INTF_ADC_DEFAULT_SAMPLE_CYCLE,
            .clock_div = INTF_ADC_DEFAULT_CLOCK_DIV,
            .vref_mv = INTF_ADC_DEFAULT_VREF_MV,
            .dma_en = true,
            .dma_buff = pmt_dma0,
            .dma_buff_len = APP_ADC_PMT_DMA_BUFF_LEN,
            .pmt_trig_ch = APP_ADC_PMT_ADC0_TRIG_CH,
            .pmt_ch_count = APP_ADC_PMT_ADC0_CH_COUNT,
            .pmt_cb = app_adc_pmt_cb_adc0,
            .pmt_cb_user_data = NULL,
        };
        cfg.pmt_ch_list[0] = 15U;
        cfg.pmt_ch_list[1] = 6U;
        (void)intf_adc_init(INTF_ADC_CH(APP_ADC_INST_0, 0), &cfg);
    }

    /* ADC1 PMT: dummy + IIN/VOUT/VCAP, triggered by PWM1 CMP11 at carrier center. */
    {
        memset(pmt_dma1, 0, sizeof(pmt_dma1));
        intf_adc_cfg_t cfg = {
            .resolution = INTF_ADC_RES_DEFAULT,
            .mode = INTF_ADC_MODE_PMT,
            .sample_cycle = INTF_ADC_DEFAULT_SAMPLE_CYCLE,
            .clock_div = INTF_ADC_DEFAULT_CLOCK_DIV,
            .vref_mv = INTF_ADC_DEFAULT_VREF_MV,
            .dma_en = true,
            .dma_buff = pmt_dma1,
            .dma_buff_len = APP_ADC_PMT_DMA_BUFF_LEN,
            .pmt_trig_ch = APP_ADC_PMT_ADC1_TRIG_CH,
            .pmt_ch_count = APP_ADC_PMT_ADC1_CH_COUNT,
            .pmt_cb = app_adc_pmt_cb_adc1,
            .pmt_cb_user_data = NULL,
        };
        cfg.pmt_ch_list[0] = 15U;
        cfg.pmt_ch_list[1] = 5U;
        cfg.pmt_ch_list[2] = 3U;
        cfg.pmt_ch_list[3] = 4U;
        (void)intf_adc_init(INTF_ADC_CH(APP_ADC_INST_1, 0), &cfg);
    }

    (void)intf_trgm_connect(INTF_TRGM_SRC_PWM1_CH10REF, INTF_TRGM_DST_ADC_PTRGI0A);
    (void)intf_trgm_connect(INTF_TRGM_SRC_PWM1_CH11REF, INTF_TRGM_DST_ADC_PTRGI1A);

    (void)intf_hrpwm_config_trigger_cmp(
        HRPWM_INST_BUCKBOOST, APP_ADC_PMT_TRIGGER_CMP_INDEX_ADC0_PWM1, APP_ADC_PMT_POSITION_RATIO_ADC0);
    (void)intf_hrpwm_config_trigger_cmp(
        HRPWM_INST_BUCKBOOST, APP_ADC_PMT_TRIGGER_CMP_INDEX_ADC1_PWM1, APP_ADC_PMT_POSITION_RATIO_ADC1);
}

static int app_adc_read_latest_raw(adc_channel_t ch, uint16_t *raw)
{
    if (!app_adc_channel_is_valid(ch) || raw == NULL) {
        return -1;
    }

    if (pmt_raw_cache_valid[ch]) {
        *raw = pmt_raw_cache[ch];
        return 0;
    }

    return intf_adc_read(app_adc_encoded_channel(ch), raw);
}

uint16_t app_adc_read_raw(adc_channel_t ch)
{
    uint16_t raw = 0;
    (void)app_adc_read_latest_raw(ch, &raw);
    return raw;
}

void app_adc_read_all(uint16_t values[ADC_CH_COUNT])
{
    if (values == NULL) {
        return;
    }
    for (adc_channel_t ch = ADC_CH_VCAP; ch < ADC_CH_COUNT; ch++) {
        values[ch] = app_adc_read_raw(ch);
    }
}

int app_adc_read_adc_voltage_mv(adc_channel_t ch, float *voltage_mv)
{
    if (!app_adc_channel_is_valid(ch) || voltage_mv == NULL) {
        return -1;
    }

    uint16_t raw = 0;
    if (app_adc_read_latest_raw(ch, &raw) != 0) {
        return -1;
    }

    app_adc_inst_t inst = (app_adc_inst_t)adc_map[ch].inst;
    *voltage_mv = (float)raw * s_adc_vref_mv[inst] / 65535.0f;
    return 0;
}

int app_adc_read_sense_voltage_mv(adc_channel_t ch, float *voltage_mv)
{
    float adc_voltage_mv;

    if (!app_adc_channel_is_valid(ch) || voltage_mv == NULL) {
        return -1;
    }
    if (app_adc_read_adc_voltage_mv(ch, &adc_voltage_mv) != 0) {
        return -1;
    }

    *voltage_mv =
        adc_voltage_mv * adc_calibration[ch].sense_gain + adc_calibration[ch].sense_offset_mv;
    return 0;
}

int app_adc_read_physical(adc_channel_t ch, float *value)
{
    float sense_voltage_mv;

    if (!app_adc_channel_is_valid(ch) || value == NULL) {
        return -1;
    }
    if (app_adc_read_sense_voltage_mv(ch, &sense_voltage_mv) != 0) {
        return -1;
    }

    *value =
        sense_voltage_mv * adc_calibration[ch].physical_gain + adc_calibration[ch].physical_offset;
    return 0;
}

void app_adc_set_calibration(adc_channel_t ch, const app_adc_calibration_t *cal)
{
    if (!app_adc_channel_is_valid(ch) || cal == NULL) {
        return;
    }
    adc_calibration[ch] = *cal;
}

int app_adc_get_calibration(adc_channel_t ch, app_adc_calibration_t *cal)
{
    if (!app_adc_channel_is_valid(ch) || cal == NULL) {
        return -1;
    }
    *cal = adc_calibration[ch];
    return 0;
}

void app_adc_set_vref_inst(app_adc_inst_t inst, float mv)
{
    if (inst >= APP_ADC_INST_COUNT) {
        return;
    }
    s_adc_vref_mv[inst] = mv;
    intf_adc_set_vref(INTF_ADC_CH(inst, 0), mv);
}

void app_adc_set_vref_all(float mv)
{
    for (app_adc_inst_t inst = APP_ADC_INST_0; inst < APP_ADC_INST_COUNT; inst++) {
        app_adc_set_vref_inst(inst, mv);
    }
}

void app_adc_calibrate(void)
{
    intf_adc_calibrate(INTF_ADC_CH(0, 0));
    intf_adc_calibrate(INTF_ADC_CH(1, 0));
}

void app_adc_pmt_start_inst(app_adc_inst_t inst)
{
    if (inst >= APP_ADC_INST_COUNT) {
        return;
    }
    (void)intf_adc_start(INTF_ADC_CH(inst, 0));
}

void app_adc_pmt_stop_inst(app_adc_inst_t inst)
{
    if (inst >= APP_ADC_INST_COUNT) {
        return;
    }
    (void)intf_adc_stop(INTF_ADC_CH(inst, 0));
}

void app_adc_wdog_init(
    adc_channel_t ch, uint16_t thshd_high, uint16_t thshd_low, intf_adc_wdog_cb_t cb,
    void *user_data)
{
    if (!app_adc_channel_is_valid(ch)) {
        return;
    }

    intf_adc_cfg_t cfg = {
        .resolution = INTF_ADC_RES_DEFAULT,
        .mode = INTF_ADC_MODE_ONESHOT,
        .sample_cycle = INTF_ADC_DEFAULT_SAMPLE_CYCLE,
        .clock_div = INTF_ADC_DEFAULT_CLOCK_DIV,
        .vref_mv = INTF_ADC_DEFAULT_VREF_MV,
        .wdog_en = true,
        .wdog_thshd_high = thshd_high,
        .wdog_thshd_low = thshd_low,
        .wdog_cb = cb,
        .wdog_cb_user_data = user_data,
    };

    pmt_raw_cache_valid[ch] = false;
    pmt_raw_cache[ch] = 0U;
    (void)intf_adc_init(app_adc_encoded_channel(ch), &cfg);
}

void app_adc_wdog_reenable(adc_channel_t ch)
{
    if (!app_adc_channel_is_valid(ch)) {
        return;
    }
    intf_adc_wdog_reenable(app_adc_encoded_channel(ch));
}
