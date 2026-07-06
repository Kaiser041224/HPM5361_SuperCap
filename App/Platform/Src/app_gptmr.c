/*
 * GPTMR Platform Implementation
 *
 * GPTMR1 外环定时器封装。
 * 使用与 PWM 相同的 AHB 时钟源 (120MHz)，保证频率一致性。
 * 通过 intf_gptmr 接口层访问驱动，不直接调用驱动层。
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_gptmr.h"
#include "intf_gptmr.h"

#include <stddef.h>

/* GPTMR1 实例，全局通道号 4-7 */
#define APP_GPTMR_INSTANCE      1U
#define APP_GPTMR_CH_BASE       (APP_GPTMR_INSTANCE * 4U)

#define APP_GPTMR_VOLTAGE_FREQ  50000U
#define APP_GPTMR_POWER_FREQ    25000U

extern void hpm_gptmr_driver_register(void);

static app_gptmr_callback_t s_callbacks[APP_GPTMR_CH_COUNT];

static intf_gptmr_ch_t app_ch_to_intf(app_gptmr_ch_t ch)
{
    return (intf_gptmr_ch_t)(APP_GPTMR_CH_BASE + (uint8_t)ch);
}

void app_gptmr_init(void)
{
    hpm_gptmr_driver_register();

    /* CH0: 电压外环 50kHz */
    intf_gptmr_cfg_t cfg_voltage = {
        .mode         = INTF_GPTMR_MODE_TIMER,
        .frequency_hz = APP_GPTMR_VOLTAGE_FREQ,
        .callback     = NULL,
        .enable_sync  = false,
    };
    (void)intf_gptmr_init(app_ch_to_intf(APP_GPTMR_CH_VOLTAGE), &cfg_voltage);

    /* CH1: 功率外环 25kHz */
    intf_gptmr_cfg_t cfg_power = {
        .mode         = INTF_GPTMR_MODE_TIMER,
        .frequency_hz = APP_GPTMR_POWER_FREQ,
        .callback     = NULL,
        .enable_sync  = false,
    };
    (void)intf_gptmr_init(app_ch_to_intf(APP_GPTMR_CH_POWER), &cfg_power);

    s_callbacks[APP_GPTMR_CH_VOLTAGE] = NULL;
    s_callbacks[APP_GPTMR_CH_POWER]   = NULL;
}

int app_gptmr_register_callback(app_gptmr_ch_t ch, app_gptmr_callback_t cb)
{
    if (ch >= APP_GPTMR_CH_COUNT || cb == NULL) {
        return -1;
    }

    s_callbacks[ch] = cb;

    uint32_t freq;
    switch (ch) {
    case APP_GPTMR_CH_VOLTAGE: freq = APP_GPTMR_VOLTAGE_FREQ; break;
    case APP_GPTMR_CH_POWER:   freq = APP_GPTMR_POWER_FREQ;   break;
    default:                   return -1;
    }

    intf_gptmr_cfg_t cfg = {
        .mode         = INTF_GPTMR_MODE_TIMER,
        .frequency_hz = freq,
        .callback     = cb,
        .enable_sync  = false,
    };
    return intf_gptmr_init(app_ch_to_intf(ch), &cfg);
}

int app_gptmr_start(app_gptmr_ch_t ch)
{
    if (ch >= APP_GPTMR_CH_COUNT) {
        return -1;
    }
    return intf_gptmr_start(app_ch_to_intf(ch));
}

int app_gptmr_stop(app_gptmr_ch_t ch)
{
    if (ch >= APP_GPTMR_CH_COUNT) {
        return -1;
    }
    return intf_gptmr_stop(app_ch_to_intf(ch));
}

void app_gptmr_start_all(void)
{
    for (app_gptmr_ch_t ch = APP_GPTMR_CH_VOLTAGE; ch < APP_GPTMR_CH_COUNT; ch++) {
        (void)app_gptmr_start(ch);
    }
}

void app_gptmr_stop_all(void)
{
    for (app_gptmr_ch_t ch = APP_GPTMR_CH_VOLTAGE; ch < APP_GPTMR_CH_COUNT; ch++) {
        (void)app_gptmr_stop(ch);
    }
}

int app_gptmr_set_frequency(app_gptmr_ch_t ch, uint32_t frequency_hz)
{
    if (ch >= APP_GPTMR_CH_COUNT) {
        return -1;
    }
    return intf_gptmr_set_frequency(app_ch_to_intf(ch), frequency_hz);
}
