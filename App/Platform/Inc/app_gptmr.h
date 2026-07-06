/*
 * GPTMR Platform API
 *
 * GPTMR1 外环定时器封装。
 * 使用与 PWM 相同的 AHB 时钟源 (120MHz)，保证频率一致性。
 * 通过 intf_gptmr 接口层访问驱动，不直接调用驱动层。
 *
 * 通道分配 (GPTMR1):
 *   CH0 — 电压外环 50kHz
 *   CH1 — 功率外环 25kHz
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_GPTMR_H
#define APP_GPTMR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*app_gptmr_callback_t)(void);

typedef enum {
    APP_GPTMR_CH_VOLTAGE = 0,   /* GPTMR1 CH0: 电压外环 */
    APP_GPTMR_CH_POWER   = 1,   /* GPTMR1 CH1: 功率外环 */
    APP_GPTMR_CH_COUNT,
} app_gptmr_ch_t;

void app_gptmr_init(void);

int  app_gptmr_start(app_gptmr_ch_t ch);
int  app_gptmr_stop(app_gptmr_ch_t ch);
void app_gptmr_start_all(void);
void app_gptmr_stop_all(void);

int  app_gptmr_register_callback(app_gptmr_ch_t ch, app_gptmr_callback_t cb);
int  app_gptmr_set_frequency(app_gptmr_ch_t ch, uint32_t frequency_hz);

#ifdef __cplusplus
}
#endif

#endif /* APP_GPTMR_H */
