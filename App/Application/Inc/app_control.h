/*
 * app_control.h — 系统控制编排
 *
 * 职责：状态机、运行模式、功率使能/禁能、故障检查、ISR 注册。
 * 不包含控制算法（算法在 Control/ 层），只做"何时做什么、谁来调用"的决策。
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_INIT   = 0,
    SYS_IDLE   = 1,
    SYS_RUN    = 2,
    SYS_FAULT  = 3,
} sys_state_t;

typedef enum {
    MODE_IDLE       = 0,
    MODE_BUCK_CV    = 1,
    MODE_BUCK_CC    = 2,
    MODE_STANDBY    = 3,
    MODE_BUCK_CW    = 4,
} op_mode_t;

typedef struct {
    struct {
        float i_l_a;
        float vcap_v;        /* 当前 VCAP，控制模型 VLINK */
        float i_in_a;        /* ADC 实测 I_IN 换算物理值 */
        float vout_v;        /* 当前 VOUT，控制模型 VIN */
    } raw;
    struct {
        uint16_t i_l;
        uint16_t vcap;
        uint16_t i_in;
        uint16_t vout;
    } raw_adc;
    struct {
        float i_l_a;
        float vcap_v;        /* VCAP/VLINK, MA4 @50kHz: 电压外环反馈用 (稳态平滑) */
        float vcap_fast_v;   /* VCAP/VLINK, 1阶LPF @40kHz: 电流内环前馈用 (动态快, 勿用于反馈) */
        float i_in_a;        /* ADC1 PB13 实测 I_IN 滤波值，供功率环输入功率计算 */
        float vout_v;        /* VOUT/VIN, 输入功率计算和 Buck-Boost 前馈用 */
    } filt;
    struct {
        float buckboost_a;
        float buckboost_b;
    } duty;
    struct {
        float i_load_est_a; /* VCAP/VLINK 侧电流估算，供电压环前馈 */
        float i_in_ctrl_a;  /* app_analog_signal 已按 PCB 极性修正后的 ADC 实测 I_IN */
        float p_in_w;       /* VOUT/VIN(filtered) × i_in_ctrl_a */
        float p_target_w;
        float power_pid_out;
    } ff;
} ctrl_diag_t;

extern volatile ctrl_diag_t g_ctrl_diag;

void app_control_init(void);

void app_control_tick(void);

sys_state_t app_control_get_state(void);
op_mode_t   app_control_get_mode(void);
int         app_control_set_mode(op_mode_t mode);

int  app_control_power_enable(void);
void app_control_power_disable(void);
void app_control_emergency(void);

uint32_t app_control_get_faults(void);
int      app_control_clear_faults(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_H */
