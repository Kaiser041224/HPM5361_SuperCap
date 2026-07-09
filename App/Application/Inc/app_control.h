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
    MODE_IDLE    = 0,
    MODE_CV      = 1,
    MODE_CC      = 2,
    MODE_STANDBY = 3,
    MODE_CW      = 4,
    MODE_CV_CC   = 5,
} op_mode_t;

typedef struct {
    struct {
        float i_l_a;
        float vcap_v;        /* 当前 VCAP，超级电容电容组电压 */
        float i_in_a;        /* ADC 实测 I_IN 换算物理值 */
        float vout_v;        /* 当前 VOUT，超级电容控制器输出电压 */
    } raw;
    struct {
        uint16_t i_l;
        uint16_t vcap;
        uint16_t i_in;
        uint16_t vout;
    } raw_adc;
    struct {
        float i_l_a;
        float vcap_v;        /* VCAP, LPF @40kHz(125kHz) + MA4 @50kHz: cap 侧监测/限幅参考 */
        float vcap_fast_v;   /* VCAP, 1阶LPF @40kHz: 电流内环前馈用 */
        float i_in_a;        /* ADC1 PB13 实测 I_IN 滤波值，供 CW 功率/输入电流计算 */
        float vout_v;        /* VOUT, MA4 @50kHz: 电压外环反馈和功率计算用 */
    } filt;
    struct {
        float buckboost_a;
        float buckboost_b;
    } duty;
    struct {
        float i_cap_est_a;    /* cap 侧充电/放电电流估算，来自 I_L 与占空比 */
        float i_cap_target_a; /* CC 环 cap 侧目标电流 */
        float cc_pid_out;     /* CC PID 输出，CC 模式下即 current_ref */
        float voltage_pid_out; /* CV 电压环 PID 输出，CV 模式下即 current_ref */
        float v_cap_target_v;  /* CV 电压环设定值（VCAP 目标） */
        float current_ref_a;  /* 内环电流参考，外环赋值后的最终值 */
        float i_in_a;         /* ADC 实测 I_IN，经滤波后用于 CW 环 */
        float p_in_w;         /* VOUT(filtered) × I_IN(filtered)，VIN 当前未采样 */
        float p_target_w;
        float power_pid_out;
    } ff;
} ctrl_diag_t;

extern volatile ctrl_diag_t g_ctrl_diag;

void app_control_init(void);

void app_control_tick(void);

sys_state_t app_control_get_state(void);
op_mode_t   app_control_get_mode(void);
int         app_control_set_mode(op_mode_t mode, float vout, float vcap);

int  app_control_power_enable(void);
void app_control_power_disable(void);
void app_control_emergency(void);

uint32_t app_control_get_faults(void);
int      app_control_clear_faults(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_H */
