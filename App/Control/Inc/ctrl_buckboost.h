/*
 * ctrl_buckboost.h — 四开关 Buck-Boost 控制器
 *
 * 控制链:
 *   PI 输出 → V_L_cmd (V，有符号平均电感电压命令)
 *   VOUT/VCAP 前馈: V_L_cmd → generalized_duty (0.0-1.0)
 *   单输入调制器: generalized_duty → DA, DB
 *
 * 调制器公式:
 *   DA = Dmax × generalized_duty
 *   DB = Dmax × (1 - generalized_duty)
 *
 * 理想平均电感电压:
 *   V_L = DA × VOUT - DB × VCAP
 *       = Dmax × ((VOUT + VCAP) × generalized_duty - VCAP)
 *
 * 双向功率: V_L_cmd 符号由 PI 自然决定，前馈负责转换为 generalized_duty。
 * A/B 半桥独立控制，无移相。
 *
 * 物理量约定:
 *   VOUT: 超级电容控制器输出电压，和系统 VIN 仅通过 IIN 检流电阻相连。
 *   VCAP: 超级电容电容组电压，位于 Buck-Boost cap 侧。
 *   VIN:  系统总输入电压，当前硬件未采样，不参与本控制器前馈计算。
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CTRL_BUCKBOOST_H
#define CTRL_BUCKBOOST_H

#include "algo_pid.h"

#include <stdbool.h>
#include <stdint.h>

/* 控制默认值 */
#define BUCKBOOST_P_TARGET_DEFAULT 15.0f

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_max;
    float output_max;
    float output_min;
} ctrl_buckboost_pid_params_t;

typedef enum {
    BUCKBOOST_TARGET_CV = 0,
    BUCKBOOST_TARGET_CC = 1,
    BUCKBOOST_TARGET_CW = 2,
} ctrl_buckboost_target_t;

typedef struct {
    ctrl_buckboost_pid_params_t current_pid;
    ctrl_buckboost_pid_params_t voltage_pid;
    ctrl_buckboost_pid_params_t current_cc_pid;
    ctrl_buckboost_pid_params_t power_pid;
    float i_l_limit_a;
    float v_out_limit_v;
    float p_target_w;
    float voltage_ff_gain;
    float duty_min;
    float duty_max;
} ctrl_buckboost_params_t;

typedef struct {
    bool enabled;
    ctrl_buckboost_target_t target_type;

    float v_out_target_v;
    float i_l_target_a;
    float i_cap_target_a;
    float p_target_w;
    float p_in_w;

    algo_pid_t current_pid;
    algo_pid_t voltage_pid;
    algo_pid_t current_cc_pid;
    algo_pid_t power_pid;

    volatile float current_ref;
    float voltage_pid_out;
    float cc_pid_out;
    float power_pid_out;

    float v_cmd;
    float generalized_duty;
    bool vcap_limit_active;
    float duty_a;
    float duty_b;
} ctrl_buckboost_state_t;

typedef struct {
    ctrl_buckboost_params_t params;
    ctrl_buckboost_state_t state;
} ctrl_buckboost_t;

int ctrl_buckboost_init(ctrl_buckboost_t* ctrl);
int ctrl_buckboost_enable(ctrl_buckboost_t* ctrl);
void ctrl_buckboost_disable(ctrl_buckboost_t* ctrl);
void ctrl_buckboost_emergency_stop(ctrl_buckboost_t* ctrl);

/*
 * 单输入调制器: generalized_duty (0.0-1.0) → DA, DB
 *
 *   DA = Dmax × generalized_duty
 *   DB = Dmax × (1 - generalized_duty)
 *
 * VOUT/VCAP 前馈不在调制器内完成，而是在 update_current() 中完成。
 */
void ctrl_buckboost_modulate(ctrl_buckboost_t* ctrl, float generalized_duty);

/*
 * 电流内环 update (200kHz)
 *   PI(current_ref, i_l) → V_L_cmd (V)
 *   feedforward(V_L_cmd, VOUT, VCAP) → generalized_duty
 *   modulate(generalized_duty) → DA, DB
 *
 * VOUT 与 VCAP 分别是四开关 Buck-Boost 两侧的实测电压。
 */
void ctrl_buckboost_update_current(ctrl_buckboost_t* ctrl, float i_l, float vout, float vcap);

/*
 * 电压/限流竞争外环 update (50kHz)
 *   PI(v_out_target, vout) + i_cap_ff × ff_gain → current_ref
 *   PI(i_cap_target, i_cap_est) 与电压环竞争，限制 cap 侧充电/放电电流。
 *
 * 电压反馈使用 VOUT；cap 侧电流当前由 I_L 和占空比估算。
 */
void ctrl_buckboost_update_voltage(
    ctrl_buckboost_t* ctrl, float vout, float i_cap_ff, float i_cap_est);

/*
 * 进入恒压 (CV) 模式，配置软起动
 *   - 设定 v_out_target 并 reset 电压环 PID，从零输出开始
 *   - target_type 自动设为 BUCKBOOST_TARGET_CV
 */
void ctrl_buckboost_enter_cv_mode(ctrl_buckboost_t* ctrl, float target_v);

/*
 * 进入恒功率 (CW) 模式，支持双向 (target_w 可为正/负)
 *   - 设定 p_target_w, target_type 自动设为 BUCKBOOST_TARGET_CW
 *   - 不 reset PID, 保留积分连续性实现平滑模式切换
 */
void ctrl_buckboost_enter_cw_mode(ctrl_buckboost_t* ctrl, float target_w);

/*
 * 功率外环 update (25kHz)
 *   增量式 PI(p_target, p_in) → power_pid_out
 *   双向: PID 输出作为 signed VOUT target，再由电压环产生 signed current_ref
 */
void ctrl_buckboost_update_power(ctrl_buckboost_t* ctrl, float p_in);

void ctrl_buckboost_set_vout_target(ctrl_buckboost_t* ctrl, float target_v);
void ctrl_buckboost_set_il_target(ctrl_buckboost_t* ctrl, float target_a);
void ctrl_buckboost_set_icap_target(ctrl_buckboost_t* ctrl, float target_a);
void ctrl_buckboost_set_ptarget(ctrl_buckboost_t* ctrl, float target_w);
void ctrl_buckboost_set_target_type(ctrl_buckboost_t* ctrl, ctrl_buckboost_target_t target);
void ctrl_buckboost_set_params(ctrl_buckboost_t* ctrl, const ctrl_buckboost_params_t* params);

float ctrl_buckboost_get_duty_a(const ctrl_buckboost_t* ctrl);
float ctrl_buckboost_get_duty_b(const ctrl_buckboost_t* ctrl);
float ctrl_buckboost_get_v_cmd(const ctrl_buckboost_t* ctrl);
float ctrl_buckboost_get_generalized_duty(const ctrl_buckboost_t* ctrl);
float ctrl_buckboost_get_duty_max(const ctrl_buckboost_t* ctrl);
float ctrl_buckboost_get_current_ref(const ctrl_buckboost_t* ctrl);
float ctrl_buckboost_get_ptarget(const ctrl_buckboost_t* ctrl);
float ctrl_buckboost_get_power_pid_out(const ctrl_buckboost_t* ctrl);
bool ctrl_buckboost_is_enabled(const ctrl_buckboost_t* ctrl);

#ifdef __cplusplus
}
#endif

#endif /* CTRL_BUCKBOOST_H */
