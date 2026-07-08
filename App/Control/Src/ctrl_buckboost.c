/*
 * ctrl_buckboost.c — 四开关 Buck-Boost 控制器实现
 *
 * 控制链:
 *   PI(current_ref, i_l) → V_L_cmd (有符号平均电感电压命令)
 *   VOUT/VCAP 前馈 → generalized_duty (0.0-1.0)
 *   单输入调制器 generalized_duty → DA, DB
 *
 * 调制器:
 *   DA = Dmax × generalized_duty
 *   DB = Dmax × (1 - generalized_duty)
 *
 * 理想平均电感电压:
 *   V_L = DA × VOUT - DB × VCAP
 *
 * 物理量约定:
 *   VOUT: 超级电容控制器输出电压，和系统 VIN 仅通过 IIN 检流电阻相连。
 *   VCAP: 超级电容电容组电压，位于 Buck-Boost cap 侧。
 *   VIN:  系统总输入电压，当前硬件未采样。
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ctrl_buckboost.h"

#include <stddef.h>
#include <string.h>

#ifndef ATTR_RAMFUNC
# define ATTR_RAMFUNC __attribute__((section(".fast")))
#endif

/* 物理 PWM duty 默认范围: 最终写入 HRPWM 的 DA/DB 上限 */
#define BUCKBOOST_DUTY_MIN_DEFAULT 0.0f
#define BUCKBOOST_DUTY_MAX_DEFAULT 0.95f

/* 控制保护默认值 */
#define BUCKBOOST_I_L_LIMIT_DEFAULT      32.0f
#define BUCKBOOST_V_OUT_LIMIT_DEFAULT    32.0f
#define BUCKBOOST_V_OUT_TARGET_DEFAULT   24.0f
#define BUCKBOOST_I_CAP_TARGET_DEFAULT   27.0f
#define BUCKBOOST_BUS_SUM_MIN_V          1.0f
#define BUCKBOOST_VCAP_LIMIT_ENTER_RATIO 1.00f
#define BUCKBOOST_VCAP_LIMIT_EXIT_RATIO  0.96f

/* 功率环 PID 输出范围 */
#define BUCKBOOST_POWER_PID_OUT_MAX 27.0f
#define BUCKBOOST_POWER_PID_OUT_MIN (-BUCKBOOST_POWER_PID_OUT_MAX)
/* 电流环 PID 输出为有符号平均电感电压命令 V_L_cmd */
#define BUCKBOOST_CURRENT_PID_OUT_MIN (-32.0f)
#define BUCKBOOST_CURRENT_PID_OUT_MAX (32.0f)

/* 电压环 PID 输出为电流命令 current_ref (A)，范围 ±i_l_limit */
#define BUCKBOOST_VOLTAGE_PID_OUT_MIN (-BUCKBOOST_I_L_LIMIT_DEFAULT)
#define BUCKBOOST_VOLTAGE_PID_OUT_MAX (BUCKBOOST_I_L_LIMIT_DEFAULT)

/* generalized_duty 命令范围: 单输入调制器的最终命令钳位 */
#define BUCKBOOST_GENERALIZED_DUTY_MIN 0.0f
#define BUCKBOOST_GENERALIZED_DUTY_MAX 0.98f

static inline __attribute__((always_inline)) float clampf(float x, float lo, float hi) {
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

static inline float absf_fast(float x) { return (x < 0.0f) ? -x : x; }

typedef struct {
    float d_min;
    float d_max;
    float g_min;
    float g_max;
    float inv_d_max;
} buckboost_mod_ctx_t;

typedef struct {
    buckboost_mod_ctx_t mod;
    float vout;
    float vcap;
    float inv_bus_sum;
} buckboost_current_ctx_t;

ATTR_RAMFUNC
static bool buckboost_modulation_params_valid(float d_min, float d_max) {
    return algo_pid_finite(d_min) && algo_pid_finite(d_max) && d_min >= 0.0f && d_max > 0.0f
        && d_max <= 1.0f && d_min <= (d_max * 0.5f);
}

ATTR_RAMFUNC
static bool buckboost_prepare_mod_ctx(float d_min, float d_max, buckboost_mod_ctx_t* ctx) {
    if (ctx == NULL || !buckboost_modulation_params_valid(d_min, d_max)) {
        return false;
    }

    float inv_d_max = 1.0f / d_max;
    if (!algo_pid_finite(inv_d_max)) {
        return false;
    }

    float g_min = d_min * inv_d_max;
    if (!algo_pid_finite(g_min)) {
        return false;
    }

    float g_max_by_duty_min = 1.0f - g_min;
    float g_max = clampf(BUCKBOOST_GENERALIZED_DUTY_MAX, g_min, g_max_by_duty_min);
    if (!algo_pid_finite(g_max)) {
        return false;
    }

    ctx->d_min = d_min;
    ctx->d_max = d_max;
    ctx->g_min = g_min;
    ctx->g_max = g_max;
    ctx->inv_d_max = inv_d_max;
    return true;
}

ATTR_RAMFUNC
static bool buckboost_prepare_current_ctx(
    const ctrl_buckboost_t* ctrl, float i_l, float vout, float vcap, buckboost_current_ctx_t* ctx) {
    if (ctrl == NULL || ctx == NULL) {
        return false;
    }
    if (!algo_pid_finite(i_l) || !algo_pid_finite(ctrl->state.current_ref)) {
        return false;
    }
    if (!buckboost_prepare_mod_ctx(ctrl->params.duty_min, ctrl->params.duty_max, &ctx->mod)) {
        return false;
    }

    float vout_abs = absf_fast(vout);
    float vcap_abs = absf_fast(vcap);
    if (!algo_pid_finite(vout_abs) || !algo_pid_finite(vcap_abs)) {
        return false;
    }

    float bus_sum = vout_abs + vcap_abs;
    if (!algo_pid_finite(bus_sum) || bus_sum <= BUCKBOOST_BUS_SUM_MIN_V) {
        return false;
    }

    float inv_bus_sum = 1.0f / bus_sum;
    if (!algo_pid_finite(inv_bus_sum)) {
        return false;
    }

    ctx->vout = vout_abs;
    ctx->vcap = vcap_abs;
    ctx->inv_bus_sum = inv_bus_sum;
    return true;
}

/* ============================================================================
 * 单输入调制器: generalized_duty → DA, DB
 *
 * DA = Dmax × generalized_duty
 * DB = Dmax × (1 - generalized_duty)
 * ============================================================================ */

ATTR_RAMFUNC
static bool buckboost_modulate_generalized(
    float generalized_duty, const buckboost_mod_ctx_t* ctx, float* out_da, float* out_db,
    float* out_effective_generalized_duty) {
    if (ctx == NULL || out_da == NULL || out_db == NULL) {
        return false;
    }
    if (!algo_pid_finite(generalized_duty)) {
        *out_da = 0.0f;
        *out_db = 0.0f;
        if (out_effective_generalized_duty != NULL) {
            *out_effective_generalized_duty = 0.0f;
        }
        return false;
    }

    float g = clampf(generalized_duty, ctx->g_min, ctx->g_max);
    float da = ctx->d_max * g;
    float db = ctx->d_max * (1.0f - g);

    *out_da = da;
    *out_db = db;
    if (out_effective_generalized_duty != NULL) {
        *out_effective_generalized_duty = g;
    }
    return true;
}

/* ============================================================================
 * VOUT/VCAP 前馈: V_L_cmd → generalized_duty
 *
 * V_L = Dmax × ((VOUT + VCAP) × g - VCAP)
 * g   = (V_L / Dmax + VCAP) / (VOUT + VCAP)
 * ============================================================================ */

ATTR_RAMFUNC
static bool buckboost_vl_cmd_to_generalized_duty(
    float v_l_cmd, const buckboost_current_ctx_t* ctx, float* out_generalized_duty) {
    if (out_generalized_duty == NULL) {
        return false;
    }

    if (ctx == NULL || !algo_pid_finite(v_l_cmd)) {
        *out_generalized_duty = 0.0f;
        return false;
    }

    float v_l_per_duty = v_l_cmd * ctx->mod.inv_d_max;
    if (!algo_pid_finite(v_l_per_duty)) {
        *out_generalized_duty = 0.0f;
        return false;
    }

    float numerator = v_l_per_duty + ctx->vcap;
    if (!algo_pid_finite(numerator)) {
        *out_generalized_duty = 0.0f;
        return false;
    }

    float generalized_duty = numerator * ctx->inv_bus_sum;
    if (!algo_pid_finite(generalized_duty)) {
        *out_generalized_duty = 0.0f;
        return false;
    }

    *out_generalized_duty = clampf(generalized_duty, ctx->mod.g_min, ctx->mod.g_max);
    return true;
}

/* ============================================================================
 * VCAP 动态限幅: 仅在 cap 侧过压区启用，低压区释放以保持最大调制性能
 *
 * 进入: |VCAP| >= limit
 * 退出: |VCAP| <= limit * BUCKBOOST_VCAP_LIMIT_EXIT_RATIO
 *
 * 正向过压时，最大允许 g 按目标最高输出电压反推:
 *   g_limit = V_LIMIT / (VOUT + V_LIMIT)
 * 这样 VCAP > V_LIMIT 时会产生抑制继续升压的调制边界。
 * ============================================================================ */

ATTR_RAMFUNC
static bool
    buckboost_update_vcap_limit_state(ctrl_buckboost_t* ctrl, float vcap_abs, float voltage_limit) {
    if (ctrl == NULL || !algo_pid_finite(vcap_abs) || !algo_pid_finite(voltage_limit)
        || voltage_limit <= 0.0f) {
        return false;
    }

    float enter = voltage_limit * BUCKBOOST_VCAP_LIMIT_ENTER_RATIO;
    float exit = voltage_limit * BUCKBOOST_VCAP_LIMIT_EXIT_RATIO;
    if (!algo_pid_finite(enter) || !algo_pid_finite(exit) || exit < 0.0f || exit > enter) {
        return false;
    }

    if (ctrl->state.vcap_limit_active) {
        if (vcap_abs <= exit) {
            ctrl->state.vcap_limit_active = false;
        }
    } else if (vcap_abs >= enter) {
        ctrl->state.vcap_limit_active = true;
    }

    return true;
}

ATTR_RAMFUNC
static bool buckboost_limit_generalized_duty_by_vcap(
    ctrl_buckboost_t* ctrl, float vcap, float voltage_limit, const buckboost_current_ctx_t* ctx,
    float* io_generalized_duty) {
    if (ctrl == NULL || ctx == NULL || io_generalized_duty == NULL
        || !algo_pid_finite(*io_generalized_duty)) {
        return false;
    }

    float limit = absf_fast(voltage_limit);
    float vcap_abs = absf_fast(vcap);
    if (!algo_pid_finite(limit) || limit <= 0.0f || !algo_pid_finite(vcap_abs)) {
        return false;
    }
    if (!buckboost_update_vcap_limit_state(ctrl, vcap_abs, limit)) {
        return false;
    }
    if (!ctrl->state.vcap_limit_active) {
        return true;
    }

    float limit_sum = ctx->vout + limit;
    if (!algo_pid_finite(limit_sum) || limit_sum <= BUCKBOOST_BUS_SUM_MIN_V) {
        return false;
    }

    float dynamic_g = limit / limit_sum;
    if (!algo_pid_finite(dynamic_g)) {
        return false;
    }
    dynamic_g = clampf(dynamic_g, ctx->mod.g_min, ctx->mod.g_max);

    if (vcap >= 0.0f) {
        if (*io_generalized_duty > dynamic_g) {
            *io_generalized_duty = dynamic_g;
        }
    } else if (*io_generalized_duty < dynamic_g) {
        *io_generalized_duty = dynamic_g;
    }

    return algo_pid_finite(*io_generalized_duty);
}

ATTR_RAMFUNC
static void buckboost_set_safe_output(ctrl_buckboost_t* ctrl) {
    if (ctrl == NULL) {
        return;
    }
    ctrl->state.v_cmd = 0.0f;
    ctrl->state.generalized_duty = 0.0f;
    ctrl->state.vcap_limit_active = false;
    ctrl->state.duty_a = 0.0f;
    ctrl->state.duty_b = 0.0f;
}

/* ============================================================================
 * 初始化 / 使能 / 禁能
 * ============================================================================ */

int ctrl_buckboost_init(ctrl_buckboost_t* ctrl) {
    if (ctrl == NULL)
        return -1;
    memset(ctrl, 0, sizeof(*ctrl));

    ctrl->params.duty_min = BUCKBOOST_DUTY_MIN_DEFAULT;
    ctrl->params.duty_max = BUCKBOOST_DUTY_MAX_DEFAULT;
    ctrl->params.i_l_limit_a = BUCKBOOST_I_L_LIMIT_DEFAULT;
    ctrl->params.v_out_limit_v = BUCKBOOST_V_OUT_LIMIT_DEFAULT;
    ctrl->params.voltage_ff_gain = 0.8f;

    algo_pid_ctor(&ctrl->state.current_pid);

    algo_pid_cfg_t inductor_current_pid_cfg = {
        .mode = ALGO_PID_MODE_INCREMENTAL,
        .kp = 0.125f,
        .ki = 325.0f,
        .kd = 0.0f,
        .sample_time_s = 1.0f / 100000.0f,
        .out_min = BUCKBOOST_CURRENT_PID_OUT_MIN,
        .out_max = BUCKBOOST_CURRENT_PID_OUT_MAX,
        .integral_min = BUCKBOOST_CURRENT_PID_OUT_MIN,
        .integral_max = BUCKBOOST_CURRENT_PID_OUT_MAX,
        .antiwindup = ALGO_PID_ANTIWINDUP_CLAMP,
        .backcalc_coeff = 1.0f,
        .deriv_filter_coeff = 0.0f,
        .deriv_on_measurement = true,
        .rate_limit = 0.0f,
        .setpoint_weight_p = 1.0f,
        .setpoint_weight_d = 0.0f,
    };
    ctrl->state.current_pid.init(&ctrl->state.current_pid, &inductor_current_pid_cfg);

    algo_pid_ctor(&ctrl->state.voltage_pid);
    algo_pid_cfg_t voltage_pid_cfg = {
        .mode = ALGO_PID_MODE_POSITIONAL,
        .kp = 1.0f,
        .ki = 500.0f,
        .kd = 0.0f,
        .sample_time_s = 1.0f / 50000.0f,
        .out_min = BUCKBOOST_VOLTAGE_PID_OUT_MIN,
        .out_max = BUCKBOOST_VOLTAGE_PID_OUT_MAX,
        .integral_min = BUCKBOOST_VOLTAGE_PID_OUT_MIN,
        .integral_max = BUCKBOOST_VOLTAGE_PID_OUT_MAX,
        .antiwindup = ALGO_PID_ANTIWINDUP_CLAMP,
        .backcalc_coeff = 1.0f,
        .deriv_filter_coeff = 0.0f,
        .deriv_on_measurement = true,
        .rate_limit = 0.0f,
        .setpoint_weight_p = 1.0f,
        .setpoint_weight_d = 0.0f,
    };
    ctrl->state.voltage_pid.init(&ctrl->state.voltage_pid, &voltage_pid_cfg);

    algo_pid_ctor(&ctrl->state.current_cc_pid);
    algo_pid_cfg_t cc_pid_cfg = {
        .mode = ALGO_PID_MODE_INCREMENTAL,
        .kp = 0.25f,
        .ki = 1000.0f,
        .kd = 0.0f,
        .sample_time_s = 1.0f / 50000.0f,
        .out_min = -BUCKBOOST_I_L_LIMIT_DEFAULT,
        .out_max = BUCKBOOST_I_L_LIMIT_DEFAULT,
        .integral_min = -BUCKBOOST_I_L_LIMIT_DEFAULT,
        .integral_max = BUCKBOOST_I_L_LIMIT_DEFAULT,
        .antiwindup = ALGO_PID_ANTIWINDUP_CLAMP,
        .backcalc_coeff = 1.0f,
        .deriv_filter_coeff = 0.0f,
        .deriv_on_measurement = true,
        .rate_limit = 0.0f,
        .setpoint_weight_p = 1.0f,
        .setpoint_weight_d = 0.0f,
    };
    ctrl->state.current_cc_pid.init(&ctrl->state.current_cc_pid, &cc_pid_cfg);

    algo_pid_ctor(&ctrl->state.power_pid);
    algo_pid_cfg_t power_pid_cfg = {
        .mode = ALGO_PID_MODE_INCREMENTAL,
        .kp = 0.025f,
        .ki = 128.0f,
        .kd = 0.0f,
        .sample_time_s = 1.0f / 25000.0f,
        .out_min = BUCKBOOST_POWER_PID_OUT_MIN,
        .out_max = BUCKBOOST_POWER_PID_OUT_MAX,
        .integral_min = BUCKBOOST_POWER_PID_OUT_MIN,
        .integral_max = BUCKBOOST_POWER_PID_OUT_MAX,
        .antiwindup = ALGO_PID_ANTIWINDUP_CLAMP,
        .backcalc_coeff = 1.0f,
        .deriv_filter_coeff = 0.0f,
        .deriv_on_measurement = true,
        .rate_limit = 0.0f,
        .setpoint_weight_p = 1.0f,
        .setpoint_weight_d = 0.0f,
    };
    ctrl->state.power_pid.init(&ctrl->state.power_pid, &power_pid_cfg);
    ctrl->params.power_pid = (ctrl_buckboost_pid_params_t){
        .kp = power_pid_cfg.kp,
        .ki = power_pid_cfg.ki,
        .kd = power_pid_cfg.kd,
        .integral_max = BUCKBOOST_POWER_PID_OUT_MAX,
        .output_max = BUCKBOOST_POWER_PID_OUT_MAX,
        .output_min = BUCKBOOST_POWER_PID_OUT_MIN,
    };

    ctrl->state.v_out_target_v = BUCKBOOST_V_OUT_TARGET_DEFAULT;
    ctrl->state.i_cap_target_a = BUCKBOOST_I_CAP_TARGET_DEFAULT;
    ctrl->state.p_target_w = BUCKBOOST_P_TARGET_DEFAULT;
    ctrl->params.p_target_w = BUCKBOOST_P_TARGET_DEFAULT;

    return 0;
}

int ctrl_buckboost_enable(ctrl_buckboost_t* ctrl) {
    if (ctrl == NULL)
        return -1;
    ctrl->state.enabled = true;
    ctrl->state.v_cmd = 0.0f;
    ctrl->state.generalized_duty = 0.0f;
    ctrl->state.vcap_limit_active = false;
    ctrl->state.duty_a = 0.0f;
    ctrl->state.duty_b = 0.0f;
    ctrl->state.current_ref = 0.0f;
    if (ctrl->state.current_pid.reset) {
        ctrl->state.current_pid.reset(&ctrl->state.current_pid);
    }
    ctrl->state.voltage_pid_out = 0.0f;
    if (ctrl->state.voltage_pid.reset) {
        ctrl->state.voltage_pid.reset(&ctrl->state.voltage_pid);
    }
    ctrl->state.power_pid_out = 0.0f;
    if (ctrl->state.power_pid.reset) {
        ctrl->state.power_pid.reset(&ctrl->state.power_pid);
    }
    return 0;
}

void ctrl_buckboost_disable(ctrl_buckboost_t* ctrl) {
    if (ctrl == NULL)
        return;
    ctrl->state.enabled = false;
    ctrl->state.v_cmd = 0.0f;
    ctrl->state.generalized_duty = 0.0f;
    ctrl->state.vcap_limit_active = false;
    ctrl->state.duty_a = 0.0f;
    ctrl->state.duty_b = 0.0f;
    ctrl->state.voltage_pid_out = 0.0f;
    ctrl->state.power_pid_out = 0.0f;
}

void ctrl_buckboost_emergency_stop(ctrl_buckboost_t* ctrl) {
    if (ctrl == NULL)
        return;
    ctrl->state.enabled = false;
    ctrl->state.v_cmd = 0.0f;
    ctrl->state.generalized_duty = 0.0f;
    ctrl->state.vcap_limit_active = false;
    ctrl->state.duty_a = 0.0f;
    ctrl->state.duty_b = 0.0f;
    ctrl->state.current_ref = 0.0f;
    ctrl->state.voltage_pid_out = 0.0f;
    if (ctrl->state.current_pid.reset) {
        ctrl->state.current_pid.reset(&ctrl->state.current_pid);
    }
    if (ctrl->state.voltage_pid.reset) {
        ctrl->state.voltage_pid.reset(&ctrl->state.voltage_pid);
    }
}

/* ============================================================================
 * 调制器接口
 * ============================================================================ */

ATTR_RAMFUNC
void ctrl_buckboost_modulate(ctrl_buckboost_t* ctrl, float generalized_duty) {
    if (ctrl == NULL || !ctrl->state.enabled) {
        return;
    }

    float d_min = ctrl->params.duty_min;
    float d_max = ctrl->params.duty_max;
    buckboost_mod_ctx_t mod_ctx;
    if (!buckboost_prepare_mod_ctx(d_min, d_max, &mod_ctx)) {
        buckboost_set_safe_output(ctrl);
        return;
    }

    float da = 0.0f;
    float db = 0.0f;
    float g = 0.0f;
    if (!buckboost_modulate_generalized(generalized_duty, &mod_ctx, &da, &db, &g)) {
        buckboost_set_safe_output(ctrl);
        return;
    }

    ctrl->state.generalized_duty = g;
    ctrl->state.duty_a = da;
    ctrl->state.duty_b = db;
}

/* ============================================================================
 * 电流内环 update (200kHz)
 *
 * PI(current_ref, i_l) → V_L_cmd (V)
 * VOUT/VCAP feedforward → generalized_duty
 * modulate(generalized_duty) → DA, DB
 * ============================================================================ */

ATTR_RAMFUNC
void ctrl_buckboost_update_current(ctrl_buckboost_t* ctrl, float i_l, float vout, float vcap) {
    if (ctrl == NULL || !ctrl->state.enabled) {
        return;
    }

    buckboost_current_ctx_t ctx;
    if (!buckboost_prepare_current_ctx(ctrl, i_l, vout, vcap, &ctx)) {
        buckboost_set_safe_output(ctrl);
        return;
    }

    /* 1. PID: 电流误差 → 有符号平均电感电压命令 */
    float v_l_cmd =
        ctrl->state.current_pid.step(&ctrl->state.current_pid, ctrl->state.current_ref, i_l);

    if (!algo_pid_finite(v_l_cmd)) {
        buckboost_set_safe_output(ctrl);
        return;
    }

    float voltage_limit = absf_fast(ctrl->params.v_out_limit_v);
    if (!algo_pid_finite(voltage_limit) || voltage_limit <= 0.0f) {
        voltage_limit = BUCKBOOST_V_OUT_LIMIT_DEFAULT;
    }
    v_l_cmd = clampf(v_l_cmd, -voltage_limit, voltage_limit);

    /* 2. 前馈: V_L_cmd + VOUT/VCAP → generalized_duty，并按 VCAP 双向限幅 */
    float generalized_duty = 0.0f;
    if (!buckboost_vl_cmd_to_generalized_duty(v_l_cmd, &ctx, &generalized_duty)) {
        buckboost_set_safe_output(ctrl);
        return;
    }
    if (!buckboost_limit_generalized_duty_by_vcap(
            ctrl, vcap, voltage_limit, &ctx, &generalized_duty)) {
        buckboost_set_safe_output(ctrl);
        return;
    }

    /* 3. 调制器: generalized_duty → DA/DB */
    float da = 0.0f;
    float db = 0.0f;
    float effective_generalized_duty = 0.0f;
    if (!buckboost_modulate_generalized(
            generalized_duty, &ctx.mod, &da, &db, &effective_generalized_duty)) {
        buckboost_set_safe_output(ctrl);
        return;
    }

    ctrl->state.v_cmd = v_l_cmd;
    ctrl->state.generalized_duty = effective_generalized_duty;
    ctrl->state.duty_a = da;
    ctrl->state.duty_b = db;
}

/* ============================================================================
 * VOUT 电压外环 + cap 侧估算电流限流环 update (50kHz)
 * ============================================================================ */

ATTR_RAMFUNC
void ctrl_buckboost_update_voltage(
    ctrl_buckboost_t* ctrl, float vout, float i_cap_ff, float i_cap_est) {
    if (ctrl == NULL || !ctrl->state.enabled) {
        return;
    }
    if (!algo_pid_finite(vout) || !algo_pid_finite(ctrl->state.v_out_target_v)) {
        return;
    }

    float i_limit = ctrl->params.i_l_limit_a;
    if (i_limit <= 0.0f)
        i_limit = BUCKBOOST_I_L_LIMIT_DEFAULT;

    /* CV: PI + 前馈 */
    float i_ref_cv =
        ctrl->state.voltage_pid.step(&ctrl->state.voltage_pid, ctrl->state.v_out_target_v, vout)
        + i_cap_ff * ctrl->params.voltage_ff_gain;
    if (!algo_pid_finite(i_ref_cv))
        i_ref_cv = 0.0f;
    i_ref_cv = clampf(i_ref_cv, -i_limit, i_limit);
    ctrl->state.voltage_pid_out = i_ref_cv;

    /* CC: PI(i_cap_target, i_cap_est) */
    float i_cap_target = ctrl->state.i_cap_target_a;
    float i_ref_cc =
        ctrl->state.current_cc_pid.step(&ctrl->state.current_cc_pid, i_cap_target, i_cap_est);
    if (!algo_pid_finite(i_ref_cc))
        i_ref_cc = 0.0f;
    i_ref_cc = clampf(i_ref_cc, -i_limit, i_limit);
    ctrl->state.cc_pid_out = i_ref_cc;

    /* CV/CC 竞争: 取更保守的值 */
    float i_ref_cmd = (i_cap_target >= 0.0f) ? ((i_ref_cv < i_ref_cc) ? i_ref_cv : i_ref_cc)
                                             : ((i_ref_cv > i_ref_cc) ? i_ref_cv : i_ref_cc);

    if (ctrl->state.target_type == BUCKBOOST_TARGET_CV
        || ctrl->state.target_type == BUCKBOOST_TARGET_CW) {
        (void)i_ref_cmd; /* Current-loop tuning: keep outer-loop command disconnected for now. */
        // ctrl->state.current_ref = i_ref_cmd;
    }
}

/* ============================================================================
 * CW 功率外环 update (25kHz): p_in = VOUT × IIN
 * ============================================================================ */

ATTR_RAMFUNC
void ctrl_buckboost_update_power(ctrl_buckboost_t* ctrl, float p_in) {
    if (ctrl == NULL || !ctrl->state.enabled) {
        return;
    }

    ctrl->state.p_in_w = p_in;

    if (!algo_pid_finite(p_in) || !algo_pid_finite(ctrl->state.p_target_w)) {
        return;
    }

    float vout_target_cmd =
        ctrl->state.power_pid.step(&ctrl->state.power_pid, ctrl->state.p_target_w, p_in);
    if (!algo_pid_finite(vout_target_cmd)) {
        vout_target_cmd = 0.0f;
    }

    float v_limit = ctrl->params.v_out_limit_v;
    if (!algo_pid_finite(v_limit) || v_limit <= 0.0f) {
        v_limit = BUCKBOOST_POWER_PID_OUT_MAX;
    }
    vout_target_cmd = clampf(vout_target_cmd, -v_limit, v_limit);

    ctrl->state.power_pid_out = vout_target_cmd;

    /* CW 模式级联: 功率环输出 → signed v_out_target_v → 电压环 → signed current_ref */
    if (ctrl->state.target_type == BUCKBOOST_TARGET_CW) {
        ctrl->state.v_out_target_v = vout_target_cmd;
    }
}

/* ============================================================================
 * 参数配置
 * ============================================================================ */

void ctrl_buckboost_set_vout_target(ctrl_buckboost_t* ctrl, float target_v) {
    if (ctrl != NULL)
        ctrl->state.v_out_target_v = target_v;
}

void ctrl_buckboost_enter_cv_mode(ctrl_buckboost_t* ctrl, float target_v) {
    if (ctrl == NULL || !algo_pid_finite(target_v) || target_v <= 0.0f)
        return;
    ctrl->state.v_out_target_v = target_v;
    ctrl->state.target_type = BUCKBOOST_TARGET_CV;
    if (ctrl->state.voltage_pid.reset) {
        ctrl->state.voltage_pid.reset(&ctrl->state.voltage_pid);
    }
    ctrl->state.voltage_pid_out = 0.0f;
}

void ctrl_buckboost_enter_cw_mode(ctrl_buckboost_t* ctrl, float target_w) {
    if (ctrl == NULL || !algo_pid_finite(target_w))
        return;
    ctrl->state.p_target_w = target_w;
    ctrl->params.p_target_w = target_w;
    ctrl->state.target_type = BUCKBOOST_TARGET_CW;
    float i_cap_limit = absf_fast(ctrl->state.i_cap_target_a);
    if (!algo_pid_finite(i_cap_limit) || i_cap_limit <= 0.0f) {
        i_cap_limit = BUCKBOOST_I_CAP_TARGET_DEFAULT;
    }
    ctrl->state.i_cap_target_a = (target_w < 0.0f) ? -i_cap_limit : i_cap_limit;
}

void ctrl_buckboost_set_il_target(ctrl_buckboost_t* ctrl, float target_a) {
    if (ctrl == NULL)
        return;
    if (!algo_pid_finite(target_a)) {
        ctrl->state.current_ref = 0.0f;
        ctrl->state.i_l_target_a = 0.0f;
        if (ctrl->state.current_pid.reset) {
            ctrl->state.current_pid.reset(&ctrl->state.current_pid);
        }
        buckboost_set_safe_output(ctrl);
        return;
    }
    float i_l_limit = absf_fast(ctrl->params.i_l_limit_a);
    if (!algo_pid_finite(i_l_limit) || i_l_limit <= 0.0f) {
        i_l_limit = BUCKBOOST_I_L_LIMIT_DEFAULT;
    }
    /* 不 reset 内环 PID：级联时外环周期更新 ref，reset 会破坏积分连续性 */
    ctrl->state.target_type = BUCKBOOST_TARGET_CC;
    ctrl->state.current_ref = clampf(target_a, -i_l_limit, i_l_limit);
    ctrl->state.i_l_target_a = target_a;
}

void ctrl_buckboost_set_icap_target(ctrl_buckboost_t* ctrl, float target_a) {
    if (ctrl == NULL)
        return;
    ctrl->state.i_cap_target_a = target_a;
}

void ctrl_buckboost_set_ptarget(ctrl_buckboost_t* ctrl, float target_w) {
    if (ctrl == NULL || !algo_pid_finite(target_w))
        return;
    ctrl->state.p_target_w = target_w;
    ctrl->params.p_target_w = target_w;
    if (ctrl->state.target_type == BUCKBOOST_TARGET_CW) {
        float i_cap_limit = absf_fast(ctrl->state.i_cap_target_a);
        if (!algo_pid_finite(i_cap_limit) || i_cap_limit <= 0.0f) {
            i_cap_limit = BUCKBOOST_I_CAP_TARGET_DEFAULT;
        }
        ctrl->state.i_cap_target_a = (target_w < 0.0f) ? -i_cap_limit : i_cap_limit;
    }
}

void ctrl_buckboost_set_target_type(ctrl_buckboost_t* ctrl, ctrl_buckboost_target_t target) {
    if (ctrl != NULL)
        ctrl->state.target_type = target;
}

void ctrl_buckboost_set_params(ctrl_buckboost_t* ctrl, const ctrl_buckboost_params_t* params) {
    if (ctrl == NULL || params == NULL)
        return;
    ctrl->params = *params;

    if (ctrl->state.current_pid.set_gains) {
        ctrl->state.current_pid.set_gains(
            &ctrl->state.current_pid, params->current_pid.kp, params->current_pid.ki,
            params->current_pid.kd);
    }
    if (ctrl->state.voltage_pid.set_gains) {
        ctrl->state.voltage_pid.set_gains(
            &ctrl->state.voltage_pid, params->voltage_pid.kp, params->voltage_pid.ki,
            params->voltage_pid.kd);
        float i_limit = (algo_pid_finite(params->i_l_limit_a) && params->i_l_limit_a > 0.0f)
                          ? params->i_l_limit_a
                          : BUCKBOOST_I_L_LIMIT_DEFAULT;
        ctrl->state.voltage_pid._out_min = -i_limit;
        ctrl->state.voltage_pid._out_max = i_limit;
        ctrl->state.voltage_pid._integral_min = -i_limit;
        ctrl->state.voltage_pid._integral_max = i_limit;
    }
    if (ctrl->state.power_pid.set_gains) {
        ctrl->state.power_pid.set_gains(
            &ctrl->state.power_pid, params->power_pid.kp, params->power_pid.ki,
            params->power_pid.kd);
        float v_limit = (algo_pid_finite(params->v_out_limit_v) && params->v_out_limit_v > 0.0f)
                          ? params->v_out_limit_v
                          : BUCKBOOST_V_OUT_LIMIT_DEFAULT;
        ctrl->state.power_pid._out_min = BUCKBOOST_POWER_PID_OUT_MIN;
        ctrl->state.power_pid._out_max = v_limit;
        ctrl->state.power_pid._integral_min = BUCKBOOST_POWER_PID_OUT_MIN;
        ctrl->state.power_pid._integral_max = v_limit;
    }
    if (algo_pid_finite(params->p_target_w)) {
        ctrl->state.p_target_w = params->p_target_w;
    }
    /* 同步前馈增益 (set_params 覆盖 params,需显式写回) */
    ctrl->params.voltage_ff_gain = params->voltage_ff_gain;
}

/* ============================================================================
 * 输出读取
 * ============================================================================ */

ATTR_RAMFUNC
float ctrl_buckboost_get_duty_a(const ctrl_buckboost_t* ctrl) {
    return (ctrl != NULL) ? ctrl->state.duty_a : 0.0f;
}

ATTR_RAMFUNC
float ctrl_buckboost_get_duty_b(const ctrl_buckboost_t* ctrl) {
    return (ctrl != NULL) ? ctrl->state.duty_b : 0.0f;
}

float ctrl_buckboost_get_v_cmd(const ctrl_buckboost_t* ctrl) {
    return (ctrl != NULL) ? ctrl->state.v_cmd : 0.0f;
}

float ctrl_buckboost_get_generalized_duty(const ctrl_buckboost_t* ctrl) {
    return (ctrl != NULL) ? ctrl->state.generalized_duty : 0.0f;
}

float ctrl_buckboost_get_duty_max(const ctrl_buckboost_t* ctrl) {
    return (ctrl != NULL) ? ctrl->params.duty_max : 0.0f;
}

float ctrl_buckboost_get_current_ref(const ctrl_buckboost_t* ctrl) {
    return (ctrl != NULL) ? ctrl->state.current_ref : 0.0f;
}

float ctrl_buckboost_get_ptarget(const ctrl_buckboost_t* ctrl) {
    return (ctrl != NULL) ? ctrl->state.p_target_w : 0.0f;
}

float ctrl_buckboost_get_power_pid_out(const ctrl_buckboost_t* ctrl) {
    return (ctrl != NULL) ? ctrl->state.power_pid_out : 0.0f;
}

bool ctrl_buckboost_is_enabled(const ctrl_buckboost_t* ctrl) {
    return (ctrl != NULL) ? ctrl->state.enabled : false;
}
