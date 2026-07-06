/*
 * PID Controller Implementation
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#include "algo_pid.h"

#include <stddef.h>

static inline float algo_pid_clamp(float x, float lo, float hi)
{
    if (!algo_pid_finite(x)) return lo;
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

const algo_pid_cfg_t ALGO_PID_CFG_DEFAULT = {
    .mode                  = ALGO_PID_MODE_POSITIONAL,
    .kp                    = 1.0f,
    .ki                    = 0.0f,
    .kd                    = 0.0f,
    .sample_time_s         = 0.001f,
    .out_min               = -1.0f,
    .out_max               = 1.0f,
    .integral_min          = -1.0f,
    .integral_max          = 1.0f,
    .antiwindup            = ALGO_PID_ANTIWINDUP_CLAMP,
    .backcalc_coeff        = 1.0f,
    .deriv_filter_coeff    = 0.0f,
    .deriv_on_measurement  = true,
    .rate_limit            = 0.0f,
    .setpoint_weight_p     = 1.0f,
    .setpoint_weight_d     = 0.0f,
};

static int   algo_pid_init_impl(algo_pid_t *self, const algo_pid_cfg_t *cfg);
static float algo_pid_step_pos_impl(algo_pid_t *self, float sp, float pv);
static float algo_pid_step_inc_impl(algo_pid_t *self, float sp, float pv);
static void  algo_pid_reset_impl(algo_pid_t *self);
static void  algo_pid_set_gains_impl(algo_pid_t *self, float kp, float ki, float kd);
static void  algo_pid_get_gains_impl(const algo_pid_t *self, float *kp, float *ki, float *kd);

static inline float algo_pid_deriv(const algo_pid_t *self,
                                   float sp, float pv,
                                   float prev_sp, float prev_pv)
{
    if (self->_deriv_on_pv) {
        return self->_kd_invT * (prev_pv - pv);
    } else {
        float d_sp = self->_spw_d * (sp - prev_sp);
        float d_pv = pv - prev_pv;
        return self->_kd_invT * (d_sp - d_pv);
    }
}

static inline float algo_pid_filter_d(algo_pid_t *self, float raw)
{
    if (!self->_has_deriv_filter) return raw;

    float f = self->_deriv_fc * raw + (1.0f - self->_deriv_fc) * self->_deriv_state;
    self->_deriv_state = f;
    return f;
}

/* ── Anti-windup: discrete backcalc_coeff is used directly, no implicit Ts ── */

static inline float algo_pid_antiwindup(algo_pid_t *self, float raw, float sat)
{
    if (self->_aw_strategy == ALGO_PID_ANTIWINDUP_BACKCALC) {
        self->_integral += self->_backcalc_coeff * (sat - raw);
        self->_integral  = algo_pid_clamp(self->_integral,
                                          self->_integral_min, self->_integral_max);
    } else if (self->_aw_strategy == ALGO_PID_ANTIWINDUP_CLAMP && raw != sat) {
        self->_integral -= (raw - sat);
        self->_integral  = algo_pid_clamp(self->_integral,
                                          self->_integral_min, self->_integral_max);
    }
    return sat;
}

static inline float algo_pid_ratelimit(algo_pid_t *self, float out)
{
    if (!self->_has_rate_limit) return out;

    float max_d = self->_rate_limit_T;
    float delta = out - self->_prev_output;
    if (delta >  max_d) return self->_prev_output + max_d;
    if (delta < -max_d) return self->_prev_output - max_d;
    return out;
}

/* ── Prime state on first valid input to avoid derivative kick ────────── */

ALGO_ATTR_RAMFUNC
static void algo_pid_prime(algo_pid_t *self, float sp, float pv)
{
    if (self->_primed) return;

    self->_prev_error         = sp - pv;
    self->_prev2_error        = self->_prev_error;
    self->_prev_setpoint      = sp;
    self->_prev2_setpoint     = sp;
    self->_prev_measurement   = pv;
    self->_prev2_measurement  = pv;
    self->_prev_dout          = 0.0f;
    self->_primed             = true;
}

/* ── Positional step ─────────────────────────────────────────────────── */

ALGO_ATTR_RAMFUNC
static float algo_pid_step_pos_impl(algo_pid_t *self, float sp, float pv)
{
    if (self == NULL || !self->_initialized) return 0.0f;

    if (!algo_pid_finite(sp) || !algo_pid_finite(pv)) {
        return self->_prev_output;
    }

    algo_pid_prime(self, sp, pv);

    float err   = sp - pv;
    float p_out = self->_kp * (self->_spw_p * sp - pv);

    self->_integral += self->_ki_T * err;
    self->_integral  = algo_pid_clamp(self->_integral,
                                      self->_integral_min, self->_integral_max);

    float d_raw = algo_pid_deriv(self, sp, pv,
                                  self->_prev_setpoint, self->_prev_measurement);
    float d_out = algo_pid_filter_d(self, d_raw);

    float raw  = p_out + self->_integral + d_out;
    float sat  = algo_pid_clamp(raw, self->_out_min, self->_out_max);

    float out  = algo_pid_antiwindup(self, raw, sat);
    out        = algo_pid_ratelimit(self, out);

    if (!algo_pid_finite(out)) {
        return self->_prev_output;
    }

    self->_prev_error       = err;
    self->_prev_setpoint    = sp;
    self->_prev_measurement = pv;
    self->_prev_output      = out;
    self->_prev_dout        = d_out;

    return out;
}

/* ── Incremental (velocity) step ─────────────────────────────────────── */

ALGO_ATTR_RAMFUNC
static float algo_pid_step_inc_impl(algo_pid_t *self, float sp, float pv)
{
    if (self == NULL || !self->_initialized) return 0.0f;

    if (!algo_pid_finite(sp) || !algo_pid_finite(pv)) {
        return self->_prev_output;
    }

    algo_pid_prime(self, sp, pv);

    float err    = sp - pv;
    float dp     = self->_kp * (err - self->_prev_error);
    float di     = self->_ki_T * err;

    float d_raw  = algo_pid_deriv(self, sp, pv,
                                  self->_prev_setpoint, self->_prev_measurement);
    float d_out  = algo_pid_filter_d(self, d_raw);
    float dd     = d_out - self->_prev_dout;

    float out    = self->_prev_output + dp + di + dd;
    out          = algo_pid_clamp(out, self->_out_min, self->_out_max);
    out          = algo_pid_ratelimit(self, out);

    if (!algo_pid_finite(out)) {
        return self->_prev_output;
    }

    self->_prev2_error       = self->_prev_error;
    self->_prev_error        = err;
    self->_prev2_setpoint    = self->_prev_setpoint;
    self->_prev_setpoint     = sp;
    self->_prev2_measurement = self->_prev_measurement;
    self->_prev_measurement  = pv;
    self->_prev_output       = out;
    self->_prev_dout         = d_out;

    return out;
}

/* ── Public methods ──────────────────────────────────────────────────── */

static int algo_pid_init_impl(algo_pid_t *self, const algo_pid_cfg_t *cfg)
{
    if (self == NULL)                            return -1;
    if (cfg == NULL)                             return -2;
    if (!algo_pid_finite(cfg->sample_time_s))    return -3;
    if (cfg->sample_time_s <= 0.0f)              return -3;
    if (!algo_pid_finite(cfg->out_min))          return -4;
    if (!algo_pid_finite(cfg->out_max))          return -4;
    if (cfg->out_min >= cfg->out_max)            return -4;
    if (!algo_pid_finite(cfg->integral_min))     return -5;
    if (!algo_pid_finite(cfg->integral_max))     return -5;
    if (cfg->integral_min >= cfg->integral_max)  return -5;
    if (!algo_pid_finite(cfg->kp))               return -6;
    if (!algo_pid_finite(cfg->ki))               return -7;
    if (!algo_pid_finite(cfg->kd))               return -8;
    if (cfg->mode != ALGO_PID_MODE_POSITIONAL &&
        cfg->mode != ALGO_PID_MODE_INCREMENTAL)  return -9;
    if (!algo_pid_finite(cfg->backcalc_coeff))   return -9;
    if (cfg->backcalc_coeff < 0.0f)              return -9;
    if (cfg->antiwindup != ALGO_PID_ANTIWINDUP_NONE &&
        cfg->antiwindup != ALGO_PID_ANTIWINDUP_CLAMP &&
        cfg->antiwindup != ALGO_PID_ANTIWINDUP_BACKCALC) return -10;
    if (!algo_pid_finite(cfg->deriv_filter_coeff)) return -11;
    if (cfg->deriv_filter_coeff < 0.0f || cfg->deriv_filter_coeff > 1.0f) return -11;
    if (!algo_pid_finite(cfg->rate_limit))       return -12;
    if (cfg->rate_limit < 0.0f)                  return -12;
    if (!algo_pid_finite(cfg->setpoint_weight_p)) return -13;
    if (cfg->setpoint_weight_p < 0.0f || cfg->setpoint_weight_p > 1.0f) return -13;
    if (!algo_pid_finite(cfg->setpoint_weight_d)) return -14;
    if (cfg->setpoint_weight_d < 0.0f || cfg->setpoint_weight_d > 1.0f) return -14;

    self->_kp             = cfg->kp;
    self->_ki             = cfg->ki;
    self->_kd             = cfg->kd;
    self->_spw_p          = cfg->setpoint_weight_p;
    self->_spw_d          = cfg->setpoint_weight_d;
    self->_out_min        = cfg->out_min;
    self->_out_max        = cfg->out_max;
    self->_integral_min   = cfg->integral_min;
    self->_integral_max   = cfg->integral_max;
    self->_aw_strategy    = cfg->antiwindup;
    self->_backcalc_coeff = cfg->backcalc_coeff;
    self->_deriv_fc        = cfg->deriv_filter_coeff;
    self->_deriv_on_pv     = cfg->deriv_on_measurement;

    float Ts    = cfg->sample_time_s;
    float invT  = 1.0f / Ts;
    self->_inv_T         = invT;
    self->_ki_T          = cfg->ki * Ts;
    self->_kd_invT       = cfg->kd * invT;
    self->_rate_limit_T  = cfg->rate_limit * Ts;
    self->_has_rate_limit    = (cfg->rate_limit > 0.0f);
    self->_has_deriv_filter  = (cfg->deriv_filter_coeff > 0.0f &&
                                cfg->deriv_filter_coeff < 1.0f);

    self->step = (cfg->mode == ALGO_PID_MODE_INCREMENTAL)
                  ? algo_pid_step_inc_impl
                  : algo_pid_step_pos_impl;

    self->_integral           = 0.0f;
    self->_prev_error         = 0.0f;
    self->_prev2_error        = 0.0f;
    self->_prev_setpoint      = 0.0f;
    self->_prev2_setpoint     = 0.0f;
    self->_prev_measurement   = 0.0f;
    self->_prev2_measurement  = 0.0f;
    self->_prev_output        = 0.0f;
    self->_prev_dout          = 0.0f;
    self->_deriv_state        = 0.0f;
    self->_primed             = false;
    self->_initialized        = true;

    return 0;
}

static void algo_pid_reset_impl(algo_pid_t *self)
{
    if (self == NULL || !self->_initialized) return;

    self->_integral           = 0.0f;
    self->_prev_error         = 0.0f;
    self->_prev2_error        = 0.0f;
    self->_prev_setpoint      = 0.0f;
    self->_prev2_setpoint     = 0.0f;
    self->_prev_measurement   = 0.0f;
    self->_prev2_measurement  = 0.0f;
    self->_prev_output        = 0.0f;
    self->_prev_dout          = 0.0f;
    self->_deriv_state        = 0.0f;
    self->_primed             = false;
}

static void algo_pid_set_gains_impl(algo_pid_t *self, float kp, float ki, float kd)
{
    if (self == NULL || !self->_initialized) return;

    if (algo_pid_finite(kp)) {
        self->_kp = kp;
    }
    if (algo_pid_finite(ki)) {
        self->_ki   = ki;
        self->_ki_T = ki / self->_inv_T;
    }
    if (algo_pid_finite(kd)) {
        self->_kd      = kd;
        self->_kd_invT = kd * self->_inv_T;
    }
}

static void algo_pid_get_gains_impl(const algo_pid_t *self, float *kp, float *ki, float *kd)
{
    if (self == NULL || !self->_initialized) return;

    if (kp) *kp = self->_kp;
    if (ki) *ki = self->_ki;
    if (kd) *kd = self->_kd;
}

/* ── Constructor ─────────────────────────────────────────────────────── */

void algo_pid_ctor(algo_pid_t *self)
{
    if (self == NULL) return;

    self->init      = algo_pid_init_impl;
    self->step      = NULL;
    self->reset     = algo_pid_reset_impl;
    self->set_gains = algo_pid_set_gains_impl;
    self->get_gains = algo_pid_get_gains_impl;

    self->_kp               = 0.0f;
    self->_ki               = 0.0f;
    self->_kd               = 0.0f;
    self->_ki_T             = 0.0f;
    self->_kd_invT          = 0.0f;
    self->_inv_T            = 0.0f;
    self->_rate_limit_T     = 0.0f;
    self->_spw_p            = 0.0f;
    self->_spw_d            = 0.0f;
    self->_out_min          = 0.0f;
    self->_out_max          = 0.0f;
    self->_integral_min     = 0.0f;
    self->_integral_max     = 0.0f;
    self->_backcalc_coeff   = 0.0f;
    self->_deriv_fc         = 0.0f;
    self->_deriv_on_pv      = false;
    self->_integral         = 0.0f;
    self->_prev_error       = 0.0f;
    self->_prev2_error      = 0.0f;
    self->_prev_setpoint    = 0.0f;
    self->_prev2_setpoint   = 0.0f;
    self->_prev_measurement = 0.0f;
    self->_prev2_measurement = 0.0f;
    self->_prev_output      = 0.0f;
    self->_prev_dout        = 0.0f;
    self->_deriv_state      = 0.0f;
    self->_primed           = false;
    self->_aw_strategy      = ALGO_PID_ANTIWINDUP_NONE;
    self->_has_rate_limit   = false;
    self->_has_deriv_filter = false;
    self->_initialized      = false;
}
