/*
 * HRPWM Interface - High-Performance PWM hardware-independent contract
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef INTF_HRPWM_H
#define INTF_HRPWM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t intf_hrpwm_inst_t;
typedef uint8_t intf_hrpwm_ch_t;
typedef uint8_t intf_hrpwm_pair_t;

/* PWM中断回调函数类型 */
typedef void (*intf_hrpwm_irq_callback_t)(void);

typedef enum {
    INTF_HRPWM_FAULT_SRC_INTERNAL_0 = 0,
    INTF_HRPWM_FAULT_SRC_INTERNAL_1,
    INTF_HRPWM_FAULT_SRC_INTERNAL_2,
    INTF_HRPWM_FAULT_SRC_INTERNAL_3,
    INTF_HRPWM_FAULT_SRC_EXTERNAL_0,
    INTF_HRPWM_FAULT_SRC_EXTERNAL_1,
    INTF_HRPWM_FAULT_SRC_DEBUG,
} intf_hrpwm_fault_src_t;

typedef enum {
    INTF_HRPWM_FAULT_MODE_FORCE_LOW = 0,
    INTF_HRPWM_FAULT_MODE_FORCE_HIGH,
    INTF_HRPWM_FAULT_MODE_HIGH_Z,
} intf_hrpwm_fault_mode_t;

typedef enum {
    INTF_HRPWM_FAULT_RECOVERY_IMMEDIATELY = 0,
    INTF_HRPWM_FAULT_RECOVERY_ON_RELOAD,
    INTF_HRPWM_FAULT_RECOVERY_ON_HW_EVENT,
    INTF_HRPWM_FAULT_RECOVERY_ON_FAULT_CLEAR,
} intf_hrpwm_fault_recovery_t;

typedef enum {
    INTF_HRPWM_ALIGN_EDGE = 0,
    INTF_HRPWM_ALIGN_CENTER,
} intf_hrpwm_align_t;

typedef struct {
    uint32_t frequency_hz;
    float duty;
    uint32_t deadtime_ns;
    uint8_t jitter_cmp;
    intf_hrpwm_align_t align;
    bool invert_high_side;
    bool invert_low_side;
} intf_hrpwm_pair_cfg_t;

typedef struct {
    intf_hrpwm_fault_src_t source;
    intf_hrpwm_fault_mode_t mode;
    intf_hrpwm_fault_recovery_t recovery;
    bool active_low;
} intf_hrpwm_fault_cfg_t;

/* 移相配置 */
typedef struct {
    intf_hrpwm_inst_t inst;         /* PWM实例 (0或1) */
    intf_hrpwm_pair_t ref_pair;     /* 参考pair (0或1) */
    intf_hrpwm_pair_t target_pair;  /* 目标pair (0或1) */
    float phase_deg;                /* 移相角度 (0-max_phase_deg) */
} intf_hrpwm_phase_cfg_t;

/* 移相限制配置 */
typedef struct {
    float max_phase_deg;            /* 最大移相角度，默认180.0 */
    float max_duty_ref;             /* 参考pair最大占空比限制，默认1.0 */
    float max_duty_target;          /* 目标pair最大占空比限制，默认1.0 */
} intf_hrpwm_phase_limit_t;

typedef struct {
    uint8_t instance_id;
    struct {
        int (*init_pair)(intf_hrpwm_ch_t ch, const intf_hrpwm_pair_cfg_t *cfg);
        int (*set_duty)(intf_hrpwm_ch_t ch, float duty);
        int (*set_duty_direct)(intf_hrpwm_ch_t ch, float duty);
        int (*set_duty_direct_dual)(intf_hrpwm_ch_t ch_a, float duty_a,
                                    intf_hrpwm_ch_t ch_b, float duty_b);
        int (*set_frequency)(uint32_t frequency_hz);
        int (*set_jitter)(intf_hrpwm_ch_t ch, uint8_t jitter_cmp);
        int (*start)(intf_hrpwm_ch_t ch);
        int (*stop)(intf_hrpwm_ch_t ch);
        int (*force_low)(intf_hrpwm_ch_t ch);
        int (*force_release)(intf_hrpwm_ch_t ch);
        int (*config_fault)(const intf_hrpwm_fault_cfg_t *cfg);
        int (*clear_fault)(void);
        int (*config_reload_irq)(intf_hrpwm_irq_callback_t callback);
        int (*enable_reload_irq)(void);
        int (*disable_reload_irq)(void);
        int (*set_phase)(const intf_hrpwm_phase_cfg_t *cfg);
        int (*config_phase_limit)(const intf_hrpwm_phase_limit_t *limit);
        int (*config_trigger_cmp)(uint8_t cmp_index, float position_ratio);
        int (*set_trigger_cmp_position)(uint8_t cmp_index, float position_ratio);
    };
} intf_hrpwm_t;

int intf_hrpwm_register(const intf_hrpwm_t *ops);
int intf_hrpwm_init_pair(intf_hrpwm_ch_t ch, const intf_hrpwm_pair_cfg_t *cfg);
int intf_hrpwm_set_duty(intf_hrpwm_ch_t ch, float duty);
int intf_hrpwm_set_duty_direct(intf_hrpwm_ch_t ch, float duty);
int intf_hrpwm_set_duty_direct_dual(intf_hrpwm_ch_t ch_a, float duty_a,
                                    intf_hrpwm_ch_t ch_b, float duty_b);
int intf_hrpwm_set_frequency(intf_hrpwm_inst_t inst, uint32_t frequency_hz);
int intf_hrpwm_set_jitter(intf_hrpwm_ch_t ch, uint8_t jitter_cmp);
int intf_hrpwm_start(intf_hrpwm_ch_t ch);
int intf_hrpwm_stop(intf_hrpwm_ch_t ch);
int intf_hrpwm_force_low(intf_hrpwm_ch_t ch);
int intf_hrpwm_force_release(intf_hrpwm_ch_t ch);
int intf_hrpwm_config_fault(intf_hrpwm_inst_t inst, const intf_hrpwm_fault_cfg_t *cfg);
int intf_hrpwm_clear_fault(intf_hrpwm_inst_t inst);

/* 中断配置接口 */
int intf_hrpwm_config_reload_irq(intf_hrpwm_inst_t inst, intf_hrpwm_irq_callback_t callback);
int intf_hrpwm_enable_reload_irq(intf_hrpwm_inst_t inst);
int intf_hrpwm_disable_reload_irq(intf_hrpwm_inst_t inst);

/* 移相配置接口 */
int intf_hrpwm_set_phase(const intf_hrpwm_phase_cfg_t *cfg);
int intf_hrpwm_config_phase_limit(intf_hrpwm_inst_t inst, const intf_hrpwm_phase_limit_t *limit);

/* PWM 触发信号配置 (用于 ADC 同步等) */
int intf_hrpwm_config_trigger_cmp(intf_hrpwm_inst_t inst, uint8_t cmp_index, float position_ratio);
int intf_hrpwm_set_trigger_cmp_position(intf_hrpwm_inst_t inst, uint8_t cmp_index, float position_ratio);

#ifdef __cplusplus
}
#endif

#endif /* INTF_HRPWM_H */
