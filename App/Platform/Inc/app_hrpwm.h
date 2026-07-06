/*
 * HRPWM Platform API
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_HRPWM_H
#define APP_HRPWM_H

#include <stdint.h>

typedef enum {
    HRPWM_BUCKBOOST_A = 0,  /* PWM1 pair 0: Buck-Boost 主半桥 */
    HRPWM_BUCKBOOST_B,      /* PWM1 pair 1: Buck-Boost 同步整流 */
    HRPWM_PAIR_COUNT,
} hrpwm_pair_t;

typedef enum {
    HRPWM_INST_BUCKBOOST = 1, /* PWM1: Buck-Boost */
    HRPWM_INST_COUNT = 2,
} hrpwm_inst_t;

void app_hrpwm_init(void);
void app_hrpwm_set_duty(hrpwm_pair_t pair, float duty);
void app_hrpwm_set_duty_direct(hrpwm_pair_t pair, float duty);
void app_hrpwm_set_duty_direct_dual(hrpwm_pair_t pair_a, float duty_a,
                                     hrpwm_pair_t pair_b, float duty_b);
void app_hrpwm_set_frequency(hrpwm_inst_t inst, uint32_t freq_hz);
void app_hrpwm_set_jitter(hrpwm_pair_t pair, uint8_t jitter_cmp);
void app_hrpwm_start(hrpwm_pair_t pair);
void app_hrpwm_stop(hrpwm_pair_t pair);
void app_hrpwm_stop_all(void);
void app_hrpwm_start_all(void);
void app_hrpwm_force_low(hrpwm_pair_t pair);
void app_hrpwm_force_release(hrpwm_pair_t pair);
void app_hrpwm_emergency_stop(void);
void app_hrpwm_resume(void);
void app_hrpwm_config_fault(void);
void app_hrpwm_clear_fault(void);

#endif /* APP_HRPWM_H */
