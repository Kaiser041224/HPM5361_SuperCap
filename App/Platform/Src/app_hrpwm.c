/*
 * HRPWM Platform Implementation
 *
 * Current hardware mapping:
 *   PWM1: ch4/ch5 (Buck-Boost A), ch6/ch7 (Buck-Boost B)
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_hrpwm.h"

#include "intf_attr.h"
#include "intf_hrpwm.h"

#include <stdbool.h>

ATTR_PLACE_AT_FAST_RAM_INIT static const intf_hrpwm_ch_t pair_to_ch[HRPWM_PAIR_COUNT] = {4, 6};

extern void hpm_hrpwm_driver_register(void);

ATTR_RAMFUNC
static bool hrpwm_pair_is_valid(hrpwm_pair_t pair) { return pair < HRPWM_PAIR_COUNT; }

static bool hrpwm_inst_is_valid(hrpwm_inst_t inst) { return inst == HRPWM_INST_BUCKBOOST; }

ATTR_RAMFUNC
static intf_hrpwm_ch_t hrpwm_pair_channel(hrpwm_pair_t pair) { return pair_to_ch[pair]; }

void app_hrpwm_init(void)
{
    hpm_hrpwm_driver_register();

    intf_hrpwm_pair_cfg_t cfg[HRPWM_PAIR_COUNT] = {
        [HRPWM_BUCKBOOST_A] = {
            .frequency_hz = 200000,
            .duty = 0.0f,
            .deadtime_ns = 15,
            .jitter_cmp = 4,
            .align = INTF_HRPWM_ALIGN_CENTER,
            .invert_high_side = false,
            .invert_low_side = false,
        },
        [HRPWM_BUCKBOOST_B] = {
            .frequency_hz = 200000,
            .duty = 0.0f,
            .deadtime_ns = 15,
            .jitter_cmp = 4,
            .align = INTF_HRPWM_ALIGN_CENTER,
            .invert_high_side = false,
            .invert_low_side = false,
        },
    };

    for (hrpwm_pair_t pair = HRPWM_BUCKBOOST_A; pair < HRPWM_PAIR_COUNT; pair++) {
        (void)intf_hrpwm_init_pair(hrpwm_pair_channel(pair), &cfg[pair]);
    }
}

ATTR_RAMFUNC
void app_hrpwm_set_duty(hrpwm_pair_t pair, float duty)
{
    if (!hrpwm_pair_is_valid(pair)) {
        return;
    }
    (void)intf_hrpwm_set_duty(hrpwm_pair_channel(pair), duty);
}

ATTR_RAMFUNC
void app_hrpwm_set_duty_direct(hrpwm_pair_t pair, float duty)
{
    if (!hrpwm_pair_is_valid(pair)) {
        return;
    }
    (void)intf_hrpwm_set_duty_direct(hrpwm_pair_channel(pair), duty);
}

ATTR_RAMFUNC
void app_hrpwm_set_duty_direct_dual(
    hrpwm_pair_t pair_a, float duty_a, hrpwm_pair_t pair_b, float duty_b)
{
    if (!hrpwm_pair_is_valid(pair_a) || !hrpwm_pair_is_valid(pair_b)) {
        return;
    }
    (void)intf_hrpwm_set_duty_direct_dual(
        hrpwm_pair_channel(pair_a), duty_a, hrpwm_pair_channel(pair_b), duty_b);
}

void app_hrpwm_set_frequency(hrpwm_inst_t inst, uint32_t freq_hz)
{
    if (!hrpwm_inst_is_valid(inst)) {
        return;
    }
    (void)intf_hrpwm_set_frequency((intf_hrpwm_inst_t)inst, freq_hz);
}

void app_hrpwm_set_jitter(hrpwm_pair_t pair, uint8_t jitter_cmp)
{
    if (!hrpwm_pair_is_valid(pair)) {
        return;
    }
    (void)intf_hrpwm_set_jitter(hrpwm_pair_channel(pair), jitter_cmp);
}

void app_hrpwm_start(hrpwm_pair_t pair)
{
    if (!hrpwm_pair_is_valid(pair)) {
        return;
    }
    (void)intf_hrpwm_start(hrpwm_pair_channel(pair));
}

void app_hrpwm_stop(hrpwm_pair_t pair)
{
    if (!hrpwm_pair_is_valid(pair)) {
        return;
    }
    (void)intf_hrpwm_stop(hrpwm_pair_channel(pair));
    (void)intf_hrpwm_stop((intf_hrpwm_ch_t)(hrpwm_pair_channel(pair) + 1U));
}

void app_hrpwm_stop_all(void)
{
    for (hrpwm_pair_t pair = HRPWM_BUCKBOOST_A; pair < HRPWM_PAIR_COUNT; pair++) {
        app_hrpwm_stop(pair);
    }
}

void app_hrpwm_start_all(void)
{
    for (hrpwm_pair_t pair = HRPWM_BUCKBOOST_A; pair < HRPWM_PAIR_COUNT; pair++) {
        app_hrpwm_start(pair);
    }
}

void app_hrpwm_start_counter_only(void)
{
    (void)intf_hrpwm_start_counter_only((intf_hrpwm_inst_t)HRPWM_INST_BUCKBOOST);
}

ATTR_RAMFUNC
void app_hrpwm_force_low(hrpwm_pair_t pair)
{
    if (!hrpwm_pair_is_valid(pair)) {
        return;
    }
    (void)intf_hrpwm_force_low(hrpwm_pair_channel(pair));
    (void)intf_hrpwm_force_low((intf_hrpwm_ch_t)(hrpwm_pair_channel(pair) + 1U));
}

ATTR_RAMFUNC
void app_hrpwm_force_release(hrpwm_pair_t pair)
{
    if (!hrpwm_pair_is_valid(pair)) {
        return;
    }
    (void)intf_hrpwm_force_release(hrpwm_pair_channel(pair));
    (void)intf_hrpwm_force_release((intf_hrpwm_ch_t)(hrpwm_pair_channel(pair) + 1U));
}

ATTR_RAMFUNC
void app_hrpwm_emergency_stop(void)
{
    for (hrpwm_pair_t pair = HRPWM_BUCKBOOST_A; pair < HRPWM_PAIR_COUNT; pair++) {
        app_hrpwm_force_low(pair);
    }
}

void app_hrpwm_resume(void)
{
    for (hrpwm_pair_t pair = HRPWM_BUCKBOOST_A; pair < HRPWM_PAIR_COUNT; pair++) {
        app_hrpwm_force_release(pair);
    }
}

void app_hrpwm_config_fault(void)
{
    intf_hrpwm_fault_cfg_t fault_cfg = {
        .source = INTF_HRPWM_FAULT_SRC_EXTERNAL_0,
        .mode = INTF_HRPWM_FAULT_MODE_FORCE_LOW,
        .recovery = INTF_HRPWM_FAULT_RECOVERY_ON_FAULT_CLEAR,
        .active_low = true,
    };
    (void)intf_hrpwm_config_fault((intf_hrpwm_inst_t)HRPWM_INST_BUCKBOOST, &fault_cfg);
}

void app_hrpwm_clear_fault(void)
{
    (void)intf_hrpwm_clear_fault((intf_hrpwm_inst_t)HRPWM_INST_BUCKBOOST);
}
