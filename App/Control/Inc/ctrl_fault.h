/*
 * Control - Fault & Protection
 *
 * 保护阈值管理 + 故障检测 + 锁存 + 清除。
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CTRL_FAULT_H
#define CTRL_FAULT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FAULT_NONE          = 0,
    /* VIN/VLINK use the control-model names: VIN = current VOUT, VLINK = current VCAP. */
    FAULT_OV_VIN        = (1 << 0),
    FAULT_UV_VIN        = (1 << 1),
    FAULT_OC_IIN        = (1 << 2),
    FAULT_OC_IL         = (1 << 3),
    FAULT_OV_VLINK      = (1 << 4),
    FAULT_UV_VLINK      = (1 << 5),
    FAULT_OC_ICOIL      = (1 << 6),
    FAULT_OC_ILF        = (1 << 7),
    FAULT_OT            = (1 << 8),
    FAULT_ADC           = (1 << 9),
    FAULT_PWM           = (1 << 10),
    FAULT_CAN_BUSOFF    = (1 << 11),
    FAULT_DRVPWR        = (1 << 12),
    FAULT_HARDWARE      = (1 << 13),
    FAULT_TIMEOUT       = (1 << 14),
} fault_code_t;

typedef struct {
    float v_in_ov_mv;
    float v_in_uv_mv;
    float i_in_oc_ma;
    float i_l_oc_ma;
    float v_link_ov_mv;
    float v_link_uv_mv;
    float i_coil_oc_ma;
    float i_lf_oc_ma;
    float temp_ot_c;
    uint32_t timeout_ms;
} ctrl_fault_thresholds_t;

/* ============================================================================
 * 公开接口
 * ============================================================================ */

void     ctrl_fault_init(const ctrl_fault_thresholds_t *thresholds);
void     ctrl_fault_set_thresholds(const ctrl_fault_thresholds_t *thresholds);

uint32_t ctrl_fault_check(void);
uint32_t ctrl_fault_get_active(void);

int      ctrl_fault_clear(uint32_t code);
int      ctrl_fault_clear_all(void);
bool     ctrl_fault_is_hardware(void);

void     ctrl_fault_set_callback(void (*cb)(uint32_t active_faults));

#ifdef __cplusplus
}
#endif

#endif /* CTRL_FAULT_H */
