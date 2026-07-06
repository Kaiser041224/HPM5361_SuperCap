/*
 * TRGM Driver - HPM Trigger Mux implementation (C17 OOP)
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "intf_trgm.h"

static int trgm_connect_impl(intf_trgm_src_t src, intf_trgm_dst_t dst) {
    static const uint32_t src_map[] = {
        [INTF_TRGM_SRC_PWM0_CH8REF] = HPM_TRGM0_INPUT_SRC_PWM0_CH8REF,
        [INTF_TRGM_SRC_PWM0_CH9REF] = HPM_TRGM0_INPUT_SRC_PWM0_CH9REF,
        [INTF_TRGM_SRC_PWM0_CH10REF] = HPM_TRGM0_INPUT_SRC_PWM0_CH10REF,
        [INTF_TRGM_SRC_PWM0_CH11REF] = HPM_TRGM0_INPUT_SRC_PWM0_CH11REF,
        [INTF_TRGM_SRC_PWM1_CH8REF] = HPM_TRGM0_INPUT_SRC_PWM1_CH8REF,
        [INTF_TRGM_SRC_PWM1_CH9REF] = HPM_TRGM0_INPUT_SRC_PWM1_CH9REF,
        [INTF_TRGM_SRC_PWM1_CH10REF] = HPM_TRGM0_INPUT_SRC_PWM1_CH10REF,
        [INTF_TRGM_SRC_PWM1_CH11REF] = HPM_TRGM0_INPUT_SRC_PWM1_CH11REF,
        [INTF_TRGM_SRC_SYNT_CH0]    = HPM_TRGM0_INPUT_SRC_SYNT0_CH0,
        [INTF_TRGM_SRC_SYNT_CH1]    = HPM_TRGM0_INPUT_SRC_SYNT0_CH1,
        [INTF_TRGM_SRC_SYNT_CH2]    = HPM_TRGM0_INPUT_SRC_SYNT0_CH2,
        [INTF_TRGM_SRC_SYNT_CH3]    = HPM_TRGM0_INPUT_SRC_SYNT0_CH3,
    };

    static const uint32_t dst_map[] = {
        [INTF_TRGM_DST_ADC_PTRGI0A]  = HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A,
        [INTF_TRGM_DST_ADC_PTRGI0B]  = HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B,
        [INTF_TRGM_DST_ADC_PTRGI0C]  = HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0C,
        [INTF_TRGM_DST_ADC_PTRGI1A]  = HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI1A,
        [INTF_TRGM_DST_ADC_PTRGI1B]  = HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI1B,
        [INTF_TRGM_DST_ADC_PTRGI1C]  = HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI1C,
        [INTF_TRGM_DST_GPTMR0_SYNCI] = HPM_TRGM0_OUTPUT_SRC_GPTMR0_SYNCI,
        [INTF_TRGM_DST_GPTMR1_SYNCI] = HPM_TRGM0_OUTPUT_SRC_GPTMR1_SYNCI,
        [INTF_TRGM_DST_GPTMR2_SYNCI] = HPM_TRGM0_OUTPUT_SRC_GPTMR2_SYNCI,
        [INTF_TRGM_DST_GPTMR3_SYNCI] = HPM_TRGM0_OUTPUT_SRC_GPTMR3_SYNCI,
    };

    if (src >= sizeof(src_map) / sizeof(src_map[0]) || dst >= sizeof(dst_map) / sizeof(dst_map[0]))
        return -1;

    trgm_output_t cfg;
    cfg.invert = false;
    cfg.type   = trgm_output_pulse_at_input_rising_edge;
    cfg.input = src_map[src];

    trgm_output_config(HPM_TRGM0, dst_map[dst], &cfg);
    return 0;
}

static const intf_trgm_t trgm_ops = {
    .instance_id = 0,
    .connect = trgm_connect_impl,
};

void hpm_trgm_driver_register(void) { intf_trgm_register(&trgm_ops); }
