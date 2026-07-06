/*
 * app_control.c — 系统控制编排实现
 *
 * Current hardware path:
 *   ADC0 PMT: IL, drives Buck-Boost current loop
 *   ADC1 PMT: VCAP(VLINK)/VOUT(VIN)/IIN cache, sampled from the same PWM1 trigger
 *   PWM1: Buck-Boost A/B outputs
 *
 * Voltage naming:
 *   VCAP: SuperCap node, mapped to the control model's VLINK.
 *   VOUT: three-port external voltage, mapped to the control model's VIN.
 */

#include "app_control.h"

#include "app_adc.h"
#include "app_analog_signal.h"
#include "app_gptmr.h"
#include "app_hrpwm.h"
#include "ctrl_buckboost.h"
#include "ctrl_fault.h"

#include "intf_attr.h"

#include <stddef.h>

ATTR_PLACE_AT_FAST_RAM_BSS volatile ctrl_diag_t g_ctrl_diag;

#define CTRL_FREQ_DIVIDER 2U

ATTR_PLACE_AT_FAST_RAM_BSS static ctrl_buckboost_t g_buckboost;
ATTR_PLACE_AT_FAST_RAM_BSS static volatile uint32_t s_buckboost_vcap_sample_seq;
ATTR_PLACE_AT_FAST_RAM_BSS static uint32_t s_buckboost_vcap_consumed_seq;

static inline void buckboost_vcap_sample_reset(void)
{
    s_buckboost_vcap_sample_seq = 0U;
    s_buckboost_vcap_consumed_seq = 0U;
}

static inline bool ctrl_finite(float x)
{
    union {
        float f;
        uint32_t u;
    } v = {.f = x};
    return (v.u & 0x7F800000u) != 0x7F800000u;
}

ATTR_RAMFUNC
static void buckboost_current_loop_isr(void)
{
    uint16_t raw_i_l;
    if (app_adc_get_pmt_raw(ADC_CH_I_L, &raw_i_l) != 0) {
        goto exit;
    }
    g_ctrl_diag.raw_adc.i_l = raw_i_l;
    float phys_i_l;
    app_analog_signal_convert_raw(ADC_CH_I_L, raw_i_l, &phys_i_l);
    g_ctrl_diag.raw.i_l_a = phys_i_l;
    g_ctrl_diag.filt.i_l_a = app_analog_signal_lpf_step_fast(ADC_CH_I_L, phys_i_l);

    uint16_t raw_vlink;
    if (app_adc_get_pmt_raw(APP_ADC_CH_VLINK, &raw_vlink) != 0) {
        goto exit;
    }
    g_ctrl_diag.raw_adc.vcap = raw_vlink;
    float phys_vlink;
    app_analog_signal_convert_raw(APP_ADC_CH_VLINK, raw_vlink, &phys_vlink);
    g_ctrl_diag.raw.vcap_v = phys_vlink;
    g_ctrl_diag.filt.vcap_fast_v = app_analog_signal_lpf_step_fast(APP_ADC_CH_VLINK, phys_vlink);
    s_buckboost_vcap_sample_seq++;

    ATTR_PLACE_AT_FAST_RAM_BSS static uint8_t s_ctrl_div = 0;
    if (++s_ctrl_div >= CTRL_FREQ_DIVIDER) {
        s_ctrl_div = 0;
        float vin = g_ctrl_diag.filt.vout_v;       /* current VOUT == control VIN */
        float vlink = g_ctrl_diag.filt.vcap_fast_v; /* current VCAP == control VLINK */

        ctrl_buckboost_update_current(&g_buckboost, g_ctrl_diag.filt.i_l_a, vin, vlink);

        g_ctrl_diag.duty.buckboost_a = ctrl_buckboost_get_duty_a(&g_buckboost);
        g_ctrl_diag.duty.buckboost_b = ctrl_buckboost_get_duty_b(&g_buckboost);
        app_hrpwm_set_duty_direct_dual(
            HRPWM_BUCKBOOST_A, g_ctrl_diag.duty.buckboost_a, HRPWM_BUCKBOOST_B,
            g_ctrl_diag.duty.buckboost_b);
    }
exit:;
}

ATTR_RAMFUNC
static void buckboost_voltage_loop_isr(void)
{
    uint32_t vcap_sample_seq = s_buckboost_vcap_sample_seq;
    if (vcap_sample_seq == 0U || vcap_sample_seq == s_buckboost_vcap_consumed_seq) {
        goto exit;
    }
    s_buckboost_vcap_consumed_seq = vcap_sample_seq;

    float vlink_fb = app_analog_signal_ma_step(APP_ADC_CH_VLINK, g_ctrl_diag.filt.vcap_fast_v);
    if (!ctrl_finite(vlink_fb)) {
        goto exit;
    }
    g_ctrl_diag.filt.vcap_v = vlink_fb;

    float i_l = g_ctrl_diag.filt.i_l_a;
    float g = g_buckboost.state.generalized_duty;
    float d_max = g_buckboost.params.duty_max;
    float i_load_ff = 0.0f;
    if (ctrl_finite(i_l) && ctrl_finite(g) && ctrl_finite(d_max)) {
        float gc = (g < 0.0f) ? 0.0f : (g > 1.0f) ? 1.0f : g;
        i_load_ff = i_l * d_max * (1.0f - gc);
    }
    g_ctrl_diag.ff.i_load_est_a = i_load_ff;

    ctrl_buckboost_update_voltage(&g_buckboost, vlink_fb, i_load_ff, i_load_ff);

exit:;
}

ATTR_RAMFUNC
static void buckboost_power_loop_isr(void)
{
    uint16_t raw_vin, raw_i_in;
    if (app_adc_get_pmt_raw(APP_ADC_CH_VIN, &raw_vin) != 0
        || app_adc_get_pmt_raw(ADC_CH_I_IN, &raw_i_in) != 0) {
        goto exit;
    }
    g_ctrl_diag.raw_adc.vout = raw_vin;
    g_ctrl_diag.raw_adc.i_in = raw_i_in;

    float phys_vin, phys_i_in;
    app_analog_signal_convert_raw(APP_ADC_CH_VIN, raw_vin, &phys_vin);
    app_analog_signal_convert_raw(ADC_CH_I_IN, raw_i_in, &phys_i_in);
    g_ctrl_diag.raw.vout_v = phys_vin;
    g_ctrl_diag.raw.i_in_a = phys_i_in;

    g_ctrl_diag.filt.vout_v = app_analog_signal_ma_step(APP_ADC_CH_VIN, phys_vin);
    float i_in_lpf = app_analog_signal_lpf_step_fast(ADC_CH_I_IN, phys_i_in);
    g_ctrl_diag.filt.i_in_a = app_analog_signal_ma_step(ADC_CH_I_IN, i_in_lpf);

    float i_in = g_ctrl_diag.filt.i_in_a;
    if (!ctrl_finite(i_in)) {
        i_in = 0.0f;
    }
    g_ctrl_diag.ff.i_in_ctrl_a = i_in;

    float p_in = g_ctrl_diag.filt.vout_v * i_in;
    if (!ctrl_finite(p_in)) {
        p_in = 0.0f;
    }

    ctrl_buckboost_update_power(&g_buckboost, p_in);
    g_ctrl_diag.ff.p_in_w = p_in;
    g_ctrl_diag.ff.p_target_w = g_buckboost.state.p_target_w;
    g_ctrl_diag.ff.power_pid_out = g_buckboost.state.power_pid_out;

exit:;
}

static bool self_test(void) { return true; }

static sys_state_t s_state = SYS_INIT;
static op_mode_t s_mode = MODE_IDLE;
static bool s_self_test_ok;

static void configure_controllers_for_mode(void)
{
    switch (s_mode) {
    case MODE_BUCK_CV: ctrl_buckboost_set_target_type(&g_buckboost, BUCKBOOST_TARGET_CV); break;
    case MODE_BUCK_CC: ctrl_buckboost_set_target_type(&g_buckboost, BUCKBOOST_TARGET_CC); break;
    case MODE_BUCK_CW:
        ctrl_buckboost_enter_cw_mode(&g_buckboost, BUCKBOOST_P_TARGET_DEFAULT);
        break;
    default: break;
    }
}

void app_control_init(void)
{
    buckboost_vcap_sample_reset();

    ctrl_fault_init(NULL);
    ctrl_buckboost_init(&g_buckboost);
    ctrl_buckboost_enable(&g_buckboost);
    ctrl_buckboost_enter_cw_mode(&g_buckboost, BUCKBOOST_P_TARGET_DEFAULT);

    app_gptmr_init();
    app_gptmr_register_callback(APP_GPTMR_CH_VOLTAGE, buckboost_voltage_loop_isr);
    app_gptmr_register_callback(APP_GPTMR_CH_POWER, buckboost_power_loop_isr);

    app_adc_register_pmt_callback(APP_ADC_INST_0, buckboost_current_loop_isr);

    app_gptmr_start_all();

    s_state = SYS_INIT;
    s_mode = MODE_BUCK_CW;
    configure_controllers_for_mode();
    s_self_test_ok = false;
}

void app_control_tick(void)
{
    uint32_t faults = ctrl_fault_check();
    if (faults != 0U) {
        app_control_emergency();
        return;
    }

    switch (s_state) {
    case SYS_INIT:
        s_self_test_ok = self_test();
        if (s_self_test_ok) {
            s_state = SYS_IDLE;
            (void)app_control_power_enable();
        }
        break;
    case SYS_IDLE:
    case SYS_RUN:
    case SYS_FAULT:
        break;
    default:
        s_state = SYS_INIT;
        break;
    }
}

sys_state_t app_control_get_state(void) { return s_state; }
op_mode_t app_control_get_mode(void) { return s_mode; }

int app_control_set_mode(op_mode_t mode)
{
    switch (mode) {
    case MODE_IDLE:
    case MODE_STANDBY:
        if (s_state == SYS_FAULT) {
            return -1;
        }
        break;
    case MODE_BUCK_CV:
    case MODE_BUCK_CC:
    case MODE_BUCK_CW:
        if (s_state != SYS_IDLE && s_state != SYS_RUN) {
            return -1;
        }
        break;
    default:
        return -1;
    }

    s_mode = mode;
    configure_controllers_for_mode();
    return 0;
}

int app_control_power_enable(void)
{
    if (s_state != SYS_IDLE || !s_self_test_ok) {
        return -1;
    }

    ctrl_buckboost_enable(&g_buckboost);
    app_gptmr_start_all();

    s_state = SYS_RUN;
    return 0;
}

void app_control_power_disable(void)
{
    app_gptmr_stop_all();
    buckboost_vcap_sample_reset();
    ctrl_buckboost_disable(&g_buckboost);
    app_hrpwm_set_duty(HRPWM_BUCKBOOST_A, 0.0f);
    app_hrpwm_set_duty(HRPWM_BUCKBOOST_B, 0.0f);
    s_state = SYS_IDLE;
}

void app_control_emergency(void)
{
    app_gptmr_stop_all();
    buckboost_vcap_sample_reset();
    app_hrpwm_emergency_stop();
    ctrl_buckboost_emergency_stop(&g_buckboost);
    s_state = SYS_FAULT;
}

uint32_t app_control_get_faults(void) { return ctrl_fault_get_active(); }

int app_control_clear_faults(void)
{
    int ret = ctrl_fault_clear_all();
    if (ret == 0) {
        buckboost_vcap_sample_reset();
        s_state = SYS_INIT;
    }
    return ret;
}
