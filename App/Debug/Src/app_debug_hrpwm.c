#include "app_debug_hrpwm.h"

#include "app_debug_rtt.h"
#include "app_hrpwm.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_pwm_drv.h"
#include "hpm_soc.h"
#include "intf_clock.h"
#include "intf_hrpwm.h"

#include <stdbool.h>
#include <stddef.h>

#define APP_DBG_CMP_START_INDEX(pwm_index) ((uint8_t) ((pwm_index) * 2U))
#define PWM_IRQ_INSTANCE_COUNT (2U)

typedef struct {
    PWM_Type *base;
    const char *name;
    uint8_t pwm_index;
} app_dbg_hrpwm_probe_t;

static const app_dbg_hrpwm_probe_t app_dbg_hrpwm_probes[] = {
    {.base = HPM_PWM0, .name = "PWM0_PAIR0", .pwm_index = 0U},
    {.base = HPM_PWM0, .name = "PWM0_PAIR1", .pwm_index = 2U},
    {.base = HPM_PWM1, .name = "PWM1_PAIR0", .pwm_index = 4U},
    {.base = HPM_PWM1, .name = "PWM1_PAIR1", .pwm_index = 6U},
};

static volatile uint32_t pwm_irq_count[PWM_IRQ_INSTANCE_COUNT] = {0};
static volatile bool pwm_irq_enabled[PWM_IRQ_INSTANCE_COUNT] = {0};
static pwm_irq_user_callback_t pwm_user_callback[PWM_IRQ_INSTANCE_COUNT] = {NULL};

static const char *app_dbg_detect_alignment(uint32_t reload, uint32_t cmp_begin, uint32_t cmp_end)
{
    if (cmp_end == reload) {
        return "EDGE-like";
    }

    if ((cmp_begin + cmp_end == reload) || (cmp_begin + cmp_end + 1U == reload)) {
        return "CENTER-like";
    }

    return "UNKNOWN";
}

void app_debug_dump_hrpwm_cmp(void)
{
    app_debug_printf("[HRPWM] compare register snapshot\r\n");

    for (size_t i = 0; i < sizeof(app_dbg_hrpwm_probes) / sizeof(app_dbg_hrpwm_probes[0]); i++) {
        const app_dbg_hrpwm_probe_t *probe = &app_dbg_hrpwm_probes[i];
        uint8_t cmp_start = APP_DBG_CMP_START_INDEX(probe->pwm_index);
        uint32_t reload = pwm_get_reload_val(probe->base);
        uint32_t cmp_begin = pwm_cmp_get_cmp_value(probe->base, cmp_start);
        uint32_t cmp_end = pwm_cmp_get_cmp_value(probe->base, cmp_start + 1U);
        const char *align = app_dbg_detect_alignment(reload, cmp_begin, cmp_end);

        app_debug_printf("[HRPWM] %s: reload=%lu cmp[%u]=%lu cmp[%u]=%lu => %s\r\n", probe->name,
                         (unsigned long) reload, (unsigned int) cmp_start,
                         (unsigned long) cmp_begin, (unsigned int) (cmp_start + 1U),
                         (unsigned long) cmp_end, align);
    }
}

static void pwm0_center_irq_callback(void)
{
    pwm_irq_count[0]++;
    if (pwm_user_callback[0] != NULL) {
        pwm_user_callback[0]();
    }
}

static void pwm1_center_irq_callback(void)
{
    pwm_irq_count[1]++;
    if (pwm_user_callback[1] != NULL) {
        pwm_user_callback[1]();
    }
}

int app_debug_pwm_irq_register_callback(uint8_t inst, pwm_irq_user_callback_t callback)
{
    if (inst >= PWM_IRQ_INSTANCE_COUNT) {
        return -1;
    }
    pwm_user_callback[inst] = callback;
    app_debug_printf("[HRPWM] IRQ PWM%d user callback registered\r\n", inst);
    return 0;
}

void app_debug_pwm_irq_unregister_callback(uint8_t inst)
{
    if (inst >= PWM_IRQ_INSTANCE_COUNT) {
        return;
    }
    pwm_user_callback[inst] = NULL;
    app_debug_printf("[HRPWM] IRQ PWM%d user callback unregistered\r\n", inst);
}

void app_debug_pwm_irq_enable(uint8_t inst)
{
    if (inst >= PWM_IRQ_INSTANCE_COUNT) {
        return;
    }

    int ret;
    if (inst == 0) {
        ret = intf_hrpwm_config_reload_irq(0, pwm0_center_irq_callback);
        if (ret == 0) {
            ret = intf_hrpwm_enable_reload_irq(0);
        }
    } else {
        ret = intf_hrpwm_config_reload_irq(1, pwm1_center_irq_callback);
        if (ret == 0) {
            ret = intf_hrpwm_enable_reload_irq(1);
        }
    }

    if (ret == 0) {
        pwm_irq_enabled[inst] = true;
        pwm_irq_count[inst] = 0;
        app_debug_printf("[HRPWM] IRQ PWM%d center IRQ enabled\r\n", inst);
    } else {
        app_debug_printf("[HRPWM] IRQ PWM%d center IRQ enable FAILED\r\n", inst);
    }
}

void app_debug_pwm_irq_disable(uint8_t inst)
{
    if (inst >= PWM_IRQ_INSTANCE_COUNT) {
        return;
    }

    if (inst == 0) {
        intf_hrpwm_disable_reload_irq(0);
    } else {
        intf_hrpwm_disable_reload_irq(1);
    }

    pwm_irq_enabled[inst] = false;
    app_debug_printf("[HRPWM] IRQ PWM%d center IRQ disabled, count=%lu\r\n", inst,
                     (unsigned long) pwm_irq_count[inst]);
}

uint32_t app_debug_pwm_irq_get_count(uint8_t inst)
{
    if (inst >= PWM_IRQ_INSTANCE_COUNT) {
        return 0;
    }
    return pwm_irq_count[inst];
}

void app_debug_pwm_irq_reset_count(uint8_t inst)
{
    if (inst < PWM_IRQ_INSTANCE_COUNT) {
        pwm_irq_count[inst] = 0;
    }
}

void app_debug_pwm_irq_dump_status(void)
{
    app_debug_printf("[HRPWM] IRQ status:\r\n");
    for (uint8_t i = 0; i < PWM_IRQ_INSTANCE_COUNT; i++) {
        app_debug_printf("[HRPWM] IRQ PWM%d: %s, count=%lu\r\n", i,
                         pwm_irq_enabled[i] ? "enabled" : "disabled",
                         (unsigned long) pwm_irq_count[i]);
    }
}

void app_debug_pwm_test_frequency_sweep(uint8_t inst, uint32_t freq_start, uint32_t freq_end,
                                        uint32_t freq_step, uint32_t delay_ms)
{
    app_debug_printf("[HRPWM] TEST PWM%d frequency sweep: %lu -> %lu Hz, step=%lu Hz\r\n", inst,
                     (unsigned long) freq_start, (unsigned long) freq_end,
                     (unsigned long) freq_step);

    int32_t direction = (freq_end > freq_start) ? 1 : -1;
    uint32_t freq = freq_start;

    while (1) {
        intf_hrpwm_set_frequency(inst, freq);
        app_debug_printf("[HRPWM] TEST freq = %lu Hz\r\n", (unsigned long) freq);
        intf_clock_delay_ms(delay_ms);

        if (direction > 0) {
            if (freq >= freq_end) {
                break;
            }
            freq += freq_step;
            if (freq > freq_end) {
                freq = freq_end;
            }
        } else {
            if (freq <= freq_end || freq < freq_step) {
                break;
            }
            freq -= freq_step;
            if (freq < freq_end) {
                freq = freq_end;
            }
        }
    }

    app_debug_printf("[HRPWM] TEST PWM%d frequency sweep done\r\n", inst);
}

void app_debug_pwm_test_duty_resolution(uint8_t inst, uint8_t pair, float duty_start,
                                        float duty_end, float duty_step, uint32_t delay_ms)
{
    app_debug_printf("[HRPWM] TEST PWM%d pair%d duty resolution test\r\n", inst, pair);
    app_debug_printf("[HRPWM] TEST range: %.4f -> %.4f, step=%.4f\r\n", duty_start, duty_end,
                     duty_step);

#if defined(HRPWM_USE_EXTENDED_COUNTER) && (HRPWM_USE_EXTENDED_COUNTER == 1)
    app_debug_printf("[HRPWM] TEST mode: 28-bit counter (higher resolution)\r\n");
#else
    app_debug_printf("[HRPWM] TEST mode: 24-bit counter (standard)\r\n");
#endif

    uint8_t ch = pair * 2U + inst * 4U;
    float duty = duty_start;
    int32_t direction = (duty_end > duty_start) ? 1 : -1;

    while (1) {
        intf_hrpwm_set_duty(ch, duty);
        app_debug_printf("[HRPWM] TEST duty = %.4f (%.2f%%)\r\n", duty, duty * 100.0f);
        intf_clock_delay_ms(delay_ms);

        if (direction > 0) {
            if (duty >= duty_end) {
                break;
            }
            duty += duty_step;
            if (duty > duty_end) {
                duty = duty_end;
            }
        } else {
            if (duty <= duty_end) {
                break;
            }
            duty -= duty_step;
            if (duty < duty_end) {
                duty = duty_end;
            }
        }
    }

    app_debug_printf("[HRPWM] TEST PWM%d pair%d duty resolution test done\r\n", inst, pair);
}

void app_debug_hrpwm_run_tests(void)
{
    app_debug_printf("\r\n[HRPWM] === HRPWM Validation Tests ===\r\n");
    app_debug_dump_hrpwm_cmp();

    for (hrpwm_pair_t pair = HRPWM_BUCKBOOST_A; pair < HRPWM_PAIR_COUNT; pair++) {
        app_hrpwm_set_duty(pair, 0.5f);
    }

    app_debug_printf("[HRPWM] enabling PWM1 reload IRQ...\r\n");
    app_debug_pwm_irq_enable(1);

    app_debug_printf("[HRPWM] waiting 1s before tests...\r\n");
    intf_clock_delay_ms(1000);

    app_debug_printf("\r\n[HRPWM] --- Test 1: PWM1 Buck-Boost pair restart ---\r\n");
    app_hrpwm_stop(HRPWM_BUCKBOOST_A);
    app_hrpwm_stop(HRPWM_BUCKBOOST_B);
    intf_clock_delay_ms(20);
    app_hrpwm_start(HRPWM_BUCKBOOST_A);
    app_hrpwm_start(HRPWM_BUCKBOOST_B);
    app_debug_dump_hrpwm_cmp();
    intf_clock_delay_ms(2000);

    app_debug_printf("\r\n[HRPWM] --- Test 2: PWM1 frequency sweep ---\r\n");
    app_debug_pwm_test_frequency_sweep(1, 160000, 240000, 20000, 500);
    app_hrpwm_set_frequency(HRPWM_INST_BUCKBOOST, 200000);

    app_debug_printf("\r\n[HRPWM] --- Test 3: PWM1 duty resolution around 50% ---\r\n");
    app_debug_pwm_test_duty_resolution(1, HRPWM_BUCKBOOST_A, 0.45f, 0.55f, 0.0005f, 80);
    app_hrpwm_set_duty(HRPWM_BUCKBOOST_A, 0.5f);

    app_debug_printf("\r\n[HRPWM] === HRPWM Validation Tests Completed ===\r\n");
}

void app_debug_dump_hrpwm_freq(void)
{
    uint32_t clk = clock_get_frequency(clock_mot0);
    uint32_t ahb = clock_get_frequency(clock_ahb);
    uint32_t r0 = pwm_get_reload_val(HPM_PWM0);
    uint32_t r1 = pwm_get_reload_val(HPM_PWM1);
    uint32_t f0 = (r0 > 0U) ? clk / (r0 + 1U) : 0U;
    uint32_t f1 = (r1 > 0U) ? clk / (r1 + 1U) : 0U;

    app_debug_printf("[HRPWM] PWM0 reload=%lu freq=%luHz | PWM1 reload=%lu freq=%luHz | mot0=%luHz ahb=%luHz\r\n",
                     (unsigned long)r0, (unsigned long)f0,
                     (unsigned long)r1, (unsigned long)f1,
                     (unsigned long)clk, (unsigned long)ahb);
}
