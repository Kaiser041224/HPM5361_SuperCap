#include "app_buzzer.h"
#include "intf_gptmr.h"

#include <stddef.h>

#define APP_BUZZER_GPTMR_CH (2U)
#define APP_BUZZER_DUTY     (0.5f)

extern void hpm_gptmr_driver_register(void);

static bool buzzer_initialized;

void app_buzzer_init(void)
{
    intf_gptmr_cfg_t cfg = {
        .mode = INTF_GPTMR_MODE_PWM,
        .frequency_hz = APP_BUZZER_DEFAULT_FREQ_HZ,
        .duty = APP_BUZZER_DUTY,
        .invert_output = false,
        .callback = NULL,
        .enable_sync = false,
    };

    hpm_gptmr_driver_register();

    if (intf_gptmr_init(APP_BUZZER_GPTMR_CH, &cfg) == 0) {
        intf_gptmr_force_low(APP_BUZZER_GPTMR_CH);
        buzzer_initialized = true;
    }
}

int app_buzzer_set(bool enabled, uint32_t frequency_hz)
{
    if (!buzzer_initialized) {
        return -1;
    }

    if (!enabled) {
        return intf_gptmr_force_low(APP_BUZZER_GPTMR_CH);
    }

    if (frequency_hz == 0U) {
        frequency_hz = APP_BUZZER_DEFAULT_FREQ_HZ;
    }

    if (intf_gptmr_set_frequency(APP_BUZZER_GPTMR_CH, frequency_hz) != 0) {
        return -1;
    }

    return intf_gptmr_force_release(APP_BUZZER_GPTMR_CH);
}
