#ifndef APP_BUZZER_H
#define APP_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_BUZZER_DEFAULT_FREQ_HZ (4000U)

void app_buzzer_init(void);
int app_buzzer_set(bool enabled, uint32_t frequency_hz);

#ifdef __cplusplus
}
#endif

#endif /* APP_BUZZER_H */
