#ifndef APP_GPIO_H
#define APP_GPIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pin definitions (must match Board/HPM5361_SuperCap_board/pinmux.c) */
#define PIN_DRVPWR    ((0 << 5) | 27)  /* PA27 */

void app_gpio_init(void);
void app_gpio_set(uint16_t pin, uint8_t on);
void app_gpio_toggle(uint16_t pin);
uint8_t app_gpio_read(uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif /* APP_GPIO_H */
