#include "app_gpio.h"
#include "intf_gpio.h"

#include <stddef.h>

/* Driver registration */
extern void hpm_gpio_driver_register(void);

void app_gpio_init(void) {
    hpm_gpio_driver_register();

    intf_gpio_cfg_t drvpwr_cfg = {
        .pin = PIN_DRVPWR,
        .dir = INTF_GPIO_DIR_OUTPUT,
        .pull = INTF_GPIO_PULL_DOWN,
        .init_level = INTF_GPIO_LEVEL_LOW,
        .irq_mode = INTF_GPIO_IRQ_NONE,
        .irq_cb = NULL,
    };
    intf_gpio_init(&drvpwr_cfg);
}

void app_gpio_set(uint16_t pin, uint8_t on) {
    intf_gpio_set_level(pin, on ? INTF_GPIO_LEVEL_HIGH : INTF_GPIO_LEVEL_LOW);
}

void app_gpio_toggle(uint16_t pin) { intf_gpio_toggle(pin); }
uint8_t app_gpio_read(uint16_t pin)
{
    intf_gpio_level_t level;
    intf_gpio_get_level(pin, &level);
    return (level == INTF_GPIO_LEVEL_HIGH) ? 1 : 0;
}
