#ifndef INTF_GPIO_H
#define INTF_GPIO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Types
 * ============================================================================ */

typedef uint16_t intf_gpio_pin_t;

typedef enum {
    INTF_GPIO_DIR_INPUT  = 0,
    INTF_GPIO_DIR_OUTPUT = 1,
} intf_gpio_dir_t;

typedef enum {
    INTF_GPIO_LEVEL_LOW  = 0,
    INTF_GPIO_LEVEL_HIGH = 1,
} intf_gpio_level_t;

typedef enum {
    INTF_GPIO_PULL_NONE = 0,
    INTF_GPIO_PULL_DOWN = 1,
    INTF_GPIO_PULL_UP   = 2,
} intf_gpio_pull_t;

typedef enum {
    INTF_GPIO_IRQ_NONE          = 0,
    INTF_GPIO_IRQ_EDGE_RISING   = 1,
    INTF_GPIO_IRQ_EDGE_FALLING  = 2,
    INTF_GPIO_IRQ_EDGE_BOTH     = 3,
    INTF_GPIO_IRQ_LEVEL_HIGH    = 4,
    INTF_GPIO_IRQ_LEVEL_LOW     = 5,
} intf_gpio_irq_mode_t;

typedef void (*intf_gpio_irq_cb_t)(intf_gpio_pin_t pin, void *user_data);

typedef struct {
    intf_gpio_pin_t      pin;
    intf_gpio_dir_t      dir;
    intf_gpio_pull_t     pull;
    intf_gpio_level_t    init_level;
    intf_gpio_irq_mode_t irq_mode;
    intf_gpio_irq_cb_t   irq_cb;
    void                *irq_user_data;
} intf_gpio_cfg_t;

/* ============================================================================
 * Interface Definition (Object-Oriented C17)
 * ============================================================================ */

typedef struct {
    int (*init)(const intf_gpio_cfg_t *cfg);
    int (*set_level)(intf_gpio_pin_t pin, intf_gpio_level_t level);
    int (*get_level)(intf_gpio_pin_t pin, intf_gpio_level_t *level);
    int (*toggle)(intf_gpio_pin_t pin);
} intf_gpio_t;

/* ============================================================================
 * Registration API
 * ============================================================================ */

int intf_gpio_register(const intf_gpio_t *ops);

/* ============================================================================
 * Functional API (wraps ops)
 * ============================================================================ */

int intf_gpio_init(const intf_gpio_cfg_t *cfg);
int intf_gpio_set_level(intf_gpio_pin_t pin, intf_gpio_level_t level);
int intf_gpio_get_level(intf_gpio_pin_t pin, intf_gpio_level_t *level);
int intf_gpio_toggle(intf_gpio_pin_t pin);

#ifdef __cplusplus
}
#endif

#endif /* INTF_GPIO_H */
