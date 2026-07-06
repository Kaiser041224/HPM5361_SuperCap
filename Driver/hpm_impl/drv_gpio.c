#include "intf_gpio.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_interrupt.h"
#include "hpm_soc_irq.h"

#include <stddef.h>

/* ============================================================================
 * Pin Mapping: intf_gpio_pin_t -> HPM GPIO controller
 *
 * Pin encoding: (port << 5) | pin
 *   port 0 = GPIOA, port 1 = GPIOB
 *   pin 0-31
 * ============================================================================ */

#define GPIO_PORT(pin) ((pin) >> 5)
#define GPIO_PIN(pin)  ((pin) & 0x1F)
#define GPIO_PORT_COUNT (2U)
#define GPIO_PINS_PER_PORT (32U)
#define GPIO_PIN_COUNT (GPIO_PORT_COUNT * GPIO_PINS_PER_PORT)

/* HPM5361 has single GPIO controller */
static GPIO_Type *get_gpio_base(uint8_t port)
{
    (void)port;
    return HPM_GPIO0;
}

static uint32_t get_gpio_port_index(uint8_t port)
{
    switch (port) {
    case 0:
        return GPIO_DI_GPIOA;
    case 1:
        return GPIO_DI_GPIOB;
    default:
        return GPIO_DI_GPIOA;
    }
}

static int get_gpio_port_irq(uint8_t port)
{
    switch (port) {
    case 0:
        return IRQn_GPIO0_A;
    case 1:
        return IRQn_GPIO0_B;
    default:
        return -1;
    }
}

/* ============================================================================
 * IRQ Support
 * ============================================================================ */

typedef struct {
    intf_gpio_irq_cb_t cb;
    void *user_data;
} irq_entry_t;

static irq_entry_t irq_table[64];

static void gpio_port_isr(uint8_t port)
{
    GPIO_Type *base = HPM_GPIO0;
    uint32_t port_index = get_gpio_port_index(port);

    for (uint8_t i = 0; i < GPIO_PINS_PER_PORT; i++) {
        if (gpio_check_pin_interrupt_flag(base, port_index, i)) {
            intf_gpio_pin_t pin = (intf_gpio_pin_t)((port << 5) | i);
            gpio_clear_pin_interrupt_flag(base, port_index, i);
            if (irq_table[pin].cb != NULL) {
                irq_table[pin].cb(pin, irq_table[pin].user_data);
            }
        }
    }
}

/* GPIO0_A: GPIO port A */
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_A, isr_gpio0_a)
void isr_gpio0_a(void)
{
    gpio_port_isr(0);
}

/* GPIO0_B: GPIO port B */
SDK_DECLARE_EXT_ISR_M(IRQn_GPIO0_B, isr_gpio0_b)
void isr_gpio0_b(void)
{
    gpio_port_isr(1);
}

/* ============================================================================
 * HPM GPIO Implementation
 * ============================================================================ */

static int hpm_gpio_init(const intf_gpio_cfg_t *cfg)
{
    if (cfg == NULL) return -1;

    if (cfg->pin >= GPIO_PIN_COUNT) {
        return -1;
    }

    uint8_t port = GPIO_PORT(cfg->pin);
    uint8_t idx = GPIO_PIN(cfg->pin);
    GPIO_Type *base = get_gpio_base(port);
    uint32_t port_index;

    port_index = get_gpio_port_index(port);

    /* Set direction */
    if (cfg->dir == INTF_GPIO_DIR_OUTPUT) {
        gpio_set_pin_output_with_initial(base, port_index, idx, (uint8_t)cfg->init_level);
    } else {
        gpio_set_pin_input(base, port_index, idx);
    }

    /* Configure interrupt if needed */
    if (cfg->irq_mode != INTF_GPIO_IRQ_NONE && cfg->irq_cb != NULL) {
        gpio_interrupt_trigger_t trigger;
        switch (cfg->irq_mode) {
        case INTF_GPIO_IRQ_EDGE_RISING:  trigger = gpio_interrupt_trigger_edge_rising; break;
        case INTF_GPIO_IRQ_EDGE_FALLING: trigger = gpio_interrupt_trigger_edge_falling; break;
        case INTF_GPIO_IRQ_EDGE_BOTH:
#if defined(GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT) && (GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT == 1)
            trigger = gpio_interrupt_trigger_edge_both;
            break;
#else
            return -1;
#endif
        case INTF_GPIO_IRQ_LEVEL_HIGH:   trigger = gpio_interrupt_trigger_level_high; break;
        case INTF_GPIO_IRQ_LEVEL_LOW:    trigger = gpio_interrupt_trigger_level_low; break;
        default: return -1;
        }

        irq_table[cfg->pin].cb = cfg->irq_cb;
        irq_table[cfg->pin].user_data = cfg->irq_user_data;

        gpio_config_pin_interrupt(base, port_index, idx, trigger);
        gpio_clear_pin_interrupt_flag(base, port_index, idx);
        gpio_enable_pin_interrupt(base, port_index, idx);

        /* Enable IRQ in PLIC */
        int irq_num = get_gpio_port_irq(port);
        if (irq_num >= 0) {
            intc_m_enable_irq_with_priority(irq_num, 1);
        }
    }

    return 0;
}

static int hpm_gpio_set_level(intf_gpio_pin_t pin, intf_gpio_level_t level)
{
    if (pin >= GPIO_PIN_COUNT) {
        return -1;
    }

    uint8_t port = GPIO_PORT(pin);
    uint8_t idx = GPIO_PIN(pin);
    GPIO_Type *base = get_gpio_base(port);

    gpio_write_pin(base, get_gpio_port_index(port), idx, (uint8_t)level);
    return 0;
}

static int hpm_gpio_get_level(intf_gpio_pin_t pin, intf_gpio_level_t *level)
{
    if ((level == NULL) || (pin >= GPIO_PIN_COUNT)) {
        return -1;
    }

    uint8_t port = GPIO_PORT(pin);
    uint8_t idx = GPIO_PIN(pin);
    GPIO_Type *base = get_gpio_base(port);

    uint8_t val = gpio_read_pin(base, get_gpio_port_index(port), idx);
    *level = (val != 0) ? INTF_GPIO_LEVEL_HIGH : INTF_GPIO_LEVEL_LOW;
    return 0;
}

static int hpm_gpio_toggle(intf_gpio_pin_t pin)
{
    if (pin >= GPIO_PIN_COUNT) {
        return -1;
    }

    uint8_t port = GPIO_PORT(pin);
    uint8_t idx = GPIO_PIN(pin);
    GPIO_Type *base = get_gpio_base(port);

    gpio_toggle_pin(base, get_gpio_port_index(port), idx);
    return 0;
}

/* ============================================================================
 * Operations Structure & Registration
 * ============================================================================ */

static const intf_gpio_t hpm_gpio_ops = {
    .init      = hpm_gpio_init,
    .set_level = hpm_gpio_set_level,
    .get_level = hpm_gpio_get_level,
    .toggle    = hpm_gpio_toggle,
};

void hpm_gpio_driver_register(void)
{
    intf_gpio_register(&hpm_gpio_ops);
}
