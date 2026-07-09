#include "intf_hrpwm.h"
#include "intf_gptmr.h"
#include "intf_adc.h"
#include "intf_trgm.h"
#include "intf_uart.h"
#include "intf_gpio.h"
#include "intf_ws2812.h"
#include "intf_can.h"
#include "intf_synt.h"

#include <stddef.h>

#ifndef ATTR_RAMFUNC
#define ATTR_RAMFUNC __attribute__((section(".fast")))
#endif

/* ============================================================================
 * HRPWM Interface
 * ============================================================================ */

#define HRPWM_INSTANCE_COUNT (2U)

static const intf_hrpwm_t *hrpwm_ops[HRPWM_INSTANCE_COUNT] = {NULL};

int intf_hrpwm_register(const intf_hrpwm_t *ops)
{
    if (ops == NULL || ops->instance_id >= HRPWM_INSTANCE_COUNT) return -1;
    hrpwm_ops[ops->instance_id] = ops;
    return 0;
}

ATTR_RAMFUNC
static const intf_hrpwm_t *hrpwm_get_ops_by_ch(intf_hrpwm_ch_t ch)
{
    uint8_t inst = ch / 4;
    if (inst >= HRPWM_INSTANCE_COUNT) return NULL;
    return hrpwm_ops[inst];
}

int intf_hrpwm_init_pair(intf_hrpwm_ch_t ch, const intf_hrpwm_pair_cfg_t *cfg)
{
    const intf_hrpwm_t *ops = hrpwm_get_ops_by_ch(ch);
    if (ops && ops->init_pair) return ops->init_pair(ch, cfg);
    return -1;
}

ATTR_RAMFUNC
int intf_hrpwm_set_duty(intf_hrpwm_ch_t ch, float duty)
{
    const intf_hrpwm_t *ops = hrpwm_get_ops_by_ch(ch);
    if (ops && ops->set_duty) return ops->set_duty(ch, duty);
    return -1;
}

ATTR_RAMFUNC
int intf_hrpwm_set_duty_direct(intf_hrpwm_ch_t ch, float duty)
{
    const intf_hrpwm_t *ops = hrpwm_get_ops_by_ch(ch);
    if (ops && ops->set_duty_direct) return ops->set_duty_direct(ch, duty);
    return -1;
}

ATTR_RAMFUNC
int intf_hrpwm_set_duty_direct_dual(intf_hrpwm_ch_t ch_a, float duty_a,
                                     intf_hrpwm_ch_t ch_b, float duty_b)
{
    const intf_hrpwm_t *ops = hrpwm_get_ops_by_ch(ch_a);
    if (ops && ops->set_duty_direct_dual) return ops->set_duty_direct_dual(ch_a, duty_a, ch_b, duty_b);
    return -1;
}

int intf_hrpwm_set_frequency(intf_hrpwm_inst_t inst, uint32_t frequency_hz)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->set_frequency) return hrpwm_ops[inst]->set_frequency(frequency_hz);
    return -1;
}

int intf_hrpwm_set_jitter(intf_hrpwm_ch_t ch, uint8_t jitter_cmp)
{
    const intf_hrpwm_t *ops = hrpwm_get_ops_by_ch(ch);
    if (ops && ops->set_jitter) return ops->set_jitter(ch, jitter_cmp);
    return -1;
}

int intf_hrpwm_start(intf_hrpwm_ch_t ch)
{
    const intf_hrpwm_t *ops = hrpwm_get_ops_by_ch(ch);
    if (ops && ops->start) return ops->start(ch);
    return -1;
}

int intf_hrpwm_stop(intf_hrpwm_ch_t ch)
{
    const intf_hrpwm_t *ops = hrpwm_get_ops_by_ch(ch);
    if (ops && ops->stop) return ops->stop(ch);
    return -1;
}

int intf_hrpwm_start_counter_only(intf_hrpwm_inst_t inst)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->start_counter_only) return hrpwm_ops[inst]->start_counter_only();
    return -1;
}

ATTR_RAMFUNC
int intf_hrpwm_force_low(intf_hrpwm_ch_t ch)
{
    const intf_hrpwm_t *ops = hrpwm_get_ops_by_ch(ch);
    if (ops && ops->force_low) return ops->force_low(ch);
    return -1;
}

ATTR_RAMFUNC
int intf_hrpwm_force_release(intf_hrpwm_ch_t ch)
{
    const intf_hrpwm_t *ops = hrpwm_get_ops_by_ch(ch);
    if (ops && ops->force_release) return ops->force_release(ch);
    return -1;
}

int intf_hrpwm_config_fault(intf_hrpwm_inst_t inst, const intf_hrpwm_fault_cfg_t *cfg)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->config_fault) return hrpwm_ops[inst]->config_fault(cfg);
    return -1;
}

int intf_hrpwm_clear_fault(intf_hrpwm_inst_t inst)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->clear_fault) return hrpwm_ops[inst]->clear_fault();
    return -1;
}

int intf_hrpwm_config_reload_irq(intf_hrpwm_inst_t inst, intf_hrpwm_irq_callback_t callback)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->config_reload_irq) return hrpwm_ops[inst]->config_reload_irq(callback);
    return -1;
}

int intf_hrpwm_enable_reload_irq(intf_hrpwm_inst_t inst)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->enable_reload_irq) return hrpwm_ops[inst]->enable_reload_irq();
    return -1;
}

int intf_hrpwm_disable_reload_irq(intf_hrpwm_inst_t inst)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->disable_reload_irq) return hrpwm_ops[inst]->disable_reload_irq();
    return -1;
}

ATTR_RAMFUNC
int intf_hrpwm_set_phase(const intf_hrpwm_phase_cfg_t *cfg)
{
    if (cfg == NULL || cfg->inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[cfg->inst] == NULL) return -1;
    if (hrpwm_ops[cfg->inst]->set_phase) return hrpwm_ops[cfg->inst]->set_phase(cfg);
    return -1;
}

int intf_hrpwm_config_phase_limit(intf_hrpwm_inst_t inst, const intf_hrpwm_phase_limit_t *limit)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->config_phase_limit) return hrpwm_ops[inst]->config_phase_limit(limit);
    return -1;
}

int intf_hrpwm_config_trigger_cmp(intf_hrpwm_inst_t inst, uint8_t cmp_index, float position_ratio)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->config_trigger_cmp) return hrpwm_ops[inst]->config_trigger_cmp(cmp_index, position_ratio);
    return -1;
}

ATTR_RAMFUNC
int intf_hrpwm_set_trigger_cmp_position(intf_hrpwm_inst_t inst, uint8_t cmp_index, float position_ratio)
{
    if (inst >= HRPWM_INSTANCE_COUNT || hrpwm_ops[inst] == NULL) return -1;
    if (hrpwm_ops[inst]->set_trigger_cmp_position) return hrpwm_ops[inst]->set_trigger_cmp_position(cmp_index, position_ratio);
    return -1;
}

/* ============================================================================
 * GPTMR Interface
 * ============================================================================ */

#define GPTMR_INSTANCE_COUNT (4U)

static const intf_gptmr_t *gptmr_ops[GPTMR_INSTANCE_COUNT] = {NULL};

int intf_gptmr_register(const intf_gptmr_t *ops)
{
    if (ops == NULL || ops->instance_id >= GPTMR_INSTANCE_COUNT) return -1;
    gptmr_ops[ops->instance_id] = ops;
    return 0;
}

static const intf_gptmr_t *gptmr_get_ops(intf_gptmr_ch_t ch)
{
    uint8_t inst = ch / 4U;
    if (inst >= GPTMR_INSTANCE_COUNT || gptmr_ops[inst] == NULL) return NULL;
    return gptmr_ops[inst];
}

int intf_gptmr_init(intf_gptmr_ch_t ch, const intf_gptmr_cfg_t *cfg)
{
    const intf_gptmr_t *ops = gptmr_get_ops(ch);
    if (ops && ops->init) return ops->init(ch, cfg);
    return -1;
}

int intf_gptmr_start(intf_gptmr_ch_t ch)
{
    const intf_gptmr_t *ops = gptmr_get_ops(ch);
    if (ops && ops->start) return ops->start(ch);
    return -1;
}

int intf_gptmr_stop(intf_gptmr_ch_t ch)
{
    const intf_gptmr_t *ops = gptmr_get_ops(ch);
    if (ops && ops->stop) return ops->stop(ch);
    return -1;
}

int intf_gptmr_set_duty(intf_gptmr_ch_t ch, float duty)
{
    const intf_gptmr_t *ops = gptmr_get_ops(ch);
    if (ops && ops->set_duty) return ops->set_duty(ch, duty);
    return -1;
}

int intf_gptmr_set_frequency(intf_gptmr_ch_t ch, uint32_t frequency_hz)
{
    const intf_gptmr_t *ops = gptmr_get_ops(ch);
    if (ops && ops->set_frequency) return ops->set_frequency(ch, frequency_hz);
    return -1;
}

int intf_gptmr_force_low(intf_gptmr_ch_t ch)
{
    const intf_gptmr_t *ops = gptmr_get_ops(ch);
    if (ops && ops->force_low) return ops->force_low(ch);
    return -1;
}

int intf_gptmr_force_release(intf_gptmr_ch_t ch)
{
    const intf_gptmr_t *ops = gptmr_get_ops(ch);
    if (ops && ops->force_release) return ops->force_release(ch);
    return -1;
}

int intf_gptmr_capture_poll(intf_gptmr_ch_t ch, intf_gptmr_capture_t *capture)
{
    const intf_gptmr_t *ops = gptmr_get_ops(ch);
    if (ops && ops->capture_poll) return ops->capture_poll(ch, capture);
    return -1;
}

/* ============================================================================
 * TRGM Interface
 * ============================================================================ */

static const intf_trgm_t *trgm_ops = NULL;

int intf_trgm_register(const intf_trgm_t *ops)
{
    if (ops == NULL) return -1;
    trgm_ops = ops;
    return 0;
}

int intf_trgm_connect(intf_trgm_src_t src, intf_trgm_dst_t dst)
{
    if (trgm_ops && trgm_ops->connect) return trgm_ops->connect(src, dst);
    return -1;
}

/* ============================================================================
 * ADC Interface
 * ============================================================================ */

static const intf_adc_t *adc_ops[INTF_ADC_INSTANCE_COUNT] = {NULL};

int intf_adc_register(const intf_adc_t *ops)
{
    if (ops == NULL || ops->instance_id >= INTF_ADC_INSTANCE_COUNT) return -1;
    adc_ops[ops->instance_id] = ops;
    return 0;
}

static const intf_adc_t *adc_get_ops_by_ch(intf_adc_ch_t ch)
{
    uint8_t inst = INTF_ADC_CH_INST(ch);
    if (inst >= INTF_ADC_INSTANCE_COUNT) return NULL;
    return adc_ops[inst];
}

int intf_adc_init(intf_adc_ch_t ch, const intf_adc_cfg_t *cfg)
{
    const intf_adc_t *ops = adc_get_ops_by_ch(ch);
    if (ops && ops->init) return ops->init(ch, cfg);
    return -1;
}

int intf_adc_read(intf_adc_ch_t ch, uint16_t *value)
{
    const intf_adc_t *ops = adc_get_ops_by_ch(ch);
    if (ops && ops->read) return ops->read(ch, value);
    return -1;
}

int intf_adc_read_voltage(intf_adc_ch_t ch, float *voltage_mv)
{
    const intf_adc_t *ops = adc_get_ops_by_ch(ch);
    if (ops && ops->read_voltage) return ops->read_voltage(ch, voltage_mv);
    return -1;
}

int intf_adc_start(intf_adc_ch_t ch)
{
    const intf_adc_t *ops = adc_get_ops_by_ch(ch);
    if (ops && ops->start) return ops->start(ch);
    return -1;
}

int intf_adc_stop(intf_adc_ch_t ch)
{
    const intf_adc_t *ops = adc_get_ops_by_ch(ch);
    if (ops && ops->stop) return ops->stop(ch);
    return -1;
}

void intf_adc_set_vref(intf_adc_ch_t ch, float vref_mv)
{
    const intf_adc_t *ops = adc_get_ops_by_ch(ch);
    if (ops && ops->set_vref) ops->set_vref(vref_mv);
}

extern void adc_wdog_reenable(uint8_t inst, uint8_t ch);

void intf_adc_wdog_reenable(intf_adc_ch_t ch)
{
    uint8_t inst   = INTF_ADC_CH_INST(ch);
    uint8_t ch_idx = INTF_ADC_CH_IDX(ch);
    if (inst < INTF_ADC_INSTANCE_COUNT) {
        adc_wdog_reenable(inst, ch_idx);
    }
}

int intf_adc_calibrate(intf_adc_ch_t ch)
{
    const intf_adc_t *ops = adc_get_ops_by_ch(ch);
    if (ops && ops->calibrate) return ops->calibrate();
    return -1;
}

__attribute__((weak)) int adc_get_diag_snapshot(intf_adc_diag_snapshot_t *snapshot)
{
    (void)snapshot;
    return -1;
}

int intf_adc_get_diag_snapshot(intf_adc_diag_snapshot_t *snapshot)
{
    return adc_get_diag_snapshot(snapshot);
}

__attribute__((weak)) void adc_reset_diag_max(void) { }

void intf_adc_reset_diag_max(void)
{
    adc_reset_diag_max();
}

/* ============================================================================
 * UART Interface
 * ============================================================================ */

static const intf_uart_ops_t *uart_ops = NULL;

int intf_uart_register(const intf_uart_ops_t *ops)
{
    if (ops == NULL) return -1;
    uart_ops = ops;
    return 0;
}

int intf_uart_init(intf_uart_port_t port, const intf_uart_cfg_t *cfg)
{
    if (uart_ops && uart_ops->init) return uart_ops->init(port, cfg);
    return -1;
}

int intf_uart_transmit(intf_uart_port_t port, const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (uart_ops && uart_ops->transmit) return uart_ops->transmit(port, data, len, timeout_ms);
    return -1;
}

int intf_uart_receive(intf_uart_port_t port, uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (uart_ops && uart_ops->receive) return uart_ops->receive(port, data, len, timeout_ms);
    return -1;
}

int intf_uart_register_rx_callback(intf_uart_port_t port, intf_uart_rx_cb_t cb)
{
    if (uart_ops && uart_ops->register_rx_callback) return uart_ops->register_rx_callback(port, cb);
    return -1;
}

/* ============================================================================
 * GPIO Interface
 * ============================================================================ */

static const intf_gpio_t *gpio_ops = NULL;

int intf_gpio_register(const intf_gpio_t *ops)
{
    if (ops == NULL) return -1;
    gpio_ops = ops;
    return 0;
}

int intf_gpio_init(const intf_gpio_cfg_t *cfg)
{
    if (gpio_ops && gpio_ops->init) return gpio_ops->init(cfg);
    return -1;
}

int intf_gpio_set_level(intf_gpio_pin_t pin, intf_gpio_level_t level)
{
    if (gpio_ops && gpio_ops->set_level) return gpio_ops->set_level(pin, level);
    return -1;
}

int intf_gpio_get_level(intf_gpio_pin_t pin, intf_gpio_level_t *level)
{
    if (gpio_ops && gpio_ops->get_level) return gpio_ops->get_level(pin, level);
    return -1;
}

int intf_gpio_toggle(intf_gpio_pin_t pin)
{
    if (gpio_ops && gpio_ops->toggle) return gpio_ops->toggle(pin);
    return -1;
}

/* ============================================================================
 * WS2812 Interface
 * ============================================================================ */

static const intf_ws2812_ops_t *ws2812_ops = NULL;

int intf_ws2812_register(const intf_ws2812_ops_t *ops)
{
    if (ops == NULL) return -1;
    ws2812_ops = ops;
    return 0;
}

int intf_ws2812_init(const intf_ws2812_cfg_t *cfg)
{
    if (ws2812_ops && ws2812_ops->init) return ws2812_ops->init(cfg);
    return -1;
}

int intf_ws2812_set_pixel(intf_ws2812_pixel_t index, intf_ws2812_rgb_t color)
{
    if (ws2812_ops && ws2812_ops->set_pixel) return ws2812_ops->set_pixel(index, color);
    return -1;
}

int intf_ws2812_set_pixels(const intf_ws2812_rgb_t *colors, uint32_t count)
{
    if (ws2812_ops && ws2812_ops->set_pixels) return ws2812_ops->set_pixels(colors, count);
    return -1;
}

int intf_ws2812_update(bool blocking)
{
    if (ws2812_ops && ws2812_ops->update) return ws2812_ops->update(blocking);
    return -1;
}

int intf_ws2812_clear(void)
{
    if (ws2812_ops && ws2812_ops->clear) return ws2812_ops->clear();
    return -1;
}

bool intf_ws2812_is_busy(void)
{
    if (ws2812_ops && ws2812_ops->is_busy) return ws2812_ops->is_busy();
    return false;
}

void intf_ws2812_deinit(void)
{
    if (ws2812_ops && ws2812_ops->deinit) ws2812_ops->deinit();
}

/* ============================================================================
 * CAN Interface
 * ============================================================================ */

#define CAN_INSTANCE_COUNT (4U)

static const intf_can_t *can_ops[CAN_INSTANCE_COUNT] = {NULL};

int intf_can_register(const intf_can_t *ops)
{
    if (ops == NULL || ops->instance_id >= CAN_INSTANCE_COUNT) return -1;
    can_ops[ops->instance_id] = ops;
    return 0;
}

int intf_can_init(intf_can_inst_t inst, const intf_can_cfg_t *cfg)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->init) return can_ops[inst]->init(cfg);
    return -1;
}

void intf_can_deinit(intf_can_inst_t inst)
{
    if (inst < CAN_INSTANCE_COUNT && can_ops[inst] && can_ops[inst]->deinit)
        can_ops[inst]->deinit();
}

int intf_can_send(intf_can_inst_t inst, const intf_can_frame_t *frame,
                   uint32_t timeout_ms)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->send) return can_ops[inst]->send(frame, timeout_ms);
    return -1;
}

int intf_can_send_nonblocking(intf_can_inst_t inst,
                               const intf_can_frame_t *frame,
                               uint8_t *fifo_idx)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->send_nonblocking) return can_ops[inst]->send_nonblocking(frame, fifo_idx);
    return -1;
}

int intf_can_send_add_request(intf_can_inst_t inst, uint8_t fifo_idx)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->send_add_request) return can_ops[inst]->send_add_request(fifo_idx);
    return -1;
}

int intf_can_receive(intf_can_inst_t inst, intf_can_frame_t *frame,
                      uint32_t timeout_ms)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->receive) return can_ops[inst]->receive(frame, timeout_ms);
    return -1;
}

int intf_can_receive_nonblocking(intf_can_inst_t inst,
                                  intf_can_frame_t *frame)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->receive_nonblocking) return can_ops[inst]->receive_nonblocking(frame);
    return -1;
}

int intf_can_config_filter(intf_can_inst_t inst, uint32_t index,
                            const intf_can_filter_elem_t *elem)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->config_filter) return can_ops[inst]->config_filter(index, elem);
    return -1;
}

int intf_can_enable_interrupt(intf_can_inst_t inst, uint32_t event_mask)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->enable_interrupt) return can_ops[inst]->enable_interrupt(event_mask);
    return -1;
}

int intf_can_disable_interrupt(intf_can_inst_t inst, uint32_t event_mask)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->disable_interrupt) return can_ops[inst]->disable_interrupt(event_mask);
    return -1;
}

int intf_can_config_irq_callback(intf_can_inst_t inst,
                                  intf_can_irq_callback_t cb,
                                  void *user_data)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->config_irq_callback) return can_ops[inst]->config_irq_callback(cb, user_data);
    return -1;
}

int intf_can_get_status(intf_can_inst_t inst, intf_can_status_t *status)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->get_status) return can_ops[inst]->get_status(status);
    return -1;
}

int intf_can_read_tx_event(intf_can_inst_t inst, intf_can_tx_event_t *tx_evt)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->read_tx_event) return can_ops[inst]->read_tx_event(tx_evt);
    return -1;
}

int intf_can_get_timestamp(intf_can_inst_t inst,
                            const intf_can_tx_event_t *tx_evt,
                            intf_can_timestamp_t *ts)
{
    if (inst >= CAN_INSTANCE_COUNT || can_ops[inst] == NULL) return -1;
    if (can_ops[inst]->get_timestamp) return can_ops[inst]->get_timestamp(tx_evt, ts);
    return -1;
}

/* ============================================================================
 * SYNT Interface
 * ============================================================================ */

static const intf_synt_t *synt_ops = NULL;

int intf_synt_register(const intf_synt_t *ops)
{
    if (ops == NULL) return -1;
    synt_ops = ops;
    return 0;
}

int intf_synt_init(const intf_synt_cfg_t *cfg)
{
    if (synt_ops && synt_ops->init) return synt_ops->init(cfg);
    return -1;
}

int intf_synt_start(void)
{
    if (synt_ops && synt_ops->start) return synt_ops->start();
    return -1;
}

int intf_synt_stop(void)
{
    if (synt_ops && synt_ops->stop) return synt_ops->stop();
    return -1;
}

int intf_synt_reset(void)
{
    if (synt_ops && synt_ops->reset) return synt_ops->reset();
    return -1;
}

int intf_synt_set_reload(uint32_t reload_count)
{
    if (synt_ops && synt_ops->set_reload) return synt_ops->set_reload(reload_count);
    return -1;
}

int intf_synt_set_compare(intf_synt_ch_t ch, uint32_t cmp_count)
{
    if (synt_ops && synt_ops->set_compare) return synt_ops->set_compare(ch, cmp_count);
    return -1;
}

uint32_t intf_synt_get_count(void)
{
    if (synt_ops && synt_ops->get_count) return synt_ops->get_count();
    return 0;
}
