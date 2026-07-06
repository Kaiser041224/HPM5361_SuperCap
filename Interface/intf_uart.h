/*
 * UART Interface - C11 Abstract Interface
 *
 * Copyright (c) 2024 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _INTF_UART_H
#define _INTF_UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Types
 * ============================================================================ */

typedef uint8_t intf_uart_port_t;

typedef struct {
    uint32_t baudrate;
    uint8_t  data_bits;
    uint8_t  stop_bits;
    uint8_t  parity;
    bool     flow_ctrl;
} intf_uart_cfg_t;

typedef void (*intf_uart_rx_cb_t)(uint8_t *data, size_t len);

typedef struct {
    int  (*init)(intf_uart_port_t port, const intf_uart_cfg_t *cfg);
    int  (*transmit)(intf_uart_port_t port, const uint8_t *data, size_t len, uint32_t timeout_ms);
    int  (*receive)(intf_uart_port_t port, uint8_t *data, size_t len, uint32_t timeout_ms);
    int  (*register_rx_callback)(intf_uart_port_t port, intf_uart_rx_cb_t cb);
    void (*deinit)(intf_uart_port_t port);
} intf_uart_ops_t;

/* ============================================================================
 * API
 * ============================================================================ */

int intf_uart_register(const intf_uart_ops_t *ops);
int intf_uart_init(intf_uart_port_t port, const intf_uart_cfg_t *cfg);
int intf_uart_transmit(intf_uart_port_t port, const uint8_t *data, size_t len, uint32_t timeout_ms);
int intf_uart_receive(intf_uart_port_t port, uint8_t *data, size_t len, uint32_t timeout_ms);
int intf_uart_register_rx_callback(intf_uart_port_t port, intf_uart_rx_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* _INTF_UART_H */
