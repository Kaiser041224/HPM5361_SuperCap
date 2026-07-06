/*
 * WS2812 RGB LED Interface - C11 Abstract Interface
 *
 * Copyright (c) 2024 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _INTF_WS2812_H
#define _INTF_WS2812_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} intf_ws2812_rgb_t;

typedef uint32_t intf_ws2812_pixel_t;

typedef struct {
    uint32_t pixel_count;
    intf_ws2812_rgb_t init_color;
} intf_ws2812_cfg_t;

typedef struct {
    int  (*init)(const intf_ws2812_cfg_t *cfg);
    int  (*set_pixel)(intf_ws2812_pixel_t index, intf_ws2812_rgb_t color);
    int  (*set_pixels)(const intf_ws2812_rgb_t *colors, uint32_t count);
    int  (*update)(bool blocking);
    int  (*clear)(void);
    bool (*is_busy)(void);
    void (*deinit)(void);
} intf_ws2812_ops_t;

int intf_ws2812_register(const intf_ws2812_ops_t *ops);
int intf_ws2812_init(const intf_ws2812_cfg_t *cfg);
int intf_ws2812_set_pixel(intf_ws2812_pixel_t index, intf_ws2812_rgb_t color);
int intf_ws2812_set_pixels(const intf_ws2812_rgb_t *colors, uint32_t count);
int intf_ws2812_update(bool blocking);
int intf_ws2812_clear(void);
bool intf_ws2812_is_busy(void);
void intf_ws2812_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* _INTF_WS2812_H */