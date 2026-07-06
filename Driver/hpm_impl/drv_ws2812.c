/*
 * WS2812 Driver Implementation - DMA + GPTMR Mode
 *
 * Copyright (c) 2024 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "intf_ws2812.h"
#include "WS2812.h"
#include "board.h"
#include "hpm_dma_mgr.h"

static int drv_ws2812_init(const intf_ws2812_cfg_t *cfg)
{
    if (cfg == NULL) {
        return -1;
    }

    dma_mgr_init();
    WS2812_Init();

    return 0;
}

static int drv_ws2812_set_pixel(intf_ws2812_pixel_t index, intf_ws2812_rgb_t color)
{
    WS2812_SetPixel(index, color.r, color.g, color.b);
    return 0;
}

static int drv_ws2812_set_pixels(const intf_ws2812_rgb_t *colors, uint32_t count)
{
    if (colors == NULL) {
        return -1;
    }

    for (uint32_t i = 0; i < count; i++) {
        WS2812_SetPixel(i, colors[i].r, colors[i].g, colors[i].b);
    }

    return 0;
}

static int drv_ws2812_update(bool blocking)
{
    WS2812_Update(blocking);
    return 0;
}

static int drv_ws2812_clear(void)
{
    for (uint32_t i = 0; i < WS2812_LED_NUM; i++) {
        WS2812_SetPixel(i, 0, 0, 0);
    }
    return 0;
}

static bool drv_ws2812_is_busy(void)
{
    return WS2812_IsBusy();
}

static void drv_ws2812_deinit(void)
{
    WS2812_Clear_Busy();
}

static const intf_ws2812_ops_t drv_ws2812_ops = {
    .init       = drv_ws2812_init,
    .set_pixel  = drv_ws2812_set_pixel,
    .set_pixels = drv_ws2812_set_pixels,
    .update     = drv_ws2812_update,
    .clear      = drv_ws2812_clear,
    .is_busy    = drv_ws2812_is_busy,
    .deinit     = drv_ws2812_deinit,
};

int drv_ws2812_register(void)
{
    return intf_ws2812_register(&drv_ws2812_ops);
}