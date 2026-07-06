#ifndef _APP_WS2812_H
#define _APP_WS2812_H

#include <stdint.h>
#include <stdbool.h>

int app_ws2812_init(void);
int app_ws2812_set_pixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b);
int app_ws2812_update(bool blocking);
int app_ws2812_clear(void);
int app_ws2812_rainbow(uint8_t max_brightness);

#endif
