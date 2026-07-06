#include "app_ws2812.h"
#include "intf_ws2812.h"

#define WS2812_PIXEL_COUNT 3

extern int drv_ws2812_register(void);

int app_ws2812_init(void)
{
    drv_ws2812_register();

    intf_ws2812_cfg_t cfg = {
        .pixel_count = WS2812_PIXEL_COUNT,
        .init_color = {0, 0, 0}
    };
    return intf_ws2812_init(&cfg);
}

int app_ws2812_set_pixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    intf_ws2812_rgb_t color = {r, g, b};
    return intf_ws2812_set_pixel(index, color);
}

int app_ws2812_update(bool blocking)
{
    return intf_ws2812_update(blocking);
}

int app_ws2812_clear(void)
{
    return intf_ws2812_clear();
}

static void hsv_to_rgb(uint8_t hue, uint8_t max_brightness, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t pos = hue;
    uint8_t sat = max_brightness;
    if (pos < 85) {
        *r = pos * 3 * sat / 255;
        *g = (255 - pos * 3) * sat / 255;
        *b = 0;
    } else if (pos < 170) {
        pos -= 85;
        *r = (255 - pos * 3) * sat / 255;
        *g = 0;
        *b = pos * 3 * sat / 255;
    } else {
        pos -= 170;
        *r = 0;
        *g = pos * 3 * sat / 255;
        *b = (255 - pos * 3) * sat / 255;
    }
}

int app_ws2812_rainbow(uint8_t max_brightness)
{
    static uint8_t hue = 0;
    uint8_t r, g, b;

    hsv_to_rgb(hue, max_brightness, &r, &g, &b);
    intf_ws2812_rgb_t color2 = {r, g, b};
    intf_ws2812_set_pixel(2, color2);

    hsv_to_rgb((hue + 85) % 255, max_brightness, &r, &g, &b);
    intf_ws2812_rgb_t color1 = {r, g, b};
    intf_ws2812_set_pixel(1, color1);

    hsv_to_rgb((hue + 170) % 255, max_brightness, &r, &g, &b);
    intf_ws2812_rgb_t color0 = {r, g, b};
    intf_ws2812_set_pixel(0, color0);

    hue++;
    return intf_ws2812_update(true);
}
