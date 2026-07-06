#ifndef _BOARD_WS2812_CONF_H
#define _BOARD_WS2812_CONF_H

#include "WS2812_Type.h"
#include <stdint.h>

#define WS2812_DIN           PA10
#define WS2812_GPTMR         0
#define WS2812_GPTMR_CHANNLE 2

#define WS2812_DMA         HPM_HDMA
#define WS2812_DMA_CHANNLE (0U)
#define WS2812_DMAMUX      HPM_DMAMUX
#define WS2812_DMA_IRQ     IRQn_HDMA

#define WS2812_LED_CONNECT WS2812_CONNECT_LINE
#define WS2812_LED_NUM     4
#endif
