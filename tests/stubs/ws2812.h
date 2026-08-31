#ifndef TEST_STUB_WS2812_H
#define TEST_STUB_WS2812_H

#include <stdint.h>

#define PIXEL_MAX 2U

void neopixel_set_color(uint8_t num, uint32_t color);
void ws2812_show(void);
void ws2812_service(void);

#endif
