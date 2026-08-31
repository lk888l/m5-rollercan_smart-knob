#ifndef TEST_STUB_MAIN_H
#define TEST_STUB_MAIN_H

#include <stdint.h>

#define PI 3.14159265358979323846f
#define __DMB() ((void)0)

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1,
} GPIO_PinState;

#define SYS_SW_GPIO_Port ((void *)1)
#define SYS_SW_Pin ((uint16_t)0x0040U)

uint32_t HAL_GetTick(void);
GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin);

#endif
