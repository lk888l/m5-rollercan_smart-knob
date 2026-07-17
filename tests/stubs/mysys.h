#ifndef TEST_STUB_MYSYS_H
#define TEST_STUB_MYSYS_H

#include <stdint.h>

enum {
    MODE_SPEED = 1,
    MODE_POS,
    MODE_CURRENT,
    MODE_DIAL,
};

extern uint8_t motor_mode;
extern uint8_t error_code;
extern uint8_t over_vol_flag;
extern float mechanical_rad;
extern float motor_rps;
extern float ph_crrent_lpf;

uint32_t micros(void);

#endif
