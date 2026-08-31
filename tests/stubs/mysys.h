#ifndef TEST_STUB_MYSYS_H
#define TEST_STUB_MYSYS_H

#include <stdint.h>

enum {
    MODE_SPEED = 1,
    MODE_POS,
    MODE_CURRENT,
    MODE_DIAL,
};

enum {
    SYS_STANDBY = 0,
    SYS_RUNNING,
    SYS_ERROR,
};

enum {
    ERR_NONE = 0,
    ERR_OVER_VOLTAGE = 1 << 0,
    ERR_STALLED = 1 << 1,
    ERR_OVER_VALUE = 1 << 2,
};

extern uint8_t motor_mode;
extern uint8_t motor_output;
extern uint8_t sys_status;
extern uint8_t error_code;
extern uint8_t over_vol_flag;
extern float mechanical_rad;
extern float motor_rps;
extern float ph_crrent_lpf;

uint32_t micros(void);
uint8_t MysysLocalSavePending(void);

#endif
