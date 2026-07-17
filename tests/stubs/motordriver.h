#ifndef TEST_STUB_MOTORDRIVER_H
#define TEST_STUB_MOTORDRIVER_H

#include <stdint.h>

void MotorDriverSetCurrentReal(float phase_current_ma);
void MotorDriverSetCurrentRealContinuous(float phase_current_ma);
uint8_t MotorDriverIsOutputEnabled(void);

#endif
