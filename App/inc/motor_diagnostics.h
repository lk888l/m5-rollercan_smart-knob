#ifndef MOTOR_DIAGNOSTICS_H
#define MOTOR_DIAGNOSTICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Called only by the owner of the outer motor controller. */
bool MotorDiagnosticsStep(void);

#ifdef __cplusplus
}
#endif
#endif
