#ifndef LOCAL_RGB_H
#define LOCAL_RGB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  LOCAL_RGB_STATE_RUNNING = 0,
  LOCAL_RGB_STATE_PAUSED,
  LOCAL_RGB_STATE_MENU,
  LOCAL_RGB_STATE_EDIT,
  LOCAL_RGB_STATE_SAVING,
  LOCAL_RGB_STATE_FAULT,
} LocalRgbState;

void LocalRgbInitialize(void);
void LocalRgbTask(void);
void LocalRgbNotifySave(void);
LocalRgbState LocalRgbGetState(void);
uint32_t LocalRgbGetColor(void);

#ifdef __cplusplus
}
#endif

#endif
