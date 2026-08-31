#ifndef LOCAL_UI_H
#define LOCAL_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void LocalUiInitialize(void);
void LocalUiTask(void);
uint8_t LocalUiIsMenuActive(void);
uint8_t LocalUiIsEditing(void);

#ifdef __cplusplus
}
#endif

#endif
