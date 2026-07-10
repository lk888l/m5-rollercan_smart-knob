#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_CONTROL_COMMAND_CYCLE_MODE = 1
} AppControlCommandType;

extern volatile uint32_t app_control_step_count;
extern volatile uint32_t app_control_deadline_miss_count;
extern volatile uint32_t app_control_can_frame_count;
extern volatile uint32_t app_control_command_drop_count;
extern volatile uint32_t app_control_stack_min_words;
extern volatile uint32_t app_maintenance_stack_min_words;
extern volatile uint32_t app_storage_stack_min_words;

void App_StartScheduler(void);
bool App_NotifyCanRxFromISR(void);
bool App_PostControlCommand(AppControlCommandType type, int32_t value);

#ifdef __cplusplus
}
#endif

#endif
