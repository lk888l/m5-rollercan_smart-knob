#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdbool.h>
#include <stdint.h>

#include "can_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  APP_CONTROL_COMMAND_CYCLE_MODE = 1,
  APP_CONTROL_COMMAND_CAN_PROTOCOL = 2
} AppControlCommandType;

extern volatile uint32_t app_control_step_count;
extern volatile uint32_t app_control_deadline_miss_count;
extern volatile uint32_t app_control_can_frame_count;
extern volatile uint32_t app_control_command_drop_count;
extern volatile uint32_t app_control_stack_min_words;
extern volatile uint32_t app_maintenance_stack_min_words;
extern volatile uint32_t app_storage_stack_min_words;
extern volatile uint32_t app_communication_stack_min_words;
extern volatile uint32_t app_can_rx_count;
extern volatile uint32_t app_can_bridge_count;
extern volatile uint32_t app_can_rx_drop_count;
extern volatile uint32_t app_can_tx_drop_count;
extern volatile uint32_t app_can_hw_fifo_loss_count;
extern volatile uint32_t app_can_rx_fifo_high_water;
extern volatile uint32_t app_can_control_queue_high_water;
extern volatile uint32_t app_can_response_queue_high_water;

void App_StartScheduler(void);
bool App_NotifyCanRxFromISR(void);
bool App_PostControlCommand(AppControlCommandType type, int32_t value);
bool App_PostCanControlCommand(const CanProtocolCommand *command);

#ifdef __cplusplus
}
#endif

#endif
