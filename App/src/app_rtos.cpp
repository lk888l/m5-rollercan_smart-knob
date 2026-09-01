#include "app_rtos.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "fdcan.h"
#include "main.h"
#include "mysys.h"
#include "runtime_metrics.h"
#include "u8g2_disp_fun.h"

namespace {

constexpr UBaseType_t kStoragePriority = tskIDLE_PRIORITY + 1U;
constexpr UBaseType_t kMaintenancePriority = tskIDLE_PRIORITY + 2U;
constexpr UBaseType_t kCommunicationPriority = tskIDLE_PRIORITY + 3U;
constexpr UBaseType_t kControlPriority = tskIDLE_PRIORITY + 5U;
constexpr uint32_t kMaintenancePeriodMs = 10U;
constexpr uint32_t kStoragePeriodMs = 20U;
constexpr uint32_t kControlPeriodMs = 1U;
constexpr UBaseType_t kControlCommandQueueLength = 8U;
constexpr UBaseType_t kCanResponseQueueLength = 8U;
constexpr uint32_t kCanFramesPerPass = 16U;
constexpr uint32_t kControlCommandsPerPass = 4U;

struct ControlCommand {
    AppControlCommandType type;
    int32_t value;
    LocalProfileEdit local_edit;
    CanProtocolCommand can_command;
};

StackType_t maintenance_stack[768];
StaticTask_t maintenance_tcb;
TaskHandle_t maintenance_handle = nullptr;

StackType_t control_stack[896];
StaticTask_t control_tcb;
TaskHandle_t control_handle = nullptr;

StaticQueue_t control_command_queue_storage;
uint8_t control_command_queue_buffer[kControlCommandQueueLength * sizeof(ControlCommand)];
QueueHandle_t control_command_queue = nullptr;

StaticQueue_t can_response_queue_storage;
uint8_t can_response_queue_buffer[kCanResponseQueueLength * sizeof(CanProtocolResponse)];
QueueHandle_t can_response_queue = nullptr;

StackType_t communication_stack[768];
StaticTask_t communication_tcb;
TaskHandle_t communication_handle = nullptr;

StackType_t storage_stack[384];
StaticTask_t storage_tcb;
TaskHandle_t storage_handle = nullptr;

StaticTask_t idle_tcb;
StackType_t idle_stack[configMINIMAL_STACK_SIZE];

void MaintenanceTask(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(kMaintenancePeriodMs);
    uint32_t stack_sample_count = 0U;

    for (;;) {
        LoopMysysOnce();
        if (++stack_sample_count >= 100U) {
            app_maintenance_stack_min_words =
                static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr));
            stack_sample_count = 0U;
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

void ApplyControlCommand(const ControlCommand &command)
{
    switch (command.type) {
    case APP_CONTROL_COMMAND_CYCLE_MODE:
        MysysCycleMode();
        break;
    case APP_CONTROL_COMMAND_LOCAL_MENU_ENTER:
        MysysLocalMenuEnter();
        break;
    case APP_CONTROL_COMMAND_LOCAL_MENU_EXIT:
        MysysLocalMenuExit(&command.local_edit);
        break;
    case APP_CONTROL_COMMAND_LOCAL_TOGGLE_OUTPUT:
        MysysLocalToggleOutput();
        break;
    case APP_CONTROL_COMMAND_CAN_PROTOCOL: {
        CanProtocolResponse response{};
        const uint32_t runtime_start_cycles = RuntimeMetricsCycleNow();
        MysysHostCommandBegin(&command.can_command);
        const bool command_valid =
            FDCAN_ProcessCommand(&command.can_command, &response) && response.valid;
        MysysHostCommandEnd(&command.can_command, command_valid);
        if (command_valid) {
            if (xQueueSendToBack(can_response_queue, &response, 0U) == pdTRUE) {
                const uint32_t waiting =
                    static_cast<uint32_t>(uxQueueMessagesWaiting(can_response_queue));
                if (waiting > app_can_response_queue_high_water) {
                    app_can_response_queue_high_water = waiting;
                }
                xTaskNotifyGive(communication_handle);
            } else {
                ++app_can_tx_drop_count;
            }
        }
        RuntimeMetricsRecordCanFrame(runtime_start_cycles);
        ++app_control_can_frame_count;
        break;
    }
    default:
        break;
    }
}

void ControlTask(void *)
{
    const TickType_t period = pdMS_TO_TICKS(kControlPeriodMs);
    TickType_t next_control = xTaskGetTickCount() + period;

    MysysControlTaskBegin();

    for (;;) {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait_ticks = 0U;
        if (static_cast<int32_t>(next_control - now) > 0) {
            wait_ticks = next_control - now;
        }

        /* CAN RX and command producers wake the task early; the absolute
           deadline below keeps the control step on a 1 ms cadence. */
        ulTaskNotifyTake(pdTRUE, wait_ticks);

        now = xTaskGetTickCount();
        if (static_cast<int32_t>(now - next_control) >= 0) {
            const TickType_t lateness = now - next_control;
            if (lateness > 0U) {
                app_control_deadline_miss_count +=
                    static_cast<uint32_t>(lateness / period);
            }
            const uint32_t runtime_start_cycles = RuntimeMetricsControlBegin();
            MysysControlStep();
            RuntimeMetricsRecordControl(runtime_start_cycles);
            ++app_control_step_count;
            if ((app_control_step_count % 1000U) == 0U) {
                app_control_stack_min_words =
                    static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr));
            }
            next_control += period;
            if (static_cast<int32_t>(now - next_control) >= 0) {
                /* Do not burst several stale control iterations after a long
                   protocol operation; resume from the next real deadline. */
                next_control = now + period;
            }
        }

        /* Control deadlines win over protocol work. Commands are bounded per
           pass and any remainder stays queued for the next wake/deadline. */
        for (uint32_t index = 0U; index < kControlCommandsPerPass; ++index) {
            if (static_cast<int32_t>(xTaskGetTickCount() - next_control) >= 0) {
                break;
            }
            ControlCommand command{};
            if (xQueueReceive(control_command_queue, &command, 0U) != pdTRUE) {
                break;
            }
            ApplyControlCommand(command);
        }

    }
}

void CommunicationTask(void *)
{
    TickType_t last_stack_sample = xTaskGetTickCount();

    for (;;) {
        if ((HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) &&
            (uxQueueMessagesWaiting(can_response_queue) == 0U)) {
            const uint32_t idle_wait_ms = FDCAN_SmartKnobTelemetryActive() ? 1U : 10U;
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(idle_wait_ms));
        } else {
            ulTaskNotifyTake(pdTRUE, 0U);
        }

        CanProtocolResponse response{};
        while (xQueueReceive(can_response_queue, &response, 0U) == pdTRUE) {
            if (FDCAN_SendResponse(&response) != 0U) {
                ++app_can_tx_drop_count;
            }
        }

        for (uint32_t frame = 0U; frame < kCanFramesPerPass; ++frame) {
            const uint32_t fifo_fill =
                HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
            if (fifo_fill > app_can_rx_fifo_high_water) {
                app_can_rx_fifo_high_water = fifo_fill;
            }
            if (fifo_fill == 0U) {
                break;
            }

            CanProtocolCommand command{};
            if (!FDCAN_ReadPendingCommand(&command)) {
                ++app_can_rx_drop_count;
                break;
            }
            ++app_can_rx_count;

            if (FDCAN_IsBridgeCommand(&command)) {
                const uint32_t runtime_start_cycles = RuntimeMetricsCycleNow();
                response = {};
                if (FDCAN_ProcessCommand(&command, &response) && response.valid &&
                    (FDCAN_SendResponse(&response) != 0U)) {
                    ++app_can_tx_drop_count;
                }
                RuntimeMetricsRecordCanBridge(runtime_start_cycles);
                ++app_can_bridge_count;
            } else if (!App_PostCanControlCommand(&command)) {
                ++app_can_rx_drop_count;
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_stack_sample) >= pdMS_TO_TICKS(1000U)) {
            app_communication_stack_min_words =
                static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr));
            last_stack_sample = now;
        }

        /* A higher-priority ControlTask may have produced replies while this
           task was draining RX. Send them before an ID/baud reconfiguration. */
        while (xQueueReceive(can_response_queue, &response, 0U) == pdTRUE) {
            if (FDCAN_SendResponse(&response) != 0U) {
                ++app_can_tx_drop_count;
            }
        }
        app_can_tx_drop_count += FDCAN_ServiceSmartKnobTelemetry();
        FDCAN_ServiceReconfiguration();
    }
}

void StorageTask(void *)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(kStoragePeriodMs);
    uint32_t stack_sample_count = 0U;

    for (;;) {
        MysysStorageOnce();
        if (++stack_sample_count >= 50U) {
            app_storage_stack_min_words =
                static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr));
            stack_sample_count = 0U;
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

}  // namespace

extern "C" {
volatile uint32_t app_control_step_count = 0U;
volatile uint32_t app_control_deadline_miss_count = 0U;
volatile uint32_t app_control_can_frame_count = 0U;
volatile uint32_t app_control_command_drop_count = 0U;
volatile uint32_t app_control_stack_min_words = UINT32_MAX;
volatile uint32_t app_maintenance_stack_min_words = UINT32_MAX;
volatile uint32_t app_storage_stack_min_words = UINT32_MAX;
volatile uint32_t app_communication_stack_min_words = UINT32_MAX;
volatile uint32_t app_can_rx_count = 0U;
volatile uint32_t app_can_bridge_count = 0U;
volatile uint32_t app_can_rx_drop_count = 0U;
volatile uint32_t app_can_tx_drop_count = 0U;
volatile uint32_t app_can_hw_fifo_loss_count = 0U;
volatile uint32_t app_can_rx_fifo_high_water = 0U;
volatile uint32_t app_can_control_queue_high_water = 0U;
volatile uint32_t app_can_response_queue_high_water = 0U;
}

extern "C" void App_StartScheduler(void)
{
    control_command_queue = xQueueCreateStatic(kControlCommandQueueLength,
                                                sizeof(ControlCommand),
                                                control_command_queue_buffer,
                                                &control_command_queue_storage);
    can_response_queue = xQueueCreateStatic(kCanResponseQueueLength,
                                             sizeof(CanProtocolResponse),
                                             can_response_queue_buffer,
                                             &can_response_queue_storage);
    maintenance_handle = xTaskCreateStatic(MaintenanceTask,
                                            "Maintenance",
                                            static_cast<uint32_t>(sizeof(maintenance_stack) / sizeof(maintenance_stack[0])),
                                            nullptr,
                                            kMaintenancePriority,
                                            maintenance_stack,
                                            &maintenance_tcb);
    control_handle = xTaskCreateStatic(ControlTask,
                                       "Control",
                                       static_cast<uint32_t>(sizeof(control_stack) / sizeof(control_stack[0])),
                                       nullptr,
                                       kControlPriority,
                                       control_stack,
                                       &control_tcb);
    if (comm_type != COMM_TYPE_NONE) {
        communication_handle = xTaskCreateStatic(
            CommunicationTask,
            "Communication",
            static_cast<uint32_t>(sizeof(communication_stack) /
                                  sizeof(communication_stack[0])),
            nullptr,
            kCommunicationPriority,
            communication_stack,
            &communication_tcb);
    }
    storage_handle = xTaskCreateStatic(StorageTask,
                                       "Storage",
                                       static_cast<uint32_t>(sizeof(storage_stack) / sizeof(storage_stack[0])),
                                       nullptr,
                                       kStoragePriority,
                                       storage_stack,
                                       &storage_tcb);

    configASSERT(control_command_queue != nullptr);
    configASSERT(can_response_queue != nullptr);
    configASSERT(maintenance_handle != nullptr);
    configASSERT(control_handle != nullptr);
    configASSERT(comm_type == COMM_TYPE_NONE || communication_handle != nullptr);
    configASSERT(storage_handle != nullptr);

    RuntimeMetricsInit();
    vTaskStartScheduler();
    Error_Handler();
}

extern "C" bool App_NotifyCanRxFromISR(void)
{
    if ((communication_handle == nullptr) ||
        (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)) {
        return false;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(communication_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
    return true;
}

extern "C" bool App_PostControlCommand(AppControlCommandType type, int32_t value)
{
    if ((control_handle == nullptr) || (control_command_queue == nullptr)) {
        return false;
    }

    ControlCommand command{};
    command.type = type;
    command.value = value;
    if (xQueueSendToBack(control_command_queue, &command, 0U) != pdTRUE) {
        ++app_control_command_drop_count;
        return false;
    }

    const uint32_t waiting =
        static_cast<uint32_t>(uxQueueMessagesWaiting(control_command_queue));
    if (waiting > app_can_control_queue_high_water) {
        app_can_control_queue_high_water = waiting;
    }

    xTaskNotifyGive(control_handle);
    return true;
}

extern "C" bool App_PostLocalProfileEdit(const LocalProfileEdit *edit)
{
    if (edit == nullptr || control_handle == nullptr ||
        control_command_queue == nullptr) {
        return false;
    }

    ControlCommand command{};
    command.type = APP_CONTROL_COMMAND_LOCAL_MENU_EXIT;
    command.local_edit = *edit;
    if (xQueueSendToBack(control_command_queue, &command, 0U) != pdTRUE) {
        ++app_control_command_drop_count;
        return false;
    }

    const uint32_t waiting =
        static_cast<uint32_t>(uxQueueMessagesWaiting(control_command_queue));
    if (waiting > app_can_control_queue_high_water) {
        app_can_control_queue_high_water = waiting;
    }
    xTaskNotifyGive(control_handle);
    return true;
}

extern "C" bool App_PostCanControlCommand(const CanProtocolCommand *can_command)
{
    if ((can_command == nullptr) || (control_handle == nullptr) ||
        (control_command_queue == nullptr) || FDCAN_IsBridgeCommand(can_command)) {
        return false;
    }

    ControlCommand command{};
    command.type = APP_CONTROL_COMMAND_CAN_PROTOCOL;
    command.can_command = *can_command;
    if (xQueueSendToBack(control_command_queue, &command, 0U) != pdTRUE) {
        ++app_control_command_drop_count;
        return false;
    }

    const uint32_t waiting =
        static_cast<uint32_t>(uxQueueMessagesWaiting(control_command_queue));
    if (waiting > app_can_control_queue_high_water) {
        app_can_control_queue_high_water = waiting;
    }

    xTaskNotifyGive(control_handle);
    return true;
}

extern "C" void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,
                                               StackType_t **stack,
                                               uint32_t *stack_size)
{
    *tcb = &idle_tcb;
    *stack = idle_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *)
{
    Error_Handler();
}
