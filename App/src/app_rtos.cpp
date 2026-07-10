#include "app_rtos.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "fdcan.h"
#include "main.h"
#include "mysys.h"
#include "runtime_metrics.h"

namespace {

constexpr UBaseType_t kStoragePriority = tskIDLE_PRIORITY + 1U;
constexpr UBaseType_t kMaintenancePriority = tskIDLE_PRIORITY + 2U;
constexpr UBaseType_t kControlPriority = tskIDLE_PRIORITY + 5U;
constexpr uint32_t kMaintenancePeriodMs = 10U;
constexpr uint32_t kStoragePeriodMs = 20U;
constexpr uint32_t kControlPeriodMs = 1U;
constexpr UBaseType_t kControlCommandQueueLength = 8U;
constexpr uint32_t kCanFramesPerPass = 4U;

struct ControlCommand {
    AppControlCommandType type;
    int32_t value;
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

        ControlCommand command{};
        while (xQueueReceive(control_command_queue, &command, 0U) == pdTRUE) {
            ApplyControlCommand(command);
        }

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

        /* Protocol work shares the single-writer context, but it must not
           drain an unbounded FIFO ahead of the next control deadline. */
        for (uint32_t frame = 0U; frame < kCanFramesPerPass; ++frame) {
            if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) {
                break;
            }
            if (static_cast<int32_t>(xTaskGetTickCount() - next_control) >= 0) {
                break;
            }
            const uint32_t runtime_start_cycles = RuntimeMetricsCycleNow();
            FDCAN_ProcessPending();
            RuntimeMetricsRecordCanFrame(runtime_start_cycles);
            ++app_control_can_frame_count;
        }
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
}

extern "C" void App_StartScheduler(void)
{
    control_command_queue = xQueueCreateStatic(kControlCommandQueueLength,
                                                sizeof(ControlCommand),
                                                control_command_queue_buffer,
                                                &control_command_queue_storage);
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
    storage_handle = xTaskCreateStatic(StorageTask,
                                       "Storage",
                                       static_cast<uint32_t>(sizeof(storage_stack) / sizeof(storage_stack[0])),
                                       nullptr,
                                       kStoragePriority,
                                       storage_stack,
                                       &storage_tcb);

    configASSERT(control_command_queue != nullptr);
    configASSERT(maintenance_handle != nullptr);
    configASSERT(control_handle != nullptr);
    configASSERT(storage_handle != nullptr);

    RuntimeMetricsInit();
    vTaskStartScheduler();
    Error_Handler();
}

extern "C" bool App_NotifyCanRxFromISR(void)
{
    if ((control_handle == nullptr) ||
        (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)) {
        return false;
    }

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(control_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
    return true;
}

extern "C" bool App_PostControlCommand(AppControlCommandType type, int32_t value)
{
    if ((control_handle == nullptr) || (control_command_queue == nullptr)) {
        return false;
    }

    const ControlCommand command{type, value};
    if (xQueueSendToBack(control_command_queue, &command, 0U) != pdTRUE) {
        ++app_control_command_drop_count;
        return false;
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
