#include "runtime_metrics.h"

#include <cstdint>
#include <climits>

#include "stm32g4xx.h"

namespace {

bool metrics_enabled = false;
uint32_t last_control_start_cycles = 0U;

void UpdateMaximum(volatile uint32_t &maximum, uint32_t value)
{
    if (value > maximum) {
        maximum = value;
    }
}

}  // namespace

extern "C" {
volatile uint32_t runtime_foc_last_cycles = 0U;
volatile uint32_t runtime_foc_max_cycles = 0U;
volatile uint32_t runtime_control_last_cycles = 0U;
volatile uint32_t runtime_control_max_cycles = 0U;
volatile uint32_t runtime_control_period_min_cycles = UINT32_MAX;
volatile uint32_t runtime_control_period_max_cycles = 0U;
volatile uint32_t runtime_control_jitter_max_cycles = 0U;
volatile uint32_t runtime_can_frame_max_cycles = 0U;
volatile uint32_t runtime_can_bridge_max_cycles = 0U;
}

extern "C" void RuntimeMetricsInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
    metrics_enabled = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
}

extern "C" uint32_t RuntimeMetricsCycleNow(void)
{
    return metrics_enabled ? DWT->CYCCNT : 0U;
}

extern "C" void RuntimeMetricsRecordFoc(uint32_t start_cycles)
{
    if (!metrics_enabled) {
        return;
    }

    const uint32_t elapsed = DWT->CYCCNT - start_cycles;
    runtime_foc_last_cycles = elapsed;
    UpdateMaximum(runtime_foc_max_cycles, elapsed);
}

extern "C" uint32_t RuntimeMetricsControlBegin(void)
{
    if (!metrics_enabled) {
        return 0U;
    }

    const uint32_t now = DWT->CYCCNT;
    if (last_control_start_cycles != 0U) {
        const uint32_t period = now - last_control_start_cycles;
        const uint32_t expected = SystemCoreClock / 1000U;
        const uint32_t jitter = (period > expected) ? (period - expected)
                                                    : (expected - period);
        if (period < runtime_control_period_min_cycles) {
            runtime_control_period_min_cycles = period;
        }
        UpdateMaximum(runtime_control_period_max_cycles, period);
        UpdateMaximum(runtime_control_jitter_max_cycles, jitter);
    }
    last_control_start_cycles = now;
    return now;
}

extern "C" void RuntimeMetricsRecordControl(uint32_t start_cycles)
{
    if (!metrics_enabled) {
        return;
    }

    const uint32_t elapsed = DWT->CYCCNT - start_cycles;
    runtime_control_last_cycles = elapsed;
    UpdateMaximum(runtime_control_max_cycles, elapsed);
}

extern "C" void RuntimeMetricsRecordCanFrame(uint32_t start_cycles)
{
    if (!metrics_enabled) {
        return;
    }

    UpdateMaximum(runtime_can_frame_max_cycles, DWT->CYCCNT - start_cycles);
}

extern "C" void RuntimeMetricsRecordCanBridge(uint32_t start_cycles)
{
    if (!metrics_enabled) {
        return;
    }

    UpdateMaximum(runtime_can_bridge_max_cycles, DWT->CYCCNT - start_cycles);
}
