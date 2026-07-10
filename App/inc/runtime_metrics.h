#ifndef RUNTIME_METRICS_H
#define RUNTIME_METRICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t runtime_foc_last_cycles;
extern volatile uint32_t runtime_foc_max_cycles;
extern volatile uint32_t runtime_control_last_cycles;
extern volatile uint32_t runtime_control_max_cycles;
extern volatile uint32_t runtime_control_period_min_cycles;
extern volatile uint32_t runtime_control_period_max_cycles;
extern volatile uint32_t runtime_control_jitter_max_cycles;
extern volatile uint32_t runtime_can_frame_max_cycles;
extern volatile uint32_t runtime_can_bridge_max_cycles;

void RuntimeMetricsInit(void);
uint32_t RuntimeMetricsCycleNow(void);
void RuntimeMetricsRecordFoc(uint32_t start_cycles);
uint32_t RuntimeMetricsControlBegin(void);
void RuntimeMetricsRecordControl(uint32_t start_cycles);
void RuntimeMetricsRecordCanFrame(uint32_t start_cycles);
void RuntimeMetricsRecordCanBridge(uint32_t start_cycles);

#ifdef __cplusplus
}
#endif

#endif
