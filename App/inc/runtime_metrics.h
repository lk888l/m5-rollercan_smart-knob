#ifndef RUNTIME_METRICS_H
#define RUNTIME_METRICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t runtime_foc_last_cycles;
extern volatile uint32_t runtime_foc_max_cycles;
extern volatile uint32_t runtime_foc_cpu_last_cycles;
extern volatile uint32_t runtime_foc_cpu_max_cycles;
extern volatile uint32_t runtime_encoder_dma_last_cycles;
extern volatile uint32_t runtime_encoder_dma_max_cycles;
extern volatile uint32_t runtime_tim1_isr_last_cycles;
extern volatile uint32_t runtime_tim1_isr_max_cycles;
extern volatile uint32_t runtime_encoder_dma_isr_last_cycles;
extern volatile uint32_t runtime_encoder_dma_isr_max_cycles;
extern volatile uint32_t runtime_control_last_cycles;
extern volatile uint32_t runtime_control_max_cycles;
extern volatile uint32_t runtime_control_period_min_cycles;
extern volatile uint32_t runtime_control_period_max_cycles;
extern volatile uint32_t runtime_control_jitter_max_cycles;
extern volatile uint32_t runtime_can_frame_max_cycles;
extern volatile uint32_t runtime_can_bridge_max_cycles;

void RuntimeMetricsInit(void);
uint32_t RuntimeMetricsCycleNow(void);
uint32_t RuntimeMetricsElapsedSince(uint32_t start_cycles);
void RuntimeMetricsRecordFoc(uint32_t start_cycles);
void RuntimeMetricsRecordFocCpu(uint32_t active_cycles);
void RuntimeMetricsRecordEncoderDma(uint32_t start_cycles);
void RuntimeMetricsRecordTim1Isr(uint32_t start_cycles);
void RuntimeMetricsRecordEncoderDmaIsr(uint32_t start_cycles);
uint32_t RuntimeMetricsControlBegin(void);
void RuntimeMetricsRecordControl(uint32_t start_cycles);
void RuntimeMetricsRecordCanFrame(uint32_t start_cycles);
void RuntimeMetricsRecordCanBridge(uint32_t start_cycles);

#ifdef __cplusplus
}
#endif

#endif
