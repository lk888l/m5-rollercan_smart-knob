#ifndef FAST_CONTROL_LINK_H
#define FAST_CONTROL_LINK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t sequence;
  float iq_target_adc;
  uint32_t mode_generation;
  uint8_t driver_mode;
  uint8_t reserved[3];
} FastControlCommandSnapshot;

typedef struct {
  uint32_t sequence;
  uint16_t encoder_raw;
  uint16_t mechanical_angle_tenths;
  uint16_t bus_voltage_adc;
  uint16_t reserved;
  float phase_current_ma;
  int32_t internal_temp_raw;
} FastSensorSnapshot;

extern volatile uint32_t fast_control_command_apply_count;
extern volatile uint32_t fast_sensor_read_retry_count;
extern volatile uint32_t fast_sensor_read_failure_count;

void FastControlLinkInit(void);
void FastControlPublishCurrentAdc(float iq_target_adc);
void FastControlPublishDriverMode(uint8_t driver_mode);
bool FastControlConsumeCommandFromISR(FastControlCommandSnapshot *command);

void FastSensorPublishFromISR(uint16_t encoder_raw,
                             uint16_t mechanical_angle_tenths,
                             uint16_t bus_voltage_adc,
                             float phase_current_ma,
                             int32_t internal_temp_raw);
bool FastSensorRead(FastSensorSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
