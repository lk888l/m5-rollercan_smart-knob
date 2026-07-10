#include "fast_control_link.h"

#include <cstdint>

#include "stm32g4xx.h"

namespace {

struct CommandStorage {
    volatile uint32_t sequence;
    volatile float iq_target_adc;
    volatile uint32_t mode_generation;
    volatile uint8_t driver_mode;
};

struct SensorStorage {
    volatile uint32_t sequence;
    volatile uint16_t encoder_raw;
    volatile uint16_t mechanical_angle_tenths;
    volatile uint16_t bus_voltage_adc;
    volatile uint16_t reserved;
    volatile float phase_current_ma;
    volatile int32_t internal_temp_raw;
};

CommandStorage command_storage{};
SensorStorage sensor_storage{};
uint32_t consumed_command_sequence = UINT32_MAX;

}  // namespace

extern "C" {
volatile uint32_t fast_control_command_apply_count = 0U;
volatile uint32_t fast_sensor_read_retry_count = 0U;
volatile uint32_t fast_sensor_read_failure_count = 0U;
}

extern "C" void FastControlLinkInit(void)
{
    command_storage.sequence = 0U;
    command_storage.iq_target_adc = 0.0f;
    command_storage.mode_generation = 0U;
    command_storage.driver_mode = 0U;
    sensor_storage.sequence = 0U;
    sensor_storage.encoder_raw = 0U;
    sensor_storage.mechanical_angle_tenths = 0U;
    sensor_storage.bus_voltage_adc = 0U;
    sensor_storage.reserved = 0U;
    sensor_storage.phase_current_ma = 0.0f;
    sensor_storage.internal_temp_raw = 0;
    consumed_command_sequence = UINT32_MAX;
    __DMB();
}

extern "C" void FastControlPublishCurrentAdc(float iq_target_adc)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t next_sequence = (command_storage.sequence + 2U) & ~1U;
    command_storage.sequence = next_sequence - 1U;
    __DMB();
    command_storage.iq_target_adc = iq_target_adc;
    __DMB();
    command_storage.sequence = next_sequence;

    if (primask == 0U) {
        __enable_irq();
    }
}

extern "C" void FastControlPublishDriverMode(uint8_t driver_mode)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t next_sequence = (command_storage.sequence + 2U) & ~1U;
    command_storage.sequence = next_sequence - 1U;
    __DMB();
    command_storage.driver_mode = driver_mode;
    ++command_storage.mode_generation;
    __DMB();
    command_storage.sequence = next_sequence;

    if (primask == 0U) {
        __enable_irq();
    }
}

extern "C" bool FastControlConsumeCommandFromISR(FastControlCommandSnapshot *command)
{
    const uint32_t sequence_before = command_storage.sequence;
    if ((command == nullptr) || ((sequence_before & 1U) != 0U) ||
        (sequence_before == consumed_command_sequence)) {
        return false;
    }

    __DMB();
    const float target = command_storage.iq_target_adc;
    const uint32_t mode_generation = command_storage.mode_generation;
    const uint8_t driver_mode = command_storage.driver_mode;
    __DMB();
    if (sequence_before != command_storage.sequence) {
        return false;
    }

    consumed_command_sequence = sequence_before;
    command->sequence = sequence_before;
    command->iq_target_adc = target;
    command->mode_generation = mode_generation;
    command->driver_mode = driver_mode;
    command->reserved[0] = 0U;
    command->reserved[1] = 0U;
    command->reserved[2] = 0U;
    ++fast_control_command_apply_count;
    return true;
}

extern "C" void FastSensorPublishFromISR(uint16_t encoder_raw,
                                           uint16_t mechanical_angle_tenths,
                                           uint16_t bus_voltage_adc,
                                           float phase_current_ma,
                                           int32_t internal_temp_raw)
{
    const uint32_t next_sequence = (sensor_storage.sequence + 2U) & ~1U;
    sensor_storage.sequence = next_sequence - 1U;
    __DMB();
    sensor_storage.encoder_raw = encoder_raw;
    sensor_storage.mechanical_angle_tenths = mechanical_angle_tenths;
    sensor_storage.bus_voltage_adc = bus_voltage_adc;
    sensor_storage.phase_current_ma = phase_current_ma;
    sensor_storage.internal_temp_raw = internal_temp_raw;
    __DMB();
    sensor_storage.sequence = next_sequence;
}

extern "C" bool FastSensorRead(FastSensorSnapshot *snapshot)
{
    if (snapshot == nullptr) {
        return false;
    }

    for (uint32_t attempt = 0U; attempt < 3U; ++attempt) {
        const uint32_t sequence_before = sensor_storage.sequence;
        if ((sequence_before & 1U) != 0U) {
            continue;
        }

        __DMB();
        snapshot->encoder_raw = sensor_storage.encoder_raw;
        snapshot->mechanical_angle_tenths = sensor_storage.mechanical_angle_tenths;
        snapshot->bus_voltage_adc = sensor_storage.bus_voltage_adc;
        snapshot->reserved = 0U;
        snapshot->phase_current_ma = sensor_storage.phase_current_ma;
        snapshot->internal_temp_raw = sensor_storage.internal_temp_raw;
        __DMB();

        if (sequence_before == sensor_storage.sequence) {
            snapshot->sequence = sequence_before;
            fast_sensor_read_retry_count += attempt;
            return true;
        }
    }

    ++fast_sensor_read_failure_count;
    return false;
}
