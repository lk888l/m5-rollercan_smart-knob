#pragma once

#include <cstdint>

#include "bsp_i2c.hpp"

namespace hardware {

enum class RollerCanStatus : uint8_t {
    ok = 0,
    invalid_argument,
    bus_error,
    device_not_found,
};

const char* toString(RollerCanStatus status);

using DelayMs = void (*)(uint32_t ms);

struct RollerCanConfig {
    DelayMs delay_ms = nullptr;
    uint32_t register_select_delay_ms = 2;
};

class RollerCan {
public:
    RollerCan(bsp::I2CDevice& device, RollerCanConfig config);

    RollerCan(const RollerCan&) = delete;
    RollerCan& operator=(const RollerCan&) = delete;

    RollerCanStatus disableOutput();
    RollerCanStatus startEncoderCalibration();
    RollerCanStatus readEncoderCalibrationBusy(bool& busy);
    RollerCanStatus saveEncoderCalibration();

private:
    RollerCanStatus writeCommand(uint8_t reg, uint8_t value);
    static RollerCanStatus fromI2CStatus(bsp::I2CStatus status);

    bsp::I2CDevice& device_;
    RollerCanConfig config_;
};

} // namespace hardware
