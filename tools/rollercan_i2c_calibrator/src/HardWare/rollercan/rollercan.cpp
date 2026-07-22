#include "rollercan/rollercan.hpp"

#include <array>

namespace hardware {
namespace {

constexpr uint8_t kRegOutput = 0x00;
constexpr uint8_t kRegStartEncoderCalibration = 0xF1;
constexpr uint8_t kRegSaveEncoderCalibration = 0xF2;
constexpr uint8_t kRegEncoderCalibrationBusy = 0xF3;

} // namespace

const char* toString(RollerCanStatus status)
{
    switch (status) {
    case RollerCanStatus::ok:
        return "ok";
    case RollerCanStatus::invalid_argument:
        return "invalid_argument";
    case RollerCanStatus::bus_error:
        return "bus_error";
    case RollerCanStatus::device_not_found:
        return "device_not_found";
    default:
        return "unknown";
    }
}

RollerCan::RollerCan(bsp::I2CDevice& device, RollerCanConfig config)
    : device_(device)
    , config_(config)
{
}

RollerCanStatus RollerCan::disableOutput()
{
    return writeCommand(kRegOutput, 0x00);
}

RollerCanStatus RollerCan::startEncoderCalibration()
{
    return writeCommand(kRegStartEncoderCalibration, 0x01);
}

RollerCanStatus RollerCan::readEncoderCalibrationBusy(bool& busy)
{
    if (config_.delay_ms == nullptr) {
        return RollerCanStatus::invalid_argument;
    }

    // The original RollerCAN firmware prepares its F3 response in the receive
    // callback. Keep register selection and reading as two separate transfers so
    // the first transfer ends with STOP; do not replace this with writeRead().
    const uint8_t reg = kRegEncoderCalibrationBusy;
    RollerCanStatus status = fromI2CStatus(device_.write(&reg, 1));
    if (status != RollerCanStatus::ok) {
        return status;
    }

    config_.delay_ms(config_.register_select_delay_ms);

    uint8_t value = 0;
    status = fromI2CStatus(device_.read(&value, 1));
    if (status != RollerCanStatus::ok) {
        return status;
    }

    busy = value != 0;
    return RollerCanStatus::ok;
}

RollerCanStatus RollerCan::saveEncoderCalibration()
{
    return writeCommand(kRegSaveEncoderCalibration, 0x01);
}

RollerCanStatus RollerCan::writeCommand(uint8_t reg, uint8_t value)
{
    const std::array<uint8_t, 2> command = {reg, value};
    return fromI2CStatus(device_.write(command.data(), command.size()));
}

RollerCanStatus RollerCan::fromI2CStatus(bsp::I2CStatus status)
{
    switch (status) {
    case bsp::I2CStatus::ok:
        return RollerCanStatus::ok;
    case bsp::I2CStatus::invalid_argument:
    case bsp::I2CStatus::invalid_state:
        return RollerCanStatus::invalid_argument;
    case bsp::I2CStatus::not_found:
        return RollerCanStatus::device_not_found;
    case bsp::I2CStatus::timeout:
    case bsp::I2CStatus::io_error:
    default:
        return RollerCanStatus::bus_error;
    }
}

} // namespace hardware
