#pragma once

#include <cstdint>
#include <string>

#include "bsp_i2c.hpp"

namespace bsp {

struct LinuxI2CBusConfig {
    std::string device_path = "/dev/i2c-3";
};

class LinuxI2CBus final : public I2CBus {
public:
    explicit LinuxI2CBus(LinuxI2CBusConfig config);
    ~LinuxI2CBus() override;

    LinuxI2CBus(const LinuxI2CBus&) = delete;
    LinuxI2CBus& operator=(const LinuxI2CBus&) = delete;

    I2CStatus init() override;
    I2CStatus deinit() override;
    I2CDeviceResult createDevice(uint8_t address, uint32_t clock_hz) override;
    bool isInitialized() const override { return initialized_; }

private:
    LinuxI2CBusConfig config_;
    bool initialized_ = false;
};

} // namespace bsp
