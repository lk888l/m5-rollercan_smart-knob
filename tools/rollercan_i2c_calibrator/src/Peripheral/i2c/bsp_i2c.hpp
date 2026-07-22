#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace bsp {

enum class I2CStatus : uint8_t {
    ok = 0,
    invalid_argument,
    invalid_state,
    not_found,
    timeout,
    io_error,
};

const char* toString(I2CStatus status);

class I2CDevice {
public:
    virtual ~I2CDevice() = default;

    virtual I2CStatus write(const uint8_t* data, size_t length) = 0;
    virtual I2CStatus read(uint8_t* data, size_t length) = 0;
    virtual I2CStatus writeRead(const uint8_t* write_data,
                                size_t write_length,
                                uint8_t* read_data,
                                size_t read_length) = 0;
};

struct I2CDeviceResult {
    I2CStatus status = I2CStatus::invalid_state;
    std::unique_ptr<I2CDevice> device;

    explicit operator bool() const { return status == I2CStatus::ok && device != nullptr; }
};

class I2CBus {
public:
    virtual ~I2CBus() = default;

    virtual I2CStatus init() = 0;
    virtual I2CStatus deinit() = 0;
    virtual I2CDeviceResult createDevice(uint8_t address, uint32_t clock_hz) = 0;
    virtual bool isInitialized() const = 0;
};

} // namespace bsp
