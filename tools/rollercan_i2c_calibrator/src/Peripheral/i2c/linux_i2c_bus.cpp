#include "linux_i2c_bus.hpp"

#include <cerrno>
#include <fcntl.h>
#include <limits>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <memory>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

namespace bsp {
namespace {

I2CStatus fromErrno(int error)
{
    switch (error) {
    case EINVAL:
        return I2CStatus::invalid_argument;
    case ENOENT:
    case ENODEV:
    case ENXIO:
    case EREMOTEIO:
        return I2CStatus::not_found;
    case ETIMEDOUT:
        return I2CStatus::timeout;
    default:
        return I2CStatus::io_error;
    }
}

bool isValidLength(size_t length)
{
    return length > 0 && length <= std::numeric_limits<__u16>::max();
}

class LinuxI2CDevice final : public I2CDevice {
public:
    LinuxI2CDevice(std::string device_path, uint8_t address)
        : device_path_(std::move(device_path))
        , address_(address)
    {
        fd_ = ::open(device_path_.c_str(), O_RDWR | O_CLOEXEC);
    }

    ~LinuxI2CDevice() override
    {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    LinuxI2CDevice(const LinuxI2CDevice&) = delete;
    LinuxI2CDevice& operator=(const LinuxI2CDevice&) = delete;

    bool isOpen() const { return fd_ >= 0; }

    I2CStatus write(const uint8_t* data, size_t length) override
    {
        if (fd_ < 0 || data == nullptr || !isValidLength(length)) {
            return I2CStatus::invalid_argument;
        }

        i2c_msg message = {};
        message.addr = address_;
        message.flags = 0;
        message.len = static_cast<__u16>(length);
        message.buf = const_cast<__u8*>(data);
        return transfer(&message, 1);
    }

    I2CStatus read(uint8_t* data, size_t length) override
    {
        if (fd_ < 0 || data == nullptr || !isValidLength(length)) {
            return I2CStatus::invalid_argument;
        }

        i2c_msg message = {};
        message.addr = address_;
        message.flags = I2C_M_RD;
        message.len = static_cast<__u16>(length);
        message.buf = data;
        return transfer(&message, 1);
    }

    I2CStatus writeRead(const uint8_t* write_data,
                        size_t write_length,
                        uint8_t* read_data,
                        size_t read_length) override
    {
        if (fd_ < 0 || write_data == nullptr || !isValidLength(write_length) ||
            read_data == nullptr || !isValidLength(read_length)) {
            return I2CStatus::invalid_argument;
        }

        i2c_msg messages[2] = {};
        messages[0].addr = address_;
        messages[0].flags = 0;
        messages[0].len = static_cast<__u16>(write_length);
        messages[0].buf = const_cast<__u8*>(write_data);

        messages[1].addr = address_;
        messages[1].flags = I2C_M_RD;
        messages[1].len = static_cast<__u16>(read_length);
        messages[1].buf = read_data;
        return transfer(messages, 2);
    }

private:
    I2CStatus transfer(i2c_msg* messages, __u32 count)
    {
        i2c_rdwr_ioctl_data transfer_data = {};
        transfer_data.msgs = messages;
        transfer_data.nmsgs = count;

        if (::ioctl(fd_, I2C_RDWR, &transfer_data) < 0) {
            return fromErrno(errno);
        }
        return I2CStatus::ok;
    }

    std::string device_path_;
    uint8_t address_;
    int fd_ = -1;
};

} // namespace

LinuxI2CBus::LinuxI2CBus(LinuxI2CBusConfig config)
    : config_(std::move(config))
{
}

LinuxI2CBus::~LinuxI2CBus()
{
    deinit();
}

I2CStatus LinuxI2CBus::init()
{
    if (config_.device_path.empty()) {
        return I2CStatus::invalid_argument;
    }
    initialized_ = true;
    return I2CStatus::ok;
}

I2CStatus LinuxI2CBus::deinit()
{
    initialized_ = false;
    return I2CStatus::ok;
}

I2CDeviceResult LinuxI2CBus::createDevice(uint8_t address, uint32_t /*clock_hz*/)
{
    if (!initialized_) {
        return {I2CStatus::invalid_state, nullptr};
    }
    if (address > 0x7F) {
        return {I2CStatus::invalid_argument, nullptr};
    }

    auto device = std::make_unique<LinuxI2CDevice>(config_.device_path, address);
    if (!device->isOpen()) {
        return {fromErrno(errno), nullptr};
    }
    return {I2CStatus::ok, std::move(device)};
}

} // namespace bsp
