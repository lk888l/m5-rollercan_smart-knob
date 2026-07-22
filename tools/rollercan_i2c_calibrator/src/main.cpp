#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "linux_i2c_bus.hpp"
#include "rollercan/rollercan.hpp"
#include "rollercan_calibrator_app.hpp"

namespace {

constexpr const char* kDefaultBus = "/dev/i2c-3";
constexpr uint8_t kDefaultAddress = 0x64;
constexpr uint32_t kDefaultClockHz = 100000;
constexpr uint32_t kDefaultPollMs = 50;
constexpr uint32_t kDefaultTimeoutMs = 15000;

std::atomic_bool g_running{true};

struct Options {
    std::string bus = kDefaultBus;
    uint8_t address = kDefaultAddress;
    uint32_t poll_ms = kDefaultPollMs;
    uint32_t timeout_ms = kDefaultTimeoutMs;
    bool probe = false;
    bool confirmed = false;
    bool help = false;
};

void printUsage(const char* program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " --probe [--bus /dev/i2c-3] [--addr 0x64]\n"
        << "  " << program << " --yes [--bus /dev/i2c-3] [--addr 0x64]"
        << " [--poll-ms 50] [--timeout-ms 15000]\n\n"
        << "Options:\n"
        << "  --probe        Test communication without starting calibration\n"
        << "  --yes          Confirm that the motor is unloaded and secured\n"
        << "  --bus PATH     Linux i2c-dev node (default: /dev/i2c-3)\n"
        << "  --addr ADDR    7-bit RollerCAN I2C address (default: 0x64)\n"
        << "  --poll-ms N    Busy polling interval (default: 50)\n"
        << "  --timeout-ms N Calibration timeout (default: 15000)\n"
        << "  -h, --help     Show this help\n";
}

bool parseUnsigned(const std::string& text, unsigned long max_value, unsigned long& value)
{
    char* end = nullptr;
    const int base = text.rfind("0x", 0) == 0 || text.rfind("0X", 0) == 0 ? 16 : 10;
    value = std::strtoul(text.c_str(), &end, base);
    return end != text.c_str() && end != nullptr && *end == '\0' && value <= max_value;
}

bool parseArgs(int argc, char* argv[], Options& options)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
            return true;
        }
        if (arg == "--probe") {
            options.probe = true;
            continue;
        }
        if (arg == "--yes") {
            options.confirmed = true;
            continue;
        }
        if (arg == "--bus") {
            if (++i >= argc) {
                std::cerr << "--bus requires a value\n";
                return false;
            }
            options.bus = argv[i];
            continue;
        }
        if (arg == "--addr") {
            if (++i >= argc) {
                std::cerr << "--addr requires a value\n";
                return false;
            }
            unsigned long value = 0;
            if (!parseUnsigned(argv[i], 0x7F, value)) {
                std::cerr << "--addr must be a 7-bit address, e.g. 0x64\n";
                return false;
            }
            options.address = static_cast<uint8_t>(value);
            continue;
        }
        if (arg == "--poll-ms" || arg == "--timeout-ms") {
            const bool is_poll = arg == "--poll-ms";
            if (++i >= argc) {
                std::cerr << arg << " requires a value\n";
                return false;
            }
            unsigned long value = 0;
            const unsigned long max_value = is_poll ? 1000UL : 300000UL;
            if (!parseUnsigned(argv[i], max_value, value) || value == 0) {
                std::cerr << arg << " has an invalid value\n";
                return false;
            }
            if (is_poll) {
                options.poll_ms = static_cast<uint32_t>(value);
            } else {
                options.timeout_ms = static_cast<uint32_t>(value);
            }
            continue;
        }

        std::cerr << "unknown argument: " << arg << '\n';
        return false;
    }
    return true;
}

void delayMs(uint32_t ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void handleSignal(int)
{
    g_running.store(false);
}

} // namespace

int main(int argc, char* argv[])
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    Options options;
    if (!parseArgs(argc, argv, options)) {
        printUsage(argv[0]);
        return 1;
    }
    if (options.help) {
        printUsage(argv[0]);
        return 0;
    }
    if (!options.probe && !options.confirmed) {
        std::cerr
            << "Refusing to energize the motor without --yes.\n"
            << "Unload the shaft, secure the RollerCAN, and use a current-limited supply.\n";
        return 2;
    }

    try {
        bsp::LinuxI2CBus bus({options.bus});
        const bsp::I2CStatus bus_status = bus.init();
        if (bus_status != bsp::I2CStatus::ok) {
            std::cerr << "failed to initialize I2C bus: " << bsp::toString(bus_status) << '\n';
            return 1;
        }

        auto device_result = bus.createDevice(options.address, kDefaultClockHz);
        if (!device_result) {
            std::cerr << "failed to open " << options.bus << " for RollerCAN address 0x"
                      << std::hex << static_cast<int>(options.address) << std::dec
                      << ": " << bsp::toString(device_result.status) << '\n';
            return 1;
        }

        hardware::RollerCan roller(
            *device_result.device,
            hardware::RollerCanConfig{delayMs, 2});
        app::RollerCanCalibratorApp calibrator(
            roller,
            app::CalibratorConfig{options.poll_ms, options.timeout_ms, 500});

        std::cout << "RollerCAN: bus=" << options.bus << ", address=0x"
                  << std::hex << static_cast<int>(options.address) << std::dec << '\n';

        if (options.probe) {
            bool busy = false;
            const app::CalibrationResult result = calibrator.probe(busy);
            if (result != app::CalibrationResult::ok) {
                std::cerr << "probe failed: " << app::toString(result) << '\n';
                return 1;
            }
            std::cout << "I2C communication OK; calibration_busy=" << (busy ? 1 : 0) << '\n';
            return 0;
        }

        std::cout
            << "WARNING: calibration applies approximately 1.2 A phase current.\n"
            << "Keep hands and mechanical loads away from the shaft.\n";

        const app::CalibrationResult result = calibrator.run(g_running);
        if (result != app::CalibrationResult::ok) {
            std::cerr << "calibration failed: " << app::toString(result) << '\n';
            if (result == app::CalibrationResult::calibration_not_started) {
                std::cerr << "The command may have been rejected; check RollerCAN supply voltage, "
                             "firmware mode, I2C address, and wiring.\n";
            }
            return 1;
        }

        std::cout
            << "Calibration completed and saved.\n"
            << "Power-cycle RollerCAN, then test speed mode at low current and low speed.\n";
    } catch (const std::exception& ex) {
        std::cerr << "fatal error: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}
