#pragma once

#include <atomic>
#include <cstdint>

#include "rollercan/rollercan.hpp"

namespace app {

enum class CalibrationResult : uint8_t {
    ok = 0,
    invalid_argument,
    communication_error,
    calibration_not_started,
    timeout,
    interrupted,
};

const char* toString(CalibrationResult result);

struct CalibratorConfig {
    uint32_t poll_interval_ms = 50;
    uint32_t timeout_ms = 15000;
    uint32_t save_settle_ms = 500;
};

class RollerCanCalibratorApp {
public:
    RollerCanCalibratorApp(hardware::RollerCan& roller, CalibratorConfig config);

    CalibrationResult probe(bool& busy);
    CalibrationResult run(const std::atomic_bool& running);

private:
    void bestEffortDisableOutput();

    hardware::RollerCan& roller_;
    CalibratorConfig config_;
};

} // namespace app
