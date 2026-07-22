#include "rollercan_calibrator_app.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace app {

const char* toString(CalibrationResult result)
{
    switch (result) {
    case CalibrationResult::ok:
        return "ok";
    case CalibrationResult::invalid_argument:
        return "invalid_argument";
    case CalibrationResult::communication_error:
        return "communication_error";
    case CalibrationResult::calibration_not_started:
        return "calibration_not_started";
    case CalibrationResult::timeout:
        return "timeout";
    case CalibrationResult::interrupted:
        return "interrupted";
    default:
        return "unknown";
    }
}

RollerCanCalibratorApp::RollerCanCalibratorApp(hardware::RollerCan& roller,
                                               CalibratorConfig config)
    : roller_(roller)
    , config_(config)
{
}

CalibrationResult RollerCanCalibratorApp::probe(bool& busy)
{
    return roller_.readEncoderCalibrationBusy(busy) == hardware::RollerCanStatus::ok
               ? CalibrationResult::ok
               : CalibrationResult::communication_error;
}

CalibrationResult RollerCanCalibratorApp::run(const std::atomic_bool& running)
{
    if (config_.poll_interval_ms == 0 || config_.timeout_ms == 0) {
        return CalibrationResult::invalid_argument;
    }

    std::cout << "[1/4] Disabling motor output..." << std::endl;
    if (roller_.disableOutput() != hardware::RollerCanStatus::ok) {
        return CalibrationResult::communication_error;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "[2/4] Starting encoder electrical-angle calibration..." << std::endl;
    if (roller_.startEncoderCalibration() != hardware::RollerCanStatus::ok) {
        bestEffortDisableOutput();
        return CalibrationResult::communication_error;
    }

    const auto started_at = std::chrono::steady_clock::now();
    bool observed_busy = false;

    while (running.load()) {
        bool busy = false;
        if (roller_.readEncoderCalibrationBusy(busy) != hardware::RollerCanStatus::ok) {
            bestEffortDisableOutput();
            return CalibrationResult::communication_error;
        }

        if (busy) {
            if (!observed_busy) {
                std::cout << "[3/4] Calibration is running; keep the rotor unloaded..."
                          << std::endl;
            }
            observed_busy = true;
        } else if (!observed_busy) {
            bestEffortDisableOutput();
            return CalibrationResult::calibration_not_started;
        } else {
            break;
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started_at);
        if (elapsed.count() >= config_.timeout_ms) {
            bestEffortDisableOutput();
            return CalibrationResult::timeout;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(config_.poll_interval_ms));
    }

    if (!running.load()) {
        bestEffortDisableOutput();
        return CalibrationResult::interrupted;
    }

    std::cout << "[4/4] Saving the calibrated offset to RollerCAN flash..." << std::endl;
    if (roller_.saveEncoderCalibration() != hardware::RollerCanStatus::ok) {
        bestEffortDisableOutput();
        return CalibrationResult::communication_error;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(config_.save_settle_ms));
    if (roller_.disableOutput() != hardware::RollerCanStatus::ok) {
        return CalibrationResult::communication_error;
    }

    return CalibrationResult::ok;
}

void RollerCanCalibratorApp::bestEffortDisableOutput()
{
    const hardware::RollerCanStatus status = roller_.disableOutput();
    if (status != hardware::RollerCanStatus::ok) {
        std::cerr << "warning: failed to disable motor output: "
                  << hardware::toString(status) << '\n';
    }
}

} // namespace app
