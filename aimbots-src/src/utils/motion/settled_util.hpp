#pragma once

#include <optional>

#include "tap/architecture/timeout.hpp"

namespace src::Utils::motion {

class SettledUtil {
public:
    SettledUtil() = default;
    ~SettledUtil() = default;

    bool isSettled(
        float error,
        float errTolerance,
        std::optional<uint32_t> errorTimeout = std::nullopt,
        std::optional<float> derivative = std::nullopt,
        std::optional<float> derivTolerance = std::nullopt,
        std::optional<uint32_t> derivTimeout = std::nullopt) {
        // Sets both flags to false
        bool errorFlag = false;
        bool derivativeFlag = !derivative.has_value();
        if (fabs(error) < errTolerance) {
            if (errorTimer.isExpired()) {
                errorFlag = true;
            } else if (errorTimer.isStopped()) {  // if timer has been stopped, restart it
                errorTimer.restart(errorTimeout.value_or(0));
            }
        } else {
            errorTimer.stop();  // if error is not within tolerance, stop timer
        }
        // if derivative is not provided, derivativeFlag is true
        if (fabs(derivative.value_or(0.0f)) < derivTolerance.value_or(0.0f)) {
            if (derivativeTimer.isExpired()) {
                derivativeFlag = true;
            } else if (derivativeTimer.isStopped()) {
                derivativeTimer.restart(derivTimeout.value_or(0));
            }
        } else {
            derivativeTimer.stop();
        }
        return errorFlag && derivativeFlag;
    }

private:
    tap::arch::MilliTimeout errorTimer;
    tap::arch::MilliTimeout derivativeTimer;
};

}  // namespace src::Utils::motion