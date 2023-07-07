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
        std::optional<uint32_t> errorTimeout = std::nullopt) {
        // Sets both flags to false
        bool errorFlag = false;
        if (fabs(error) < errTolerance) {
            if (errorTimer.isExpired()) {
                errorFlag = true;
            } else if (errorTimer.isStopped()) {  // if timer has been stopped, restart it
                errorTimer.restart(errorTimeout.value_or(0));
            }
        } else {
            errorTimer.stop();  // if error is not within tolerance, stop timer
        }

        return errorFlag;
    }

private:
    tap::arch::MilliTimeout errorTimer;
    tap::arch::MilliTimeout derivativeTimer;
};

}  // namespace src::Utils::motion