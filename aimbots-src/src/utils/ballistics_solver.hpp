#pragma once

#include <optional>

#include <drivers.hpp>

#include "tap/algorithms/ballistics.hpp"

// #include "informants/vision/jetson_communicator.hpp"

namespace src::Gimbal {
class GimbalSubsystem;
}

namespace src::Informants::vision {
struct plateKinematicState;
}

namespace src::Utils {

class BallisticsSolver {
public:
    BallisticsSolver(src::Drivers *);
    ~BallisticsSolver() = default;

    struct AngularFilterConfig {
        float tQDerivativeKalman = 1.0f;   /**< The system noise covariance for the kalman filter that
                                            * is applied to the derivative of the error. */
        float tRDerivativeKalman = 0.0f;   /**< The measurement noise covariance for the kalman filter
                                            * that is applied to the derivative of the error. */
        float tQProportionalKalman = 1.0f; /**< The system noise covariance for the kalman filter that
                                            *  is applied to the proportional error. */
        float tRProportionalKalman = 0.0f; /**< The measurement noise covariance for the kalman filter
                                            * that is applied to the proportional error. */
    };

    struct BallisticsSolution {
        float pitchAngle;
        float yawAngle;
        float distanceToTarget;
        float timeToTarget;  // in seconds
    };

    std::optional<BallisticsSolution> solve();

private:
    src::Drivers *drivers;

    const float defaultProjectileSpeed = 30.0f;  // m/s

    uint32_t lastFoundTargetTime = 0;

    std::optional<BallisticsSolution> lastBallisticsSolution = {};
};

}  // namespace src::Utils