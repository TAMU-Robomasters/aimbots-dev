#pragma once

#include <optional>

#include "tap/algorithms/ballistics.hpp"

#include "src/informants/vision/jetson_communicator.hpp"

namespace src::Gimbal {
class GimbalSubsystem;
}

namespace src::Informants::vision {
struct plateKinematicState;
}

namespace src::Utils {

class BallisticsSolver {
public:
    BallisticsSolver(
        const src::Gimbal::GimbalSubsystem &gimbalSubsystem,
        const src::Informants::vision::JetsonCommunicator &jetsonCommunicator);
    ~BallisticsSolver() = default;

    struct BallisticsSolution {
        float pitchAngle;
        float yawAngle;
        float distanceToTarget;
        float timeToTarget;  // in seconds
    };

    std::optional<BallisticsSolution> solve();

private:
    const src::Gimbal::GimbalSubsystem &gimbalSubsystem;
    const src::Informants::vision::JetsonCommunicator &jetsonCommunicator;

    const float defaultProjectileSpeed = 30.0f;  // m/s

    uint32_t lastFoundTargetTime = 0;

    std::optional<BallisticsSolution> lastBallisticsSolution = {};
};

}  // namespace src::Utils