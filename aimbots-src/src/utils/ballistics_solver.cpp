#include "ballistics_solver.hpp"

namespace src::Informants::vision {
enum CVState;
}

namespace src::Utils {

BallisticsSolver::BallisticsSolver(
    const src::Gimbal::GimbalSubsystem &gimbalSubsystem,
    const src::Informants::vision::JetsonCommunicator &jetsonCommunicator)
    : gimbalSubsystem(gimbalSubsystem),
      jetsonCommunicator(jetsonCommunicator),
      pitchFilter(config.tQProportionalKalman, config.tRProportionalKalman),
      yawFilter(config.tQProportionalKalman, config.tRProportionalKalman) {}

std::optional<BallisticsSolver::BallisticsSolution> BallisticsSolver::solve() {
    if (!jetsonCommunicator.isJetsonOnline() ||
        jetsonCommunicator.getLastValidMessage().cvState < src::Informants::vision::CVState::FOUND) {
        return std::nullopt;
    }

    // If we have already solved for this target, return the same solution
    if (lastFoundTargetTime == jetsonCommunicator.getLastFoundTargetTime()) {
        return lastBallisticsSolution;
    } else {
        lastFoundTargetTime = jetsonCommunicator.getLastFoundTargetTime();
    }

    auto plateKinematicState = jetsonCommunicator.getPlateKinematicState();

    float projectileSpeed = defaultProjectileSpeed;

    ballistics::MeasuredKinematicState targetKinematicState = {
        .position = plateKinematicState.position,
        .velocity = plateKinematicState.velocity,
        .acceleration = plateKinematicState.acceleration,
    };

    // Current time - Time the target was seen
    int64_t forwardProjectionTime = static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds()) -
                                    static_cast<int64_t>(plateKinematicState.timestamp_uS);
    targetKinematicState.position = targetKinematicState.projectForward(forwardProjectionTime / MICROSECONDS_PER_SECOND);

    lastBallisticsSolution = BallisticsSolution();
    lastBallisticsSolution->distanceToTarget = targetKinematicState.position.getLength();

    if (!ballistics::findTargetProjectileIntersection(
            targetKinematicState,
            projectileSpeed,
            3,
            &lastBallisticsSolution->pitchAngle,
            &lastBallisticsSolution->yawAngle,
            &lastBallisticsSolution->timeToTarget,
            0.0f)) {
        lastBallisticsSolution = std::nullopt;
    }

    //lastBallisticsSolution->pitchAngle = pitchFilter.filterData(lastBallisticsSolution->pitchAngle);
    //lastBallisticsSolution->yawAngle = yawFilter.filterData(lastBallisticsSolution->yawAngle);

    return lastBallisticsSolution;
}

}  // namespace src::Utils