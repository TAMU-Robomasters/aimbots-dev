#include "ballistics_solver.hpp"

namespace src::Informants::vision {
enum CVState;
}

namespace src::Utils::Ballistics {

BallisticsSolver::BallisticsSolver(src::Drivers *drivers) : drivers(drivers) {}

std::optional<BallisticsSolver::BallisticsSolution> BallisticsSolver::solve() {
    if (!drivers->cvCommunicator.isJetsonOnline() ||
        drivers->cvCommunicator.getLastValidMessage().cvState < src::Informants::vision::CVState::FOUND) {
        return std::nullopt;
    }

    // If we have already solved for this target, return the same solution
    if (lastFoundTargetTime == drivers->cvCommunicator.getLastFoundTargetTime()) {
        return lastBallisticsSolution;
    } else {
        lastFoundTargetTime = drivers->cvCommunicator.getLastFoundTargetTime();
    }

    auto plateKinematicState = drivers->cvCommunicator.getPlateKinematicState();

    float projectileSpeed = defaultProjectileSpeed;

    MeasuredKinematicState targetKinematicState = {
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

    if (!findTargetProjectileIntersection(
            targetKinematicState,
            projectileSpeed,
            3,
            &lastBallisticsSolution->pitchAngle,
            &lastBallisticsSolution->yawAngle,
            &lastBallisticsSolution->timeToTarget,
            0.0f)) {
        lastBallisticsSolution = std::nullopt;
    }

    return lastBallisticsSolution;
}

bool BallisticsSolver::computeTravelTime(
    const modm::Vector3f &targetPosition,
    float bulletVelocity,
    float *travelTime,
    float *turretPitch,
    const float pitchAxisOffset) {
    float horizontalDist = hypot(targetPosition.x, targetPosition.y) + pitchAxisOffset;
    float bulletVelocitySquared = pow2(bulletVelocity);
    float sqrtTerm =
        pow2(bulletVelocitySquared) -
        ACCELERATION_GRAVITY * (ACCELERATION_GRAVITY * pow2(horizontalDist) +
                                2 * (targetPosition.z + BARREL_POSITION_FROM_GIMBAL_ORIGIN.getZ()) * bulletVelocitySquared);

    if (sqrtTerm < 0) {
        return false;
    }

    // Equation obtained from the wikipedia page on projectile motion
    *turretPitch = -atan2(bulletVelocitySquared - sqrt(sqrtTerm), (ACCELERATION_GRAVITY * horizontalDist));

    // For vertical aiming, y_f = v_0*t - 0.5*g*t^2 -> t = (v_0 - sqrt((v_0)^2 - 2*g*y_f))/g
    // We use the negative root since the collision will happen on the first instance that the
    // trajectory reaches y_f
    if (compareFloatClose(*turretPitch, 0, 1E-2)) {
        float sqrtTerm =
            pow2(bulletVelocity) - 2 * ACCELERATION_GRAVITY * (targetPosition.z + BARREL_POSITION_FROM_GIMBAL_ORIGIN.getZ());

        // If there isn't a real-valued root, there is no time where we can reach the target with
        // the given assumptions
        if (sqrtTerm < 0) {
            return false;
        }

        *travelTime = (bulletVelocity - sqrt(sqrtTerm)) / ACCELERATION_GRAVITY;
        return true;
    }

    // Equation obtained from the wikipedia page on projectile motion
    *travelTime = horizontalDist / (bulletVelocity * cos(*turretPitch));

    return !isnan(*turretPitch) && !isnan(*travelTime);
}

bool BallisticsSolver::findTargetProjectileIntersection(
    MeasuredKinematicState targetInitialState,
    float bulletVelocity,
    uint8_t numIterations,
    float *turretPitch,
    float *turretYaw,
    float *projectedTravelTime,
    const float pitchAxisOffset) {
    modm::Vector3f projectedTargetPosition = targetInitialState.position;

    if (projectedTargetPosition.x == 0 && projectedTargetPosition.y == 0 && projectedTargetPosition.z == 0) {
        return false;
    }

    for (int i = 0; i < numIterations; i++) {
        if (!computeTravelTime(projectedTargetPosition, bulletVelocity, projectedTravelTime, turretPitch, pitchAxisOffset)) {
            return false;
        }
        projectedTargetPosition = targetInitialState.projectForward(*projectedTravelTime);
    }

    float squaredTargetX = projectedTargetPosition.x * projectedTargetPosition.x;
    float squaredTargetY = projectedTargetPosition.y * projectedTargetPosition.y;

    float squaredBarrelPositionX = BARREL_POSITION_FROM_GIMBAL_ORIGIN.getX() * BARREL_POSITION_FROM_GIMBAL_ORIGIN.getX();

    // *turretYaw = atan2f(projectedTargetPosition.y, projectedTargetPosition.x);

    if (projectedTargetPosition.x - BARREL_POSITION_FROM_GIMBAL_ORIGIN.getX() >= 0) {
        *turretYaw = acos(
            (projectedTargetPosition.y * sqrt(squaredTargetX + squaredTargetY - squaredBarrelPositionX) +
             BARREL_POSITION_FROM_GIMBAL_ORIGIN.getX() * projectedTargetPosition.x) /
            (squaredTargetX + squaredTargetY));
    } else {
        *turretYaw = -1 * acos(
                              (projectedTargetPosition.y * sqrt(squaredTargetX + squaredTargetY - squaredBarrelPositionX) +
                               BARREL_POSITION_FROM_GIMBAL_ORIGIN.getX() * projectedTargetPosition.x) /
                              (squaredTargetX + squaredTargetY));
    }

    return !isnan(*turretPitch) && !isnan(*turretYaw);
}

}  // namespace src::Utils::Ballistics