#include "ballistics_solver.hpp"

namespace src::Informants::Vision {
enum CVState;
}

namespace src::Utils::Ballistics {

BallisticsSolver::BallisticsSolver(src::Drivers *drivers, src::Utils::RefereeHelper *refHelper)
    : drivers(drivers),
      refHelper(refHelper)  //
{}

std::optional<BallisticsSolver::BallisticsSolution> BallisticsSolver::solve() {
    if (!drivers->cvCommunicator.isJetsonOnline() ||
        drivers->cvCommunicator.getLastValidMessage().cvState < src::Informants::Vision::CVState::FOUND) {
        return std::nullopt;
    }

    // If we have already solved for this target, return the same solution
    if (lastPlatePredictionTime == drivers->cvCommunicator.getLastFoundTargetTime()) {
        return lastBallisticsSolution;
    } else {
        lastPlatePredictionTime = drivers->cvCommunicator.getLastFoundTargetTime();
    }

    // How far from now you want to predict? (in us)
    uint32_t forwardProjectionTime = 0 * 1000;

    auto plateKinematicState = drivers->cvCommunicator.getPlatePrediction(forwardProjectionTime);

    // float projectileSpeed = refHelper->getPredictedProjectileSpeed();

    uint16_t projectileSpeed = 15.0f;

    // for (int i = 0; i < SHOOTER_SPEED_MATRIX.getNumberOfRows(); i++) {
    //     if (SHOOTER_SPEED_MATRIX[i][0] == refHelper->getProjectileSpeedLimit()) {
    //         projectileSpeed = SHOOTER_SPEED_MATRIX[i][1];
    //         break;
    //     }
    // }

    MeasuredKinematicState targetKinematicState = {
        .position = plateKinematicState.position,
        .velocity = plateKinematicState.velocity,
        .acceleration = plateKinematicState.acceleration,
    };

    lastBallisticsSolution = BallisticsSolution();
    lastBallisticsSolution->distanceToTarget = targetKinematicState.position.getLength();

    if (!findTargetProjectileIntersection(
            targetKinematicState,
            static_cast<float>(projectileSpeed),
            3,
            &lastBallisticsSolution->pitchAngle,
            &lastBallisticsSolution->yawAngle,
            &lastBallisticsSolution->timeToTarget,
            0.0f)) {
        lastBallisticsSolution = std::nullopt;
    }

    return lastBallisticsSolution;
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

    float squaredTargetX = pow2(projectedTargetPosition.x);
    float squaredTargetY = pow2(projectedTargetPosition.y);
    float squaredBarrelPositionX = pow2(BARREL_POSITION_FROM_GIMBAL_ORIGIN.getX());

    // *turretYaw = atan2f(projectedTargetPosition.y, projectedTargetPosition.x);
    *turretYaw = acos(
        (projectedTargetPosition.y * sqrt(squaredTargetX + squaredTargetY - squaredBarrelPositionX) +
         BARREL_POSITION_FROM_GIMBAL_ORIGIN.getX() * projectedTargetPosition.x) /
        (squaredTargetX + squaredTargetY));

    if (projectedTargetPosition.x - BARREL_POSITION_FROM_GIMBAL_ORIGIN.getX() > 0) {
        *turretYaw *= -1;
    }

    return !isnan(*turretPitch) && !isnan(*turretYaw);
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
    *turretPitch = atan2(bulletVelocitySquared - sqrt(sqrtTerm), (ACCELERATION_GRAVITY * horizontalDist));

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

}  // namespace src::Utils::Ballistics