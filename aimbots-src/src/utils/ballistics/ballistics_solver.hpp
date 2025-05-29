#pragma once

#include <optional>

#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

namespace src::Gimbal {
class GimbalSubsystem;
}

namespace src::Informants::Vision {
struct PlateKinematicState;
}

namespace src::Utils::Ballistics {

// Heavily pulled from Taproot's Ballistics Solver
/**
 * Stores the 3D position, velocity, and acceleration of an object as `modm::Vector3f`s.
 * - Position Units: m
 * - Velocity Units: m/s
 * - Acceleration Units: m/s^2
 */
struct MeasuredKinematicState {
    modm::Vector3f position;      // m
    modm::Vector3f velocity;      // m/s
    modm::Vector3f acceleration;  // m/s^2

    // TODO: check if I need this
    /**
     * @param[in] dt: The amount of time to project forward.
     * @param[in] s: The position of the object.
     * @param[in] v: The velocity of the object.
     * @param[in] a: The acceleration of the object.
     *
     * @return The future position of an object using a quadratic (constant acceleration) model.
     */
    inline static float quadraticKinematicProjection(float dt, float s, float v, float a) {
        return s + v * dt + 0.5f * a * pow2(dt);
    }

    /**
     * @param[in] dt: The amount of time to project the state forward.
     *
     * @return The future 3D position of this object using a quadratic (constant acceleration)
     * model.
     */
    inline modm::Vector3f projectForward(float dt) {
        return modm::Vector3f(
            quadraticKinematicProjection(dt, position.x, velocity.x, acceleration.x),
            quadraticKinematicProjection(dt, position.y, velocity.y, acceleration.y),
            quadraticKinematicProjection(dt, position.z, velocity.z, acceleration.z));
    }
};

class BallisticsSolver {
public:
    BallisticsSolver(src::Drivers *, Vector3f);
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
        float horizontalDistanceToTarget;
        float timeToTarget;  // in seconds
    };

    std::optional<BallisticsSolution> solve(std::optional<float> projectileSpeed = std::nullopt);

    /**
     * @param[in] numIterations: The number of times to project the kinematics forward.
     *      Guidelines on choosing this parameter:
     *      - If the target is moving very slow relative to bulletVelocity, 1 is probably enough.
     *      - For higher target speeds, 2-3 is probably a good estimate.
     *      - If the target is approaching the projectile speed, this algorithm may have a difficult
     *        time converging (but it may be possible with enough iterations).
     *      - If the target is moving faster than the projectile, this algorithm will diverge.
     * @param[out] turretPitch: The world-relative turret pitch (in radians above level) at which our
     * robot should aim to hit the given target, taking into account the path a projectile takes to hit
     * the target.
     * @param[out] turretYaw: Analogue of turret pitch
     * @param[out] projectedTravelTime: The expected time between projectile launch and impact with the
     * target, in seconds.
     * @param[in] pitchAxisOffset: The distance between the pitch and yaw axes (in meters)
     * as seen from a plane parallel to the ground. A positive offset indicates that
     * the pitch axis is located behind the yaw axis.
     * @param[in] maxError: The maximum error allowed in the solution. If the error is greater than
     * this value, the solution is considered invalid.
     * @param[in] minDiff: The minimum difference between the current solution and the previous
     * solution. If the difference is less than this value, There could be numerical instability
     * and the solution is considered invalid.
     * @return Whether or not a valid aiming solution was found. Out parameters only valid if true.
     */
    bool findTargetProjectileIntersection(
        uint8_t numIterations,
        float *turretPitch,
        float *turretYaw,
        float *projectedTravelTime,
        const float pitchAxisOffset = 0,
        const float maxError = 0.001f,
        const float minDiff = 1e-7f);

    // TODO: check these tolerances 

    // The width tolerance of where a predicted shot needs to land on a plate be considered "on target"
    static constexpr float PLATE_WIDTH_TOLERANCE = 0.15f;  // 0.15cm
    // The height tolerance of where a predicted shot needs to land on a plate be considered "on target"
    static constexpr float PLATE_HEIGHT_TOLERANCE = 0.1f;  // 0.1cm
    // (Same tolerance for both types of armor panels)

    /**
     * @return true if the specified yaw and pitch angle errors are small enough such that if a
     * projectile were to be launched, the projectile would hit an armor plate at
     * targetDistance m away.
     */
    static inline bool withinAimingTolerance(float yawAngleError, float pitchAngleError, float targetDistance) {
        if (targetDistance < 0) {
            return false;
        }

        return (abs(yawAngleError) < atan2f(PLATE_WIDTH_TOLERANCE, 2.0f * targetDistance)) &&
               (abs(pitchAngleError) < atan2f(PLATE_HEIGHT_TOLERANCE, 2.0f * targetDistance));
    }

    /**
     * @param[in] stepSize: The step size for the finite difference approximation.
     * @param[in] x_init: The initial guess for the solution.
     * @return The approximate inverse Jacobian of the ballistic characteristic equation.
     */
    Matrix3f approximateInverseJacobian(Vector3f x_init, float stepSize = 0.001);

    /**
     * @param[in] x: The input to the ballistic characteristic equation.
     * x.x is the yaw angle, x.y is the pitch angle, x.z is the projectile traveltime to the target.
     * @return The output of the ballistic characteristic equation.
     */
    Vector3f ballisticCharacteristicEquation(Vector3f x);

    // TODO: move this to a better place
    Matrix3f inverseMatrix3f(Matrix3f matrix);

private:
    src::Drivers *drivers;

    const float defaultProjectileSpeed = 30.0f;  // m/s

    float bulletVelocity = 0; // m/s

    MeasuredKinematicState targetKinematicState;

    uint32_t lastPlatePredictionTime = 0;

    Vector3f barrelOriginFromGimbalOrigin;

    std::optional<BallisticsSolution> lastBallisticsSolution = {};
};

}  // namespace src::Utils::Ballistics