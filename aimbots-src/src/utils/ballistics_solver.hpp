#pragma once

#include <optional>

#include <drivers.hpp>

namespace src::Gimbal {
class GimbalSubsystem;
}

namespace src::Informants::vision {
struct plateKinematicState;
}

namespace src::Utils {

    /**
 * Stores the 3D position, velocity, and acceleration of an object as `modm::Vector3f`s.
 * - Position Units: m
 * - Velocity Units: m/s
 * - Acceleration Units: m/s^2
 */
struct MeasuredKinematicState
{
    modm::Vector3f position;      // m
    modm::Vector3f velocity;      // m/s
    modm::Vector3f acceleration;  // m/s^2

    /**
     * @param[in] dt: The amount of time to project forward.
     * @param[in] s: The position of the object.
     * @param[in] v: The velocity of the object.
     * @param[in] a: The acceleration of the object.
     *
     * @return The future position of an object using a quadratic (constant acceleration) model.
     */
    inline static float quadraticKinematicProjection(float dt, float s, float v, float a)
    {
        return s + v * dt + 0.5f * a * powf(dt, 2.0f);
    }

    /**
     * @param[in] dt: The amount of time to project the state forward.
     *
     * @return The future 3D position of this object using a quadratic (constant acceleration)
     * model.
     */
    inline modm::Vector3f projectForward(float dt)
    {
        return modm::Vector3f(
            quadraticKinematicProjection(dt, position.x, velocity.x, acceleration.x),
            quadraticKinematicProjection(dt, position.y, velocity.y, acceleration.y),
            quadraticKinematicProjection(dt, position.z, velocity.z, acceleration.z));
    }
};

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

/**
 * Computes an iterative numerical approximation of the pitch angle to aim the turret in order to
 * hit a given target and the time it will take for that target to be hit, given the velocity of a
 * bullet out of the turret and the position of the target relative to the turret.
 *
 * @param[in] targetPosition: The 3D position of a target in m. Frame requirements: RELATIVE TO
 * PROJECTILE RELEASE POSITION, Z IS OPPOSITE TO GRAVITY.
 * @param[in] bulletVelocity: The velocity of the projectile to be fired in m/s.
 * @param[out] travelTime: The expected travel time of a turret shot to hit a target from this
 * object's position.
 * @param[out] turretPitch: The pitch angle of the turret to hit the target at the given travel
 * time.
 * @param[in] pitchAxisOffset: The distance between the pitch and yaw axes (in meters)
 * as seen from a plane parallel to the ground. A positive offset indicates that
 * the pitch axis is located behind the yaw axis.
 * @return Whether or not a valid travel time was found.
 */
bool computeTravelTime(
    const modm::Vector3f &targetPosition,
    float bulletVelocity,
    float *travelTime,
    float *turretPitch,
    const float pitchAxisOffset = 0);

/**
 * @param[in] targetInitialState: The initial 3D kinematic state of a target. Frame requirements:
 * RELATIVE TO PROJECTILE RELEASE POSITION, Z IS OPPOSITE TO GRAVITY.
 * @param[in] bulletVelocity: The velocity of the projectile to be fired in m/s.
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
 * @return Whether or not a valid aiming solution was found. Out parameters only valid if true.
 */
bool findTargetProjectileIntersection(
    MeasuredKinematicState targetInitialState,
    float bulletVelocity,
    uint8_t numIterations,
    float *turretPitch,
    float *turretYaw,
    float *projectedTravelTime,
    const float pitchAxisOffset = 0);

private:
    src::Drivers *drivers;

    const float defaultProjectileSpeed = 30.0f;  // m/s

    uint32_t lastFoundTargetTime = 0;

    std::optional<BallisticsSolution> lastBallisticsSolution = {};
};

}  // namespace src::Utils