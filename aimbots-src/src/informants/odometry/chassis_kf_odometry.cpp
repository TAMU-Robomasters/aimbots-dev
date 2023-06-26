#include "chassis_kf_odometry.hpp"

namespace src::Informants::Odometry {

ChassisKFOdometry::ChassisKFOdometry(
    const tap::control::chassis::ChassisSubsystemInterface& chassis,
    float initialXPos,
    float initialYPos)
    : initialXPos(initialXPos),
      initialYPos(initialYPos),
      chassis(chassis),
      kf(KF_A, KF_C, KF_Q, KF_R, KF_P0),
      chassisAccelerationToMeasurementCovarianceInterpolator(
          CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT,
          MODM_ARRAY_SIZE(CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT))  //
{
    float initialX[static_cast<int>(OdomState::NUM_STATES)] = {initialXPos, 0.0f, 0.0f, initialYPos, 0.0f, 0.0f};
    kf.init(initialX);
}

void ChassisKFOdometry::update(float chassisYaw, float xChassisAccel, float yChassisAccel) {
    this->chassisYaw = chassisYaw;

    auto chassisVelocity = chassis.getActualVelocityChassisRelative();  // get chassis-relative velocity

    tap::control::chassis::ChassisSubsystemInterface::getVelocityWorldRelative(
        chassisVelocity,
        chassisYaw);  // convert to world-relative velocity using chassis orientation

    updateMeasurementCovariance(chassisVelocity);

    // rotate chassis-relative linear accelerations to world-relative
    tap::algorithms::rotateVector(&xChassisAccel, &yChassisAccel, chassisYaw);

    // Assume 0 velocity/acceleration in z direction
    float y[static_cast<int>(OdomInput::NUM_INPUTS)] = {};

    y[static_cast<int>(OdomInput::VEL_X)] = chassisVelocity[0][0];
    y[static_cast<int>(OdomInput::VEL_Y)] = chassisVelocity[1][0];
    y[static_cast<int>(OdomInput::ACC_X)] = xChassisAccel;
    y[static_cast<int>(OdomInput::ACC_Y)] = yChassisAccel;

    kf.performUpdate(y);

    updateChassisStateFromKF(chassisYaw);
}

void ChassisKFOdometry::updateChassisStateFromKF(float chassisYaw) {
    const auto& x = kf.getStateVectorAsMatrix();

    velocity.x = x[static_cast<int>(OdomState::VEL_X)];
    velocity.y = x[static_cast<int>(OdomState::VEL_Y)];

    location.setOrientation(chassisYaw);
    location.setPosition(x[static_cast<int>(OdomState::POS_X)], x[static_cast<int>(OdomState::POS_Y)]);
}

void ChassisKFOdometry::updateMeasurementCovariance(const modm::Matrix<float, 3, 1>& chassisVelocity) {
    const uint32_t currTime = tap::arch::clock::getTimeMicroseconds();
    const uint32_t dt = currTime - lastComputedOdometryTime;
    lastComputedOdometryTime = currTime;

    // return if first time computing odometry
    if (lastComputedOdometryTime == 0) {
        return;
    }

    // compute the acceleration in the x and y directions
    chassisMeasuredDeltaVelocity.x = tap::algorithms::lowPassFilter(
        chassisMeasuredDeltaVelocity.x,
        chassisVelocity[0][0] - prevChassisVelocity[0][0],
        CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

    chassisMeasuredDeltaVelocity.y = tap::algorithms::lowPassFilter(
        chassisMeasuredDeltaVelocity.y,
        chassisVelocity[1][0] - prevChassisVelocity[1][0],
        CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA);

    prevChassisVelocity = chassisVelocity;

    // dt is in microseconds, acceleration is dv / dt, so to get an acceleration with units m/s^2,
    // convert dt in microseconds to seconds
    const float accelMagnitude = chassisMeasuredDeltaVelocity.getLength() * 1E6 / static_cast<float>(dt);

    const float velocityCovariance = chassisAccelerationToMeasurementCovarianceInterpolator.interpolate(accelMagnitude);

    kf.getMeasurementCovariance()[0] = velocityCovariance;
    kf.getMeasurementCovariance()[2 * static_cast<int>(OdomInput::NUM_INPUTS) + 2] = velocityCovariance;
}

void ChassisKFOdometry::reset() {
    float initialX[int(OdomState::NUM_STATES)] = {initialXPos, 0.0f, 0.0f, initialYPos, 0.0f, 0.0f};
    kf.init(initialX);
}

}  // namespace src::Informants::Odometry