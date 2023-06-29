#include "auto_navigator_holonomic.hpp"

#include "src/subsystems/chassis/chassis_helper.hpp"

namespace src::Chassis::AutoNav {

AutoNavigatorHolonomic::AutoNavigatorHolonomic(
    src::Drivers* drivers,
    SmoothPIDConfig linearPIDConfig,
    SmoothPIDConfig rotationPIDConfig,
    const src::Informants::Odometry::ChassisKFOdometry& odometry,
    const SnapSymmetryConfig& snapSymmetryConfig,
    float linearTolerance,
    float angularTolerance)
    : drivers(drivers),
      xController(linearPIDConfig),
      yController(linearPIDConfig),
      rotationController(rotationPIDConfig),
      snapSymmetryConfig(snapSymmetryConfig),
      odometry(odometry),
      linearTolerance(linearTolerance),
      angularTolerance(angularTolerance)  //
{}

void AutoNavigatorHolonomic::update() {
    modm::Location2D<float> currentWorldLocation = odometry.getCurrentLocation2D();
    modm::Vector2f currentWorldVelocity = odometry.getCurrentVelocity2D();

    float worldXError = targetLocation.getX() - currentWorldLocation.getX();
    float worldYError = targetLocation.getY() - currentWorldLocation.getY();
    float worldRotationError = targetLocation.getOrientation() - currentWorldLocation.getOrientation();

    // For Chassis, WorldRelative error is the same as ChassisRelative error
    worldRotationError = Helper::findNearestChassisErrorTo(worldRotationError, snapSymmetryConfig);

    xController.runController(worldXError, currentWorldVelocity.getX());
    yController.runController(worldYError, currentWorldVelocity.getY());
    rotationController.runController(
        worldRotationError,
        -RADPS_TO_RPM(
            drivers->kinematicInformant.getIMUAngularVelocity(src::Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians)));
}

bool AutoNavigatorHolonomic::isSettled() {
    return xController.isSettled(linearTolerance) && yController.isSettled(linearTolerance) &&
           rotationController.isSettled(angularTolerance);
}

};  // namespace src::Chassis::AutoNav