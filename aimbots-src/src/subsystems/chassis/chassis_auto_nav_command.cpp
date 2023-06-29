#include "chassis_auto_nav_command.hpp"

namespace src::Chassis {

ChassisAutoNavCommand::ChassisAutoNavCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    SmoothPIDConfig linearPIDConfig,
    SmoothPIDConfig rotationPIDConfig,
    const SnapSymmetryConfig& snapSymmetryConfig)
    : drivers(drivers),
      chassis(chassis),
      snapSymmetryConfig(snapSymmetryConfig),
      autoNavigator(),
      xController(linearPIDConfig),
      yController(linearPIDConfig),
      rotationController(rotationPIDConfig)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisAutoNavCommand::initialize() {
    modm::Location2D<float> targetLocation({0.0f, 0.0f}, 0.0f);  // test
    autoNavigator.setTargetLocation(targetLocation);
}

void ChassisAutoNavCommand::execute() {
    float xError = 0.0f;
    float yError = 0.0f;
    float rotationError = 0.0f;

    modm::Location2D<float> currentWorldLocation = drivers->kinematicInformant.getRobotLocation2D();
    modm::Vector2f currentWorldVelocity = drivers->kinematicInformant.getRobotVelocity2D();

    autoNavigator.update(currentWorldLocation);

    autoNavigator.getDesiredInput(&xError, &yError, &rotationError);

    // For Chassis, WorldRelative error is the same as ChassisRelative error
    rotationError = Helper::findNearestChassisErrorTo(rotationError, snapSymmetryConfig);

    float desiredX = xController.runController(xError, currentWorldVelocity.getX());
    float desiredY = yController.runController(yError, currentWorldVelocity.getY());
    float desiredR = rotationController.runController(
        rotationError,
        -RADPS_TO_RPM(
            drivers->kinematicInformant.getIMUAngularVelocity(src::Informants::AngularAxis::YAW_AXIS, AngleUnit::Radians)));

    Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredR);

    chassis->setTargetRPMs(desiredX, desiredY, desiredR);
}

bool ChassisAutoNavCommand::isReady() { return true; }

bool ChassisAutoNavCommand::isFinished() const { return false; }

void ChassisAutoNavCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

}  // namespace src::Chassis