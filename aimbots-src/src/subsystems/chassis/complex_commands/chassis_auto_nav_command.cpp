#include "chassis_auto_nav_command.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

ChassisAutoNavCommand::ChassisAutoNavCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    SmoothPIDConfig linearPIDConfig,
    SmoothPIDConfig rotationPIDConfig,
    const SnapSymmetryConfig& snapSymmetryConfig,
    float linearSettledThreshold,
    float angularSettledThreshold)
    : drivers(drivers),
      chassis(chassis),
      snapSymmetryConfig(snapSymmetryConfig),
      autoNavigator(),
      xController(linearPIDConfig),
      yController(linearPIDConfig),
      rotationController(rotationPIDConfig),
      linearSettledThreshold(linearSettledThreshold),
      angularSettledThreshold(angularSettledThreshold)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

float path_x = 0.0f;
float path_y = 0.0f;
float myX = 0.0f;
float myY = 0.0f;
bool settled = false;

void ChassisAutoNavCommand::initialize() {
    // modm::Location2D<float> targetLocation({0.5f, 0.5f}, modm::toRadian(90.0f));  // test
    // autoNavigator.setTargetLocation(targetLocation);

    // 15 inches assumed radius = 0.381 meters
}

void ChassisAutoNavCommand::pop_path() {
    if (path.empty()) {
        return;
    }
    Point pt = path.front();
    path_x = pt.x;
    path_y = pt.x;
    autoNavigator.setTargetLocation(modm::Location2D<float>(pt.x, pt.y, 0));
    path.erase(path.begin());
}

void ChassisAutoNavCommand::load_path(vector<Point> path) {
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);  // halt motion
    this->path = path;
    pop_path();  // put new point into auto navigator
}

void ChassisAutoNavCommand::setTargetLocation(double x, double y) {
    modm::Location2D<float> currentWorldLocation = drivers->kinematicInformant.getChassisOdometry()->getRobotLocation2D();
    load_path(pathfinder.search(currentWorldLocation.getX(), currentWorldLocation.getY(), x, y));
}

float rotationErrorDisplay = 0.0f;

void ChassisAutoNavCommand::execute() {
    float xError = 0.0f;
    float yError = 0.0f;
    float rotationError = 0.0f;
    settled = isSettled();

    // no points to load and controllers at target
    if (this->path.empty() && this->isSettled()) {
        chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
        return;
    }

    if (this->isSettled()) {  // if controllers at target load new point
        this->pop_path();
    }

    modm::Location2D<float> currentWorldLocation = drivers->kinematicInformant.getChassisOdometry()->getRobotLocation2D();
    myX = currentWorldLocation.getX();
    myY = currentWorldLocation.getY();
    modm::Vector2f currentWorldVelocity = drivers->kinematicInformant.getChassisOdometry()->getRobotVelocity2D();

    autoNavigator.update(currentWorldLocation);
    autoNavigator.getDesiredInput(&xError, &yError, &rotationError);

    rotationErrorDisplay = rotationError;

    // findNearestChassisErrorTo expects a chassis-relative target angle, not an error, so we negate the error to get target
    rotationError = Helper::findNearestChassisErrorTo(-rotationError, snapSymmetryConfig);

    float desiredX = xController.runController(xError, currentWorldVelocity.getX());
    float desiredY = yController.runController(yError, currentWorldVelocity.getY());
    float desiredR = 0;
    // float desiredR = rotationController.runController(
    //     rotationError,
    //     -RADPS_TO_RPM(drivers->kinematicInformant.getChassisIMUAngularVelocity(
    //         src::Informants::AngularAxis::YAW_AXIS,
    //         AngleUnit::Radians)));

    // Rotate world-relative desired input to chassis-relative desired input
    tap::algorithms::rotateVector(&desiredX, &desiredY, -currentWorldLocation.getOrientation());

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

#endif  //#ifdef CHASSIS_COMPATIBLE