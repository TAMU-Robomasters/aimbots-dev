#include "gimbal_patrol_command.hpp"

#include "utils/robot_specific_inc.hpp"

namespace src::Gimbal {

GimbalPatrolCommand::GimbalPatrolCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalFieldRelativeController* gimbalController,
    GimbalPatrolConfig patrolConfig)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      patrolConfig(patrolConfig),
      patrolCoordinates(
          {modm::Location2D<float>({5.0f, -3.0f}, 0.0f),
           modm::Location2D<float>({-1.425, -1.131}, 0.0f),
           modm::Location2D<float>({-3.25f, 2.5f}, 0.0f)}),
      patrolCoordinateTimes({500, 500, 500})
//
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

float currPatrolCoordinateXDisplay = 0.0f;
float currPatrolCoordinateYDisplay = 0.0f;
float currPatrolCoordinateTimeDisplay = 0.0f;

void GimbalPatrolCommand::initialize() { commandStartTime = tap::arch::clock::getTimeMilliseconds(); }

float targetPatrolYawAxisAngleDisplay = 0.0f;
float targetPatrolPitchAxisAngleDisplay = 0.0f;

void GimbalPatrolCommand::execute() {
    float targetYawAxisAngle = 0.0f;
    float targetPitchAxisAngle = 0.0f;

    targetYawAxisAngle = getFieldRelativeYawPatrolAngle(AngleUnit::Degrees);
    targetPatrolYawAxisAngleDisplay = targetYawAxisAngle;
    targetPitchAxisAngle = getSinusoidalPitchPatrolAngle(AngleUnit::Degrees);
    targetPatrolPitchAxisAngleDisplay = targetPitchAxisAngle;

    controller->setTargetYaw(AngleUnit::Degrees, targetYawAxisAngle);
    controller->setTargetPitch(AngleUnit::Degrees, targetPitchAxisAngle);

    controller->runYawController();
    controller->runPitchController();  // That parameter is unused and should be unecessary, but complier is strange
}

bool GimbalPatrolCommand::isReady() { return true; }

bool GimbalPatrolCommand::isFinished() const { return false; }

void GimbalPatrolCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchOutputs(0);
}

float yawPositionPIDErrorDisplay = 0.0f;
float yawPositionPIDDerivativeDisplay = 0.0f;

void GimbalPatrolCommand::updateYawPatrolTarget() {
    yawPositionPIDErrorDisplay = this->controller->allOnlineYawControllersSettled(modm::toRadian(15.0f), 0);

    if (controller->allOnlineYawControllersSettled(modm::toRadian(15.0f), 0)) {
        if (patrolTimer.execute()) {
            patrolCoordinateIndex += patrolCoordinateIncrement;
            // if we're settled at the target angle, and the timer expires for the first time, bounce the patrol coordinate
            // index
            if (patrolCoordinateIndex == 0 || (patrolCoordinateIndex == NUM_PATROL_LOCATIONS - 1)) {
                patrolCoordinateIncrement *= -1;
            }
        } else if (patrolTimer.isExpired() || patrolTimer.isStopped()) {
            // if we're settled at the target angle, and the timer has already expired or hasn't ever been started, start the
            // timer
            patrolTimer.restart(static_cast<uint32_t>(patrolCoordinateTimes[patrolCoordinateIndex]));
        }
    }
}

float xy_angleDisplay = 0.0f;

// function assumes gimbal yaw is at 0 degrees (positive x axis)
float GimbalPatrolCommand::getFieldRelativeYawPatrolAngle(AngleUnit unit) {
    this->updateYawPatrolTarget();
    // needs to target XY positions on the field from patrolCoordinates.getRow(patrolCoordinateIndex)
    // convert that to an angle relative to the field's positive x axis
    currPatrolCoordinateXDisplay = patrolCoordinates[patrolCoordinateIndex].getX();
    currPatrolCoordinateYDisplay = patrolCoordinates[patrolCoordinateIndex].getY();
    currPatrolCoordinateTimeDisplay = patrolCoordinateTimes[patrolCoordinateIndex];

    // Matrix<float, 1, 3> demoPosition1 = Matrix<float, 1, 3>::zeroMatrix();
    // demoPosition1[0][0] = drivers->fieldRelativeInformant.getFieldRelativeRobotPosition()[0][X];
    // demoPosition1[0][1] = drivers->fieldRelativeInformant.getFieldRelativeRobotPosition()[0][Y];

    // Matrix<float, 1, 3> demoPosition2 = Matrix<float, 1, 3>::zeroMatrix();
    // demoPosition2[0][0] = -3.6675f + 1.0f;
    // demoPosition2[0][1] = -1.6675f + 1.0f;

    // This function doesn't exist anymore, presumably a transformation helper function now
    float zAngle = src::Utils::MatrixHelper::getZAngleBetweenLocations(
        drivers->kinematicInformant.getRobotLocation2D(),
        patrolCoordinates[patrolCoordinateIndex],
        AngleUnit::Radians);

    xy_angleDisplay = modm::toDegree(zAngle);

    if (unit == AngleUnit::Degrees) {
        zAngle = modm::toDegree(zAngle);
    }
    return zAngle;
}

};  // namespace src::Gimbal
