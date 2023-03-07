#include "utils/robot_specific_inc.hpp"
#ifdef TARGET_SENTRY

#include "gimbal_patrol_command.hpp"

namespace src::Gimbal {

GimbalPatrolCommand::GimbalPatrolCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalChassisRelativeController* gimbalController)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      patrolCoordinates(Matrix<float, 3, 3>::zeroMatrix()),
      patrolCoordinateIndex(0),
      patrolCoordinateIncrement(1) /* incrememnt set to -1 because index starts at 0 */
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

float currPatrolCoordinateXDisplay = 0.0f;
float currPatrolCoordinateYDisplay = 0.0f;
float currPatrolCoordinateTimeDisplay = 0.0f;

void GimbalPatrolCommand::initialize() {
#ifdef TARGET_SENTRY
    // clang-format off
    static constexpr float xy_field_relative_patrol_location_array[9] = {
        5.0f, -3.0f, 500.0f, // field coordinate x, y, time spent at this angle
        -1.425f, -1.131f, 250.0f,
        // 1.425f, 0.494f, 100.0f,
        // -1.425f, 2.119f, 250.0f,
        -3.25f, 2.5f, 1000.0f,
    };  // clang-format on
    patrolCoordinates = Matrix<float, 3, 3>(xy_field_relative_patrol_location_array);
#endif
}

float targetYawAngleDisplay = 0.0f;
float targetPitchAngleDisplay = 0.0f;

void GimbalPatrolCommand::execute() {
    float targetYawAngle = 0.0f;
    float targetPitchAngle = 0.0f;

    targetYawAngle = getFieldRelativeYawPatrolAngle(AngleUnit::Degrees);
    targetYawAngleDisplay = targetYawAngle;
    targetPitchAngle = getSinusoidalPitchPatrolAngle(AngleUnit::Degrees);
    targetPitchAngleDisplay = targetPitchAngle;

    controller->runYawController(AngleUnit::Degrees, targetYawAngle);
    controller->runPitchController(AngleUnit::Degrees, targetPitchAngle);
}

bool GimbalPatrolCommand::isReady() { return true; }

bool GimbalPatrolCommand::isFinished() const { return false; }

void GimbalPatrolCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}

float yawPositionPIDErrorDisplay = 0.0f;
float yawPositionPIDDerivativeDisplay = 0.0f;
float lastPatrolChangeTime = 0.0f;
float msBetweenLastPatrolChange = 0.0f;

void GimbalPatrolCommand::updateYawPatrolTarget() {
    yawPositionPIDErrorDisplay = this->controller->getYawPositionPID()->getError();
    yawPositionPIDDerivativeDisplay = this->controller->getYawPositionPID()->getDerivative();

    if (controller->getYawPositionPID()->isSettled(15.0f /*, 0.04f, 250.0f*/)) {
        if (patrolTimer.execute()) {
            patrolCoordinateIndex += patrolCoordinateIncrement;
            // if we're settled at the target angle, and the timer expires for the first time, bounce the patrol coordinate
            // index
            if (patrolCoordinateIndex == 0 || (patrolCoordinateIndex == patrolCoordinates.getNumberOfRows() - 1)) {
                patrolCoordinateIncrement *= -1;
            }
            msBetweenLastPatrolChange = tap::arch::clock::getTimeMilliseconds() - lastPatrolChangeTime;
            lastPatrolChangeTime = tap::arch::clock::getTimeMilliseconds();
        } else if (patrolTimer.isExpired() || patrolTimer.isStopped()) {
            // if we're settled at the target angle, and the timer has already expired or hasn't ever been started, start the
            // timer
            patrolTimer.restart(static_cast<uint32_t>(patrolCoordinates[patrolCoordinateIndex][TIME]));
        }
    }
}

float xy_angleDisplay = 0.0f;

// function assumes gimbal yaw is at 0 degrees (positive x axis)
float GimbalPatrolCommand::getFieldRelativeYawPatrolAngle(AngleUnit unit) {
    this->updateYawPatrolTarget();
    // needs to target XY positions on the field from patrolCoordinates.getRow(patrolCoordinateIndex)
    // convert that to an angle relative to the field's positive x axis
    currPatrolCoordinateXDisplay = patrolCoordinates[patrolCoordinateIndex][X];
    currPatrolCoordinateYDisplay = patrolCoordinates[patrolCoordinateIndex][Y];
    currPatrolCoordinateTimeDisplay = patrolCoordinates[patrolCoordinateIndex][TIME];

    Matrix<float, 1, 3> demoPosition1 = Matrix<float, 1, 3>::zeroMatrix();
    demoPosition1[0][0] = drivers->fieldRelativeInformant.getFieldRelativeRobotPosition()[0][X];
    demoPosition1[0][1] = drivers->fieldRelativeInformant.getFieldRelativeRobotPosition()[0][Y];

    Matrix<float, 1, 3> demoPosition2 = Matrix<float, 1, 3>::zeroMatrix();
    demoPosition2[0][0] = -3.6675f + 1.0f;
    demoPosition2[0][1] = -1.6675f + 1.0f;

    float xy_angle = src::Utils::MatrixHelper::xy_angle_between_locations(
        AngleUnit::Radians,
        drivers->fieldRelativeInformant.getFieldRelativeRobotPosition(),
        patrolCoordinates.getRow(patrolCoordinateIndex) /*demoPosition2*/);
    xy_angleDisplay = modm::toDegree(xy_angle);
#ifdef TARGET_SENTRY
    // if robot is sentry, need to rotate by 45 degrees because sentry rail is at 45 degree angle relative to field
    xy_angle -= modm::toRadian(45.0f);
#endif
    // offset by the preset "front" angle of the robot
    float robotRelativeAngle = modm::toRadian(YAW_OFFSET_ANGLE) + xy_angle;

    if (unit == AngleUnit::Degrees) {
        robotRelativeAngle = modm::toDegree(robotRelativeAngle);
    }
    return robotRelativeAngle;
}

};  // namespace src::Gimbal

#endif
