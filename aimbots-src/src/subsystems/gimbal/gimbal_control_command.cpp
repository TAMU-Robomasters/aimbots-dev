#include "gimbal_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "drivers.hpp"
#include "vision/jetson_communicator.hpp"
#include "vision/jetson_protocol.hpp"

namespace src::Gimbal {

    GimbalControlCommand::GimbalControlCommand(src::Drivers * drivers,
                                               GimbalSubsystem * gimbalSubsystem,
                                               GimbalChassisRelativeController * gimbalController,
                                               float inputYawSensitivity,
                                               float inputPitchSensitivity)
        : tap::control::Command(),
          drivers(drivers),
          gimbal(gimbalSubsystem),
          controller(gimbalController),
          userInputYawSensitivityFactor(inputYawSensitivity),
          userInputPitchSensitivityFactor(inputPitchSensitivity),
          currMode(MANUAL),
          patrolCoordinates(Matrix<float, 5, 3>::zeroMatrix()),
          patrolCoordinateIndex(0)  //,
    //   currPatrolCoordinate(Matrix<float, 1, 3>::zeroMatrix())  //
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    }

    float currPatrolCoordinateXDisplay = 0.0f;
    float currPatrolCoordinateYDisplay = 0.0f;
    float currPatrolCoordinateTimeDisplay = 0.0f;

    void GimbalControlCommand::initialize() {
        // clang-format off
    static constexpr float xy_field_relative_patrol_location_array[15] = {
        -4.0f, 0.0f, 1500.0f, // field coordinate x, y, time spent at this angle
        -2.0f, 0.0f, 1500.0f,
        0.0f, 0.0f, 1500.0f,
        2.0f, 0.0f, 1500.0f,
        4.0f, 0.0f, 1500.0f,
    };  // clang-format on
        patrolCoordinates = Matrix<float, 5, 3>(xy_field_relative_patrol_location_array);
    }

    float yawOffsetAngleDisplay = 0.0f;
    float pitchOffsetAngleDisplay = 0.0f;

    float targetYawAngleDisplay = 0.0f;
    float targetPitchAngleDisplay = 0.0f;

    void GimbalControlCommand::execute() {
        float targetYawAngle = 0.0f;
        float targetPitchAngle = 0.0f;

        Matrix<float, 1, 2> visionOffsetAngles = Matrix<float, 1, 2>::zeroMatrix();
        src::vision::CVState cvState;

        if (drivers->cvCommunicator.isJetsonOnline()) {
            cvState = drivers->cvCommunicator.lastValidMessage().cvState;
            visionOffsetAngles = drivers->cvCommunicator.getVisionOffsetAngles();
            yawOffsetAngleDisplay = visionOffsetAngles[0][0];
            pitchOffsetAngleDisplay = visionOffsetAngles[0][1];

#ifdef TARGET_SENTRY
            if (cvState == src::vision::CV_STATE_FOUND) {
                currMode = CHASE;
            } else {
                currMode = PATROL;
            }
        } else {
            currMode = PATROL;
        }
#else
        } else {
            currMode = MANUAL;
        }
#endif

        switch (currMode) {
            case PATROL:
                targetYawAngle = getFieldRelativeYawPatrolAngle(AngleUnit::Degrees);
                targetYawAngleDisplay = getFieldRelativeYawPatrolAngle(AngleUnit::Degrees);
                targetPitchAngle = getSinusoidalPitchPatrolAngle(AngleUnit::Degrees);
                targetPitchAngleDisplay = targetPitchAngle;
                break;
            case CHASE:
                // Adds vision offset angles to the current yaw and pitch angles, set to target
                targetYawAngle = gimbal->getCurrentYawAngle(AngleUnit::Degrees) + modm::toDegree(visionOffsetAngles[0][0]);
                targetPitchAngle = gimbal->getCurrentPitchAngle(AngleUnit::Degrees) + modm::toDegree(visionOffsetAngles[0][1]);
                break;
            case MANUAL:
                targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Degrees) -
                                 userInputYawSensitivityFactor * drivers->controlOperatorInterface.getGimbalYawInput();
                targetPitchAngle = gimbal->getTargetPitchAngle(AngleUnit::Degrees) -
                                   userInputPitchSensitivityFactor * drivers->controlOperatorInterface.getGimbalPitchInput();
                break;
        }

        controller->runYawController(AngleUnit::Degrees, targetYawAngle);
        controller->runPitchController(AngleUnit::Degrees, targetPitchAngle);
    }

    bool GimbalControlCommand::isReady() { return true; }

    bool GimbalControlCommand::isFinished() const { return false; }

    void GimbalControlCommand::end(bool) {
        gimbal->setYawMotorOutput(0);
        gimbal->setPitchMotorOutput(0);
    }

    float yawPositionPIDErrorDisplay = 0.0f;

    bool patrolTimerExpiredDisplay = false;
    bool patrolTimerIsStoppedDisplay = false;

    void GimbalControlCommand::updateYawPatrolTarget() {
        yawPositionPIDErrorDisplay = this->controller->getYawPositionPID()->getError();
        patrolTimerExpiredDisplay = patrolTimer.isExpired();
        patrolTimerIsStoppedDisplay = patrolTimer.isStopped();

        // currPatrolCoordinate = patrolCoordinates.getRow(patrolCoordinateIndex);
        if (controller->getYawPositionPID()->isSettled(2.0f)) {
            if (patrolTimer.execute()) {
                // if we're settled at the target angle, and the timer expires for the first time, bounce the patrol coordinate index
                patrolCoordinateIndex = (patrolCoordinateIndex + 1) % patrolCoordinates.getNumberOfRows();
            } else if (patrolTimer.isExpired() || patrolTimer.isStopped()) {
                // if we're settled at the target angle, and the timer has already expired or hasn't ever been started, start the timer
                patrolTimer.restart(static_cast<uint32_t>(patrolCoordinates[patrolCoordinateIndex][TIME]));
            }
        }
    }

    float xy_angleDisplay = 0.0f;

    // function assumes gimbal yaw is at 0 degrees (positive x axis)
    float GimbalControlCommand::getFieldRelativeYawPatrolAngle(AngleUnit unit) {
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

        float xy_angle = xy_angle_between_locations(AngleUnit::Radians, drivers->fieldRelativeInformant.getFieldRelativeRobotPosition(), /*patrolCoordinates.getRow(patrolCoordinateIndex)*/ demoPosition2);
        xy_angleDisplay = modm::toDegree(xy_angle);
#ifdef TARGET_SENTRY
        // if robot is sentry, need to rotate by 45 degrees because sentry rail is at 45 degree angle relative to field
        xy_angle -= modm::toRadian(45.0f);
#endif
        // offset by the preset "front" angle of the robot
        float robotRelativeAngle = modm::toRadian(YAW_FRONT_ANGLE) + (YAW_INPUT_DIRECTION * xy_angle);

        if (unit == AngleUnit::Degrees) {
            robotRelativeAngle = modm::toDegree(robotRelativeAngle);
        }
        return robotRelativeAngle;
    }

}  // namespace src::Gimbal