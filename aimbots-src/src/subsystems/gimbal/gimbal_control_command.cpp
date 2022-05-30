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
          patrolCoordinates(XY_FIELD_RELATIVE_PATROL_LOCATIONS),
          patrolCoordinateIndex(0),
          currPatrolCoordinate(patrolCoordinates.getRow(patrolCoordinateIndex))  //
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    }

    void GimbalControlCommand::initialize() {}

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

        // controller->runYawController(AngleUnit::Degrees, targetYawAngle);
        controller->runPitchController(AngleUnit::Degrees, targetPitchAngle);
    }

    bool GimbalControlCommand::isReady() { return true; }

    bool GimbalControlCommand::isFinished() const { return false; }

    void GimbalControlCommand::end(bool) {
        gimbal->setYawMotorOutput(0);
        gimbal->setPitchMotorOutput(0);
    }

}  // namespace src::Gimbal