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
          yawPatrolLocations(YAW_PATROL_LOCATIONS) {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
    }

    void GimbalControlCommand::initialize() {}

    float yawOffsetAngleDisplay = 0.0f;
    float pitchOffsetAngleDisplay = 0.0f;

    void GimbalControlCommand::execute() {
        float targetYawAngle = 0.0f;
        float targetPitchAngle = 0.0f;

        Matrix<float, 2, 1> visionOffsetAngles = Matrix<float, 2, 1>::zeroMatrix();
        src::vision::CVState cvState;

        if (drivers->cvCommunicator.isJetsonOnline()) {
            cvState = drivers->cvCommunicator.lastValidMessage().cvState;
            visionOffsetAngles = drivers->cvCommunicator.getVisionOffsetAngles();
            yawOffsetAngleDisplay = visionOffsetAngles[0][0];
            pitchOffsetAngleDisplay = visionOffsetAngles[1][0];

            if (cvState == src::vision::CV_STATE_FOUND) {
                currMode = CHASE;
            } else {
                currMode = MANUAL;
            }
        } else {
            currMode = MANUAL;
        }

#ifdef TARGET_SENTRY
        switch (currMode) {
            case PATROL:
                // TODO: need to update it so that it runs of a matrix of target positions for sentry so that it can go around and patrol it.
                targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Radians) - yawPatrolLocations[yawPatrolLocationIndex][0];
                targetPitchAngle = getPitchPatrolAngle(AngleUnit::Radians);
                break;
            case CHASE:
                // TODO: needs to be updated so that it is getting from the CV.
                targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Radians) + visionOffsetAngles[0][0];
                targetPitchAngle = gimbal->getTargetPitchAngle(AngleUnit::Radians) + visionOffsetAngles[1][0];
                break;
            case MANUAL:
                targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Radians) -
                                 (userInputYawSensitivityFactor * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL)) * YAW_INPUT_DIRECTION;
                targetPitchAngle = gimbal->getTargetPitchAngle(AngleUnit::Radians) -
                                   (userInputPitchSensitivityFactor * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL)) * getPitchMotorDirection();
                break;
        }

#else
        if (CHASE) {
            // TODO: needs to be updated so that it is getting from the CV.
            targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Degrees) - 0;
            targetPitchAngle = gimbal->getTargetPitchAngle(AngleUnit::Degrees) - 0;
        } else if (MANUAL) {
            targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Degrees) -
                             (userInputYawSensitivityFactor * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL)) * YAW_INPUT_DIRECTION;
            targetPitchAngle = gimbal->getTargetPitchAngle(AngleUnit::Degrees) -
                               (userInputPitchSensitivityFactor * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL)) * getPitchMotorDirection();
        }
#endif

        // controller->runYawController(AngleUnit::Radians, targetYawAngle);
        // controller->runPitchController(AngleUnit::Radians, targetPitchAngle);

        // insert code that updates if between CHASE and PATROL/MANUAL based of cv input.
    }

    bool GimbalControlCommand::isReady() { return true; }

    bool GimbalControlCommand::isFinished() const { return false; }

    void GimbalControlCommand::end(bool) {
        gimbal->setYawMotorOutput(0);
        gimbal->setPitchMotorOutput(0);
    }

}  // namespace src::Gimbal