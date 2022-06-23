#include "gimbal_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "drivers.hpp"
#include "informants/vision/jetson_communicator.hpp"
#include "informants/vision/jetson_protocol.hpp"

namespace src::Gimbal {

GimbalControlCommand::GimbalControlCommand(src::Drivers* drivers,
                                           GimbalSubsystem* gimbalSubsystem,
                                           GimbalControllerInterface* gimbalController,
                                           float inputYawSensitivity,
                                           float inputPitchSensitivity)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      userInputYawSensitivityFactor(inputYawSensitivity),
      userInputPitchSensitivityFactor(inputPitchSensitivity)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalControlCommand::initialize() {}

void GimbalControlCommand::execute() {
#ifdef TARGET_SENTRY
    float targetYawAngle = 0.0f;
    targetYawAngle = gimbal->getTargetChassisRelativeYawAngle(AngleUnit::Degrees) +
                     userInputYawSensitivityFactor * drivers->controlOperatorInterface.getGimbalYawInput();
    controller->runYawController(AngleUnit::Degrees, targetYawAngle);
#else
    // This just locks it to the the forward direction, specified by YAW_START_ANGLE
    controller->runYawController(AngleUnit::Degrees, YAW_START_ANGLE);
#endif

    float targetPitchAngle = 0.0f;
    targetPitchAngle = gimbal->getTargetChassisRelativePitchAngle(AngleUnit::Degrees) +
                       userInputPitchSensitivityFactor * drivers->controlOperatorInterface.getGimbalPitchInput();
    controller->runPitchController(AngleUnit::Degrees, targetPitchAngle);
}

bool GimbalControlCommand::isReady() { return true; }

bool GimbalControlCommand::isFinished() const { return false; }

void GimbalControlCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}

}  // namespace src::Gimbal