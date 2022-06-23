#include "gimbal_field_relative_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

namespace src::Gimbal {

GimbalFieldRelativeControlCommand::GimbalFieldRelativeControlCommand(src::Drivers* drivers,
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

void GimbalFieldRelativeControlCommand::initialize() {}

void GimbalFieldRelativeControlCommand::execute() {
    float targetYawAngle = 0.0f;
    targetYawAngle = controller->getTargetYaw(AngleUnit::Degrees) +
                     userInputYawSensitivityFactor * drivers->controlOperatorInterface.getGimbalYawInput();
    controller->runYawController(AngleUnit::Degrees, targetYawAngle);

    float targetPitchAngle = 0.0f;
    targetPitchAngle = gimbal->getTargetChassisRelativePitchAngle(AngleUnit::Degrees) +
                       userInputPitchSensitivityFactor * drivers->controlOperatorInterface.getGimbalPitchInput();
    controller->runPitchController(AngleUnit::Degrees, targetPitchAngle);
}

bool GimbalFieldRelativeControlCommand::isReady() { return true; }

bool GimbalFieldRelativeControlCommand::isFinished() const { return false; }

void GimbalFieldRelativeControlCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}

}  // namespace src::Gimbal