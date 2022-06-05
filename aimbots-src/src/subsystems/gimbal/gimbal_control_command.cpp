#include "gimbal_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "drivers.hpp"
#include "vision/jetson_communicator.hpp"
#include "vision/jetson_protocol.hpp"

namespace src::Gimbal {

GimbalControlCommand::GimbalControlCommand(src::Drivers* drivers,
                                           GimbalSubsystem* gimbalSubsystem,
                                           GimbalChassisRelativeController* gimbalController,
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
    float targetYawAngle = 0.0f;
    float targetPitchAngle = 0.0f;

    targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Degrees) -
                     userInputYawSensitivityFactor * drivers->controlOperatorInterface.getGimbalYawInput();
    targetPitchAngle = gimbal->getTargetPitchAngle(AngleUnit::Degrees) -
                       userInputPitchSensitivityFactor * drivers->controlOperatorInterface.getGimbalPitchInput();

    controller->runYawController(AngleUnit::Degrees, targetYawAngle);
    controller->runPitchController(AngleUnit::Degrees, targetPitchAngle);
}

bool GimbalControlCommand::isReady() { return true; }

bool GimbalControlCommand::isFinished() const { return false; }

void GimbalControlCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}

}  // namespace src::Gimbal