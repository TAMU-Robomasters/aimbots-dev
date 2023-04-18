#include "gimbal_field_relative_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

namespace src::Gimbal {

GimbalFieldRelativeControlCommand::GimbalFieldRelativeControlCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalControllerInterface* gimbalController)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalFieldRelativeControlCommand::initialize() {}

void GimbalFieldRelativeControlCommand::execute() {
    float quickTurnOffset = 0.0f;

    if (drivers->remote.keyPressed(Remote::Key::Q)) wasQPressed = true;

    if (wasQPressed && !drivers->remote.keyPressed(Remote::Key::Q)) {
        wasQPressed = false;
        quickTurnOffset -= 90.0f;
    }

    if (drivers->remote.keyPressed(Remote::Key::E)) wasEPressed = true;

    if (wasEPressed && !drivers->remote.keyPressed(Remote::Key::E)) {
        wasEPressed = false;
        quickTurnOffset += 90.0f;
    }

    float targetYawAngle = controller->getTargetYaw(AngleUnit::Degrees) + quickTurnOffset +
                           drivers->controlOperatorInterface.getGimbalYawInput();
    controller->runYawController(AngleUnit::Degrees, targetYawAngle);

    float targetPitchAngle =
        gimbal->getTargetPitchAngle(AngleUnit::Degrees) + drivers->controlOperatorInterface.getGimbalPitchInput();
    controller->runPitchController(AngleUnit::Degrees, targetPitchAngle);
}

bool GimbalFieldRelativeControlCommand::isReady() { return true; }

bool GimbalFieldRelativeControlCommand::isFinished() const { return false; }

void GimbalFieldRelativeControlCommand::end(bool) {
    gimbal->setAllDesiredYawOutputs(0);
    gimbal->setAllDesiredPitchOutputs(0);
}

}  // namespace src::Gimbal