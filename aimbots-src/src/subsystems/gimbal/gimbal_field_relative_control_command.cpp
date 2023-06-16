#include "gimbal_field_relative_control_command.hpp"
#ifndef ENGINEER

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

float gimbalYawInputDisplay = 0.0f;

void GimbalFieldRelativeControlCommand::execute() {
    float quickTurnOffset = 0.0f;

    if (drivers->remote.keyPressed(Remote::Key::Q)) wasQPressed = true;

    if (wasQPressed && !drivers->remote.keyPressed(Remote::Key::Q)) {
        wasQPressed = false;
        quickTurnOffset += M_PI_2;
    }

    if (drivers->remote.keyPressed(Remote::Key::E)) wasEPressed = true;

    if (wasEPressed && !drivers->remote.keyPressed(Remote::Key::E)) {
        wasEPressed = false;
        quickTurnOffset -= M_PI_2;
    }

    gimbalYawInputDisplay =
        controller->getTargetYaw(AngleUnit::Radians) + drivers->controlOperatorInterface.getGimbalYawInput();

    controller->setTargetYaw(
        AngleUnit::Radians,
        controller->getTargetYaw(AngleUnit::Radians) + quickTurnOffset -
            drivers->controlOperatorInterface.getGimbalYawInput());

    controller->setTargetPitch(
        AngleUnit::Radians,
        controller->getTargetPitch(AngleUnit::Radians) + drivers->controlOperatorInterface.getGimbalPitchInput());

    controller->runYawController();
    controller->runPitchController();
}

bool GimbalFieldRelativeControlCommand::isReady() { return true; }

bool GimbalFieldRelativeControlCommand::isFinished() const { return false; }

void GimbalFieldRelativeControlCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchOutputs(0);
}

}  // namespace src::Gimbal
#endif