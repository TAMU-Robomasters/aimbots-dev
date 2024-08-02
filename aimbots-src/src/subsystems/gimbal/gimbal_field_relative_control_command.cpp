#include "gimbal_field_relative_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>
#ifdef GIMBAL_COMPATIBLE

namespace src::Gimbal {

GimbalFieldRelativeControlCommand::GimbalFieldRelativeControlCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalControllerInterface* gimbalController,
    std::optional<float> quickTurnOffset)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      quickTurnOffset(quickTurnOffset)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalFieldRelativeControlCommand::initialize() {}

float gimbalYawInputDisplay = 0.0f;
float targetPitchAngleDisplay = 0;

void GimbalFieldRelativeControlCommand::execute() {
    /*if (drivers->remote.keyPressed(Remote::Key::V) && !wasVPressed) wasVPRessed = true;

    if (wasVPRessed && !drivers->remote.keyPressed(Remote::Key::V)) {
        forceYawOffset = !forceYawOffset;
    }

    if (forceYawOffset) {
        gimbal->setYawAxisAngleOffset(M_PI);
    } else {
        gimbal->setYawAxisAngleOffset(0);
    }*/

    float quickTurnOffset = 0.0f;

    if (drivers->remote.keyPressed(Remote::Key::Q) && !ignoreQuickTurns) wasQPressed = true;

    if (wasQPressed && !drivers->remote.keyPressed(Remote::Key::Q)) {
        wasQPressed = false;
        quickTurnOffset += this->quickTurnOffset.value_or(M_PI_2);
    }

    if (drivers->remote.keyPressed(Remote::Key::E) && !ignoreQuickTurns) wasEPressed = true;

    if (wasEPressed && !drivers->remote.keyPressed(Remote::Key::E)) {
        wasEPressed = false;
        quickTurnOffset -= this->quickTurnOffset.value_or(M_PI_2);
    }

    gimbalYawInputDisplay = drivers->controlOperatorInterface.getGimbalYawInput();
    targetPitchAngleDisplay = controller->getTargetPitch(AngleUnit::Radians);


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
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

}  // namespace src::Gimbal

#endif