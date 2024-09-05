#include "gimbal_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "informants/vision/jetson_communicator.hpp"
#include "informants/vision/jetson_protocol.hpp"

#include "drivers.hpp"
#ifdef GIMBAL_COMPATIBLE


namespace src::Gimbal {

GimbalControlCommand::GimbalControlCommand(
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

void GimbalControlCommand::initialize() {}

void GimbalControlCommand::execute() {
    float targetYawAxisAngle = 0.0f;
    float targetPitchAxisAngle = 0.0f;

    targetYawAxisAngle =
        controller->getTargetYaw(AngleUnit::Radians) - drivers->controlOperatorInterface.getGimbalYawInput();

    targetPitchAxisAngle =
        controller->getTargetPitch(AngleUnit::Radians) + drivers->controlOperatorInterface.getGimbalPitchInput();

    controller->setTargetYaw(AngleUnit::Radians, targetYawAxisAngle);
    controller->setTargetPitch(AngleUnit::Radians, targetPitchAxisAngle);

    controller->runYawController();
    controller->runPitchController();
}

bool GimbalControlCommand::isReady() { return true; }

bool GimbalControlCommand::isFinished() const { return false; }

void GimbalControlCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

}  // namespace src::Gimbal

#endif