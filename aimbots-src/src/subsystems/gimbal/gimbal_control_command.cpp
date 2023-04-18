#include "gimbal_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "informants/vision/jetson_communicator.hpp"
#include "informants/vision/jetson_protocol.hpp"

#include "drivers.hpp"

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
#ifdef TARGET_SENTRY
    float targetYawAngle = 0.0f;
    targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Degrees) + drivers->controlOperatorInterface.getGimbalYawInput();
    controller->runYawController(AngleUnit::Degrees, targetYawAngle);
#else
    // This just locks it to the the forward direction, specified by YAW_OFFSET_ANGLE
    controller->runYawController(AngleUnit::Degrees, YAW_OFFSET_ANGLE);
#endif

    float targetPitchAngle = 0.0f;
    targetPitchAngle =
        gimbal->getTargetPitchAngle(AngleUnit::Degrees) + drivers->controlOperatorInterface.getGimbalPitchInput();
    controller->runPitchController(AngleUnit::Degrees, targetPitchAngle);
}

bool GimbalControlCommand::isReady() { return true; }

bool GimbalControlCommand::isFinished() const { return false; }

void GimbalControlCommand::end(bool) {
    for (auto i = 0; i < YAW_MOTOR_COUNT; i++) gimbal->desiredYawMotorOutputs[i] = 0;
    for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) gimbal->desiredPitchMotorOutputs[i] = 0;
}

}  // namespace src::Gimbal