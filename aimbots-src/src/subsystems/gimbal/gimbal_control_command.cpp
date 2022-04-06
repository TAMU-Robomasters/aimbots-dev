#include "gimbal_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

namespace src::Gimbal {

//
// NOTE: This function assumes the hardstops are in degrees.
// TODO: Validate that this functions as it should.
//
constexpr float getPitchMotorDirection()
{
    constexpr float intialDirection = (PITCH_HARDSTOP_HIGH < PITCH_HARDSTOP_LOW) ? 1.0f : -1.0f;

    // If 0 is somewhere in our available arc of pitch, then we need
    // to flip the direction, because the previous condition would be
    // incorrect.
    if constexpr (constAbs(PITCH_HARDSTOP_HIGH - PITCH_HARDSTOP_LOW) > 180.0f) {
        return intialDirection * -1.0f;
    }

    return intialDirection;
}

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
      userInputPitchSensitivityFactor(inputPitchSensitivity) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalControlCommand::initialize() {}

void GimbalControlCommand::execute() {
    float targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Degrees) -
                           (userInputYawSensitivityFactor * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_HORIZONTAL)) * YAW_MOTOR_DIRECTION;
    controller->runYawController(AngleUnit::Degrees, targetYawAngle);

    float targetPitchAngle = gimbal->getTargetPitchAngle(AngleUnit::Degrees) -
                             (userInputPitchSensitivityFactor * drivers->remote.getChannel(tap::communication::serial::Remote::Channel::RIGHT_VERTICAL)) * getPitchMotorDirection();
    controller->runPitchController(AngleUnit::Degrees, targetPitchAngle);
}

bool GimbalControlCommand::isReady() { return true; }

bool GimbalControlCommand::isFinished() const { return false; }

void GimbalControlCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}

}  // namespace src::Gimbal