#include "gimbal_control_command.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

namespace src::Gimbal {

// FIXME: Remove this.
static uint8_t ledIndex = 0;
static tap::gpio::Leds::LedPin pins[8] = {
    tap::gpio::Leds::A,
    tap::gpio::Leds::B,
    tap::gpio::Leds::C,
    tap::gpio::Leds::D,
    tap::gpio::Leds::E,
    tap::gpio::Leds::F,
    tap::gpio::Leds::G,
    tap::gpio::Leds::H,
};

GimbalControlCommand::GimbalControlCommand(src::Drivers* drivers,
                                           GimbalSubsystem* gimbalSubsystem,
                                           GimbalChassisRelativeController* gimbalController,
                                           float inputSensitivity)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      userInputSensitivityFactor(inputSensitivity) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalControlCommand::initialize() {}

static float currentYawAngle = 0.0f;
static float currentPitchAngle = 0.0f;

void GimbalControlCommand::execute() {
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();

    // These are static varibles so we watch the values while debugging.
    currentYawAngle = gimbal->getCurrentYawAngle(AngleUnit::Degrees);
    currentPitchAngle = gimbal->getCurrentPitchAngle(AngleUnit::Degrees);

    // NOTE: For now we don't want to run this. We don't know how the motor is oriented.
    //       So, depending on how the motor is mounted, target angle could be inaccessible,
    //       which could cause us to either damage the motors or the hardware.

    /*
    float targetYawAngle = gimbal->getTargetYawAngle(AngleUnit::Radians) +
                           userInputSensitivityFactor * drivers->remote.getChannel(tap::Remote::Channel::RIGHT_HORIZONTAL);
    controller->runYawController(targetYawAngle);

    float targetPitchAngle = gimbal->getTargetPitchAngle(AngleUnit::Radians) +
                             userInputSensitivityFactor * drivers->remote.getChannel(tap::Remote::Channel::RIGHT_VERTICAL);
    controller->runPitchController(targetPitchAngle);
    */

    // FIXME: Remove this.
    // This just moves the LED so we know that this code is running.
    if (currentTime % 500 == 0) {
        drivers->leds.set(pins[ledIndex], true);
        if (ledIndex > 0)
            drivers->leds.set(pins[ledIndex - 1], false);
        else
            drivers->leds.set(pins[7], false);
        ledIndex = (ledIndex + 1) % 8;
    }
}

bool GimbalControlCommand::isReady() { return true; }

bool GimbalControlCommand::isFinished() const { return false; }

void GimbalControlCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}

}  // namespace src::Gimbal