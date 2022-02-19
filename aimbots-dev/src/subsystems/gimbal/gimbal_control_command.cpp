#include "gimbal_control_command.hpp"

#include <tap/architecture/clock.hpp>

namespace src::Gimbal {

GimbalControlCommand::GimbalControlCommand(src::Drivers* drivers,
                                           GimbalSubsystem* gimbalSubsystem,
                                           GimbalChassisRelativeController* gimbalController,
                                           float inputSensitivity)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      userInputSensitivityFactor(inputSensitivity),
      previousTime(0) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalControlCommand::initialize() {
    controller->initialize();
    previousTime = tap::arch::clock::getTimeMilliseconds();
}

void GimbalControlCommand::execute() {
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t deltaTime   = currentTime - previousTime;
    previousTime         = currentTime;

    float targetYawAngle = gimbal->getTargetYawAngleInRadians() +
                           userInputSensitivityFactor * drivers->remote.getChannel(tap::Remote::Channel::RIGHT_HORIZONTAL);
    controller->runYawController(deltaTime, targetYawAngle);

    float targetPitchAngle = gimbal->getTargetPitchAngleInRadians() +
                           userInputSensitivityFactor * drivers->remote.getChannel(tap::Remote::Channel::RIGHT_VERTICAL);
    controller->runPitchController(deltaTime, targetPitchAngle);
}

bool GimbalControlCommand::isReady() { return !isFinished(); }

bool GimbalControlCommand::isFinished() const { return !controller->isOnline(); }

void GimbalControlCommand::end(bool interrupted) {
    (void)interrupted;

    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}

}  // namespace src::Gimbal