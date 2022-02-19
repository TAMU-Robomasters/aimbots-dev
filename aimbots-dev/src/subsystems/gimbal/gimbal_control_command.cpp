#include "gimbal_control_command.hpp"

#include <tap/architecture/clock.hpp>

namespace src::Gimbal {

GimbalControlCommand::GimbalControlCommand(src::Drivers* drivers, GimbalSubsystem* gimbalSubsystem, GimbalChassisRelativeController* gimbalController)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController),
      previousTime(0) { }

void GimbalControlCommand::initialize() {
    controller->initialize();
    previousTime = tap::arch::clock::getTimeMilliseconds();
}

void GimbalControlCommand::execute() {
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t deltaTime   = currentTime - previousTime;
    previousTime         = currentTime;

    // FIXME: Factor in user input
    float targetYawAngle = gimbal->getTargetYawAngleInRadians() +
                           0.0f;
    controller->runYawController(deltaTime, targetYawAngle);

    // FIXME: Factor in user input
    float targetPitchAngle = gimbal->getTargetPitchAngleInRadians() +
                             0.0f;
    controller->runPitchController(deltaTime, targetPitchAngle);
}

bool GimbalControlCommand::isReady() { return !isFinished(); }

bool GimbalControlCommand::isFinished() const { return !controller->isOnline(); }

void GimbalControlCommand::end(bool interrupted) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}

}  // namespace src::Gimbal