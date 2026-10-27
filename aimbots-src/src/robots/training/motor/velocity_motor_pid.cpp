#include "velocity_motor_pid.hpp"

#ifdef TARGET_TRAINING

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

namespace src::Training {

VelocityMotorPid::VelocityMotorPid(src::Drivers* drivers, TrainingMotorSubsystem* motor, const SmoothPIDConfig& pidConfig)
    : drivers(drivers),
      motor(motor),
      pid(pidConfig)  //
{}

void VelocityMotorPid::initialize() {
    // TODO(week3):
    // 1. Reset the taproot PID inside the wrapper: pid.pid.reset() (clears the I term and the last error).
    // 2. Set pid.lastTime to the current time in ms (tap::arch::clock::getTimeMilliseconds()).
    //    The wrapper computes dt = now - lastTime. If you skip this, the first run after the command
    //    starts sees a dt of however long the command was off (seconds!), and the I term jumps.
}

void VelocityMotorPid::setTargetVelocity(float targetRpm) {
    // TODO(week3): store the target, clamped to -MOTOR_MAX_SPEED_RPM..MOTOR_MAX_SPEED_RPM.
    // tap::algorithms::limitVal<float>(value, min, max) does the clamp.
    (void)targetRpm;  // delete this line once you use targetRpm
}

void VelocityMotorPid::runVelocityController(std::optional<float> velocityLimit) {
    // TODO(week3):
    // 1. Start from the stored target. If velocityLimit has a value, clamp it to +-velocityLimit.value().
    // 2. error = target - measured speed (motor->getMotorRPM()).
    // 3. Run the PID on the error: pid.runControllerDerivateError(error) returns the output.
    //    (It works out dt and the derivative of the error for you.)
    // 4. Send that output to the motor: motor->setDesiredOutput(output).
    (void)velocityLimit;  // delete this line once you use velocityLimit
}

void VelocityMotorPid::setPidGains(float kp, float ki, float kd) {
    // TODO(week3): set the three gains on pid.pid (setP / setI / setD), then call initialize() so the
    // old I term doesn't carry over into the new gains.
    (void)kp;  // delete these three lines once you use the gains
    (void)ki;
    (void)kd;
}

bool VelocityMotorPid::isOnline() const {
    // TODO(week3): ask the motor subsystem.
    return false;
}

}  // namespace src::Training

#endif  // TARGET_TRAINING
