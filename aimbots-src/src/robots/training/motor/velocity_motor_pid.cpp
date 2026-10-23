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
    pid.pid.reset();
    // 2. Set pid.lastTime to the current time in ms (tap::arch::clock::getTimeMilliseconds()).
    //    The wrapper computes dt = now - lastTime. If you skip this, the first run after the command
    //    starts sees a dt of however long the command was off (seconds!), and the I term jumps.
    pid.lastTime = tap::arch::clock::getTimeMilliseconds();
}

void VelocityMotorPid::setTargetVelocity(float targetRpm) {
    // TODO(week3): store the target, clamped to -MOTOR_MAX_SPEED_RPM..MOTOR_MAX_SPEED_RPM.
    // tap::algorithms::limitVal<float>(value, min, max) does the clamp.
    this->targetRpm = tap::algorithms::limitVal<float>(
        targetRpm,
        -MOTOR_MAX_SPEED_RPM,
        MOTOR_MAX_SPEED_RPM);
}

void VelocityMotorPid::runVelocityController(std::optional<float> velocityLimit) {
    // TODO(week3):
    // 1. Start from the stored target. If velocityLimit has a value, clamp it to +-velocityLimit.value().
    float target = targetRpm;
    if(velocityLimit.has_value()){
        target = tap::algorithms::limitVal<float>(
            target,
            -velocityLimit.value(),
            velocityLimit.value());
    }
    // 2. error = target - measured speed (motor->getMotorRPM()).
    float error = target - motor->getMotorRPM();
    // 3. Run the PID on the error: pid.runControllerDerivateError(error) returns the output.
    //    (It works out dt and the derivative of the error for you.)
    float output = pid.runControllerDerivateError(error);
    // 4. Send that output to the motor: motor->setDesiredOutput(output).
    motor->setDesiredOutput(output);
}

void VelocityMotorPid::setPidGains(float kp, float ki, float kd) {
    // TODO(week3): set the three gains on pid.pid (setP / setI / setD), then call initialize() so the
    // old I term doesn't carry over into the new gains.
    pid.pid.setP(kp);
    pid.pid.setI(ki);
    pid.pid.setD(kd);

    initialize();
}

bool VelocityMotorPid::isOnline() const {
    // TODO(week3): ask the motor subsystem.
    return motor->isMotorOnline();
}

}  // namespace src::Training

#endif  // TARGET_TRAINING
