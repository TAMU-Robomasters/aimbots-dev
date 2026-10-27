#include "velocity_pid_tuning_command.hpp"

#ifdef TARGET_TRAINING

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"

// ---- MCUViewer: plot these four together while you tune ---------------------------------------
float tuningTargetRpmDisplay = 0.0f;   // what the square wave asks for
float tuningCurrentRpmDisplay = 0.0f;  // what the motor actually does
float tuningErrorRpmDisplay = 0.0f;    // target - current
float tuningOutputDisplay = 0.0f;      // your PID's voltage command (watch for it pinning at maxOutput)

// ---- MCUViewer: live tuning ---------------------------------------------------------------------
// Type new values into the *Debug variables, then set the matching update flag to 1. The code copies
// them in and sets the flag back to 0. Everything here resets on reboot, so copy gains you like into
// training_control.cpp.
bool updateVelocityPidDebug = false;
float velocityPDebug = 0.0f;
float velocityIDebug = 0.0f;
float velocityDDebug = 0.0f;

bool updateTuningConfigDebug = false;
float tuningAmplitudeRpmDebug = 0.0f;
float tuningFrequencyHzDebug = 0.0f;

namespace src::Training {

VelocityPidTuningCommand::VelocityPidTuningCommand(
    src::Drivers* drivers,
    TrainingMotorSubsystem* motor,
    VelocityMotorPid* controller,
    VelocityTuningConfig config)
    : drivers(drivers),
      motor(motor),
      controller(controller),
      config(config)  //
{
    addSubsystemRequirement(motor);
}

void VelocityPidTuningCommand::initialize() {
    initTime = tap::arch::clock::getTimeMilliseconds();
    controller->initialize();
}

void VelocityPidTuningCommand::execute() {
    if (updateVelocityPidDebug) {
        controller->setPidGains(velocityPDebug, velocityIDebug, velocityDDebug);
        updateVelocityPidDebug = false;
    }

    const float target = getSquareWaveTarget();
    controller->setTargetVelocity(target);
    controller->runVelocityController();

    tuningTargetRpmDisplay = controller->getTargetVelocity();
    tuningCurrentRpmDisplay = controller->getCurrentVelocity();
    tuningErrorRpmDisplay = controller->getError();
    tuningOutputDisplay = controller->getOutput();
}

float VelocityPidTuningCommand::getSquareWaveTarget() {
    if (updateTuningConfigDebug) {
        config.amplitudeRpm = tuningAmplitudeRpmDebug;
        config.frequencyHz = tuningFrequencyHzDebug;
        updateTuningConfigDebug = false;
    }

    const float amplitude = tap::algorithms::limitVal(config.amplitudeRpm, 0.0f, MOTOR_MAX_SPEED_RPM);
    if (config.frequencyHz <= 0.0f) {
        return 0.0f;  // no frequency set yet: hold still instead of dividing by zero
    }

    const float periodMs = 1000.0f / config.frequencyHz;
    // Where we are inside the current period: goes 0 -> periodMs, then starts over at 0.
    const float timeInPeriodMs = fmodf(static_cast<float>(getRelativeTime()), periodMs);

    // TODO(week3): return +amplitude during the first half of the period and -amplitude during the
    // second half. (Sketch the wave against timeInPeriodMs first.)
    (void)amplitude;  // delete these two lines once you use them
    (void)timeInPeriodMs;
    return 0.0f;
}

void VelocityPidTuningCommand::end(bool) {
    // Stop the motor. Without this, it keeps the last voltage and keeps spinning.
    motor->setDesiredOutput(0.0f);
}

bool VelocityPidTuningCommand::isFinished() const { return false; }

}  // namespace src::Training

#endif  // TARGET_TRAINING
