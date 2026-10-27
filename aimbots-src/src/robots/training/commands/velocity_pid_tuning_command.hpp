#pragma once

#ifdef TARGET_TRAINING

#include "tap/architecture/clock.hpp"
#include "tap/control/command.hpp"

#include "robots/training/motor/training_motor_subsystem.hpp"
#include "robots/training/motor/velocity_motor_pid.hpp"
#include "drivers.hpp"

namespace src::Training {

/**
 * The target speed for tuning is a square wave: +amplitude for half a period, then -amplitude for the
 * other half, forever. Pick the values in training_control.cpp (README section 4).
 */
struct VelocityTuningConfig {
    float amplitudeRpm = 0.0f;  // clamped to MOTOR_MAX_SPEED_RPM
    float frequencyHz = 0.0f;   // full periods (+ then -) per second
};

/**
 * @brief WEEK 3: drives the motor's target speed as a square wave so you can tune your PID in
 * MCUViewer. Based on src::Gimbal::GimbalVelocityTunningCommand.
 *
 * Almost everything is written for you. You fill in getSquareWaveTarget() in the .cpp.
 */
class VelocityPidTuningCommand : public tap::control::Command {
   public:
    VelocityPidTuningCommand(
        src::Drivers* drivers,
        TrainingMotorSubsystem* motor,
        VelocityMotorPid* controller,
        VelocityTuningConfig config);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

    const char* getName() const override { return "Velocity PID Tuning"; }

   private:
    src::Drivers* drivers;
    TrainingMotorSubsystem* motor;
    VelocityMotorPid* controller;
    VelocityTuningConfig config;

    uint32_t initTime = 0;

    /** @return the target speed (RPM) for right now. */
    float getSquareWaveTarget();

    /** ms since this command started. */
    uint32_t getRelativeTime() const { return tap::arch::clock::getTimeMilliseconds() - initTime; }
};

}  // namespace src::Training

#endif  // TARGET_TRAINING
