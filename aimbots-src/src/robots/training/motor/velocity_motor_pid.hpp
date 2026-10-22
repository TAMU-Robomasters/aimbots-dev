#pragma once

#ifdef TARGET_TRAINING

#include <optional>

#include "utils/pid/smooth_pid_wrap.hpp"
#include "utils/tools/common_types.hpp"

#include "robots/training/motor/training_motor_subsystem.hpp"
#include "drivers.hpp"

namespace src::Training {

/**
 * @brief WEEK 3: one velocity PID for a voltage-controlled GM6020.
 *
 * Same idea as src::Gimbal::GimbalFieldRelativeController, minus the IMU and the cascade: a
 * command sets a target speed, then calls runVelocityController() every execute(). That compares the
 * target to the motor's measured RPM, runs ONE PID on the error, and sends the PID's output (a
 * voltage command) to the motor subsystem.
 *
 * Don't write your own PID math: use the SmoothPID member below (the team's wrapper around
 * taproot's tap::algorithms::SmoothPid). See training/week3/README.md, section 2.
 *
 * Fill in the TODOs in velocity_motor_pid.cpp.
 */
class VelocityMotorPid {
   public:
    VelocityMotorPid(src::Drivers* drivers, TrainingMotorSubsystem* motor, const SmoothPIDConfig& pidConfig);

    /** Call from your command's initialize(): clears the PID's memory from the last time it ran. */
    void initialize();

    /** @param targetRpm the speed you want. Never let the stored target go past +-MOTOR_MAX_SPEED_RPM. */
    void setTargetVelocity(float targetRpm);

    /**
     * Runs the PID once and sends its output to the motor. Call every execute().
     * @param velocityLimit if given, also clamp the target to +-velocityLimit (RPM) for this run.
     */
    void runVelocityController(std::optional<float> velocityLimit = std::nullopt);

    /** Live tuning from MCUViewer: replace the gains and reset the PID. */
    void setPidGains(float kp, float ki, float kd);

    bool isOnline() const;

    float getTargetVelocity() const { return targetRpm; }        // RPM
    float getCurrentVelocity() const { return motor->getMotorRPM(); }  // RPM
    float getError() { return pid.getError(); }                  // RPM, from the last run
    float getOutput() { return pid.getOutput(); }                // voltage command, from the last run

   private:
    src::Drivers* drivers;
    TrainingMotorSubsystem* motor;

    SmoothPID pid;
    float targetRpm = 0.0f;
};

}  // namespace src::Training

#endif  // TARGET_TRAINING
