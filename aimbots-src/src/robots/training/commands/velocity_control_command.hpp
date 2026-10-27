#pragma once

#ifdef TARGET_TRAINING

#include "tap/control/command.hpp"

#include "robots/training/motor/training_motor_subsystem.hpp"
#include "robots/training/motor/velocity_motor_pid.hpp"
#include "drivers.hpp"

namespace src::Training {

/**
 * @brief WEEK 3: spins the GM6020 at a speed set by the RIGHT stick's vertical axis.
 *
 *   stick fully up     -> +MAX_SPEED_RPM
 *   stick centered     -> 0
 *   stick fully down   -> -MAX_SPEED_RPM
 *
 * Uses YOUR remote parser (week 2) and YOUR VelocityMotorPid (step 1).
 * Fill in the TODOs in velocity_control_command.cpp. See training/week3/README.md, section 5.
 */
class VelocityControlCommand : public tap::control::Command {
   public:
    VelocityControlCommand(src::Drivers* drivers, TrainingMotorSubsystem* motor, VelocityMotorPid* controller);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

    const char* getName() const override { return "Velocity Control"; }

   private:
    // Fastest the stick can ask for, in RPM. Kept at the subsystem's safety limit (90 RPM).
    static constexpr float MAX_SPEED_RPM = MOTOR_MAX_SPEED_RPM;

    src::Drivers* drivers;
    TrainingMotorSubsystem* motor;
    VelocityMotorPid* controller;
};

}  // namespace src::Training

#endif  // TARGET_TRAINING
