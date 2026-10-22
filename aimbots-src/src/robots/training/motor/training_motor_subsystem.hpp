#pragma once

#ifdef TARGET_TRAINING

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

#include "utils/tools/common_types.hpp"
#include "drivers.hpp"

namespace src::Training {

// ---- Set these to match YOUR motor (training/week3/README.md, section 1) ----------------------
// GM6020 DIP switch ID 1 -> MOTOR5, ID 2 -> MOTOR6, ID 3 -> MOTOR7, ID 4 -> MOTOR8.
static constexpr MotorID TRAINING_MOTOR_ID = MotorID::MOTOR5;
// The CAN port on the dev board the motor is plugged into.
static constexpr CANBus TRAINING_MOTOR_BUS = CANBus::CAN_BUS1;
// Flip this if "positive" spins the wrong way for you.
static constexpr bool TRAINING_MOTOR_INVERTED = false;

// Safety limit: nothing in week 3 should ever ask the motor for more than this.
static constexpr float MOTOR_MAX_SPEED_RPM = 90.0f;

/**
 * @brief WEEK 3: owns the one GM6020 on the bench. You do NOT need to edit this file (except the
 * constants above).
 *
 * This is what a subsystem is really for (week 1's board subsystem was a stand-in): it wraps a
 * piece of hardware so only one command at a time can drive it. Commands (through your
 * VelocityMotorPid) call setDesiredOutput(); refresh() sends it to the motor every 2 ms.
 */
class TrainingMotorSubsystem : public tap::control::Subsystem {
   public:
    TrainingMotorSubsystem(src::Drivers* drivers);

    void initialize() override;
    void refresh() override;

    const char* getName() const override { return "Training Motor"; }

    /** Voltage command for the GM6020, -GM6020_MAX_OUTPUT..GM6020_MAX_OUTPUT (clamped in refresh()). */
    void setDesiredOutput(float output) { desiredOutput = output; }
    float getDesiredOutput() const { return desiredOutput; }

    /** Shaft speed reported by the motor, in RPM. Positive = the motor's positive direction. */
    float getMotorRPM() const { return motor.getInternalEncoder().getShaftRPM(); }

    /** true if the motor has sent us feedback in the last 100 ms. */
    bool isMotorOnline() const { return motor.isMotorOnline(); }

   private:
    DJIMotor motor;
    float desiredOutput = 0.0f;
};

}  // namespace src::Training

#endif  // TARGET_TRAINING
