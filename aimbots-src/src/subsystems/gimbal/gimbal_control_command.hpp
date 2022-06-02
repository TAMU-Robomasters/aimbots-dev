#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <tap/control/command.hpp>

namespace src::Gimbal {

enum gimbalControlMode {
    PATROL,
    CHASE,
    MANUAL
};

class GimbalControlCommand : public tap::control::Command {
   public:
    GimbalControlCommand(src::Drivers*,
                         GimbalSubsystem*,
                         GimbalChassisRelativeController*,
                         float inputYawSensitivity,
                         float inputPitchSensitiity);

    char const* getName() const override { return "Gimbal Control Command"; }

    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    // implements sin function with current time (millis) as function input
    float getSinusoidalPitchPatrolAngle(AngleUnit unit) {
        constexpr float intialDirection = (PITCH_SOFTSTOP_HIGH < PITCH_SOFTSTOP_LOW) ? 1.0f : -1.0f;
        float angle = modm::toRadian(PITCH_PATROL_AMPLITUDE) * sin(PITCH_PATROL_FREQUENCY * tap::arch::clock::getTimeMilliseconds() / 1000.0f) + modm::toRadian((intialDirection * PITCH_PATROL_OFFSET) + PITCH_HORIZON_ANGLE);
        if (unit == AngleUnit::Radians) {
            return angle;
        }
        return modm::toDegree(angle);
    }

    void updateYawPatrolTarget();

    // function assumes gimbal yaw is at 0 degrees (positive x axis)
    float getFieldRelativeYawPatrolAngle(AngleUnit unit);

   private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalChassisRelativeController* controller;

    float userInputYawSensitivityFactor;
    float userInputPitchSensitivityFactor;

    gimbalControlMode currMode;

    MilliTimeout patrolTimer;
    Matrix<float, 5, 3> patrolCoordinates;
    int patrolCoordinateIndex;
    int patrolCoordinateIncrement;
};

}  // namespace src::Gimbal