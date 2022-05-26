#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <tap/control/command.hpp>

namespace src::Gimbal {

    enum gimbalCommandMode {
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

       private:
        src::Drivers* drivers;

        GimbalSubsystem* gimbal;
        GimbalChassisRelativeController* controller;

        float getPitchPatrolAngle(AngleUnit angle_unit) {
            float angle = modm::toRadian(PITCH_PATROL_AMPLITUDE) * sin(PITCH_PATROL_FREQUENCY * tap::arch::clock::getTimeMilliseconds() / 1000.0f) - modm::toRadian(PITCH_PATROL_OFFSET) + PITCH_HORIZON_ANGLE;
            if (angle_unit == AngleUnit::Radians) {
                return angle;
            }
            return modm::toDegree(angle);
        }

        float userInputYawSensitivityFactor;
        float userInputPitchSensitivityFactor;

        gimbalCommandMode currMode;

        Matrix<float, 8, 2> yawPatrolLocations;
        int yawPatrolLocationIndex = 0;
    };

}  // namespace src::Gimbal