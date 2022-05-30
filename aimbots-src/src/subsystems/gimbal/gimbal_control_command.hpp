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

        float getSinusoidalPitchPatrolAngle(AngleUnit unit) {
            constexpr float intialDirection = (PITCH_SOFTSTOP_HIGH < PITCH_SOFTSTOP_LOW) ? 1.0f : -1.0f;
            float angle = modm::toRadian(PITCH_PATROL_AMPLITUDE) * sin(PITCH_PATROL_FREQUENCY * tap::arch::clock::getTimeMilliseconds() / 1000.0f) + modm::toRadian((intialDirection * PITCH_PATROL_OFFSET) + PITCH_HORIZON_ANGLE);
            if (unit == AngleUnit::Radians) {
                return angle;
            }
            return modm::toDegree(angle);
        }

        // function assumes gimbal yaw is at 0 degrees (positive x axis)
        float getFieldRelativeYawPatrolAngle(AngleUnit) {
            // needs to target XY positions on the field from patrolCoordinates[patrolCoordinateIndex]
            // convert that to an angle, and if we're settled at the target angle, start timer with value patrolCoordinates[patrolCoordinateIndex][2]
            // if timer is up, increment patrolCoordinateIndex (or bounce) and reset timer
            return 0.0f;
        }

       private:
        src::Drivers* drivers;

        GimbalSubsystem* gimbal;
        GimbalChassisRelativeController* controller;

        float userInputYawSensitivityFactor;
        float userInputPitchSensitivityFactor;

        gimbalCommandMode currMode;

#ifdef TARGET_SENTRY
        Matrix<float, 8, 3> patrolCoordinates;
        Matrix<float, 1, 3> currPatrolCoordinate;
        int patrolCoordinateIndex = 0;
#endif
    };

}  // namespace src::Gimbal