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

        // implements sin function with current time (millis) as function input
        float getSinusoidalPitchPatrolAngle(AngleUnit unit) {
            constexpr float intialDirection = (PITCH_SOFTSTOP_HIGH < PITCH_SOFTSTOP_LOW) ? 1.0f : -1.0f;
            float angle = modm::toRadian(PITCH_PATROL_AMPLITUDE) * sin(PITCH_PATROL_FREQUENCY * tap::arch::clock::getTimeMilliseconds() / 1000.0f) + modm::toRadian((intialDirection * PITCH_PATROL_OFFSET) + PITCH_HORIZON_ANGLE);
            if (unit == AngleUnit::Radians) {
                return angle;
            }
            return modm::toDegree(angle);
        }

        void updateYawPatrolTarget() {
            if (controller->getYawPositionPID()->isSettled(10.0f)) {
                if (patrolTimer.execute()) {
                    // if we're settled at the target angle, and the timer expires for the first time, bounce the patrol coordinate index
                    patrolCoordinateIndex = (patrolCoordinateIndex + 1) % patrolCoordinates.getNumberOfRows();
                } else if (patrolTimer.isExpired() || patrolTimer.isStopped()) {
                    // if we're settled at the target angle, and the timer has already expired or hasn't ever been started, start the timer
                    patrolTimer.restart(static_cast<uint32_t>(*patrolCoordinates.getRow(patrolCoordinateIndex)[2]));
                }
            }
            currPatrolCoordinate = patrolCoordinates.getRow(patrolCoordinateIndex);
        }

        // function assumes gimbal yaw is at 0 degrees (positive x axis)
        float getFieldRelativeYawPatrolAngle(AngleUnit unit) {
            this->updateYawPatrolTarget();
            // needs to target XY positions on the field from patrolCoordinates.getRow(patrolCoordinateIndex)
            // convert that to an angle relative to the field's positive x axis
            float xy_angle = xy_angle_between_locations(AngleUnit::Radians, drivers->fieldRelativeInformant.getFieldRelativeRobotPosition(), currPatrolCoordinate);
#ifdef TARGET_SENTRY
            // if robot is sentry, need to rotate left by 45 degrees because sentry rail is at 45 degree angle relative to field
            xy_angle += modm::toRadian(45.0f);
#endif
            // offset by the preset "front" angle of the robot
            xy_angle += modm::toRadian(YAW_FRONT_ANGLE);

            if (unit == AngleUnit::Degrees) {
                xy_angle = modm::toDegree(xy_angle);
            }
            return xy_angle;
        }

       private:
        src::Drivers* drivers;

        GimbalSubsystem* gimbal;
        GimbalChassisRelativeController* controller;

        float userInputYawSensitivityFactor;
        float userInputPitchSensitivityFactor;

        gimbalCommandMode currMode;

        MilliTimeout patrolTimer;
        Matrix<float, 8, 3> patrolCoordinates;
        int patrolCoordinateIndex = 0;
        Matrix<float, 1, 3> currPatrolCoordinate;
    };

}  // namespace src::Gimbal