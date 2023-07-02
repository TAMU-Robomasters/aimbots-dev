#include "utils/robot_specific_inc.hpp"

#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <tap/control/command.hpp>

namespace src::Gimbal {

class GimbalPatrolCommand : public tap::control::Command {
public:
    GimbalPatrolCommand(src::Drivers*, GimbalSubsystem*, GimbalChassisRelativeController*, float, float, float, float);

    char const* getName() const override { return "Gimbal Patrol Command"; }

    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    // implements sin function with current time (millis) as function input
    float getSinusoidalPitchPatrolAngle(AngleUnit unit) {
        float angle = pitchPatrolAmplitude * sin(pitchPatrolFrequency * getTimeSinceCommandInitialize() / 1000.0f) +
                      pitchPatrolOffset + pitchOffsetAngle;

        return unit == AngleUnit::Radians ? angle : modm::toDegree(angle);
    }

    void updateYawPatrolTarget();

    // function assumes gimbal yaw is at 0 degrees (positive x axis)
    float getFieldRelativeYawPatrolAngle(AngleUnit unit);

    uint32_t getTimeSinceCommandInitialize() { return tap::arch::clock::getTimeMilliseconds() - commandStartTime; }

private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalChassisRelativeController* controller;

    float pitchPatrolAmplitude;
    float pitchPatrolFrequency;
    float pitchPatrolOffset;
    float pitchOffsetAngle;

    uint32_t commandStartTime = 0;

    Matrix<float, 3, 3> patrolCoordinates;

    MilliTimeout patrolTimer;
    int patrolCoordinateIndex;
    int patrolCoordinateIncrement;
};

}  // namespace src::Gimbal