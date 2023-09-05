#include "utils/robot_specific_inc.hpp"

#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <tap/control/command.hpp>

#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/controllers/gimbal_field_relative_controller.hpp"

namespace src::Gimbal {

struct GimbalPatrolConfig {
    float pitchPatrolAmplitude;
    float pitchPatrolFrequency;
    float pitchPatrolOffset;
};

class GimbalPatrolCommand : public tap::control::Command {
public:
    GimbalPatrolCommand(src::Drivers*, GimbalSubsystem*, GimbalFieldRelativeController*, GimbalPatrolConfig);

    char const* getName() const override { return "Gimbal Patrol Command"; }

    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    // implements sin function with current time (millis) as function input
    float getSinusoidalPitchPatrolAngle(AngleUnit unit) {
        float angle = patrolConfig.pitchPatrolAmplitude *
                          sin(patrolConfig.pitchPatrolFrequency * getTimeSinceCommandInitialize() / 1000.0f) +
                      patrolConfig.pitchPatrolOffset;

        return unit == AngleUnit::Radians ? angle : modm::toDegree(angle);
    }

    void updateYawPatrolTarget();

    // function assumes gimbal yaw is at 0 degrees (positive x axis)
    float getFieldRelativeYawPatrolAngle(AngleUnit unit);

    uint32_t getTimeSinceCommandInitialize() { return tap::arch::clock::getTimeMilliseconds() - commandStartTime; }

private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalFieldRelativeController* controller;

    GimbalPatrolConfig patrolConfig;

    uint32_t commandStartTime = 0;

    static constexpr size_t NUM_PATROL_LOCATIONS = 4;
    std::array<modm::Location2D<float>, NUM_PATROL_LOCATIONS> patrolCoordinates;
    std::array<uint32_t, NUM_PATROL_LOCATIONS> patrolCoordinateTimes;

    MilliTimeout patrolTimer;
    int patrolCoordinateIndex = 0;
    int patrolCoordinateIncrement = 1;
};

}  // namespace src::Gimbal