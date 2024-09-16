#include "utils/tools/robot_specific_defines.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#pragma once

#ifdef ALL_SENTRIES

#include <drivers.hpp>
#include <subsystems/gimbal/control/gimbal.hpp>
#include <tap/control/command.hpp>

#include "subsystems/chassis/complex_commands/sentry_match_chassis_control_command.hpp"
#include "subsystems/gimbal/control/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"

namespace src::Gimbal {

struct GimbalPatrolConfig {
    float pitchPatrolAmplitude;
    float pitchPatrolFrequency;
    float pitchPatrolOffset;
};

class GimbalPatrolCommand : public tap::control::Command {
public:
    GimbalPatrolCommand(
        src::Drivers*,
        GimbalSubsystem*,
        GimbalFieldRelativeController*,
        GimbalPatrolConfig,
        src::Chassis::ChassisMatchStates&);

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

    src::Chassis::ChassisMatchStates& chassisState;

    uint32_t commandStartTime = 0;

    static constexpr size_t NUM_PATROL_LOCATIONS = 4;
    std::array<modm::Location2D<float>, NUM_PATROL_LOCATIONS> safePatrolCoordinates;
    std::array<uint32_t, NUM_PATROL_LOCATIONS> safePatrolCoordinateTimes;

    std::array<modm::Location2D<float>, NUM_PATROL_LOCATIONS> capPatrolCoordinates;
    std::array<uint32_t, NUM_PATROL_LOCATIONS> capPatrolCoordinateTimes;

    std::array<modm::Location2D<float>, NUM_PATROL_LOCATIONS> aggroPatrolCoordinates;
    std::array<uint32_t, NUM_PATROL_LOCATIONS> aggroPatrolCoordinateTimes;

    MilliTimeout patrolTimer;
    int patrolCoordinateIndex = 0;
    int patrolCoordinateIncrement = 1;
};

}  // namespace src::Gimbal

#endif