#include <cmath>
#include "utils/tools/robot_specific_defines.hpp"


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

    float yawPatrolAngularVelocityDegreesPerSec;

    // sector scan: once sectorScanSwitchTimeMillis has elapsed since patrol first started, the
    // yaw stops spinning 360 and instead sweeps back and forth between sectorScanStartAngle and
    // sectorScanEndAngle (field-relative, radians) at yawPatrolAngularVelocityDegreesPerSec.
    // The sweep always takes the short way between the two angles; the span is asserted to be
    // nonzero and < 180 degrees.
    uint32_t sectorScanSwitchTimeMillis;
    float sectorScanStartAngle;
    float sectorScanEndAngle;
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
                          sin(M_2_PI * patrolConfig.pitchPatrolFrequency * getTimeSinceCommandInitialize() / 1000.0f) +
                      patrolConfig.pitchPatrolOffset;

        return unit == AngleUnit::Radians ? angle : modm::toDegree(angle);
    }

    // sidesteps patrolCoordinates/chassisState/timer entirely -- just spins continuously
    // at a constant rate set by yawPatrolAngularVelocityDegreesPerSec, starting from
    // wherever the gimbal was pointing when this command initialized (yawPatrolStartAngle).
    // That start offset is what keeps patrol from snapping the yaw back to field-relative
    // zero every time it takes over from the chase command.
    float getConstantVelocityYawPatrolAngle(AngleUnit unit) {
        float angleRadians = yawPatrolStartAngle +
                             modm::toRadian(
                                 patrolConfig.yawPatrolAngularVelocityDegreesPerSec * getTimeSinceCommandInitialize() /
                                 1000.0f);

        return unit == AngleUnit::Radians ? angleRadians : modm::toDegree(angleRadians);
    }

    void updateYawPatrolTarget();

    // function assumes gimbal yaw is at 0 degrees (positive x axis)
    float getFieldRelativeYawPatrolAngle(AngleUnit unit);

    // switches from the 360 spin to the back-and-forth sector sweep once
    // sectorScanSwitchTimeMillis has elapsed since patrol first started, keeping the yaw
    // target continuous across the switch
    void updateScanMode();

    // triangle-wave sweep between sectorScanStartAngle and sectorScanEndAngle at the patrol
    // angular velocity; only valid while inSectorScan
    float getSectorScanYawPatrolAngle(AngleUnit unit, float dtSeconds);

    uint32_t getTimeSinceCommandInitialize() { return tap::arch::clock::getTimeMilliseconds() - commandStartTime; }

private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalFieldRelativeController* controller;

    GimbalPatrolConfig patrolConfig;

    src::Chassis::ChassisMatchStates& chassisState;

    uint32_t commandStartTime = 0;

    // field-relative gimbal yaw (radians) latched at initialize() so the constant-velocity
    // patrol spin continues from the gimbal's current heading instead of resetting to zero
    float yawPatrolStartAngle = 0.0f;

    // sector scan state
    bool inSectorScan = false;
    float sectorScanSpan = 0.0f;       // signed wrapped span start->end (radians), |span| < pi
    float sectorSweepOffset = 0.0f;    // current sweep position along the span, in [0, |span|]
    float sectorSweepDirection = 1.0f;
    uint32_t lastExecuteTime = 0;

    // latched on the very first initialize() and never reset, so chase interruptions don't
    // restart the sector-scan switch countdown
    uint32_t patrolFirstStartTime = 0;

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
