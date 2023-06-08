#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_controller_interface.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <utils/common_types.hpp>
#include <utils/pid/smooth_pid_wrap.hpp>

#include "utils/filters/ema.hpp"
#include "utils/kinematic_state_vector.hpp"

namespace src::Gimbal {
class GimbalFieldRelativeController : public GimbalControllerInterface {
public:
    GimbalFieldRelativeController(src::Drivers*, GimbalSubsystem*);

    void initialize() override;

    void BuildPIDControllers() {
        for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
            yawPositionPIDs[i] = new SmoothPID(YAW_POSITION_PID_CONFIG);
            yawPositionCascadePIDs[i] = new SmoothPID(YAW_POSITION_CASCADE_PID_CONFIG);
            yawVelocityPIDs[i] = new SmoothPID(YAW_VELOCITY_PID_CONFIG);

            yawVelocityFilters[i] = new EMAFilter(0.02);  // smoothing yaw velocity heavily for use in velocity controller
        }
        for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
            pitchPositionPIDs[i] = new SmoothPID(PITCH_POSITION_PID_CONFIG);
            pitchPositionCascadePIDs[i] = new SmoothPID(PITCH_POSITION_CASCADE_PID_CONFIG);
            pitchVelocityPIDs[i] = new SmoothPID(PITCH_VELOCITY_PID_CONFIG);
        }
    }

    void runYawController(std::optional<float> velocityLimit = std::nullopt) override;
    void runPitchController() override;

    bool isOnline() const;

    void setTargetYaw(AngleUnit unit, float targetYaw) override {
        targetYaw = (unit == AngleUnit::Radians) ? targetYaw : modm::toRadian(targetYaw);
        fieldRelativeYawTarget.setValue(targetYaw);
    }

    void setTargetPitch(AngleUnit unit, float targetPitch) override {
        targetPitch = (unit == AngleUnit::Radians) ? targetPitch : modm::toRadian(targetPitch);

        // convert chassis-relative pitch soft stops to field-relative angles
        float chassisPitchInGimbalDirection = drivers->kinematicInformant.getChassisPitchInGimbalDirection();
        float softHigh = chassisPitchInGimbalDirection + PITCH_AXIS_SOFTSTOP_HIGH;
        float softLow = chassisPitchInGimbalDirection + PITCH_AXIS_SOFTSTOP_LOW;

        targetPitch =
            tap::algorithms::limitVal(targetPitch, softLow, softHigh);  // this doesn't work if robot is upside down
        fieldRelativePitchTarget.setValue(targetPitch);
    }

    float getTargetYaw(AngleUnit unit) const override {
        return (unit == AngleUnit::Radians) ? fieldRelativeYawTarget.getValue()
                                            : modm::toDegree(fieldRelativeYawTarget.getValue());
    }

    float getTargetPitch(AngleUnit unit) const override {
        return (unit == AngleUnit::Radians) ? fieldRelativePitchTarget.getValue()
                                            : modm::toDegree(fieldRelativePitchTarget.getValue());
    }

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;

    tap::algorithms::ContiguousFloat fieldRelativeYawTarget;
    tap::algorithms::ContiguousFloat fieldRelativePitchTarget;

    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawPositionPIDs;
    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchPositionPIDs;

    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawPositionCascadePIDs;
    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchPositionCascadePIDs;

    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawVelocityPIDs;
    std::array<EMAFilter*, YAW_MOTOR_COUNT> yawVelocityFilters;

    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchVelocityPIDs;
};

}  // namespace src::Gimbal