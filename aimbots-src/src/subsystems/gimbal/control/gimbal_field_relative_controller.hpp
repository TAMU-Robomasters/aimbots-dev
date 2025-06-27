#pragma once

#include <optional>

#include <drivers.hpp>
#include <subsystems/gimbal/control/gimbal_controller_interface.hpp>
#include <subsystems/gimbal/control/gimbal.hpp>
#include <utils/tools/common_types.hpp>
#include <utils/pid/smooth_pid_wrap.hpp>

#include "utils/filters/ema.hpp"
#ifdef GIMBAL_COMPATIBLE

namespace src::Gimbal {
class GimbalFieldRelativeController : public GimbalControllerInterface {
public:
    GimbalFieldRelativeController(src::Drivers*, GimbalSubsystem*, bool forCV=false);

    void initialize() override;

    void BuildPIDControllers() {
        for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
            yawPositionPIDs[i] = new SmoothPID(YAW_POSITION_PID_CONFIG);
            if (forCV) {
                yawPositionCascadePIDs[i] = new SmoothPID(VISION_YAW_POSITION_CASCADE_PID_CONFIG);
                yawVelocityPIDs[i] = new SmoothPID(VISION_YAW_VELOCITY_PID_CONFIG);
            } else {
                yawPositionCascadePIDs[i] = new SmoothPID(YAW_POSITION_CASCADE_PID_CONFIG);
                yawVelocityPIDs[i] = new SmoothPID(YAW_VELOCITY_PID_CONFIG);
            }

            yawVelocityFilters[i] = new src::Utils::Filters::EMAFilter(0.1);
            pitchVelocityFilters[i] = new src::Utils::Filters::EMAFilter(0.1);
            // smoothing yaw velocity heavily for display purposes
        }
        for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
            pitchPositionPIDs[i] = new SmoothPID(PITCH_POSITION_PID_CONFIG);
            if (forCV) {
                pitchPositionCascadePIDs[i] = new SmoothPID(VISION_PITCH_POSITION_CASCADE_PID_CONFIG);
                pitchVelocityPIDs[i] = new SmoothPID(VISION_PITCH_VELOCITY_PID_CONFIG);
            } else {
                pitchPositionCascadePIDs[i] = new SmoothPID(PITCH_POSITION_CASCADE_PID_CONFIG);
                pitchVelocityPIDs[i] = new SmoothPID(PITCH_VELOCITY_PID_CONFIG);
            }
        }
    }

    void runYawController(std::optional<float> velocityLimit = std::nullopt) override;
    void runPitchController(std::optional<float> velocityLimit = std::nullopt) override;

    void runYawVelocityController(std::optional<float> velocityLimit = std::nullopt);
    void runPitchVelocityController(std::optional<float> velocityLimit = std::nullopt);

    bool isOnline() const;

    void setTargetYaw(AngleUnit unit, float targetYaw) override {
        targetYaw = (unit == AngleUnit::Radians) ? targetYaw : modm::toRadian(targetYaw);
        fieldRelativeYawTarget.setWrappedValue(targetYaw);
    }

    void setTargetPitch(AngleUnit unit, float targetPitch) override {
        targetPitch = (unit == AngleUnit::Radians) ? targetPitch : modm::toRadian(targetPitch);

        // convert chassis-relative pitch soft stops to field-relative angles
        float chassisPitchInGimbalDirection = drivers->kinematicInformant.getChassisPitchAngleInGimbalDirection();
        float softHigh = chassisPitchInGimbalDirection + PITCH_AXIS_SOFTSTOP_HIGH;
        float softLow = chassisPitchInGimbalDirection + PITCH_AXIS_SOFTSTOP_LOW;

        targetPitch =
            tap::algorithms::limitVal(targetPitch, softLow, softHigh);  // this doesn't work if robot is upside down
        fieldRelativePitchTarget.setWrappedValue(targetPitch);
    }

    // for PID testing
    void setTargetVelocityYaw(AngleUnit unit, float targetVelocityYaw) {
        targetVelocityYaw = (unit == AngleUnit::Radians) ? targetVelocityYaw : modm::toRadian(targetVelocityYaw);
        fieldRelativeVelocityYawTarget = targetVelocityYaw;
    }

    void setTargetVelocityPitch(AngleUnit unit, float targetVelocityPitch) {
        targetVelocityPitch = (unit == AngleUnit::Radians) ? targetVelocityPitch : modm::toRadian(targetVelocityPitch);
        fieldRelativeVelocityPitchTarget = targetVelocityPitch;
    }

    void setTargetAccelerationYaw(AngleUnit unit, float targetAccelerationYaw) {
        targetAccelerationYaw = (unit == AngleUnit::Radians) ? targetAccelerationYaw : modm::toRadian(targetAccelerationYaw);
        fieldRelativeAccelerationYawTarget = targetAccelerationYaw;
    }

    void setTargetAccelerationPitch(AngleUnit unit, float targetAccelerationPitch) {
        targetAccelerationPitch = (unit == AngleUnit::Radians) ? targetAccelerationPitch : modm::toRadian(targetAccelerationPitch);
        fieldRelativeAccelerationPitchTarget = targetAccelerationPitch;
    }

    bool allOnlineYawControllersSettled(float errTolerance, uint32_t errTimeout) {
        bool controllersSettled = false;
        for (int i = 0; i < YAW_MOTOR_COUNT; i++) {
            if (gimbal->isYawMotorOnline(i)) {
                controllersSettled = yawPositionCascadePIDs[i]->isSettled(errTolerance, errTimeout);
            }
        }
        return controllersSettled;
    }

    bool allOnlinePitchControllersSettled(float errTolerance, uint32_t errTimeout) {
        bool controllersSettled = false;
        for (int i = 0; i < PITCH_MOTOR_COUNT; i++) {
            if (gimbal->isPitchMotorOnline(i)) {
                controllersSettled = pitchPositionCascadePIDs[i]->isSettled(errTolerance, errTimeout);
            }
        }
        return controllersSettled;
    }

    float getTargetYaw(AngleUnit unit) const override {
        return (unit == AngleUnit::Radians) ? fieldRelativeYawTarget.getWrappedValue()
                                            : modm::toDegree(fieldRelativeYawTarget.getWrappedValue());
    }

    float getTargetPitch(AngleUnit unit) const override {
        return (unit == AngleUnit::Radians) ? fieldRelativePitchTarget.getWrappedValue()
                                            : modm::toDegree(fieldRelativePitchTarget.getWrappedValue());
    }

    float getTargetVelocityYaw(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? fieldRelativeVelocityYawTarget
                                            : modm::toDegree(fieldRelativeVelocityYawTarget);
    }

    float getTargetVelocityPitch(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? fieldRelativeVelocityPitchTarget
                                            : modm::toDegree(fieldRelativeVelocityPitchTarget);
    }

    float getTargetAccelerationYaw(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? fieldRelativeAccelerationYawTarget
                                            : modm::toDegree(fieldRelativeAccelerationYawTarget);
    }

    float getTargetAccelerationPitch(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? fieldRelativeAccelerationPitchTarget
                                            : modm::toDegree(fieldRelativeAccelerationPitchTarget);
    }

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    bool forCV;

    tap::algorithms::WrappedFloat fieldRelativeYawTarget;
    tap::algorithms::WrappedFloat fieldRelativePitchTarget;

    float fieldRelativeVelocityYawTarget;
    float fieldRelativeVelocityPitchTarget;

    float fieldRelativeAccelerationYawTarget;
    float fieldRelativeAccelerationPitchTarget;
 
    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawPositionPIDs;
    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchPositionPIDs;

    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawPositionCascadePIDs;
    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchPositionCascadePIDs;

    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawVelocityPIDs;
    std::array<src::Utils::Filters::EMAFilter*, YAW_MOTOR_COUNT> yawVelocityFilters;

    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchVelocityPIDs;
    std::array<src::Utils::Filters::EMAFilter*, PITCH_MOTOR_COUNT> pitchVelocityFilters;

    float kYawBallisticVelocity = 20.0f; // ballistic feedforward coefficient
    float kPitchBallisticVelocity = 0.0f; // ballistic feedforward coefficient
    float kBallisticAcceleration = 0.0f; // ballistic feedforward coefficient

    uint32_t yawOuterLoopCounter = -1;
    uint32_t pitchOuterLoopCounter = -1;
    float fieldRelativeVelocityTarget = 0.0f;
};

}  // namespace src::Gimbal
#endif