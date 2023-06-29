#pragma once

#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"

#include "chassis.hpp"
#include "chassis_helper.hpp"
#include "drivers.hpp"

namespace src::Chassis {

class ChassisAutoNavCommand : public TapCommand {
public:
    ChassisAutoNavCommand(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        SmoothPIDConfig linearPIDConfig,
        SmoothPIDConfig rotationPIDConfig,
        const SnapSymmetryConfig& snapSymmetryConfig = SnapSymmetryConfig());
    ~ChassisAutoNavCommand() = default;

    void initialize() override;
    void execute() override;

    void setTargetLocation(const modm::Location2D<float>& targetLocation) {
        autoNavigator.setTargetLocation(targetLocation);
    }

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    const char* getName() const override { return "Chassis Auto Nav"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    const SnapSymmetryConfig& snapSymmetryConfig;

    AutoNav::AutoNavigatorHolonomic autoNavigator;

    SmoothPID xController;
    SmoothPID yController;
    SmoothPID rotationController;

    tap::algorithms::Ramp xRamp;
    tap::algorithms::Ramp yRamp;
    tap::algorithms::Ramp rotationRamp;

    float linearVelocityRampValue = 1.0f;
    float rotationVelocityRampValue = modm::toRadian(1.0f / 500);
};

static constexpr SmoothPIDConfig defaultLinearConfig = {
    .kp = 0.5f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.1f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig defaultRotationConfig = {
    .kp = 1.25f,
    .ki = 0.0f,
    .kd = 0.25f,
    .maxICumulative = 0.1f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

}  // namespace src::Chassis