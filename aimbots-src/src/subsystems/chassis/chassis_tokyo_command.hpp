#pragma once

#include "tap/algorithms/ramp.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Chassis {

class ChassisTokyoCommand : public TapCommand {
public:
    ChassisTokyoCommand(src::Drivers*, ChassisSubsystem*, src::Gimbal::GimbalSubsystem*, int spinDirectionOverride = 0);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    void setRotationDirection(bool rotateLeft) {rotationDirection = (rotateLeft ? 1 : -1);}

    const char* getName() const override { return "Chassis Follow Gimbal"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;
    int spinDirectionOverride;

    float rotationDirection;

    tap::algorithms::Ramp rotationSpeedRamp;
};

}  // namespace src::Chassis