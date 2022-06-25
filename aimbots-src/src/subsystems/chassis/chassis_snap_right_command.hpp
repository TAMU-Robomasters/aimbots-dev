#pragma once

#include "drivers.hpp"
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/gimbal.hpp"
//
#include "tap/algorithms/ramp.hpp"
#include <tap/control/command.hpp>
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Chassis {
    class ChassisSnapRightCommand : public tap::control::Command {
    public:
    ChassisSnapRightCommand(src::Drivers*, ChassisSubsystem*, src::Gimbal::GimbalSubsystem*);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Chassis Snap Right"; }

   private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;


    float rotationDirection;
    float initYaw;

    // tap::algorithms::Ramp rotationSpeedRamp;

    tap::algorithms::Ramp rotationSpeedRamp;

    // tap::algorithms::SmoothPid rotationSpeedRamp;

    };

}
