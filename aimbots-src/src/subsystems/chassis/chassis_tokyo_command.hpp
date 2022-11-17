#pragma once
#ifndef ENGINEER
#include "drivers.hpp"
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/gimbal.hpp"
//
#include "tap/algorithms/ramp.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Chassis {

class ChassisTokyoCommand : public TapCommand {
   public:
    ChassisTokyoCommand(src::Drivers*, ChassisSubsystem*, src::Gimbal::GimbalSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Chassis Follow Gimbal"; }

   private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    float rotationDirection;

    tap::algorithms::Ramp rotationSpeedRamp;
};

}  // namespace src::Chassis
#endif