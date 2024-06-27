#pragma once

#include "tap/control/command.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

class ChassisIdleCommand : public TapCommand {
public:
    ChassisIdleCommand(src::Drivers*, ChassisSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Chassis Idle Drive"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
};

}  // namespace src::Chassis

#endif  //#ifdef CHASSIS_COMPATIBLE