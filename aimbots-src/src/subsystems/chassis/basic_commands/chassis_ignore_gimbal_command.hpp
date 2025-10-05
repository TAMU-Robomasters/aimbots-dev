#pragma once

#include "tap/control/command.hpp"

#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"


#include "subsystems/chassis/control/chassis_helper.hpp"
#include "drivers.hpp"

#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

// field relative chassis locking command
class ChassisIgnoreGimbalCommand : public TapCommand {
public:
    ChassisIgnoreGimbalCommand(
        src::Drivers*,
        ChassisSubsystem*,
        src::Gimbal::GimbalSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Chassis Ignore Gimbal"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    SmoothPID rotationController;
};

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE

#endif //#ifdef GIMBAL_UNTETHERED