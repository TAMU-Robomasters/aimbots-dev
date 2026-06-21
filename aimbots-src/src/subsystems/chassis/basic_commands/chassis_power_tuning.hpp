#pragma once

#include "tap/control/command.hpp"

#include "subsystems/chassis/control/chassis.hpp"
#include "utils/tools/common_types.hpp"


#include "subsystems/chassis/control/chassis_helper.hpp"
#include "drivers.hpp"

#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

// field relative chassis locking command
class ChassisTuningCommand : public TapCommand {
public:
    ChassisTuningCommand(
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
};

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE

#endif //#ifdef GIMBAL_UNTETHERED