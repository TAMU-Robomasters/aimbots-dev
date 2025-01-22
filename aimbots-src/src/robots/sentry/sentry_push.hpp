
#pragma once

#ifdef TARGET_SENTRY

#include "utils/tools/robot_specific_defines.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"


#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/tools/common_types.hpp"

#include "subsystems/shooter/control/shooter.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "subsystems/feeder/control/feeder.hpp"
#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/control/chassis_helper.hpp"



#include "drivers.hpp"



#ifdef CHASSIS_COMPATIBLE


namespace SentryControl{



class SentryPushCommand : public TapComprisedCommand {
public:
    SentryPushCommand(
        src::Drivers*,
        src::Gimbal::GimbalSubsystem*,
        src::Feeder::FeederSubsystem*,
        src::Chassis::ChassisSubsystem*,
        src::Shooter::ShooterSubsystem*,
        src::Utils::RefereeHelperTurreted*);
       

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Push Command"; }

private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbal;
    src::Feeder::FeederSubsystem* feeder;
    src::Chassis::ChassisSubsystem* chassis;
    src::Shooter::ShooterSubsystem* shooter;
    src::Utils::RefereeHelperTurreted* refHelper;
   

    

};

}


#endif
#endif

