
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

#include "sentry_push_hill.hpp"



#include "drivers.hpp"



#ifdef CHASSIS_COMPATIBLE


namespace SentryControl{

enum SentryMatchStates { START = 0, PUSH, ATTACK, RETREAT, PATROL, CONTROL};

class SentryIntelligenceCommand : public TapComprisedCommand {
public:
    SentryIntelligenceCommand(
        src::Drivers*,
        src::Gimbal::GimbalSubsystem*,
        src::Feeder::FeederSubsystem*,
        src::Chassis::ChassisSubsystem*,
        src::Shooter::ShooterSubsystem*,
        src::Utils::RefereeHelperTurreted*,
        
       SentryMatchStates& sentryState);
       

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Intelligence Command"; }

private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbal;
    src::Feeder::FeederSubsystem* feeder;
    src::Chassis::ChassisSubsystem* chassis;
    src::Shooter::ShooterSubsystem* shooter;
    src::Utils::RefereeHelperTurreted* refHelper;
   
    SentryMatchStates& sentryState;
    SentryControl::SentryMatchStates lastSentryState;
    

};

}


#endif
#endif

