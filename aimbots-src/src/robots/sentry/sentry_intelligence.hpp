/*
#pragma once

#ifdef TARGET_SENTRY

#include "utils/tools/robot_specific_defines.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "subsystems/shooter/shooter_constants.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/tools/common_types.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "subsystems/chassis/complex_commands/sentry_match_chassis_control_command.hpp"
#include "subsystems/feeder/basic_commands/full_auto_feeder_command.hpp"
#include "subsystems/feeder/basic_commands/stop_feeder_command.hpp"
//
#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"
#include "subsystems/shooter/basic_commands/stop_shooter_command.hpp"
#include "subsystems/shooter/control/shooter.hpp"
//
#include <subsystems/gimbal/control/gimbal_controller_interface.hpp>


#include "subsystems/chassis/complex_commands/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/complex_commands/chassis_auto_nav_tokyo_command.hpp"
#include "subsystems/chassis/basic_commands/chassis_tokyo_command.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"

#include "subsystems/gimbal/complex_commands/gimbal_patrol_command.hpp"
#include "subsystems/gimbal/basic_commands/gimbal_chase_command.hpp"


#include "drivers.hpp"



#ifdef CHASSIS_COMPATIBLE


namespace SentryControl{

class SentryIntelligenceCommand : public TapComprisedCommand {
public:
    SentryIntelligenceCommand(
        src::Drivers*,
        src::Gimbal::GimbalSubsystem*,
        src::Feeder::FeederSubsystem*,
        src::Chassis::ChassisSubsystem*,
        src::Shooter::ShooterSubsystem*,

        //Gimbal
        src::Gimbal::GimbalFieldRelativeController*,
        src::Utils::Ballistics::BallisticsSolver*,
        src::Gimbal::GimbalPatrolCommand*,
        src::Gimbal::GimbalChaseCommand*,

        src::Utils::RefereeHelperTurreted*,
        src::Chassis::ChassisAutoNavCommand*,
        src::Chassis::ChassisAutoNavTokyoCommand*,
        src::Chassis::ChassisTokyoCommand*,

        //feeder
        src::Gimbal::GimbalControllerInterface*,
        src::Feeder::StopFeederCommand*,
        src::Feeder::FullAutoFeederCommand*,
        src::Shooter::StopShooterCommand*,
        src::Shooter::RunShooterCommand*);

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

    //Gimbal
    src::Gimbal::GimbalFieldRelativeController* controller;
    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver;
    src::Gimbal::GimbalPatrolCommand* patrolCommand;
    src::Gimbal::GimbalChaseCommand* chaseCommand;

    src::Utils::RefereeHelperTurreted* refHelper;
    modm::Location2D<float> waypointTarget;
    src::Chassis::ChassisAutoNavCommand* autoNavCommand;
    src::Chassis::ChassisAutoNavTokyoCommand* autoNavTokyoCommand;
    src::Chassis::ChassisTokyoCommand* tokyoCommand;
    
    //feeder
    src::Gimbal::GimbalControllerInterface* fieldRelativeGimbalController;
    src::Feeder::StopFeederCommand* stopFeederCommand;
    src::Feeder::FullAutoFeederCommand* fullAutoFeederCommand;
    src::Shooter::StopShooterCommand* stopShooterCommand;
    src::Shooter::RunShooterCommand* runShooterCommand;



};

}


#endif
#endif

*/