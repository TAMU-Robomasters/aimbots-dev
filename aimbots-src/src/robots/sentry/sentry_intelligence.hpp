
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
#include "subsystems/chassis/control/chassis_helper.hpp"
#


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
        src::Utils::RefereeHelperTurreted*
        src::Chassis::ChassisAutoNavCommand*,
        const SnapSymmetryConfig& snapSymmetryConfig);

        /*

        //Gimbal
        src::Gimbal::GimbalFieldRelativeController*,
        src::Utils::Ballistics::BallisticsSolver*,
        src::Gimbal::GimbalPatrolCommand*,
        src::Gimbal::GimbalChaseCommand*,
        src::Gimbal::GimbalPatrolConfig*,

        src::Utils::RefereeHelperTurreted*,
        src::Chassis::ChassisAutoNavCommand*,
        src::Chassis::ChassisAutoNavTokyoCommand*,
        src::Chassis::ChassisTokyoCommand*,
        const defaultLinearConfig& defaultLinearConfig,
        const defaultRotationConfig& defaultRotationConfig,
        const SnapSymmetryConfig& snapSymmetryConfig,

        //feeder
        src::Gimbal::GimbalControllerInterface*,
        src::Feeder::StopFeederCommand*,
        src::Feeder::FullAutoFeederCommand*,
        src::Shooter::StopShooterCommand*,
        src::Shooter::RunShooterCommand*)
        */

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
    src::Chassis::ChassisAutoNavCommand* autoNavCommand;

    //Gimbal
    /*
    src::Gimbal::GimbalFieldRelativeController* controller;
    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver;
    src::Gimbal::GimbalPatrolCommand* patrolCommand;
    src::Gimbal::GimbalChaseCommand* chaseCommand;
    src::Gimbal::GimbalPatrolConfig patrolConfig,
    src::Utils::RefereeHelperTurreted* refHelper;
    modm::Location2D<float> waypointTarget;
    src::Chassis::ChassisAutoNavCommand* autoNavCommand;
    src::Chassis::ChassisAutoNavTokyoCommand* autoNavTokyoCommand;
    src::Chassis::ChassisTokyoCommand* tokyoCommand;
    const defaultLinearConfig& defaultLinearConfig;
    const defaultRotationConfig& defaultRotationConfig,
    const SnapSymmetryConfig& snapSymmetryConfig,
    
    //feeder
    src::Gimbal::GimbalControllerInterface* fieldRelativeGimbalController;
    src::Feeder::StopFeederCommand* stopFeederCommand;
    src::Feeder::FullAutoFeederCommand* fullAutoFeederCommand;
    src::Shooter::StopShooterCommand* stopShooterCommand;
    src::Shooter::RunShooterCommand* runShooterCommand;
    */
    float proportional_HP = 0;
    float proportional_Ammo = 0;
    float current_HP = 0;
    float current_Time = 0;
    float previous_HP = 400; //starting HP
    float previous_time = 300; // 5:00 in seconds
    float previous_Ammo = 750; // 750 starting ammo 
    


};

}


#endif
#endif

