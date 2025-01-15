
#include "sentry_intelligence.hpp"

#ifdef TARGET_SENTRY


namespace SentryControl{
  
//static constexpr int BASE_BURST_LENGTH = 3;
//static constexpr int ANNOYED_BURST_LENGTH = 10;

//static constexpr float MAX_FEEDER_SPEED = 2550.0f;  // 23.6 bps
//static constexpr float MIN_FEEDER_SPEED = 760.0f;   // 7 bps



SentryIntelligenceCommand::SentryIntelligenceCommand(
    src::Drivers* drivers,
    src::Gimbal::GimbalSubsystem* gimbal,
    src::Feeder::FeederSubsystem* feeder,
    src::Chassis::ChassisSubsystem* chassis,
    src::Shooter::ShooterSubsystem* shooter,
    src::Utils::RefereeHelperTurreted* refHelper,
    src::Chassis::ChassisAutoNavCommand& autoNavCommand)
    //const src::Chassis::ChassisAutoNavCommand::defaultLinearConfig& defaultLinearConfig,
    //const src::Chassis::defaultRotationConfig& defaultRotationConfig,
    //const src::Chassis::SnapSymmetryConfig& snapSymmetryConfig)
    /*
    //Gimbal
    src::Gimbal::GimbalFieldRelativeController* controller,
    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver,
    src::Gimbal::GimbalPatrolCommand* patrolCommand,
    src::Gimbal::GimbalChaseCommand* chaseCommand,
    src::Gimbal::GimbalPatrolConfig* patrolConfig,

    
    src::Chassis::ChassisAutoNavCommand* autoNavCommand,
    src::Chassis::ChassisAutoNavTokyoCommand* autoNavTokyoCommand,
    src::Chassis::ChassisTokyoCommand* tokyoCommand,
    const defaultLinearConfig7 defaultLinearConfig,
    const defaultRotationConfig& defaultRotationConfig,
    

    //feeder
    src::Gimbal::GimbalControllerInterface* fieldRelativeGimbalController,
    src::Feeder::StopFeederCommand* stopFeederCommand,
    src::Feeder::FullAutoFeederCommand* fullAutoFeederCommand,
    src::Shooter::StopShooterCommand* stopShooterCommand,
    src::Shooter::RunShooterCommand* runShooterCommand) 
    */
    : TapComprisedCommand(drivers),
      drivers(drivers),
      gimbal(gimbal),
      feeder(feeder),
      chassis(chassis),
      autoNavCommand(autoNavCommand) {
      //autoNavCommand(drivers, chassis, src::Chassis::defaultLinearConfig, src::Chassis::defaultRotationConfig, src::Chassis::SnapSymmetryConfig) {
  addSubsystemRequirement(chassis);
  this->comprisedCommandScheduler.registerSubsystem(chassis);
  addSubsystemRequirement(feeder);
  this->comprisedCommandScheduler.registerSubsystem(feeder);
  addSubsystemRequirement(gimbal);
  this->comprisedCommandScheduler.registerSubsystem(gimbal);

}

//float balYawErrorDisplay = 0.0f;
//float balYawAtanDisplay = 0.0f;
//float balPitchErrorDisplay = 0.0f;
//float balPitchAtanDisplay = 0.0f;
  

void SentryIntelligenceCommand::initialize() {
  

  

  //initialize gimabal command
  //scheduleIfNotScheduled(this->comprisedCommandScheduler, patrolCommand);

}
   



void SentryIntelligenceCommand::execute() {

   if (chassisState != lastChassisState) {
      // Perform setup for the next state
      switch (chassisState) {
        case ChassisMatchStates::START:
          //Call start command
          break;
        case ChassisMatchStates::RETREAT:
          //Call retreat command
          break;
        case ChassisMatchStates::ATTACK:
          //cal attack command
          break;
        case ChassisMatchStates::PATROL:
            default:
            break;
            }

            lastChassisState = chassisState;
        }
   
}

void SentryIntelligenceCommand::end(bool interrupted) {

  /*
  // feeder
    descheduleIfScheduled(this->comprisedCommandScheduler, fullAutoFeederCommand, interrupted);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, stopFeederCommand);
  //gimbal
    descheduleIfScheduled(this->comprisedCommandScheduler, patrolCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, chaseCommand, interrupted);
  //chasis
     descheduleIfScheduled(this->comprisedCommandScheduler, autoNavCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, autoNavTokyoCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, tokyoCommand, interrupted);
  */
}

bool SentryIntelligenceCommand::isReady() { return true; }

bool SentryIntelligenceCommand::isFinished() const { 
  //when time == 0 or health = 0
  
  return false; 
  }

}


#endif

