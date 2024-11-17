
#include "sentry_intelligence.hpp"

#ifdef TARGET_SENTRY


namespace SentryControl{
  
//static constexpr int BASE_BURST_LENGTH = 3;
//static constexpr int ANNOYED_BURST_LENGTH = 10;

//static constexpr float MAX_FEEDER_SPEED = 2550.0f;  // 23.6 bps
//static constexpr float MIN_FEEDER_SPEED = 760.0f;   // 7 bps

// watch varables
float proportional_HP_Display = 0;
float current_HP_Display = 0;
bool in_excute = false;

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

  in_excute = true;

  //while(true) {
    current_HP = drivers->refSerial.getRobotData().currentHp;
    //current_Ammo = drivers->refSerial.getRobotData().current;
    current_Time = tap::arch::clock::getTimeMilliseconds();

    //proportional_Ammo = (previous_Ammo-current_Ammo)/(previous_time-current_Time)
    proportional_HP = (previous_HP-current_HP)/(current_Time-previous_time);
    current_HP_Display = current_HP;

    //:
    //0 = perviousHP = CurrentHP (start postion, nobody has attacked me; or i am dead ) [ATTACK]
    //1 greater than = run away (you are lossing health too quick) [RUN]
    //less than 1 means you are attacking (but health is not severely affected) [ATTACK]


    //
    /*
    if (!drivers->cvCommunicator.isJetsonOnline()) {
          scheduleIfNotScheduled(this->comprisedCommandScheduler, patrolCommand);
          this->comprisedCommandScheduler.run();
          return;
      }

      if (drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::FOUND ||
          drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::FIRE) {
          scheduleIfNotScheduled(this->comprisedCommandScheduler, chaseCommand);
          chaseTimeout.restart(chaseTimeoutMillis);

          /*
          float yawTargetGimbal = fieldRelativeGimbalController->getTargetYaw(AngleUnit::Radians);
              tap::algorithms::WrappedFloat yawCurrentGimbal =
                  drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat();

              float pitchTargetGimbal = fieldRelativeGimbalController->getTargetPitch(AngleUnit::Radians);
              tap::algorithms::WrappedFloat pitchCurrentGimbal =
                  drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat();

              isErrorCloseEnoughToShoot = ballisticsSolver->withinAimingTolerance(
                  yawCurrentGimbal.minDifference(yawTargetGimbal),
                  pitchCurrentGimbal.minDifference(pitchTargetGimbal),
                  targetDepth);

              balYawErrorDisplay = abs(yawCurrentGimbal.minDifference(yawTargetGimbal));
              balYawAtanDisplay = atan2f(0.35f, 2.0f * targetDepth);

              balPitchErrorDisplay = abs(pitchCurrentGimbal.minDifference(pitchTargetGimbal));
              balPitchAtanDisplay = atan2f(1.0f, 2.0f * targetDepth);

            */
      /*
      }else if (chaseTimeout.isExpired()) {
          scheduleIfNotScheduled(this->comprisedCommandScheduler, patrolCommand);
      }

      this->comprisedCommandScheduler.run();
    */
    proportional_HP_Display = proportional_HP;

    if(proportional_HP > 1){
      //evade
      autoNavCommand.setTargetLocation(1.5f, 6.0f); // fake target location 
    }
    if(proportional_HP <= 1){
      //attack
      autoNavCommand.setTargetLocation(9.0f, 0.5f); // fake target loaction 
    }


    previous_HP = current_HP;
    previous_time = current_Time;
  //}
   
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

