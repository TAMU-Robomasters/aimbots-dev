
#include "sentry_intelligence.hpp"

#ifdef TARGET_SENTRY


namespace SentryControl{
  
static constexpr int BASE_BURST_LENGTH = 3;
static constexpr int ANNOYED_BURST_LENGTH = 10;

static constexpr float MAX_FEEDER_SPEED = 2550.0f;  // 23.6 bps
static constexpr float MIN_FEEDER_SPEED = 760.0f;   // 7 bps

SentryIntelligenceCommand::SentryIntelligenceCommand(
    src::Drivers* drivers,
    src::Gimbal::GimbalSubsystem* gimbal,
    src::Feeder::FeederSubsystem* feeder,
    src::Chassis::ChassisSubsystem* chassis,
    src::Shooter::ShooterSubsystem* shooter,
    src::Utils::RefereeHelperTurreted* refHelper,
    src::Chassis::ChassisAutoNavCommand* autoNavCommand)
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
    const SnapSymmetryConfig& snapSymmetryConfig,

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
      chassis(chassis)
      //autoNavCommand(drivers, chassis, //defaultLinearConfig, defaultRotationConfig, snapSymmetryConfig)
      //controller(controller)

      /*
      ballisticsSolver(ballisticsSolver),
      patrolCommand(drivers, gimbal, controller, patrolConfig, chassisState),
      chaseCommand(drivers, gimbal, controller, refHelper, ballisticsSolver, 30.0f),
  //feeder
      shooter(shooter),
      refHelper(refHelper),
      fieldRelativeGimbalController(fieldRelativeGimbalController),
      stopFeederCommand(drivers, feeder),
      fullAutoFeederCommand(drivers, feeder, refHelper, 1, UNJAM_TIMER_MS),
      stopShooterCommand(drivers, shooter),
      runShooterCommand(drivers, shooter, refHelper),

    //chasis
    autoNavCommand(drivers, chassis, defaultLinearConfig, defaultRotationConfig, snapSymmetryConfig),
    autoNavTokyoCommand(
        drivers,
        chassis,
        defaultLinearConfig,
        tokyoConfig,
        randomizeSpinRate,
        randomizerConfig),  // velocity ramp value
    tokyoCommand(drivers, chassis, gimbal, tokyoConfig, 0, randomizeSpinRate, randomizerConfig),
    evadeTimeout(EVADE_DURATION_MS)
    */
{

  addSubsystemRequirement(chassis);
  this->comprisedCommandScheduler.registerSubsystem(chassis);
  addSubsystemRequirement(feeder);
  this->comprisedCommandScheduler.registerSubsystem(feeder);
  addSubsystemRequirement(gimbal);
  this->comprisedCommandScheduler.registerSubsystem(gimbal);

}

float balYawErrorDisplay = 0.0f;
float balYawAtanDisplay = 0.0f;
float balPitchErrorDisplay = 0.0f;
float balPitchAtanDisplay = 0.0f;
  

void SentryIntelligenceCommand::initialize() {
  

  

  //initialize gimabal command
  //scheduleIfNotScheduled(this->comprisedCommandScheduler, patrolCommand);

}
   



void SentryIntelligenceCommand::execute() {

  current_HP = drivers->refSerial.getRobotData().currentHp;
  //current_Ammo = drivers->refSerial.getRobotData().current;
  current_Time = drivers->refSerial.getGameData().stageTimeRemaining;

  //proportional_Ammo = (previous_Ammo-current_Ammo)/(previous_time-current_Time)
  proportional_HP = (previous_HP-current_HP)/(previous_time-current_Time);
  
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
  
  if(proportional_HP > 1.00){
    //evade
    autoNavCommand.setTargetLocation(1.5f, 6.0f); // fake target location 
  }
  if(proportional_HP < 0){
    //attack
    autoNavCommand.setTargetLocation(9.0f, 0.5f); // fake target loaction 
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

