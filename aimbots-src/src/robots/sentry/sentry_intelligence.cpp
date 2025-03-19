
#include "sentry_intelligence.hpp"


#ifdef TARGET_SENTRY


namespace SentryControl{

 const char* watchState = "START";

SentryIntelligenceCommand::SentryIntelligenceCommand(
    src::Drivers* drivers,
    src::Gimbal::GimbalSubsystem* gimbal,
    src::Feeder::FeederSubsystem* feeder,
    src::Chassis::ChassisSubsystem* chassis,
    src::Shooter::ShooterSubsystem* shooter,
    src::Utils::RefereeHelperTurreted* refHelp,
    SentryMatchStates& sentryState
      )
    : TapComprisedCommand(drivers),
      drivers(drivers),
      gimbal(gimbal),
      feeder(feeder),
      chassis(chassis),
      sentryState(sentryState),
      sentryPushHill(drivers, gimbal, feeder, chassis, shooter, refHelper),
      sentryAttack(drivers, gimbal, feeder, chassis, shooter, refHelper),
      sentryRetreat(drivers, gimbal, feeder, chassis, shooter, refHelper)
      //sentryPatrol(drivers, gimbal, feeder, chassis, shooter, refHelp)
      {
      //autoNavCommand(drivers, chassis, src::Chassis::defaultLinearConfig, src::Chassis::defaultRotationConfig, src::Chassis::SnapSymmetryConfig) {
  addSubsystemRequirement(chassis);
  this->comprisedCommandScheduler.registerSubsystem(chassis);
  addSubsystemRequirement(feeder);
  this->comprisedCommandScheduler.registerSubsystem(feeder);
  addSubsystemRequirement(gimbal);
  this->comprisedCommandScheduler.registerSubsystem(gimbal);

}


  

void SentryIntelligenceCommand::initialize() {
  sentryState = SentryMatchStates::START;
  lastSentryState = SentryMatchStates::PATROL;
}
   


void SentryIntelligenceCommand::execute() {


  
  if (sentryState == SentryMatchStates::START || sentryState != lastSentryState) {
    // Perform setup for the next state
    switch (sentryState) {
      case SentryMatchStates::START:
        // calls sentry_push(Hill
        lastSentryState = sentryState;
        sentryState = SentryMatchStates::PUSH;
        watchState = "START TO NOW PUSH";
        break;
      case SentryMatchStates::PUSH:
        lastSentryState = sentryState;
        sentryPushHill.execute();
        watchState = "PUSH";
        break;
      case SentryMatchStates::ATTACK:
        lastSentryState = sentryState;
        sentryAttack.execute();
        watchState = "ATTACK";
        break;
      case SentryMatchStates::RETREAT:
        lastSentryState = sentryState;
        sentryRetreat.execute();
        watchState = "RETREAT";
        // move back to the home base
        break;
      case SentryMatchStates::PATROL:
        lastSentryState = sentryState;
        sentryAttack.execute();
        watchState = "PATROL";
      // go to the other base

        break;
      case SentryMatchStates::CONTROL:
        lastSentryState = sentryState;
        sentryAttack.execute();
        watchState = "CONTROL";
        break;
    }

  }

  if(sentryPushHill.exit()!= sentryState){
    sentryState == sentryPushHill.exit();
  }

   
}

void SentryIntelligenceCommand::end(bool interrupted) {

 
}

bool SentryIntelligenceCommand::isReady() { return true; }

bool SentryIntelligenceCommand::isFinished() const { 
  
  
  return false; 
  }

}


#endif


// starts
//push hill - no robots at the hill -- push center
//control hill enemies found -- push center
// retreat hp< 200 -- push center
// control center
//attack

//start with the start state and the assigned to push hill
//push hill checks whether enemies spotted, if enemies spotted then control hill, or less health then rereat
// if neither then push center
// if enemy spotted for more than 5 secs then attack
// if nobosy noticed then control center
//if lost sight for 5 second then go to control center
//if enemy robot spotted then go back to attack


//attack
//push:hill - 



