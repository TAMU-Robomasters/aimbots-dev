
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
    SentryMatchStates& sentryState)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      gimbal(gimbal),
      feeder(feeder),
      chassis(chassis),
      sentryState(sentryState),
      sentryPushHill(drivers, gimbal, feeder, chassis, shooter, refHelp){
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


  
  if (sentryState != lastSentryState) {
    // Perform setup for the next state
    switch (sentryState) {
      case SentryMatchStates::START:
        // calls sentry_push(Hill)
        sentryState = SentryMatchStates::PUSH;
        watchState = "PUSH";
        sentryPushHill.execute();
        break;
      case SentryMatchStates::PUSH:
       
        break;
      case SentryMatchStates::ATTACK:
        
        break;
      case SentryMatchStates::RETREAT:
        watchState = "RETREAT";
        break;
      case SentryMatchStates::PATROL:
        break;
      case SentryMatchStates::CONTROL:
        break;
    }

    lastSentryState = sentryState;
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

