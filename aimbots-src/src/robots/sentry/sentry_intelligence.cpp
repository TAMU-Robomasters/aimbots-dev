
#include "sentry_intelligence.hpp"

#ifdef TARGET_SENTRY


namespace SentryControl{
  

SentryIntelligenceCommand::SentryIntelligenceCommand(
    src::Drivers* drivers,
    src::Gimbal::GimbalSubsystem* gimbal,
    src::Feeder::FeederSubsystem* feeder,
    src::Chassis::ChassisSubsystem* chassis,
    src::Shooter::ShooterSubsystem* shooter,
    src::Utils::RefereeHelperTurreted* refHelp,
    
    SentryMatchStates& chassisState)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      gimbal(gimbal),
      feeder(feeder),
      chassis(chassis){
      //autoNavCommand(drivers, chassis, src::Chassis::defaultLinearConfig, src::Chassis::defaultRotationConfig, src::Chassis::SnapSymmetryConfig) {
  addSubsystemRequirement(chassis);
  this->comprisedCommandScheduler.registerSubsystem(chassis);
  addSubsystemRequirement(feeder);
  this->comprisedCommandScheduler.registerSubsystem(feeder);
  addSubsystemRequirement(gimbal);
  this->comprisedCommandScheduler.registerSubsystem(gimbal);

}


  

void SentryIntelligenceCommand::initialize() {


}
   


void SentryIntelligenceCommand::execute() {


  
  if (sentryState != lastSentryState) {
    // Perform setup for the next state
    switch (sentryState) {
      case SentryMatchStates::PUSH:
       
        break;
      case SentryMatchStates::ATTACK:
        
        break;
      case SentryMatchStates::RETREAT:
        
        break;
      case SentryMatchStates::PATROL:
        break;
      case SentryMatchStates::CONTROL:
        break;
    }

    lastSentryState = sentryState;
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

