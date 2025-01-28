
#include "sentry_push_hill.hpp"

#ifdef TARGET_SENTRY


namespace SentryControl{
  

SentryPushHillCommand::SentryPushHillCommand(
    src::Drivers* drivers,
    src::Gimbal::GimbalSubsystem* gimbal,
    src::Feeder::FeederSubsystem* feeder,
    src::Chassis::ChassisSubsystem* chassis,
    src::Shooter::ShooterSubsystem* shooter,
    src::Utils::RefereeHelperTurreted* refHelper)
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


  

void SentryPushHillCommand::initialize() {
  
}
   

void SentryPushHillCommand::execute() {
  
  //move towards the location - autonav
  //look in the direction of the push - gimbal
  //shoot enemies that are spotted - feeder
   
}


void SentryPushHillCommand::end(bool interrupted) {

 
}

bool SentryPushHillCommand::isReady() { return true; }

bool SentryPushHillCommand::isFinished() const { 
  
  
  return false; 
}

  
void SentryPushHillCommand::exit() {
  //---drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::FOUND
  //if no robot is spotted 
  //change sentryState push center
  //if hp<200 
  //change sentryState Retreat
  //enemies are spotted
  //change sentryState ControlHill

}

}


#endif

