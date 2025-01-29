
#include "sentry_push_state.hpp"

#ifdef TARGET_SENTRY


namespace SentryControl{
  

SentryPushStateCommand::SentryPushStateCommand(
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


  

void SentryPushStateCommand::initialize() {
  
}
   

void SentryPushStateCommand::execute() {
  
  //move towards the location - autonav
  //look in the direction of the push - gimbal
  //shoot enemies that are spotted - feeder
   
}


void SentryPushStateCommand::end(bool interrupted) {

 
}

bool SentryPushStateCommand::isReady() { return true; }

bool SentryPushStateCommand::isFinished() const { 
  
  
  return false; 
}

  
SentryMatchStates SentryPushStateCommand::exit() {
  //---drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::FOUND
  //if no robot is spotted 
  //change sentryState push center
  //if hp<200 
  //change sentryState Retreat
  //enemies are spotted
  //change sentryState ControlHill

  if (drivers->refSerial.getRobotData().currentHp < 200){
    return SentryMatchStates::RETREAT;
  }
  
  return SentryMatchStates::PUSH;
}

}


#endif

