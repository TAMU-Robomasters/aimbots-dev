
#include "sentry_attack_state.hpp"

#ifdef TARGET_SENTRY


namespace SentryControl{
  

SentryAttackStateCommand::SentryAttackStateCommand(
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


  

void SentryAttackStateCommand::initialize() {
  
}
   

void SentryAttackStateCommand::execute() {
  
  exit();
  //move towards the location - autonav
  //look in the direction of the push - gimbal
  //shoot enemies that are spotted - feeder
   
}


void SentryAttackStateCommand::end(bool interrupted) {

 
}

bool SentryAttackStateCommand::isReady() { return true; }

bool SentryAttackStateCommand::isFinished() const { 
  
  
  return false; 
}

  
SentryMatchStates SentryAttackStateCommand::exit() {
  if(drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::FOUND){
  }
  //---drivers->cvCommunicator.getLastValidMessage().cvState == src::Informants::Vision::FOUND
  //if no robot is spotted 
  //change sentryState push center
  //if hp<200 
  //change sentryState Retreat
  //enemies are spotted
  //change sentryState ControlHill



  return SentryMatchStates::ATTACK;
}

}


#endif

