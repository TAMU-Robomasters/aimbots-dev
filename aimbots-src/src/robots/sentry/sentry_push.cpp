
#include "sentry_push.hpp"

#ifdef TARGET_SENTRY


namespace SentryControl{
  

SentryPushCommand::SentryPushCommand(
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


  

void SentryPushCommand::initialize() {


}
   



void SentryPushCommand::execute() {
 
   
}

void SentryPushCommand::end(bool interrupted) {

 
}

bool SentryPushCommand::isReady() { return true; }

bool SentryPushCommand::isFinished() const { 
  
  
  return false; 
  }

}


#endif

