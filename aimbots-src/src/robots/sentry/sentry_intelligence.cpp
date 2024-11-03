#include "sentry_intelligence.hpp"

#ifdef TARGET_SENTRY


namespace SentryControl{

SentryIntelligenceCommand::SentryIntelligenceCommand(
    src::Drivers* drivers) 
    : TapComprisedCommand(drivers),
      drivers(drivers)
      
{
   
}


  

void SentryIntelligenceCommand::initialize() {
  float previous_HP= 5400 //starting HP
  float previous_time = 300 // 5:00 in seconds
  float previous_Ammo = 750 // 750 starting ammo
   
}


void SentryIntelligenceCommand::execute() {

  current_HP = drivers->refSerial.getRobotData().current_HP
  current_Ammo = drivers->refSerial.getRobotData().current_HP
  current_Time = drivers->ref_system.getRobotData().current_Time

  proportional_Ammo = (previous_Ammo-current_Ammo)/(previous_time-current_Time)
  proportional_HP = (previous_HP-current_HP)/(previous_time-current_Time)

  if target found and with in range
  //feed proportionaly
  //else
  //look for target

  //if health loss is high
  // evade
  //else if heatlh loss is low
  // attack
  
   
}

void SentryIntelligenceCommand::end(bool interrupted) {
    


}

bool SentryIntelligenceCommand::isReady() { return true; }

bool SentryIntelligenceCommand::isFinished() const { 
  //when time == 0 or health = 0
  
  return false; 
  }

}


#endif