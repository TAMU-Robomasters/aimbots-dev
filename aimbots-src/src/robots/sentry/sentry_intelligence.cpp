#include "sentry_intelligence.hpp"

#ifdef TARGET_SENTRY




SentryIntelligenceCommand::SentryIntelligenceCommand(
    src::Drivers* driver) 
    : TapComprisedCommand(drivers),
      drivers(drivers)
{
   
}


void SentryIntelligenceCommand::initialize() {
    
}


void SentryIntelligenceCommand::execute() {
   
}

void SentryIntelligenceCommand::end(bool interrupted) {
    
}

bool SentryIntelligenceCommand::isReady() { return true; }

bool SentryIntelligenceCommand::isFinished() const { return false; }




#endif