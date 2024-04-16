#include "cocker.hpp"

#ifdef COCKER_COMPATIBLE

namespace src::Cocker {

CockerSubsystem::CockerSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    cockerMotors{
        buildMotor(ONE),
        buildMotor(TWO)
    },
    motorPIDs{
        SmoothPID(COCKER_POSITION_PID_CONFIG),
        SmoothPID(COCKER_POSITION_PID_CONFIG)
    }
{
}

void CockerSubsystem::initialize() {
    ForAllCockerMotors(&DJIMotor::initialize);
}

// refreshes the velocity PID given the target RPM and the current RPM
void CockerSubsystem::refresh() {
    ForAllCockerMotors(&CockerSubsystem::setDesiredOutputToMotor);
}



}  // namespace src::Indexer

#endif // #ifdef COCKER_COMPATIBLE