#include "cocker.hpp"

#ifdef COCKER_COMPATIBLE

namespace src::Cocker {

CockerSubsystem::CockerSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
    drivers(drivers)
{
}

void CockerSubsystem::initialize() {
}

// refreshes the velocity PID given the target RPM and the current RPM
void CockerSubsystem::refresh() {

}



}  // namespace src::Indexer

#endif // #ifdef COCKER_COMPATIBLE