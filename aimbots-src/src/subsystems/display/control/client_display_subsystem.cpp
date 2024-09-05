#include "subsystems/display/control/client_display_subsystem.hpp"

#include "tap/drivers.hpp"

namespace src::Utils::ClientDisplay {

ClientDisplaySubsystem::ClientDisplaySubsystem(tap::Drivers* drivers) : Subsystem(drivers) {}

}  // namespace src::Utils::ClientDisplay