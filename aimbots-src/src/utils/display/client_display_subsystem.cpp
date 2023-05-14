#include "client_display_subsystem.hpp"

#include "tap/drivers.hpp"

namespace src::utils::display {

ClientDisplaySubsystem::ClientDisplaySubsystem(tap::Drivers* drivers) : Subsystem(drivers) {}

}  // namespace src::utils::display