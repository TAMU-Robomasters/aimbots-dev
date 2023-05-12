#include "communication_request_subsystem.hpp"

#include "drivers.hpp"

namespace src::Communication {
CommunicationRequestSubsystem::CommunicationRequestSubsystem(src::Drivers *drivers) : Subsystem(drivers), robotTransmiter(drivers) {}

void CommunicationRequestSubsystem::refresh() { robotTransmiter.send(); }
}  // namespace src::Communication