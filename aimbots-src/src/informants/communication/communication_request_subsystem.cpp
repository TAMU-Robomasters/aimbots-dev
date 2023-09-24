#include "communication_request_subsystem.hpp"

#include "drivers.hpp"

#ifdef REF_COMM_COMPATIBLE

namespace src::Communication {
CommunicationRequestSubsystem::CommunicationRequestSubsystem(src::Drivers *drivers) : Subsystem(drivers), robotTransmiter(drivers) {}

void CommunicationRequestSubsystem::refresh() { robotTransmiter.send(); }
}  // namespace src::Communication

#endif // #ifdef REF_COMM_COMPATIBLE