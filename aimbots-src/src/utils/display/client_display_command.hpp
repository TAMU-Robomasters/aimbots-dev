#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "client_display_command.hpp"
#include "client_display_subsystem.hpp"
#include "hud_indicator.hpp"

namespace src::utils::display {
ClientDisplayCommand::ClientDisplayCommand(
    tap::Drivers &drivers,
    tap::control::CommandScheduler &commandScheduler,
    ClientDisplaySubsystem &clientDisplay
    // the rest of the commands
    )
    : Command(),
      Protothread(),
      drivers(drivers),
      clientDisplay(clientDisplay),
      addSubsystemsToScheduler(&clientDispaly)
// the rest of the commands
{}

void ClientDisplayCommand::initialize() {
    // the rest of the commands
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run() {
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refserial.getRefSerialRecivingState());

    // PT_CALL(someIndcator.sendInitialGraphics());

    while (true) {
        // PT_CALL(someIndcator.update());

        PT_YIELD();
    }
    PT_END();
}

}  // namespace src::utils::display
