#include "client_display_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "client_display_subsystem.hpp"
#include "hud_indicator.hpp"

namespace src::utils::display {
ClientDisplayCommand::ClientDisplayCommand(
    tap::Drivers &drivers,
    tap::control::CommandScheduler &commandScheduler,
    ClientDisplaySubsystem &clientDisplay)
    : Command(),
      refSerialTransmitter(&drivers),
      drivers(drivers),
      commandScheduler(commandScheduler) {
    addSubsystemRequirement(&clientDisplay);
}

void ClientDisplayCommand::initialize() {
    // the rest of the commands
    // initalize each of display commands
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run() {
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());


    // PT_CALL(someIndcator.sendInitialGraphics());

    while (true) {
        // PT_CALL(someIndcator.update());

        PT_YIELD();
    }
    PT_END();
}

}  // namespace src::utils::display
