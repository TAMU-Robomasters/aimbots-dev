#include "client_display_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "client_display_subsystem.hpp"
#include "reticle_indicator.hpp"

using namespace tap::control;

namespace src::Utils::ClientDisplay {

ClientDisplayCommand::ClientDisplayCommand(
    tap::Drivers &drivers,
    tap::control::CommandScheduler &commandScheduler,
    ClientDisplaySubsystem &clientDisplay)
    : Command(),
      drivers(drivers),
      commandScheduler(commandScheduler),
      refSerialTransmitter(&drivers),
      reticleIndicator(drivers, refSerialTransmitter) {
    addSubsystemRequirement(&clientDisplay);
}

void ClientDisplayCommand::initialize() { this->restarting = true; }

void ClientDisplayCommand::restartDisplay() {
    // the rest of the commands
    // initalize each of display commands
    HudIndicator::resetGraphicNameGenerator();
    restart();
    reticleIndicator.initialize();

    this->restarting = false;
}

bool isHUDRunningDisplay = false;

void ClientDisplayCommand::execute() {
    run();
    isHUDRunningDisplay = true;
}

bool ClientDisplayCommand::run() {
    if (!this->isRunning()) {
        restart();
        this->restartDisplay();
    }

    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());

    // PT_CALL(chassisOrientation.sendInitialGraphics());
    // PT_CALL(cvDisplay.sendInitialGraphics());
    // PT_CALL(booleanHudIndicators.sendInitialGraphics());
    PT_CALL(reticleIndicator.sendInitialGraphics());

    // If a restart is attempted, stop updating
    while (!this->restarting) {
        // PT_CALL(chassisOrientation.update());
        // PT_CALL(cvDisplay.update());
        // PT_CALL(booleanHudIndicators.update());
        PT_CALL(reticleIndicator.update());
        PT_YIELD();
    }
    PT_END();
}

}  // namespace src::Utils::ClientDisplay
