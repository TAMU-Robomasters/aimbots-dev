#include "client_display_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "subsystems/hopper/hopper.hpp"

#include "client_display_subsystem.hpp"
#include "hud_indicator.hpp"
#include "reticle_indicator.hpp"

// using namespace src::Hopper;
using namespace src::Chassis;
using namespace src::Gimbal;
using namespace src::Utils::Ballistics;

using namespace tap::control;

namespace src::Utils::ClientDisplay {

ClientDisplayCommand::ClientDisplayCommand(
    tap::Drivers &drivers,
    tap::control::CommandScheduler &commandScheduler,
    ClientDisplaySubsystem &clientDisplay,
    // const HopperSubsystem *hopper,
    //const GimbalSubsystem &gimbal,
    const ChassisSubsystem &chassis)
    : Command(),
      drivers(drivers),
      commandScheduler(commandScheduler),
      refSerialTransmitter(&drivers),
      booleanHudIndicators(commandScheduler, refSerialTransmitter, /*hopper,*/ chassis),
      /*chassisOrientation(drivers, refSerialTransmitter, gimbal),*/
      reticleIndicator(drivers, refSerialTransmitter)  //,
     /*cvDisplay(refSerialTransmitter, ballisticsSolver)  */
{
    addSubsystemRequirement(&clientDisplay);
}

void ClientDisplayCommand::initialize() {
    // the rest of the commands
    // initalize each of display commands
    HudIndicator::resetGraphicNameGenerator();
    restart();
    // chassisOrientation.initialize();
    // cvDisplay.initialize();
    booleanHudIndicators.initialize();
    reticleIndicator.initialize();
}

bool isCommandRunningDisplay = false;

void ClientDisplayCommand::execute() { run(); isCommandRunningDisplay = true; }

bool ClientDisplayCommand::run() {
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());



    // PT_CALL(chassisOrientation.sendInitialGraphics());
    // PT_CALL(cvDisplay.sendInitialGraphics());
    PT_CALL(booleanHudIndicators.sendInitialGraphics());
    PT_CALL(reticleIndicator.sendInitialGraphics());
    while (true) {
        // PT_CALL(chassisOrientation.update());
        // PT_CALL(cvDisplay.update());
        PT_CALL(booleanHudIndicators.update());
        PT_CALL(reticleIndicator.update());
        PT_YIELD();
    }
    PT_END();
}

}  // namespace src::Utils::ClientDisplay
