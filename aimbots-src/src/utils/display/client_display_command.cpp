#include "client_display_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

 //#include "subsystems/chassis/chassis.hpp"
// #include "subsystems/gimbal/gimbal.hpp"
// #include "subsystems/hopper/hopper.hpp"

#include "client_display_subsystem.hpp"
#include "hud_indicator.hpp"
//#ifndef TARGET_ENGINEER
    #include "reticle_indicator.hpp"
//#endif

// using namespace src::Hopper;
// using namespace src::Chassis;
// using namespace src::Gimbal;
// using namespace src::Utils::Ballistics;

using namespace tap::control;

namespace src::Utils::ClientDisplay {

ClientDisplayCommand::ClientDisplayCommand(
    tap::Drivers &drivers,
    tap::control::CommandScheduler &commandScheduler,
    ClientDisplaySubsystem &clientDisplay//,
//     const HopperSubsystem *hopper,
    // const GimbalSubsystem &gimbal,
 //    const ChassisSubsystem &chassis
    )
    : Command(),
      drivers(drivers),
      commandScheduler(commandScheduler),
      refSerialTransmitter(&drivers),
    //  booleanHudIndicators( commandScheduler, refSerialTransmitter, hopper, chassis),
      /*chassisOrientation(drivers, refSerialTransmitter, gimbal),*/
    //#ifndef TARGET_ENGINEER
        reticleIndicator(drivers, refSerialTransmitter)  //,
    //#endif
     /*cvDisplay(refSerialTransmitter, ballisticsSolver)  */
{
    addSubsystemRequirement(&clientDisplay);
    this->restartHud();
}

void ClientDisplayCommand::initialize() { this->restarting = true; }

void ClientDisplayCommand::restartHud()
{
    // add more indicators here in the future when restart occurs
    HudIndicator::resetGraphicNameGenerator();
   // booleanHudIndicators.initialize();
    //#ifndef TARGET_ENGINEER
        reticleIndicator.initialize();
   // #endif

    this->restarting = false;
}

void ClientDisplayCommand::execute() { run(); }

bool ClientDisplayCommand::run() 
{

    if (!this->isRunning())
    {
        // force thread restart
        restart();
        // reset elements
        this->restartHud();
    }
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());
  //  PT_CALL(booleanHudIndicators.sendInitialGraphics());
   // #ifndef TARGET_ENGINEER
        PT_CALL(reticleIndicator.sendInitialGraphics());
    //#endif

    while (!this->restarting)
    {
        // PT_CALL(chassisOrientation.sendInitialGraphics());
        // PT_CALL(cvDisplay.sendInitialGraphics());
      //   PT_CALL(booleanHudIndicators.update());
     // #ifndef TARGET_ENGINEER
            PT_CALL(reticleIndicator.update());
       // #endif
         PT_YIELD();
        
    }

    PT_END();
}

}  // namespace src::Utils::ClientDisplay
