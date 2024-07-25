
#pragma once
#include <array>
#include <tuple>

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
//
#include "modm/math/geometry/polygon_2d.hpp"
#include "modm/math/utils/misc.hpp"
#include "modm/processing/protothread.hpp"
//
 //#include "subsystems/chassis/chassis.hpp"
// #include "subsystems/gimbal/gimbal.hpp"
 //#include "subsystems/hopper/hopper.hpp"

 #include "boolean_hud_indicators.hpp"
// #include "chassis_orientation_indicator.hpp"
// #include "client_display_subsystem.hpp"
// #include "computer_vision_display.hpp"
//#ifndef TARGET_ENGINEER
    #include "reticle_indicator.hpp"
//#endif

 //using namespace src::Hopper;
 //using namespace src::Chassis;
// using namespace src::Gimbal;
// using namespace src::Utils::Ballistics;

namespace tap::control {
class Subsystem;
}

namespace tap {
class Drivers;
}

namespace src::Utils::ClientDisplay {

class ClientDisplaySubsystem;

class ClientDisplayCommand : public tap::control::Command, ::modm::pt::Protothread {
public:
    /**
     * @param[in] drivers Global drivers instance
     * @param[in] commandscheduler CommandScheduler instance
     */
    ClientDisplayCommand(
        tap::Drivers &drivers,
        tap::control::CommandScheduler &commandScheduler,
        ClientDisplaySubsystem &clientDisplay//,
      //   const HopperSubsystem *hopper,
        // const GimbalSubsystem &gimbal,
      //   const ChassisSubsystem &chassis  //,
         //BallisticsSolver &ballisticsSolver
    );

    const char *getName() const override { return "Client Display"; }

    void initialize() override;

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }

private:
    tap::Drivers &drivers;
    tap::control::CommandScheduler &commandScheduler;
    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;

    /* Actual hud elements */
    //BooleanHUDIndicators booleanHudIndicators;
    // ChassisOrientationIndicator chassisOrientation;
    //#ifndef TARGET_ENGINEER
        ReticleIndicator reticleIndicator;
    //#endif
    // CVDisplay cvDisplay;

    bool restarting = true;

    bool run();
    void restartHud();
};

}  // namespace src::Utils::ClientDisplay