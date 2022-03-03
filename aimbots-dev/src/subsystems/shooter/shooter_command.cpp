#include "drivers.hpp"
#include "subsystems/shooter/shooter_command.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"
#include "tap/communication/gpio/leds.hpp"

//#ifndef TARGET_ENGINEER

namespace src::Shooter {

    ShooterCommand::ShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter){
        this->drivers = drivers;
        this->shooter = shooter;
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
    }

    void ShooterCommand::initialize(){
        drivers->leds.set(tap::gpio::Leds::B, true);
        drivers->leds.set(tap::gpio::Leds::C, false);
        drivers->leds.set(tap::gpio::Leds::D, false);
        drivers->leds.set(tap::gpio::Leds::E, true);
        drivers->leds.set(tap::gpio::Leds::F, false);
        drivers->leds.set(tap::gpio::Leds::G, false);
        drivers->leds.set(tap::gpio::Leds::H, true);
        //shooter->initialize();
        // shooter->calculateShooter(100.0f);
        // shooter->setDesiredOutputs();
    }
    
    // set the flywheel to a certain speed once the command is called
    void ShooterCommand::execute(){
        // drivers->leds.set(tap::gpio::Leds::A, true);
        drivers->leds.set(tap::gpio::Leds::B, true);
        drivers->leds.set(tap::gpio::Leds::C, false);
        drivers->leds.set(tap::gpio::Leds::D, false);
        drivers->leds.set(tap::gpio::Leds::E, true);
        drivers->leds.set(tap::gpio::Leds::F, false);
        drivers->leds.set(tap::gpio::Leds::G, false);
        drivers->leds.set(tap::gpio::Leds::H, true);

        shooter->calculateShooter(3000.0f); //3000 is a reasonable speed 
        shooter->setDesiredOutputs();
    }
    void ShooterCommand::end(bool interrupted){
        (void)interrupted;
        shooter->targetRPMs[0] = 0.0f;
        shooter->targetRPMs[1] = 0.0f;
        shooter->setDesiredOutputs();
        
    }

    bool ShooterCommand::isReady(){
        return true;
    }

    bool ShooterCommand::isFinished() const {
        return false;
    }

}//namespace src::Shooter

//#endif //#ifndef TARGET_ENGINEER