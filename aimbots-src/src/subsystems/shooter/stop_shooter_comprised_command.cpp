#include "stop_shooter_comprised_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"


namespace src::Shooter {

StopShooterComprisedCommand::StopShooterComprisedCommand(src::Drivers* drivers, ShooterSubsystem* shooter)
    : TapComprisedCommand(drivers),
      brake_command(drivers, shooter, 500.0f),
      stop_command(drivers, shooter) {
    this->comprisedCommandScheduler.registerSubsystem(shooter);
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void StopShooterComprisedCommand::initialize() {
    this->comprisedCommandScheduler.addCommand(&brake_command);
    brakeFinished = false;
}

// Run the brake command until it's finished, then run the stop command to keep the flywheels at 0 speed
void StopShooterComprisedCommand::execute() {
    if (brake_command.isFinished() && !brakeFinished) {
        brakeFinished = true;
        this->comprisedCommandScheduler.addCommand(&stop_command);
    }
    this->comprisedCommandScheduler.run();  // taproot docs say this is safe 👍
}
/* TAPROOT WIKI COMPRISED COMMAND FORMAT -----------------------\
void ExtendAndGrabCommand::initialize()                         |
{                                                               |
    prevExtendWristFinished = false;                            |
    comprisedCommandScheduler.addCommand(&extendWrist);         |
}                                                               |
                                                                |
void ExtendAndGrabCommand::refresh()                            |
{                                                               |
    if (extendWrist.isFinished() && !prevExtendWristFinished)   |
    {                                                           |
        prevExtendWristFinished = true;                         |
        comprisedCommandScheduler.addCommand(&grabBin);         |
    }                                                           |
    comprisedCommandScheduler.run();                            |
}                                                               |
----------------------------------------------------------------/
*/

void StopShooterComprisedCommand::end(bool) {}

bool StopShooterComprisedCommand::isReady() { return true; }

// this command will auto-deschedule when we want to move
bool StopShooterComprisedCommand::isFinished() const { return false; }

}  // namespace src::Shooter
