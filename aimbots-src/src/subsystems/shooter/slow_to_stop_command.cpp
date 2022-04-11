#include "subsystems/shooter/slow_to_stop_command.hpp"

#include "drivers.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Shooter {

SlowToStopCommand::SlowToStopCommand(src::Drivers* drivers, ShooterSubsystem* shooter) : TapComprisedCommand(drivers),
                                                                                         brake_command(drivers, shooter),
                                                                                         stop_command(drivers, shooter) {
    this->comprisedCommandScheduler.registerSubsystem(shooter);
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void SlowToStopCommand::initialize() {
    this->comprisedCommandScheduler.addCommand(&brake_command);
    brakeFinished = false;
}

// Run the brake command until it's finished, then run the stop command to keep the flywheels at 0 speed
void SlowToStopCommand::execute() {
    if (!this->comprisedCommandScheduler.isCommandScheduled(&brake_command) && !brakeFinished) {
        brakeFinished = true;
        this->comprisedCommandScheduler.addCommand(&stop_command);
    }
    this->comprisedCommandScheduler.run();  // taproot docs say this is safe 👍
}

void SlowToStopCommand::end(bool) {
}

bool SlowToStopCommand::isReady() {
    return true;
}

// this command will auto-deschedule when we want to move
bool SlowToStopCommand::isFinished() const {
    return false;
}

}  // namespace src::Shooter
