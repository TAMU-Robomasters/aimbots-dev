#include "subsystems/shooter/slow_to_stop_command.hpp"

#include "drivers.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

//#ifndef TARGET_ENGINEER

namespace src::Shooter {

SlowToStopCommand::SlowToStopCommand(src::Drivers* drivers, ShooterSubsystem* shooter) : TapComprisedCommand(drivers),
                                                                                         brake_command(drivers, shooter),
                                                                                         stop_command(drivers, shooter) {
    this->comprisedCommandScheduler.registerSubsystem(shooter);
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void SlowToStopCommand::initialize() {
    this->comprisedCommandScheduler.addCommand(&brake_command);
}

// Run the brake command until it's finished, then run the stop command to keep the flywheels at 0 speed
void SlowToStopCommand::execute() {
}

void SlowToStopCommand::end(bool) {
}

bool SlowToStopCommand::isReady() {
    return true;
}

bool SlowToStopCommand::isFinished() const {
    return false;
}

}  // namespace src::Shooter

//#endif //#ifndef TARGET_ENGINEER