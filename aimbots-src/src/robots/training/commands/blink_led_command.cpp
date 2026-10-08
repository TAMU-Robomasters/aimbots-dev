#include "blink_led_command.hpp"

#ifdef TARGET_TRAINING

#include "modm/platform.hpp"

namespace src::Training {
using Led = modm::platform::GpioH12;   

// TODO(week1): find which pin the LED you want is wired to in the RoboMaster
// Development Board Type C user manual / schematic, then name it here, e.g.
//     using Led = modm::platform::Gpio??;

BlinkLedCommand::BlinkLedCommand(src::Drivers* drivers, TrainingBoardSubsystem* board)
    : drivers(drivers),
      board(board)  //
{
    // Tells the scheduler this command "owns" the board subsystem while it runs.
    addSubsystemRequirement(board);
}

// Called once, when the command gets scheduled (e.g. you flip the switch).
void BlinkLedCommand::initialize() {
    // TODO(week1): make sure the LED pin is an output, turn the LED on, and start your timer.
    Led::setOutput();
    Led::set();
    timer.restart(BLINK_PERIOD_MS);
}

// Called every scheduler loop (every 2 ms) while the command is scheduled.
void BlinkLedCommand::execute() {
    // TODO(week1): when your timer runs out, toggle the LED and restart the timer.
    // Do NOT use modm::delay here -- it would freeze the whole robot.
    if(timer.isExpired()){
        Led::toggle();
        timer.restart(BLINK_PERIOD_MS);
    }
}

// Called once, when the command stops (e.g. you flip the switch back).
void BlinkLedCommand::end(bool) {
    // TODO(week1): leave the LED off when the command ends.
    Led::reset();
}

// Returning true ends the command. Should a blink command ever finish on its own?
bool BlinkLedCommand::isFinished() const {
    // TODO(week1)
    return false;
}

}  // namespace src::Training

#endif  // TARGET_TRAINING
