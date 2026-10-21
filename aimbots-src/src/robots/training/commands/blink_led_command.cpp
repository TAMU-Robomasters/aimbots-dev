#include "blink_led_command.hpp"

#ifdef TARGET_TRAINING

#include "modm/platform.hpp"

namespace src::Training {


using Led = modm::platform::GpioH12;

BlinkLedCommand::BlinkLedCommand(src::Drivers* drivers, TrainingBoardSubsystem* board)
    : drivers(drivers),
      board(board)  //
{
    // Tells the scheduler this command "owns" the board subsystem while it runs.
    addSubsystemRequirement(board);
}

// Called once, when the command gets scheduled (e.g. you flip the switch).
void BlinkLedCommand::initialize() {
    Led::setOutput();
    Led::set();
    blinkTimer.restart(BLINK_PERIOD_MS);
}

// Called every scheduler loop (every 2 ms) while the command is scheduled.
void BlinkLedCommand::execute() {
    if (blinkTimer.isExpired()) {
        Led::toggle();
        blinkTimer.restart(BLINK_PERIOD_MS);
    }
}

// Called once, when the command stops (e.g. you flip the switch back).
void BlinkLedCommand::end(bool) {
    Led::reset();
    blinkTimer.stop();
}

// Blinking runs until the mapping ends it (switch leaves UP), so it never finishes on its own.
bool BlinkLedCommand::isFinished() const { 
    return false; 
}

}  // namespace src::Training

#endif  // TARGET_TRAINING
