#include "buzzer_command.hpp"

#ifdef TARGET_TRAINING

#include "tap/communication/sensors/buzzer/buzzer.hpp"

namespace src::Training {

BuzzerCommand::BuzzerCommand(src::Drivers* drivers, TrainingBoardSubsystem* board)
    : drivers(drivers),
      board(board)  //
{
    addSubsystemRequirement(board);
}

void BuzzerCommand::initialize() {
    // TODO(week2): start in a known state (buzzer off, timer started, ...).
}

void BuzzerCommand::execute() {
    // TODO(week2):
    // 1. Read the right stick from YOUR remote: drivers->dt7Remote.getRightVertical() / getRightHorizontal()
    //    (both -1..1). This only compiles after you've added dt7Remote to drivers.hpp.
    // 2. Map vertical -> MIN_FREQUENCY_HZ..MAX_FREQUENCY_HZ, horizontal -> MIN_NOTE_MS..MAX_NOTE_MS.
    // 3. Every L ms flip between "note on" and "note off" (same MilliTimeout pattern as week 1).
    //
    // Play a note:   tap::buzzer::playNote(&drivers->pwm, frequencyHz);
    // Silence:       tap::buzzer::silenceBuzzer(&drivers->pwm);
    //
    // WARNING: do NOT call playNote every execute(). Only call it when a note starts or the
    // frequency changed by more than FREQUENCY_DEADBAND_HZ, and call it TWICE in a row when you do.
    // The README ("buzzer gotchas") explains why.
}

void BuzzerCommand::end(bool) {
    // TODO(week2): leave the buzzer silent.
}

bool BuzzerCommand::isFinished() const { return false; }

}  // namespace src::Training

#endif  // TARGET_TRAINING
