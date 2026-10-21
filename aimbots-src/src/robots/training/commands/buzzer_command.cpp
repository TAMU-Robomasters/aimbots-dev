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
    tap::buzzer::silenceBuzzer(&drivers->pwm);
    noteOn = false;
    lastFrequencyHz = 0;
    // 0 ms: expires right away, so the first execute() starts the first note.
    noteTimer.restart(0);
}

void BuzzerCommand::execute() {
    // Linear map from the stick's -1..1 to [low, high]: low + (stick + 1) / 2 * (high - low).
    float vertical = drivers->dt7Remote.getRightVertical();
    float horizontal = drivers->dt7Remote.getRightHorizontal();
    uint32_t frequencyHz =
        MIN_FREQUENCY_HZ + static_cast<uint32_t>((vertical + 1.0f) / 2.0f * (MAX_FREQUENCY_HZ - MIN_FREQUENCY_HZ));
    uint32_t noteMs = MIN_NOTE_MS + static_cast<uint32_t>((horizontal + 1.0f) / 2.0f * (MAX_NOTE_MS - MIN_NOTE_MS));

    if (noteTimer.isExpired()) {
        // Flip between note on and note off, then wait the note length again.
        noteOn = !noteOn;
        if (noteOn) {
            playFrequency(frequencyHz);
        } else {
            tap::buzzer::silenceBuzzer(&drivers->pwm);
        }
        noteTimer.restart(noteMs);
    } else if (noteOn) {
        // Follow the stick mid-note, but only past the deadband: every playNote restarts the waveform.
        uint32_t change = frequencyHz > lastFrequencyHz ? frequencyHz - lastFrequencyHz : lastFrequencyHz - frequencyHz;
        if (change > FREQUENCY_DEADBAND_HZ) {
            playFrequency(frequencyHz);
        }
    }
}

void BuzzerCommand::end(bool) {
    tap::buzzer::silenceBuzzer(&drivers->pwm);
    noteOn = false;
    noteTimer.stop();
}

void BuzzerCommand::playFrequency(uint32_t frequencyHz) {
    // Twice: playNote sets the 50% duty before it changes the period, so the first call's duty is
    // based on the OLD period. The second call runs with the new period already set.
    tap::buzzer::playNote(&drivers->pwm, frequencyHz);
    tap::buzzer::playNote(&drivers->pwm, frequencyHz);
    lastFrequencyHz = frequencyHz;
}

bool BuzzerCommand::isFinished() const { return false; }

}  // namespace src::Training

#endif  // TARGET_TRAINING
