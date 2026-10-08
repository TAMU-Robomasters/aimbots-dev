#pragma once

#ifdef TARGET_TRAINING

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "robots/training/training_board_subsystem.hpp"
#include "drivers.hpp"

namespace src::Training {

/**
 * @brief WEEK 2: beeps the dev board buzzer, controlled by the RIGHT stick.
 *
 *   stick up / down     -> pitch (frequency)
 *   stick left / right  -> note length (beep on for L ms, off for L ms, repeat)
 *
 * Fill in the TODOs in buzzer_command.cpp. See training/week2/README.md, section 7.
 */
class BuzzerCommand : public tap::control::Command {
   public:
    BuzzerCommand(src::Drivers* drivers, TrainingBoardSubsystem* board);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

    const char* getName() const override { return "Buzzer"; }

   private:
    // Roughly what a ~30 year old can hear: 20 Hz up to ~15 kHz (the top end drops with age).
    static constexpr uint32_t MIN_FREQUENCY_HZ = 20;     // stick fully DOWN
    static constexpr uint32_t MAX_FREQUENCY_HZ = 15000;  // stick fully UP

    static constexpr uint32_t MIN_NOTE_MS = 50;    // stick fully LEFT
    static constexpr uint32_t MAX_NOTE_MS = 1000;  // stick fully RIGHT

    // Ignore frequency changes smaller than this (the sticks jitter a little even when you don't
    // touch them; one stick count is ~11 Hz with a linear map). The README's "buzzer gotchas" say why
    // this matters.
    static constexpr uint32_t FREQUENCY_DEADBAND_HZ = 30;

    src::Drivers* drivers;
    TrainingBoardSubsystem* board;

    // TODO(week2): what do you need to remember between execute() calls?
    // (a timer, whether the buzzer is currently on, the frequency you last sent, ...)
};

}  // namespace src::Training

#endif  // TARGET_TRAINING
