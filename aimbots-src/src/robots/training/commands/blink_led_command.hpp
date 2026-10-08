#pragma once

#ifdef TARGET_TRAINING

#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"

#include "robots/training/training_board_subsystem.hpp"
#include "drivers.hpp"

namespace src::Training {

/**
 * @brief WEEK 1: blinks one of the dev board LEDs while the command is scheduled.
 *
 * Fill in the TODOs in blink_led_command.cpp. See training/week1/README.md.
 */
class BlinkLedCommand : public tap::control::Command {
   public:
    BlinkLedCommand(src::Drivers* drivers, TrainingBoardSubsystem* board);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

    const char* getName() const override { return "Blink LED"; }

   private:
    // How long the LED stays in each state (on OR off), in milliseconds.
    static constexpr uint32_t BLINK_PERIOD_MS = 250;

    src::Drivers* drivers;
    TrainingBoardSubsystem* board;

    // TODO(week1): you will need something to keep time without blocking.
    tap::arch::MilliTimeout timer;

    // Look at tap::arch::MilliTimeout in taproot/src/tap/architecture/timeout.hpp.
};

}  // namespace src::Training

#endif  // TARGET_TRAINING
