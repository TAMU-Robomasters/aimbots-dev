#pragma once

#include "tap/control/governor/command_governor_interface.hpp"

#include "utils/ref_system/ref_helper_interface.hpp"
#include "utils/tools/common_types.hpp"

namespace src::Utils {

// Governor that gates a command on the referee system reporting the match is live.
// isReady() is true only once the game has started (GameStage::IN_GAME); isFinished() fires
// whenever the game is not in progress. Pair with a GovernorWithFallbackCommand so a pre-game
// command runs until the match starts, then hands off to the in-game command.
class GameStartedGovernor : public tap::control::governor::CommandGovernorInterface {
public:
    GameStartedGovernor(RefereeHelperInterface* refHelper) : refHelper(refHelper) {}

    bool isReady() override { return refHelper->getGameStage() == GamePeriod::IN_GAME; }

    bool isFinished() override { return refHelper->getGameStage() != GamePeriod::IN_GAME; }

private:
    RefereeHelperInterface* refHelper;
};

}  // namespace src::Utils
