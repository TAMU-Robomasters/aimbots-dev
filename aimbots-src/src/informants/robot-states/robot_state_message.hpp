#pragma once
#include "utils/common_types.hpp"

namespace src::robotStates {
#ifdef TARGET_SENTRY
struct robot_state_message_team {
    /* data */
    uint32_t standardX, standardY, heroX, heroY, sentryX, sentryY;
};

#else
struct robot_state_message {
    /* data */
    unit8_t robotID;
    unit32_t x, y;
};

struct robt_state_message_enemy {
    unit8_t robotID;
    unit32_t x, y;
    unit16_t timestamp;
}

#endif
}  // namespace src::robotStates
