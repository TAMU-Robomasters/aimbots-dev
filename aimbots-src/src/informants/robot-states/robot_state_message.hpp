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
    uint8_t robotID;
    uint32_t x, y;
};

struct robt_state_message_enemy {
    uint8_t robotID;
    uint32_t x, y;
    uint16_t timestamp;
};

#endif
}  // namespace src::robotStates
