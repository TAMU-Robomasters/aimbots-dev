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

struct robot_state_message_enemy {
    uint8_t robotID;
    uint32_t x, y;
    uint16_t timestamp;
};

#endif

enum class MessageType : uint8_t {
#ifdef TARGET_SENTRY
    TEAM_MESSAGE_STANDARD,
    TEAM_MESSAGE_HERO,
#else
    ROBOT_STATE,
    ENEMY_STATE,
#endif
    NUM_MESSAGE_TYPES,
};

}  // namespace src::robotStates
