#pragma once
#include "utils/common_types.hpp"

namespace src::Communication {
#ifdef TARGET_SENTRY
struct robot_state_message_team {
    /* data */
    uint16_t standardX, standardY, heroX, heroY, sentryX, sentryY;
};

#else
struct robot_state_message {
    /* data */
    uint8_t robotID;
    uint16_t x, y;
};

struct robot_state_message_enemy {
    uint8_t robotID;
    uint16_t x, y;
    uint16_t timestamp;
};

#endif

enum class MessageType : uint8_t {
#ifdef TARGET_SENTRY
    TEAM_MESSAGE_STANDARD,
    TEAM_MESSAGE_HERO,
    TEAM_MESSAGE_SENTRY,
#else
    ROBOT_STATE,
    ENEMY_STATE,
#endif
    NUM_MESSAGE_TYPES,
};

static constexpr uint16_t SENTRY_REQUEST_ROBOT_ID = 0x200;

static constexpr uint16_t SENTRY_RESPONSE_MESSAGE_ID = 0x201;

}  // namespace src::Communication
