#pragma once
#include "utils/common_types.hpp"

namespace src::robotStates {
#ifdef TARGET_SENTRY
struct robot_state_message_team {
    /* data */
    int robotId;
    Matrix<float, 3, 1> position;
    int health;
};

struct robt_state_message_enemy {
    int robotID;
    Matrix<float, 3, 1> lastPosition;
    bool found;
}
#else
struct robot_state_message {
    /* data */
    Matrix<float, 3, 1> position;
    int health;
};

#endif
}  // namespace src::robotStates
