#pragma once
#include "utils/common_types.hpp"

#ifdef REF_COMM_COMPATIBLE

namespace src::Communication {

enum class MessageType : uint8_t {
    PATROL_LOCATION = 0,
    FOLLOW =1,
    RETURN_TO_BASE = 2,
    
};

static constexpr uint16_t SENTRY_REQUEST_ROBOT_ID = 0x200;
static constexpr uint16_t SENTRY_RESPONSE_MESSAGE_ID = 0x201;

static constexpr uint16_t STANDARD_REQUEST_ROBOT_ID = 0x200;
static constexpr uint16_t STANDARD_RESPONSE_MESSAGE_ID = 0x201;
}  // namespace src::Communication

#endif  // #ifdef REF_COMM_COMPATIBLE