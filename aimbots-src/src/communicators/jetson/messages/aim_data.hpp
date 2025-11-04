#pragma once

#include <cstdint>
#include "drivers.hpp"
#include "communicators/jetson/jetson_message.hpp"

#ifdef JETSON_COMPATIBLE

namespace src::Communication {

enum CVState : uint8_t {
    NOT_FOUND = 0,
    FOUND = 1,
    FIRE = 2,
};

struct AimMessage {
    uint8_t magic;
    float targetX;
    float targetY;
    float targetZ;
    uint8_t delay;  // ms
    CVState cvState;
} __attribute__((packed)); // tell compiler to not adding padding

class AimData : public DataFromJetson<AimMessage> {
public:
    AimData(src::Drivers* driver, uint8_t messageID);
};

} // namespace src::Communication

#endif // #ifdef JETSON_COMPATIBLE