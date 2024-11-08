#pragma once

namespace src::Informants::Vision {

enum CVState : uint8_t {
    NOT_FOUND = 0,
    FOUND = 1,
    FIRE = 2,
};

static constexpr uint8_t JETSON_MESSAGE_MAGIC = 'a';

struct JetsonMessage {
    uint8_t magic;
    float targetX;
    float targetY;
    float targetZ;
    float targetVX;
    float targetVY;
    float targetVZ;
    float targetAX;
    float targetAY;
    float targetAZ;
    uint8_t delay;  // ms
    CVState cvState;
} __attribute__((packed));

static_assert(sizeof(JetsonMessage) == 39, "JetsonMessage is not the correct size");

static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);
}  // namespace src::Informants::Vision