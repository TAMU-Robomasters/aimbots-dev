#pragma once

namespace src::Informants::Vision {

enum CVState : uint8_t {
    NOT_FOUND = 0,
    FOUND = 1,
    FIRE = 2,
};

enum CVDetectionType : bool {
    RUNE = false,
    ROBOT = true,
};

static constexpr uint8_t JETSON_MESSAGE_RECEIVE_MAGIC = 'a';
static constexpr uint8_t JETSON_MESSAGE_SEND_MAGIC = 'b';

struct JetsonMessageReceive {
    uint8_t magic;
    float targetX;
    float targetY;
    float targetZ;
    uint8_t delay;  // ms
    CVState cvState;
} __attribute__((packed));

struct JetsonMessageSend{
    uint8_t magic;
    CVDetectionType detectionType;
} __attribute__((packed));

static_assert(sizeof(JetsonMessageReceive) == 15, "JetsonMessageReceive is not the correct size");
static_assert(sizeof(JetsonMessageSend) == 2, "JetsonMessageSend is not the correct size");

static constexpr size_t JETSON_MESSAGE_RECEIVE_SIZE = sizeof(JetsonMessageReceive);
static constexpr size_t JETSON_MESSAGE_SEND_SIZE = sizeof(JetsonMessageSend);
}  // namespace src::Informants::Vision