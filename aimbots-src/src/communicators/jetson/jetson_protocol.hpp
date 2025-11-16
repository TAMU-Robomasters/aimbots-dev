#pragma once

namespace src::Informants::Vision {

enum CVState : uint8_t {
    NOT_FOUND = 0,
    FOUND = 1,
    FIRE = 2,
};

static constexpr uint8_t JETSON_MESSAGE_MAGIC = 'a'; // header for jetson -> devboard comms
static constexpr uint8_t JETSON_AIM_MESSAGE = 'd';
// Query messages are just `JETSON_MESSAGE_MAGIC` + uint8_t messageType
static constexpr uint8_t JETSON_TRANSFORMATION_QUERY = 't';
static constexpr uint8_t EMBEDDED_MESSAGE_MAGIC = 'b'; // header for devboard -> jetson comms

struct JetsonMessage {
    uint8_t magic;
    uint8_t messageType;
    float pitch;
    float yaw;
    CVState cvState;
} __attribute__((packed));

struct EmbeddedTransformationMessage {
    uint8_t magic;
    float matrix[16];
} __attribute__((packed));

static_assert(sizeof(JetsonMessage) == 11, "JetsonMessage is not the correct size");
static_assert(sizeof(EmbeddedTransformationMessage) == 65, "EmbeddedTransformationMessage is not the correct size");

static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);
}  // namespace src::Informants::Vision