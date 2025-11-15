#pragma once

namespace src::Informants::Vision {

enum CVState : uint8_t {
    NOT_FOUND = 0,
    FOUND = 1,
    FIRE = 2,
};

static constexpr uint8_t JETSON_MESSAGE_MAGIC = 'a'; // header for jetson -> devboard comms
static constexpr uint8_t JETSON_LOCALIZATION_MESSAGE_HEADER = 'l';
// Query messages are just `JETSON_MESSAGE_MAGIC` + uint8_t messageType
static constexpr uint8_t JETSON_ODOMETRY_QUERY = 'q';
static constexpr uint8_t EMBEDDED_MESSAGE_MAGIC = 'b'; // header for devboard -> jetson comms

struct JetsonMessage {
    uint8_t magic;
    uint8_t messageType;
    float targetX; // relative to camera in meters
    float targetY; // relative to camera in meters
    float targetZ; // relative to camera in meters
    float x; // location in meters
    float y; // location in meters
    float theta; // orientation in radians
    CVState  cvState;
    uint8_t delay;  // ms
} __attribute__((packed));

struct EmbeddedOdometryMessage {
    uint8_t magic;
    float x; // location in meters
    float y; // location in meters
    float theta; // orientation in radians
} __attribute__((packed));

static_assert(sizeof(JetsonMessage) == 28, "JetsonMessage is not the correct size");
static_assert(sizeof(EmbeddedOdometryMessage) == 13, "EmbeddedOdometryMessage is not the correct size");



static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);
}  // namespace src::Informants::Vision