#pragma once
#include <algorithm>

namespace src::Informants::Vision {

enum CVState : uint8_t {
    NOT_FOUND = 0,
    FOUND = 1,
    FIRE = 2,
};

static constexpr uint8_t JETSON_MESSAGE_MAGIC = 'a'; // header for jetson -> devboard comms
static constexpr uint8_t EMBEDDED_MESSAGE_MAGIC = 'b'; // header for devboard -> jetson comms

static constexpr uint8_t JETSON_AIM_MESSAGE = 'd';
// Query messages are just `JETSON_MESSAGE_MAGIC` + uint8_t messageType
static constexpr uint8_t JETSON_TRANSFORMATION_QUERY = 't';

static constexpr uint8_t JETSON_LOCALIZATION_MESSAGE = 'l';
// Query messages are just `JETSON_MESSAGE_MAGIC` + uint8_t messageType
static constexpr uint8_t JETSON_ODOMETRY_QUERY = 'q';

struct JetsonAimMessage {
    uint8_t magic;
    uint8_t messageType;
    float pitch;
    float yaw;
    uint8_t timeUntilNextFire;
    CVState cvState;
} __attribute__((packed));

struct JetsonLocalizationMessage {
    uint8_t magic;
    uint8_t messageType;
    float x; // location in meters
    float y; // location in meters
    float theta; // orientation in radians
} __attribute__((packed));

struct EmbeddedTransformationMessage {
    uint8_t magic;
    float yaw;
    float pitch;
    float matrix[16];
} __attribute__((packed));

struct EmbeddedOdometryMessage {
    uint8_t magic;
    float x; // location in meters
    float y; // location in meters
    float theta; // orientation in radians
} __attribute__((packed));

static_assert(sizeof(JetsonAimMessage) == 12, "JetsonAimMessage is not the correct size");
static_assert(sizeof(EmbeddedOdometryMessage) == 13, "EmbeddedOdometryMessage is not the correct size");
static_assert(sizeof(EmbeddedTransformationMessage) == 73, "EmbeddedTransformationMessage is not the correct size");

static constexpr size_t JETSON_AIM_MESSAGE_SIZE = sizeof(JetsonAimMessage);
static constexpr size_t JETSON_LOCALIZATION_MESSAGE_SIZE = sizeof(JetsonLocalizationMessage);

static constexpr size_t JETSON_MAX_MESSAGE_SIZE = std::max(JETSON_AIM_MESSAGE_SIZE, JETSON_LOCALIZATION_MESSAGE_SIZE);
}  // namespace src::Informants::Vision