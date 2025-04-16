#pragma once

namespace src::Informants::Vision {

enum CVState : uint8_t {
    NOT_FOUND = 0,
    FOUND = 1,
    FIRE = 2,
};

static constexpr uint8_t JETSON_MESSAGE_MAGIC = 'a';
static constexpr uint8_t DEVBOARD_MESSAGE_MAGIC = 'b';

static constexpr uint8_t RED_TEAM = 0;
static constexpr uint8_t BLUE_TEAM = 1;

static constexpr uint8_t TEST_BENCH_ROBOT_TYPE = 0b00000000;
static constexpr uint8_t SENTRY_ROBOT_TYPE = 0b00000001;
static constexpr uint8_t ENGINEER_MESSAGE_MAGIC = 0b00000010;
static constexpr uint8_t STANDARD_MESSAGE_MAGIC = 0b00000011;
static constexpr uint8_t HERO_MESSAGE_MAGIC = 0b00000100;

struct JetsonMessage {
    uint8_t typeAndTeam;
    uint8_t magic;
    float targetX;
    float targetY;
    float targetZ;
    uint8_t delay;  // ms
    CVState cvState;
} __attribute__((packed));

static_assert(sizeof(JetsonMessage) == 16, "JetsonMessage is not the correct size");

static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);
}  // namespace src::Informants::Vision