#pragma once

namespace src::Informants::vision {

enum CVState : uint8_t {
    LOOK = 0,
    SHOOT = 1,
};

static constexpr uint8_t JETSON_MESSAGE_MAGIC = 'a';

struct JetsonMessage {
    uint8_t magic;
    float targetYawOffset;
    float targetPitchOffset;
    CVState cvState;
} __attribute__((packed));

static_assert(sizeof(JetsonMessage) == 10, "JetsonMessage is not the correct size");

static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);
}  // namespace src::Informants::vision