#pragma once

namespace src::Informants::vision {

enum CVState : uint8_t {
    CV_STATE_UNSURE = 0,
    CV_STATE_FOUND = 1,
};

static constexpr uint64_t JETSON_MESSAGE_MAGIC = 0xdeadbeefdeadbef;

struct JetsonMessage {
    uint64_t magic;
    float targetYawOffset;
    float targetPitchOffset;
    CVState cvState;
} __attribute__((packed));

static_assert(sizeof(JetsonMessage) == 17, "JetsonMessage is not the correct size");

static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);
}  // namespace src::Informants::vision