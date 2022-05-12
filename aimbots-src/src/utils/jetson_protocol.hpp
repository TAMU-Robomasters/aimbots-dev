#pragma once
#include <cstdint>

namespace utils {

enum CVState : uint8_t {
    CV_STATE_PATROL = 0,
    CV_STATE_FIRE = 1,
};

static constexpr uint64_t JETSON_MESSAGE_MAGIC = 0xabcdef0123456789;

struct JetsonMessage {
    uint64_t magic;
    float targetYawOffset;
    float targetPitchOffset;
    CVState cvState;
} __attribute__((packed));

static_assert(sizeof(JetsonMessage) == 17);
}