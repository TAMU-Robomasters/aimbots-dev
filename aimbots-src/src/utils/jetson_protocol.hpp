#pragma once
#include <cstdint>

namespace utils {

enum CVState : int {
    CV_STATE_PATROL = 0,
    CV_STATE_FIRE   = 1,
};

static constexpr uint64_t JETSON_MESSAGE_MAGIC = 0xdeadbeefdeadbeef;

struct JetsonMessage {
    uint64_t magic;
    float    targetYawOffset;
    float    targetPitchOffset;
    CVState  cvState;
};

}