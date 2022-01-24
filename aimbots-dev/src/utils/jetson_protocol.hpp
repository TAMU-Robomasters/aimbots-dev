#pragma once
#include <cstdint>

enum CVState : uint8_t {
    CV_STATE_PATROL = 0,
    CV_STATE_FIRE   = 1,
};

// We need to get rid of any struct padding, so we tell the compiler not to pad
// it by giving it the `packed` attribute.
struct __attribute__((packed)) JetsonMessage {
    CVState cvState;
    float   yaw;
    float   pitch;
    float   targetYawOffset;
    float   targetPitchOffset;
    float   horizontalSpeed;
    float   forwardSpeed;
};

struct ChassisPacket {
    CVState cvState;
    float   horizontalSpeed;
    float   forwardSpeed;
};

struct GimbalPacket {
    CVState cvState;
    float   yaw;
    float   pitch;
    float   targetYawOffset;
    float   targetPitchOffset;
};