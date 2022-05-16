#pragma once

namespace src::vision {

    enum CVState : uint8_t {
        CV_STATE_PATROL = 0,
        CV_STATE_FIRE = 1,
    };

    typedef enum {
        aimAtTarget = 'a',
    } jetsonMessageTypes;

    static constexpr uint8_t JETSON_END_BYTE = 'e';

    struct JetsonMessage {
        uint8_t header;
        int16_t targetYawOffset;
        int16_t targetPitchOffset;
        CVState cvState;
        uint8_t end;
    } __attribute__((packed));

    static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);

    static_assert(JETSON_MESSAGE_SIZE == 7, "JetsonMessage is not the correct size");
}