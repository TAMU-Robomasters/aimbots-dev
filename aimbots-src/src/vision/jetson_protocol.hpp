#pragma once

namespace src::vision {

    enum CVState : uint8_t {
        CV_STATE_PATROL = 0,
        CV_STATE_FIRE = 1,
    };

    typedef enum {
        aimAtTarget = 'a',
    } jetsonMessageTypes;

    static constexpr uint64_t JETSON_MESSAGE_MAGIC = 0xdeadbeefdeadbef;

    static constexpr uint8_t JETSON_END_BYTE = 'e';

    struct JetsonMessage {
        uint64_t magic;
        float targetYawOffset;
        float targetPitchOffset;
        CVState cvState;
    } __attribute__((packed));

    static constexpr size_t JETSON_MESSAGE_SIZE = sizeof(JetsonMessage);

    static_assert(JETSON_MESSAGE_SIZE == 17, "JetsonMessage is not the correct size");
}