#pragma once

namespace src::Informants::SupperCap {

static constexpr uint8_t SUPPER_CAP_MESSAGE_RECIEVED_MAGIC = 'c';

enum SupperCapCommand : char {
    CHARGE = 'c',
    DISCHARGE = 'd',
    STOP = 's'
};

// change this for ASCII data
// we don't yet know what format supercap will be sending back
struct SupperCapMessageRecieved {
    uint8_t magic;
    float voltage;
    float power;
    float percent;
    float inputPower;
      // ms
    // CVState cvState;
} __attribute__((packed));

//sent command
struct SupperCapMessageSent {
    uint8_t magic;
    char command;
    uint8_t charge;
} __attribute__((packed));

// reminder that floats are 4 bytes :)
static_assert(sizeof(SupperCapMessageRecieved) == 17, "Supper Cap Message Recived is not the correct size");
static_assert(sizeof(SupperCapMessageSent) == 3, "Supper Cap Message Sent is not the correct size");

static constexpr size_t SUPPER_CAP_MESSAGE_SIZE = sizeof(SupperCapMessageRecieved);
static constexpr size_t SUPPER_CAP_MESSAGE_SENT_SIZE = sizeof(SupperCapMessageSent);
}  // namespace src::Informants::SupperCap