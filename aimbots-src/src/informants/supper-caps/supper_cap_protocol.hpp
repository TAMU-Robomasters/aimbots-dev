#pragma once

namespace src::Informants::SupperCap {

static constexpr uint8_t SUPPER_CAP_MESSAGE_RECIEVED_MAGIC = 'c';

enum SupperCapCommand : char { CHARGE = 'c', DISCHARGE = 'd', STOP = 's' };

// change this for ASCII data
// we don't yet know what format supercap will be sending back
struct SupperCapMessageRecieved {
    uint8_t magic;
    float voltage;
    float power;
    float percent;
    float inputPower;
    uint8_t delay = 0;  // this is to help balance out the size of the message
} __attribute__((packed));

// sent command
struct SupperCapMessageSent {
    uint8_t magic;
    char command;
    float charge;
} __attribute__((packed));

// reminder that floats are 4 bytes :)
static_assert(sizeof(SupperCapMessageRecieved) == 18, "Supper Cap Message Recived is not the correct size");
static_assert(sizeof(SupperCapMessageSent) == 6, "Supper Cap Message Sent is not the correct size");

static constexpr uint8_t SUPPER_CAP_MESSAGE_SIZE = sizeof(SupperCapMessageRecieved);
static constexpr uint8_t SUPPER_CAP_MESSAGE_SENT_SIZE = sizeof(SupperCapMessageSent);
}  // namespace src::Informants::SupperCap