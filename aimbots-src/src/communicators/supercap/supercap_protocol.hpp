#pragma once

namespace src::Communicators::SuperCap {

static constexpr uint8_t SUPERCAP_MESSAGE_RECIEVED_MAGIC = 'c';

enum SuperCapCommand : char { CHARGE = 'c', DISCHARGE = 'd', STOP = 's' };

// change this for ASCII data
// we don't yet know what format supercap will be sending back
struct SuperCapMessageRecieved {
    uint8_t magic;
    float voltage;
    float power;
    float percent;
    float inputPower;
    uint8_t delay = 0;  // this is to help balance out the size of the message
} __attribute__((packed));

// sent command
struct SuperCapMessageSent {
    uint8_t magic;
    char command;
    float charge;
} __attribute__((packed));

// reminder that floats are 4 bytes :)
static_assert(sizeof(SuperCapMessageRecieved) == 18, "Super Cap Message Recived is not the correct size");
static_assert(sizeof(SuperCapMessageSent) == 6, "Super Cap Message Sent is not the correct size");

static constexpr uint8_t SUPERCAP_MESSAGE_SIZE = sizeof(SuperCapMessageRecieved);
static constexpr uint8_t SUPERCAP_MESSAGE_SENT_SIZE = sizeof(SuperCapMessageSent);
}  // namespace src::Communicators::SuperCap