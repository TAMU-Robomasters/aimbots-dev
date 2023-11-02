#pragma once

namespace src::Informants::SupperCap {

static constexpr uint8_t SUPPER_CAP_MESSAGE_MAGIC = 'c';

// change this for ASCII data
// we don't yet know what format supercap will be sending back
struct SupperCapMessage {
    uint8_t magic;
    float voltage;
    float current;
    uint8_t delay;  // ms
    // CVState cvState;
} __attribute__((packed));

// reminder that floats are 4 bytes :)
static_assert(sizeof(SupperCapMessage) == 10, "Supper Cap Message is not the correct size");

static constexpr size_t SUPPER_CAP_MESSAGE_SIZE = sizeof(SupperCapMessage);
}  // namespace src::Informants::SupperCap