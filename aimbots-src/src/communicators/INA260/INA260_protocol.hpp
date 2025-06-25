#pragma once

#ifndef ALL_SENTRIES

namespace src::Informants::INA260 {

static constexpr uint8_t INA260_MESSAGE_MAGIC = 'c';

struct INA260Message {
    uint8_t magic;
    float power;
    float voltage_mV;
    float current_mA;
} __attribute__((packed));

static_assert(sizeof(INA260Message) == 13, "INA260Message is not the correct size");

static constexpr size_t INA260_MESSAGE_SIZE = sizeof(INA260Message);
}  // namespace src::Informants::INA260
#endif // ALL_SENTRIES