#pragma once


namespace utils::POWER_COM {

    static constexpr uint8_t POWER_MESSAGE_MAGIC = 'c';

    struct PowerMessage {
        uint8_t magic;
        uint8_t data;
    } __attribute__((packed));

    static_assert(sizeof(PowerMessage) == 2, "Power magic is not the correctsize");

    static constexpr size_t POWER_MESSAGE_SIZE = sizeof(PowerMessage);
    

}