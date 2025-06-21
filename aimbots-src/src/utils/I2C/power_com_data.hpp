#pragma once
#include <cstdint>

#include <modm/platform/gpio/gpio_G6.hpp>
#include <modm/platform/gpio/gpio_G3.hpp>
#include <modm/platform/gpio/gpio_F0.hpp>
#include <modm/platform/i2c/i2c_master_2.hpp>

namespace utils::POWER_COM_DATA {

    using DATA_READY_PIN = modm::platform::GpioF0;
    using POWER_COM_MASTER = modm::platform::I2cMaster2;
//clock 100Khz
//adress 0x55
    static constexpr uint8_t I2C_ADRESS = 0x50;

    enum class Register : uint8_t {
        WHO_AM_I = 0x00,
        VOLTAGE = 0x01,
        CURRENT = 0x02,
    };

    enum class RegisterData : uint8_t {
        DEVICE_ID = 0x55,
    };

    

}