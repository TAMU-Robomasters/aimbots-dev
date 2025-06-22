#pragma once
#include <cstdint>

#include <modm/platform/gpio/gpio_G6.hpp>
#include <modm/platform/gpio/gpio_G3.hpp>
#include <modm/platform/gpio/gpio_F0.hpp>
#include <modm/platform/i2c/i2c_master_2.hpp>

namespace utils::POWER_COM_DATA {

    using DATA_READY_PIN = modm::platform::GpioF0;
    using POWER_COM_MASTER = modm::platform::I2cMaster2;
    static constexpr uint8_t I2C_ADRESS = 0b1000000;

    enum class Register : uint8_t {

        //[READ/WRITE]
        //All-register reset, shunt voltage and bus
        //voltage ADC conversion times and
        //averaging, operating mode.
        CONFIG = 0x00,

        //[READ ONLY]
        //Contains the value of the current flowing
        //through the shunt resistor
        CURRENT = 0x01,

        //[READ ONLY]
        //Bus voltage measurement data. 
        BUS_VOLTAGE = 0x02,

        //[READ ONLY]
        //Contains the value of the calculated
        //power being delivered to the load.
        POWER = 0x03,

    };

    enum class RegisterData : uint8_t {

        MANUFACTURER_ID = 0xFE,

        DIE_ID = 0xFF,

        //[READ/WRITE]
        //Alert configuration and Conversion
        //Ready flag.
        MASK_AND_ENABLE = 0x06,

        //[READ/WRITE]
        //Contains the limit value to compare to
        //the selected Alert function. 
        ALERT_LIMIT = 0x07,
    };

    

}