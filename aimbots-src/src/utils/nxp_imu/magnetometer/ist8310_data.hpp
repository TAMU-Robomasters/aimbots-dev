#pragma once
#include <cstdint>

#include <modm/platform/gpio/gpio_G6.hpp>
#include <modm/platform/gpio/gpio_G3.hpp>
#include <modm/platform/i2c/i2c_master_3.hpp>

namespace utils::Ist8310Data {

using RESET_PIN = modm::platform::GpioG6;
using DATA_READY_PIN = modm::platform::GpioG3;
using IST_I2C_MASTER = modm::platform::I2cMaster3;

// All this came from the ist8310 datasheet: https://intofpv.com/attachment.php?aid=8104
static constexpr uint8_t I2C_ADDRESS = 0x0E;

// Convertion from raw data to microteslas (uT)
static constexpr float RAW_TO_uT_FACTOR = 0.3f;

enum class Register : uint8_t {
    // [READ ONLY]
    // Device ID
    WHO_AM_I = 0x00,

    // [READ ONLY]
    // Data status (1 when ready to read)
    STATUS_1 = 0x02,

    // [READ ONLY]
    // Low byte for x-axis data
    X_DATA_LOW = 0x03,

    // [READ ONLY]
    // High byte for x-axis data
    X_DATA_HIGH = 0x04,

    // [READ ONLY]
    // Low byte for y-axis data
    Y_DATA_LOW = 0x05,

    // [READ ONLY]
    // High byte for y-axis data
    Y_DATA_HIGH = 0x06,

    // [READ ONLY]
    // Low byte for z-axis data
    Z_DATA_LOW = 0x07,

    // [READ ONLY]
    // High byte for z-axis data
    Z_DATA_HIGH = 0x08,

    // [READ ONLY]
    // Data status
    STATUS_2 = 0x09,

    // [READ/WRITE]
    // Chip control setting 1
    CONTROL_1 = 0x0A,

    // [READ/WRITE]
    // Chip control setting 2
    CONTROL_2 = 0x0B,

    // [READ ONLY]
    // Low byte for temperature data
    TEMP_DATA_LOW = 0x1C,

    // [READ ONLY]
    // High byte for temperature data
    TEMP_DATA_HIGH = 0x1D,

    // [READ/WRITE]
    // Pulse duration control data
    PULSE_DURATION_CONTROL = 0x42,

    // [READ/WRITE]
    // Data averaging control data
    DATA_AVERAGE_CONTROL = 0x41,
};

enum class RegisterData : uint8_t {
    STAT1_DATA_READY_MASK = 0x01,
    DEVICE_ID = 0x10,
    DATA_READY_FIELDS = 0x08,
    OPERATING_MODE_SINGLE_MEASURE = 0x0B,
    NORMAL_PULSE_DURATION = 0xC0,
    AVERAGE_MODE = 0x09,
};

}