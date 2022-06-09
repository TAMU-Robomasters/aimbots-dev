#pragma once

#include <modm/processing/resumable.hpp>
#include <modm/architecture/interface/i2c_device.hpp>

#include "ist8310_data.hpp"

namespace utils {

class Ist8310 : modm::I2cDevice<Ist8310Data::IST_I2C_MASTER> {
   public:
    Ist8310();

    void init();
    void update();

    inline float getX() { return x; };
    inline float getY() { return y; };
    inline float getZ() { return z; };

   private:
    inline modm::ResumableResult<bool> readRegister(Ist8310Data::Register reg) {
        RF_BEGIN();

        raw_data_buffer[0] = uint8_t(reg);
        transaction.configureWriteRead(raw_data_buffer, 1, raw_data_buffer, 1);

        RF_END_RETURN_CALL(runTransaction());
    }

    inline modm::ResumableResult<bool> writeToRegister(Ist8310Data::Register reg, uint8_t data) {
        RF_BEGIN();

        raw_data_buffer[0] = uint8_t(reg);
        raw_data_buffer[1] = data;

        transaction.configureWrite(raw_data_buffer, 2);

        RF_END_RETURN_CALL(runTransaction());
    }

    inline uint8_t readDeviceID() {
        readRegister(Ist8310Data::Register::WHO_AM_I);
        modm::delay_us(250);
        return raw_data_buffer[0];
    }

    inline uint8_t readStatus1() {
        readRegister(Ist8310Data::Register::TEMP_DATA_LOW);
        modm::delay_us(250);
        return raw_data_buffer[0];
    }

    inline bool canReadAxisData() {
        readRegister(Ist8310Data::Register::STATUS_1);

        // Don't continue if the data-ready bit isn't set
        if (!(raw_data_buffer[0] & uint8_t(Ist8310Data::RegisterData::STAT1_DATA_READY_MASK)))
            return false;

        return true;
    }

    inline void setOperatingMode() {
        uint8_t output = uint8_t(Ist8310Data::RegisterData::OPERATING_MODE_SINGLE_MEASURE);
        writeToRegister(Ist8310Data::Register::CONTROL_1, output);
        modm::delay_us(250);
    }

    inline void enableDataReadyParameters() {
        uint8_t output = uint8_t(Ist8310Data::RegisterData::DATA_READY_FIELDS);
        writeToRegister(Ist8310Data::Register::CONTROL_2, output);
        modm::delay_us(250);
    }

    inline void setDataAverageParameters() {
        uint8_t output = uint8_t(Ist8310Data::RegisterData::AVERAGE_MODE);
        writeToRegister(Ist8310Data::Register::DATA_AVERAGE_CONTROL, output);
        modm::delay_us(250);
    }

    inline void setDefaultPulseDuration() {
        uint8_t output = uint8_t(Ist8310Data::RegisterData::NORMAL_PULSE_DURATION);
        writeToRegister(Ist8310Data::Register::PULSE_DURATION_CONTROL, output);
        modm::delay_us(250);
    }

    float x;
    float y;
    float z;

    bool isDeviceVerified;
    uint8_t raw_data_buffer[6];
};
}