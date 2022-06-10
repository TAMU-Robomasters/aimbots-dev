#pragma once

#include <modm/processing/resumable.hpp>
#include <modm/processing/protothread.hpp>
#include <modm/architecture/interface/i2c_device.hpp>

#include "ist8310_data.hpp"

namespace utils {

class Ist8310 : modm::I2cDevice<Ist8310Data::IST_I2C_MASTER>, modm::pt::Protothread {
   public:
    Ist8310();

    void init();
    void update();

    inline float getX() { return x; };
    inline float getY() { return y; };
    inline float getZ() { return z; };

   private:
    inline modm::ResumableResult<bool> readRegister(Ist8310Data::Register reg, size_t length = 1) {
        RF_BEGIN();

        // Getting a segfault would be really annoying, so let's not do that...
        if(length > 6) length = 6;

        raw_data_buffer[0] = uint8_t(reg);
        while(!transaction.configureWriteRead(raw_data_buffer, 1, raw_data_buffer, length));

        RF_END_RETURN_CALL(runTransaction());
    }

    inline modm::ResumableResult<bool> writeToRegister(Ist8310Data::Register reg, Ist8310Data::RegisterData data) {
        RF_BEGIN();

        raw_data_buffer[0] = uint8_t(reg);
        raw_data_buffer[1] = uint8_t(data);

        while(!transaction.configureWrite(raw_data_buffer, 2));

        RF_END_RETURN_CALL(runTransaction());
    }

    void initializeHardware() {
        while(!readRegister(Ist8310Data::Register::WHO_AM_I).getResult());
        modm::delay_ms(1);

        if(raw_data_buffer[0] == uint8_t(Ist8310Data::RegisterData::DEVICE_ID))
            isDeviceVerified = true;
        else
            isDeviceVerified = false;

        while(!writeToRegister(Ist8310Data::Register::CONTROL_2, Ist8310Data::RegisterData::DATA_READY_FIELDS).getResult());
        modm::delay_ms(1);
        while(!writeToRegister(Ist8310Data::Register::DATA_AVERAGE_CONTROL, Ist8310Data::RegisterData::AVERAGE_MODE).getResult());
        modm::delay_ms(1);
        while(!writeToRegister(Ist8310Data::Register::PULSE_DURATION_CONTROL, Ist8310Data::RegisterData::NORMAL_PULSE_DURATION).getResult());
        modm::delay_ms(1);
        while(!writeToRegister(Ist8310Data::Register::CONTROL_1, Ist8310Data::RegisterData::OPERATING_MODE_SINGLE_MEASURE).getResult());
        modm::delay_ms(1);
    }

    bool PT_readingRawAxisData() {
        PT_BEGIN();

        PT_CALL(readRegister(Ist8310Data::Register::X_DATA_LOW, 6));
        modm::delay_ms(5);

        PT_END();
    }

    float x;
    float y;
    float z;

    bool isDeviceVerified;
    uint8_t raw_data_buffer[6];
};
}