#pragma once

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"
#include "power_com_data.hpp"


namespace utils {

class POWER_COM : modm::I2cDevice<POWER_COM_DATA::POWER_COM_MASTER>, modm::pt::Protothread
{
public:
    POWER_COM();

    bool begin();
    void reset(void);
    float readCurrent(void);
    float readBusVoltage(void);
    float readPower(void);
    void setMode(INA260_MeasurementMode mode);
    INA260_MeasurementMode getMode(void);

    bool conversionReady(void);
    bool alertFunctionFlag(void);

    float getAlertLimit(void);
    INA260_AlertLatch getAlertLatch(void);
    void setAlertLatch(INA260_AlertLatch state);
    INA260_AlertPolarity getAlertPolarity(void);
    void setAlertPolarity(INA260_AlertPolarity polarity);
    INA260_AlertType getAlertType(void);
    void setAlertType(INA260_AlertType alert);

    

private:
    inline modm::ResumableResult<bool> readRegister(POWER_COM_DATA::Register reg, size_t length = 1)
    {
        RF_BEGIN();

        if(length > 3) length = 3;

        raw_data_buffer[0] = uint8_t(reg);
        while(!transaction.configureWriteRead(raw_data_buffer, 1, raw_data_buffer, length));

        RF_END_RETURN_CALL(runTransaction());
    };

    inline modm::ResumableResult<bool> writeToRegister(POWER_COM_DATA::Register reg, POWER_COM_DATA::RegisterData data) {
        RF_BEGIN();

        raw_data_buffer[0] = uint8_t(reg);
        raw_data_buffer[1] = uint8_t(data);

        while(!transaction.configureWrite(raw_data_buffer, 2));

        RF_END_RETURN_CALL(runTransaction());
    }

    void initializeHardware() {
        //while(!readRegister(POWER_COM_DATA::Register::WHO_AM_I).getResult());
        modm::delay_ms(1);

        if(raw_data_buffer[0] == uint8_t(POWER_COM_DATA::RegisterData::DIE_ID))
            isDeviceVerified = true;
        else
            isDeviceVerified = false;
    }

    bool PT_readingRawData() {
        PT_BEGIN();
        PT_CALL(readRegister(POWER_COM_DATA::Register::CURRENT, 3));
        modm::delay_ms(5);
        
        PT_END();
    }

    float voltage;
    float current;
    float power;

    bool isDeviceVerified;
    uint8_t raw_data_buffer[3];
};
}

