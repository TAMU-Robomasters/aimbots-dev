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

    void init();
    void update();
    inline float getCurrent(){return current;};
    inline float getBusVoltage(){return voltage;};
    inline float getPower(){return power;};
    //void setMode(INA260_MeasurementMode mode);
    //INA260_MeasurementMode getMode(void);

    // bool conversionReady(void);
    // bool alertFunctionFlag(void);

    //float getAlertLimit(void);
    //INA260_AlertLatch getAlertLatch(void);
    //void setAlertLatch(INA260_AlertLatch state);
    //INA260_AlertPolarity getAlertPolarity(void);
    //void setAlertPolarity(INA260_AlertPolarity polarity);
    //INA260_AlertType getAlertType(void);
    //void setAlertType(INA260_AlertType alert);

    //INA260_ConversionTime getCurrentConversionTime(void);
    //void setCurrentConversionTime(INA260_ConversionTime time);
    //INA260_ConversionTime getVoltageConversionTime(void);
    //void setVoltageConversionTime(INA260_ConversionTime time);
    //INA260_AveragingCount getAveragingCount(void);
    //void setAveragingCount(INA260_AveragingCount count);


private:
    inline modm::ResumableResult<bool> readRegister(POWER_COM_DATA::Register reg, size_t length = 1)
    {
        RF_BEGIN();

        if(length > 3) length = 3;

        raw_data_buffer[0] = uint8_t(reg);
        while(!transaction.configureWriteRead(raw_data_buffer, 1, raw_data_buffer, length));

        RF_END_RETURN_CALL(runTransaction());
    };

    inline modm::ResumableResult<bool> writeToRegister(POWER_COM_DATA::Register reg, POWER_COM_DATA::Register data) {
        RF_BEGIN();

        raw_data_buffer[0] = uint8_t(reg);
        raw_data_buffer[1] = uint8_t(data);

        while(!transaction.configureWrite(raw_data_buffer, 2));

        RF_END_RETURN_CALL(runTransaction());
    }

    void initializeHardware() {
        while(!readRegister(POWER_COM_DATA::Register::INA260_REG_DIE_UID).getResult());
        modm::delay_ms(1);

        //uint16_t device_id = raw_data_buffer[0];
        // uint8_t mask = ~((1<<0)|(1<<1)|(1<<2)|(1<<3));
        // device_id = device_id & mask;
        //device_id = device_id >> 4;

        //if(device_id == 320)
            isDeviceVerified = true;
        //else
            //isDeviceVerified = false;

        
    }

    bool PT_readingRawData() {
        PT_BEGIN();
        PT_CALL(readRegister(POWER_COM_DATA::Register::INA260_REG_CURRENT));
        current = raw_data_buffer[0];
        PT_CALL(readRegister(POWER_COM_DATA::Register::INA260_REG_BUSVOLTAGE));
        voltage = raw_data_buffer[1];
        PT_CALL(readRegister(POWER_COM_DATA::Register::INA260_REG_POWER));
        power = raw_data_buffer[2];
        
        modm::delay_ms(5);
        
        PT_END();
    }

    uint8_t voltage;
    uint8_t current;
    uint8_t power;

    bool isDeviceVerified;
    uint8_t raw_data_buffer[3];
};
}

