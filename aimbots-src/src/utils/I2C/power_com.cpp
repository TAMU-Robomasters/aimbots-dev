#include "power_com.hpp"
#include <cstdint>

#include "modm/platform/gpio/base.hpp"
#include "modm/platform/gpio/gpio_F0.hpp"
#include "modm/platform/gpio/gpio_F1.hpp"
#include "tap/board/board.hpp"
//f0 = data f1 = clock
#include "power_com_data.hpp"

#if defined(PLATFORM_HOSTED)
#define DELAY_MS(ms)
#define DELAY_US(us)
#else
#define DELAY_MS(ms) modm::delay_ms(ms);
#define DELAY_US(us) modm::delay_us(us);
#endif

namespace utils {

    static POWER_COM* instance;

    MODM_ISR(EXIT2){
        POWER_COM_DATA::DATA_READY_PIN::acknowledgeExternalInterruptFlag();
        if(instance)
            instance->update();
    }

    POWER_COM::POWER_COM()
        : modm::I2cDevice<POWER_COM_DATA::POWER_COM_MASTER>(POWER_COM_DATA::I2C_ADDRES),
          modm::pt::Protothread() {}

    void POWER_COM::init(){
        instance = this;

        POWER_COM_DATA::POWER_COM_MASTER::connect<modm::platform::GpioF1::Scl, modm::platform::GpioF0::Sda>();
        POWER_COM_DATA::POWER_COM_MASTER::initialize<Board::SystemClock, 400'000>();

        initializeHardware();

        POWER_COM_DATA::DATA_READY_PIN::setInput(modm::platform::Gpio::InputType::PullUp);
        POWER_COM_DATA::DATA_READY_PIN::enableExternalInterruptVector(0);
        POWER_COM_DATA::DATA_READY_PIN::enableExternalInterrupt();
        POWER_COM_DATA::DATA_READY_PIN::setInputTrigger(modm::platform::Gpio::InputTrigger::FallingEdge);
    }

    void POWER_COM::update() {
        while(PT_readingRawData());
        current = (float)((int16_t)raw_data_buffer[1]) * 1.25f;
        voltage = (float)((int16_t)raw_data_buffer[2]) * 1.25f;
        power = (float)((int16_t)raw_data_buffer[3]) * 10;
    }

}