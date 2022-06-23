#include "ist8310.hpp"
#include <cstdint>

#include "modm/platform/gpio/base.hpp"
#include "modm/platform/gpio/gpio_C9.hpp"
#include "modm/platform/gpio/gpio_A8.hpp"
#include "tap/board/board.hpp"

#include "ist8310_data.hpp"

#if defined(PLATFORM_HOSTED)
#define DELAY_MS(ms)
#define DELAY_US(us)
#else
#define DELAY_MS(ms) modm::delay_ms(ms);
#define DELAY_US(us) modm::delay_us(us);
#endif

namespace utils {

static Ist8310* instance;

MODM_ISR(EXTI3) {
    Ist8310Data::DATA_READY_PIN::acknowledgeExternalInterruptFlag();
    if(instance)
        instance->update();
}

Ist8310::Ist8310()
    : modm::I2cDevice<Ist8310Data::IST_I2C_MASTER>(Ist8310Data::I2C_ADDRESS),
      modm::pt::Protothread() {}

void Ist8310::init() {
    instance = this;

    Ist8310Data::IST_I2C_MASTER::connect<modm::platform::GpioA8::Scl, modm::platform::GpioC9::Sda>();
    Ist8310Data::IST_I2C_MASTER::initialize<Board::SystemClock, 400'000>();

    initializeHardware();

    Ist8310Data::DATA_READY_PIN::setInput(modm::platform::Gpio::InputType::PullUp);
    Ist8310Data::DATA_READY_PIN::enableExternalInterruptVector(0);
    Ist8310Data::DATA_READY_PIN::enableExternalInterrupt();
    Ist8310Data::DATA_READY_PIN::setInputTrigger(modm::platform::Gpio::InputTrigger::FallingEdge);
}

void Ist8310::update() {
    //if (!isDeviceVerified) return;

    // TODO: Make this non-blocking if it takes too long...
    while(PT_readingRawAxisData());

    x = Ist8310Data::RAW_TO_uT_FACTOR * (float)(((uint16_t)raw_data_buffer[1] << 8) | raw_data_buffer[0]);
    y = Ist8310Data::RAW_TO_uT_FACTOR * (float)(((uint16_t)raw_data_buffer[3] << 8) | raw_data_buffer[2]);
    z = Ist8310Data::RAW_TO_uT_FACTOR * (float)(((uint16_t)raw_data_buffer[5] << 8) | raw_data_buffer[4]);
}

}