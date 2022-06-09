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

static constexpr uint8_t RESET_DELAY_MS = 50;
static constexpr uint8_t IO_DELAY_US = 150;

Ist8310::Ist8310()
    : modm::I2cDevice<Ist8310Data::IST_I2C_MASTER>(Ist8310Data::I2C_ADDRESS) {}

uint8_t dbg_test = 0;

void Ist8310::init() {
    instance = this;

    Ist8310Data::DATA_READY_PIN::setInput(modm::platform::Gpio::InputType::PullUp);
    Ist8310Data::DATA_READY_PIN::enableExternalInterruptVector(0);
    Ist8310Data::DATA_READY_PIN::enableExternalInterrupt();
    Ist8310Data::DATA_READY_PIN::setInputTrigger(modm::platform::Gpio::InputTrigger::FallingEdge);

    Ist8310Data::IST_I2C_MASTER::connect<modm::platform::GpioA8::Scl, modm::platform::GpioC9::Sda>();
    Ist8310Data::IST_I2C_MASTER::initialize<Board::SystemClock, 400'000>();

    raw_data_buffer[0] = 0;

    // Works...
    uint8_t dev_id = readDeviceID();
    if (dev_id == uint8_t(Ist8310Data::RegisterData::DEVICE_ID))
        isDeviceVerified = true;
    else
        isDeviceVerified = false;

    raw_data_buffer[0] = 0;

    // Does not work...
    readRegister(Ist8310Data::Register::WHO_AM_I);
    modm::delay_us(250);
    uint8_t test_1 = raw_data_buffer[0];
    dbg_test = test_1;
    if(test_1 == uint8_t(Ist8310Data::RegisterData::DEVICE_ID))
        dbg_test = 88;

    raw_data_buffer[0] = 0;

    // Works...
    readRegister(Ist8310Data::Register::WHO_AM_I);
    modm::delay_us(250);
    uint8_t test_2 = raw_data_buffer[0];
    dbg_test = test_2;
    if(test_2 == uint8_t(Ist8310Data::RegisterData::DEVICE_ID))
        dbg_test = 88;

    raw_data_buffer[0] = 0;

    // Does not work...
    readRegister(Ist8310Data::Register::WHO_AM_I);
    modm::delay_us(250);
    uint8_t test_3 = raw_data_buffer[0];
    dbg_test = test_3;
    if(test_3 == uint8_t(Ist8310Data::RegisterData::DEVICE_ID))
        dbg_test = 88;

    raw_data_buffer[0] = 0;

    // Works...
    readRegister(Ist8310Data::Register::WHO_AM_I);
    modm::delay_us(250);
    uint8_t test_4 = raw_data_buffer[0];
    dbg_test = test_4;
    if(test_4 == uint8_t(Ist8310Data::RegisterData::DEVICE_ID))
        dbg_test = 88;
}

void Ist8310::update() {
    if (!isDeviceVerified) return;

    //readRegister(Ist8310Data::Register::X_DATA_LOW);
    //uint8_t xl = raw_data_buffer[0];
    //readRegister(Ist8310Data::Register::X_DATA_HIGH);
    //uint8_t xh = raw_data_buffer[0];
    //readRegister(Ist8310Data::Register::Y_DATA_LOW);
    //uint8_t yl = raw_data_buffer[0];
    //readRegister(Ist8310Data::Register::Y_DATA_HIGH);
    //uint8_t yh = raw_data_buffer[0];
    //readRegister(Ist8310Data::Register::Z_DATA_LOW);
    //uint8_t zl = raw_data_buffer[0];
    //readRegister(Ist8310Data::Register::Z_DATA_HIGH);
    //uint8_t zh = raw_data_buffer[0];

    //x = Ist8310Data::RAW_TO_uT_FACTOR * (float)((xh << 8) | xl);
    //y = Ist8310Data::RAW_TO_uT_FACTOR * (float)((yh << 8) | yl);
    //z = Ist8310Data::RAW_TO_uT_FACTOR * (float)((zh << 8) | zl);
}
}