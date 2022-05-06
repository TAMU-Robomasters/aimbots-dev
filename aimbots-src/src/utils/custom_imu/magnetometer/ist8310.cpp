#include "ist8310.hpp"
#include <cstdint>

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

static constexpr uint8_t RESET_DELAY_MS = 50;
static constexpr uint8_t WAIT_DELAY_US = 250;

Ist8310::Ist8310()
    : modm::I2cDevice<Ist8310Data::IST_I2C_MASTER>(Ist8310Data::I2C_ADDRESS),
      x(0.0f),
      y(0.0f),
      z(0.0f),
      isDeviceVerified(false) {}

uint8_t dbg_device_id = 0;
bool ping_success;

void Ist8310::init(uint8_t* data) {
    raw_data_buffer = data;

    Ist8310Data::IST_I2C_MASTER::connect<modm::platform::GpioA8::Scl, modm::platform::GpioC9::Sda>();
    Ist8310Data::IST_I2C_MASTER::initialize<Board::SystemClock, 400000>();

    uint8_t dev_id = readDeviceID();
    dbg_device_id = dev_id;
    if (dev_id != uint8_t(Ist8310Data::RegisterData::DEVICE_ID))
        isDeviceVerified = true;
    else
        isDeviceVerified = false;

    setDefaultPulseDuration();
}

void Ist8310::update() {
    if (!isDeviceVerified) return;
    if (!(tryReadAxisData())) return;

    x = Ist8310Data::RAW_TO_uT_FACTOR * (float)((((uint16_t)raw_data_buffer[1]) << 8) | raw_data_buffer[0]);
    y = Ist8310Data::RAW_TO_uT_FACTOR * (float)((((uint16_t)raw_data_buffer[3]) << 8) | raw_data_buffer[2]);
    z = Ist8310Data::RAW_TO_uT_FACTOR * (float)((((uint16_t)raw_data_buffer[5]) << 8) | raw_data_buffer[4]);
}
}