#include "ist8310.hpp"
#include <cstdint>

#include "modm/platform/i2c/i2c_master_1.hpp"
#include "drivers.hpp"

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
static constexpr uint8_t WAIT_DELAY_US= 250;

Ist8310::Ist8310()
    : modm::I2cDevice<Ist8310Data::IST_I2C_MASTER>(Ist8310Data::I2C_ADDRESS),
      x(0.0f),
      y(0.0f),
      z(0.0f),
      isDeviceVerified(false) { }

void Ist8310::init() {
    // Reset the device so we can assume a default configuration
    Ist8310Data::RESET_PIN::setOutput(false);
    DELAY_MS(RESET_DELAY_MS);
    Ist8310Data::RESET_PIN::setOutput(true);
    DELAY_MS(RESET_DELAY_MS);

    uint8_t dev_id = readDeviceID();
    if(dev_id != uint8_t(Ist8310Data::RegisterData::DEVICE_ID)) {
        // FIXME: Propagate this error in some intelligent way...
        isDeviceVerified = false;
        return;
    }

    isDeviceVerified = true;

    setInterruptFlag(true);
    setDefaultPulseDuration();
}

void Ist8310::update() {
    if(!isDeviceVerified) return;
    if(!(tryReadAxisData())) return;

    x = Ist8310Data::RAW_TO_uT_FACTOR * (float)((((uint16_t)raw_data_buffer[1]) << 8) | raw_data_buffer[0]);
    y = Ist8310Data::RAW_TO_uT_FACTOR * (float)((((uint16_t)raw_data_buffer[3]) << 8) | raw_data_buffer[2]);
    z = Ist8310Data::RAW_TO_uT_FACTOR * (float)((((uint16_t)raw_data_buffer[5]) << 8) | raw_data_buffer[4]);
}

}