#include "rev_encoder_protocol.hpp"
#include "drivers.hpp"

namespace src::Informants {

using Spi2Master = modm::platform::SpiMaster2;
using Spi2Hal = modm::platform::SpiHal2;
using Spi2Nss = modm::platform::GpioB12;
using Spi2Sck = modm::platform::GpioB13;
using Spi2Miso = modm::platform::GpioB14;
using Spi2Mosi = modm::platform::GpioB15;

float positionDisplay = 0.0f;
float velocityDisplay = 0.0f;

RevEncoder::RevEncoder(src::Drivers* drivers) : drivers(drivers) {}

void RevEncoder::initialize() {
    Spi2Master::connect<
        Spi2Sck::Sck,
        Spi2Miso::Miso,
        Spi2Mosi::Mosi
    >();

    Spi2Nss::setOutput(modm::platform::Gpio::OutputType::PushPull);
    Spi2Nss::set();

    Spi2Hal::initialize(
        Spi2Hal::Prescaler::Div16,
        Spi2Hal::MasterSelection::Master,
        Spi2Hal::DataMode::Mode1,
        Spi2Hal::DataOrder::MsbFirst,
        Spi2Hal::DataSize::Bit8
    );
}

uint8_t debug_byte[2];
void RevEncoder::readData() {
    uint8_t rx[2];

    Spi2Nss::reset();
    Spi2Master::transferBlocking(nullptr, rx, 2);
    Spi2Nss::set();

    debug_byte[0] = rx[0];
    debug_byte[1] = rx[1];

    uint16_t raw = (uint16_t(rx[0]) << 8) | uint16_t(rx[1]);

    raw = static_cast<uint16_t>(0u - raw);

    float rawAngle = M_TWOPI * (raw / 65535.0f);

    if (!angleFilterInitialized) {
        filteredAngle = rawAngle;
        angleFilterInitialized = true;
    }
    else {
        float angleError = fmodf((rawAngle - filteredAngle) + M_PI, M_TWOPI) - M_PI;
        filteredAngle += kPositionAlpha * angleError;

        filteredAngle = fmodf(filteredAngle, M_TWOPI);
        if (filteredAngle < 0.0f) {
            filteredAngle += M_TWOPI;
        }
    }

    data = static_cast<uint16_t>((filteredAngle / M_TWOPI) * 65535.0f);
}

uint16_t debug_data = 0;
void RevEncoder::execute() {
    readData();
    revEncoderVelocity();
    debug_data = data;
    positionDisplay = data;
}

float debug_vel = 0.0f;
void RevEncoder::revEncoderVelocity() {
    currentTimeMs = tap::arch::clock::getTimeMilliseconds();
    dtMs = currentTimeMs - lastTimeMs;

    tap::algorithms::WrappedFloat currentAngle(M_TWOPI * (data / 65535.0f),-M_PI,M_PI);

    if (lastTimeMs != 0 && dtMs > 0) {
        float dt = dtMs / 1000.0f;

        float dAngle = currentAngle.minDifference(lastAngle);
        float rawVelocity = dAngle / dt;

        if (!velocityFilterInitialized) {
            filteredVelocity = rawVelocity;
            velocityFilterInitialized = true;
        }
        else {
            filteredVelocity += kVelocityAlpha * (rawVelocity - filteredVelocity);
        }

        velocity = filteredVelocity * (30.0/M_PI);
        debug_vel = velocity;
    }

    velocityDisplay = velocity;
    lastAngle = currentAngle;
    lastTimeMs = std::move(currentTimeMs);
}

uint16_t RevEncoder::getData() const {
    return data;
}

float RevEncoder::getVelocity() const {
    return velocity;
}

} // namespace src::Informants