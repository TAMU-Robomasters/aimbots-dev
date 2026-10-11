#include "rev_encoder_protocol.hpp"
#include "drivers.hpp"

namespace src::Informants
{
    // DM-MC02: SPI3 on the 24-pin expansion header (SCK pin 23, MISO pin 21, MOSI pin 19,
    // CS = GPIO pin 13). SPI2 is the onboard BMI088 on this board. The Type C used SPI2 PB12-15.
    using Spi3Master = modm::platform::SpiMaster3;
    using Spi3Hal = modm::platform::SpiHal3;
    using Spi3Nss = modm::platform::GpioE14;
    using Spi3Sck = modm::platform::GpioC10;
    using Spi3Miso = modm::platform::GpioC11;
    using Spi3Mosi = modm::platform::GpioC12;

    float positionDisplay = 0.0f;
    float velocityDisplay = 0.0f;

    RevEncoder::RevEncoder(src::Drivers* drivers) : drivers(drivers) {}

    void RevEncoder::initialize()
    {
        Spi3Master::connect<
            Spi3Sck::Sck,
            Spi3Miso::Miso,
            Spi3Mosi::Mosi
        >();

        Spi3Nss::setOutput(modm::platform::Gpio::OutputType::PushPull);
        Spi3Nss::set();

        Spi3Hal::initialize(
            Spi3Hal::Prescaler::Div32,  // 80 MHz / 32 = 2.5 MHz (Type C: 42 MHz / 16 = 2.625 MHz)
            Spi3Hal::MasterSelection::Master,
            Spi3Hal::DataMode::Mode1,
            Spi3Hal::DataOrder::MsbFirst,
            Spi3Hal::DataSize::Bit8
        );
        startupThreshold.restart(500);
    }

    uint8_t debug_byte[2];

    void RevEncoder::readData()
    {
        uint8_t rx[2];

        Spi3Nss::reset();
        Spi3Master::transferBlocking(nullptr, rx, 2);
        Spi3Nss::set();

        debug_byte[0] = rx[0];
        debug_byte[1] = rx[1];

        uint16_t raw = (static_cast<uint16_t>(rx[0]) << 8) | static_cast<uint16_t>(rx[1]);

        raw = static_cast<uint16_t>(0u - raw);

        // goofy ass wrap crap
        if (!unwrappedInitialized)
        {
            unwrappedPosition = static_cast<int64_t>(raw);
            lastRawCount = raw;
            unwrappedInitialized = true;
        }
        else
        {
            int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(lastRawCount);

            if (delta > 32767)
            {
                delta -= 65536;
            }
            else if (delta < -32767)
            {
                delta += 65536;
            }

            #ifndef INVERTED_YAW_ENCODER
                unwrappedPosition += static_cast<int64_t>(delta);
            #else
                unwrappedPosition -= static_cast<int64_t>(delta);
            #endif
            lastRawCount = raw;
        }

        float rawAngle = M_TWOPI * (static_cast<float>(raw) / 65535.0f);

        if (!angleFilterInitialized)
        {
            filteredAngle = rawAngle;
            angleFilterInitialized = true;
        }
        else
        {
            float angleError = rawAngle - filteredAngle;

            while (angleError > M_PI)
            {
                angleError -= M_TWOPI;
            }
            while (angleError < -M_PI)
            {
                angleError += M_TWOPI;
            }

            filteredAngle += kPositionAlpha * angleError;

            while (filteredAngle >= M_TWOPI)
            {
                filteredAngle -= M_TWOPI;
            }
            while (filteredAngle < 0.0f)
            {
                filteredAngle += M_TWOPI;
            }
        }

        data = static_cast<uint16_t>((filteredAngle / M_TWOPI) * 65535.0f);
    }

    uint16_t debug_data = 0;

    void RevEncoder::execute()
    {
        if(!startupThreshold.execute())
        readData();
        revEncoderVelocity();
        debug_data = data;
        positionDisplay = data;
    }

    float debug_vel = 0.0f;

    void RevEncoder::revEncoderVelocity()
    {
        currentTimeMs = tap::arch::clock::getTimeMilliseconds();
        dtMs = currentTimeMs - lastTimeMs;

        tap::algorithms::WrappedFloat currentAngle(
            M_TWOPI * (static_cast<float>(data) / 65535.0f),
            -M_PI,
            M_PI
        );

        if (lastTimeMs != 0 && dtMs > 0)
        {
            float dt = static_cast<float>(dtMs) / 1000.0f;

            float dAngle = currentAngle.minDifference(lastAngle);
            float rawVelocity = dAngle / dt;

            if (!velocityFilterInitialized)
            {
                filteredVelocity = rawVelocity;
                velocityFilterInitialized = true;
            }
            else
            {
                filteredVelocity += kVelocityAlpha * (rawVelocity - filteredVelocity);
            }

            velocity = filteredVelocity * (30.0f / M_PI);
            debug_vel = velocity;
        }

        velocityDisplay = velocity;
        lastAngle = currentAngle;
        lastTimeMs = currentTimeMs;
    }

    uint16_t RevEncoder::getData() const
    {
        return data;
    }

    float RevEncoder::getAngleRadians() const
    {
        return RevEncoderValueToRadians(data);
    }

    float RevEncoder::getVelocity() const
    {
        return velocity;
    }

    int64_t RevEncoder::getUnwrappedPosition() const
    {
        return unwrappedPosition;
    }

    float RevEncoder::getUnwrappedPositionRadians() const
    {
        return RevEncoderValueToRadians(unwrappedPosition);
    }

}  // namespace src::Informants