#include "rev_encoder_protocol.hpp"
#include "drivers.hpp"

namespace src::Informants
{
    using Spi2Master = modm::platform::SpiMaster2;
    using Spi2Hal = modm::platform::SpiHal2;
    using Spi2Nss = modm::platform::GpioB12;
    using Spi2Sck = modm::platform::GpioB13;
    using Spi2Miso = modm::platform::GpioB14;
    using Spi2Mosi = modm::platform::GpioB15;

    float positionDisplay = 0.0f;
    float velocityDisplay = 0.0f;

    RevEncoder::RevEncoder(src::Drivers* drivers) : drivers(drivers) {}

    void RevEncoder::initialize()
    {
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
        startupThreshold.restart(500);
    }

    uint8_t debug_byte[2];

    void RevEncoder::readData()
    {
        uint8_t rx[2];

        Spi2Nss::reset();
        Spi2Master::transferBlocking(nullptr, rx, 2);
        Spi2Nss::set();

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

            unwrappedPosition += static_cast<int64_t>(delta);
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

    uint16_t RevEncoder::getAngleRadians() const
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

}  // namespace src::Informants