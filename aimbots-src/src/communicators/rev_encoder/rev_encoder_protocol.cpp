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

    using MuxS0 = modm::platform::GpioE11;
    using MuxS1 = modm::platform::GpioE13;
    using MuxS2 = modm::platform::GpioE14;


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

        MuxS0::disconnect();
        MuxS1::disconnect();
        MuxS2::disconnect();

        MuxS0::setOutput(modm::platform::Gpio::OutputType::PushPull);
        MuxS1::setOutput(modm::platform::Gpio::OutputType::PushPull);
        MuxS2::setOutput(modm::platform::Gpio::OutputType::PushPull);

        MuxS0::reset();
        MuxS1::reset();
        MuxS2::reset();

        Spi2Hal::initialize(
            Spi2Hal::Prescaler::Div64,
            Spi2Hal::MasterSelection::Master,
            Spi2Hal::DataMode::Mode0,
            Spi2Hal::DataOrder::MsbFirst,
            Spi2Hal::DataSize::Bit8
        );
        startupThreshold.restart(500);
    }

    void RevEncoder::selectEncoder(EncoderID encoder) 
    {
        uint8_t encoderValue = static_cast<uint8_t>(encoder);
        (encoderValue & 0b001) ? MuxS0::set() : MuxS0::reset();
        (encoderValue & 0b010) ? MuxS1::set() : MuxS1::reset();
        (encoderValue & 0b100) ? MuxS2::set() : MuxS2::reset();
    }

    uint8_t debug_byte[2];
    uint16_t debug_allData[5];
    uint16_t debug_data = 0;

    void RevEncoder::readData()
    {
        static uint32_t last = 0;
        static EncoderID selected = EncoderID::REV_ENCODER_1;
        static bool prepared = false;

        if (!prepared)
        {
            Spi2Nss::set();          // disable 138
            selectEncoder(selected); // set A/B/C
            prepared = true;
            last = tap::arch::clock::getTimeMilliseconds();
            return;
        }

        if (tap::arch::clock::getTimeMilliseconds() - last >= 2)
        {
            uint8_t rx[2] = {0, 0};

            Spi2Nss::reset(); // enable 138
            Spi2Master::transferBlocking(nullptr, rx, 2);
            Spi2Nss::set();   // disable 138

            uint16_t raw =
                (static_cast<uint16_t>(rx[0]) << 8) |
                static_cast<uint16_t>(rx[1]);

            uint8_t i = static_cast<uint8_t>(selected);

            if (i < NUM_ENCODERS)
            {
                if (raw != 0 && raw != 0xFFFF)
                {
                    allData[i] = raw;
                    data = raw;
                }
            }

            debug_byte[0] = rx[0];
            debug_byte[1] = rx[1];

            uint8_t next = static_cast<uint8_t>(selected) + 1;
            if (next >= NUM_ENCODERS)
            {
                next = 0;
            }

            selected = static_cast<EncoderID>(next);
            prepared = false;
        }

        for (uint8_t i = 0; i < NUM_ENCODERS; i++)
        {
            debug_allData[i] = allData[i];
        }

        debug_data = data;
    }

    void RevEncoder::execute()
    {
        if (!startupThreshold.execute()) {
            readData();
            revEncoderVelocity();
        }
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

    float RevEncoder::getVelocity() const
    {
        return velocity;
    }

    int64_t RevEncoder::getUnwrappedPosition() const
    {
        return unwrappedPosition;
    }

}  // namespace src::Informants