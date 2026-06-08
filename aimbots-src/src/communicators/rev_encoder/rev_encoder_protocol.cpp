#include "rev_encoder_protocol.hpp"
#include "drivers.hpp"

using Spi2Master = modm::platform::SpiMaster2; // Master
using Spi2Hal = modm::platform::SpiHal2; // Hardware Abstraction Layer
using Spi2Nss = modm::platform::GpioB12; // Negative SPI Select
using Spi2Sck = modm::platform::GpioB13; // Clock
using Spi2Miso = modm::platform::GpioB14; // Master In Slave Out
using Spi2Mosi = modm::platform::GpioB15; // Master Out Slave In

using MuxS0 = modm::platform::GpioE11; // Mux LSB C2
using MuxS1 = modm::platform::GpioE13; //C3
using MuxS2 = modm::platform::GpioE14; // Mux MSB  C4

namespace src::Informants 
{

    // Constructor
    RevEncoder::RevEncoder(src::Drivers* drivers) : drivers(drivers) {}

    // Initialize and Connect all GPIO Pins
    void RevEncoder::initialize() 
    {
        // Connects Clock, MISO, and MOSI to Master
        Spi2Master::connect<
            Spi2Sck::Sck,
            Spi2Miso::Miso,
            Spi2Mosi::Mosi
        >();

        // Initalize Negative SPI Select and set it High
        Spi2Nss::setOutput(modm::platform::Gpio::OutputType::PushPull);
        Spi2Nss::set();

        // Overhead for SPI Protocol
        Spi2Hal::initialize(
            Spi2Hal::Prescaler::Div64, // SPI Clock Speed
            Spi2Hal::MasterSelection::Master, // Set SPI as Master
            Spi2Hal::DataMode::Mode0, // Clock Idle Low (Mode3 for High)
            Spi2Hal::DataOrder::MsbFirst, // Most Significant Bit First
            Spi2Hal::DataSize::Bit8 // 16-Bit Read/Write
        );

        // Disconnect from TIM_1, Initialize Mux Select and Set Low
        MuxS0::disconnect();
        MuxS1::disconnect();
        MuxS2::disconnect();
        MuxS0::setOutput(modm::platform::Gpio::OutputType::PushPull);
        MuxS1::setOutput(modm::platform::Gpio::OutputType::PushPull);
        MuxS2::setOutput(modm::platform::Gpio::OutputType::PushPull);
        MuxS0::reset();
        MuxS1::reset();
        MuxS2::reset();

        // 500ms Startup Timer
        startupThreshold.restart(500);
    } // void RevEncoder::initialize

    // Main Loop
    void RevEncoder::execute()
    {
        // Runs when Startup Timer is Expired and has been Executed
        if (!startupThreshold.execute()) {
            readEncoders(); // Read All Encoders' Raw Data
            readVelocity(); // Calculate All Encoders' Velocity
            // Wrap on NUM_ENCODER, else Increment
            int nextEncoder = selectedEncoder + 1;
            selectedEncoder = (nextEncoder > NUM_ENCODERS) ? 0 : selectedEncoder++;
        }
    } // void RevEncoder::execute

    // Debug Variables (Angle)
    uint16_t rawAngle_debug[5];
    int16_t revolution_debug[5];
    float wrappedAngle_debug[5];
    float unwrappedAngle_debug[5];

    // Read Angle and Revolution of Encoder
    void RevEncoder::readEncoders() 
    {
        //if (!readReady) {
            Spi2Nss::set(); // Not Ready To Read
            selectEncoder(selectedEncoder); // Set MUX
            //readReady = true;
            //lastTimeMsAngle = tap::arch::clock::getTimeMilliseconds(); chance its not needed
            //return;
        //}

        //currentTimeMsAngle = tap::arch::clock::getTimeMilliseconds();
        //if (currentTimeMsAngle - lastTimeMsAngle >= 2) {
            uint8_t tx[4] = {0x00, 0x00, 0x00, 0x00}; // Write for 16-Bit Angle and Multi-Turn
            uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00}; // Read 16-Bit Angle then Multi-Turn

            Spi2Nss::reset(); // Read Ready
            Spi2Master::transferBlocking(tx, rx, 4); // Read
            Spi2Nss::set(); // Not Ready To Read

            // [0, 65535]
            uint16_t rawAngle = 
            (static_cast<uint16_t>(rx[0]) << 8) | 
            static_cast<uint16_t>(rx[1]);

            // [-32768, 32767]
            int16_t revolution =
            (static_cast<int16_t>(rx[2]) << 8) | 
            static_cast<int16_t>(rx[3]);

            // Raw Angle Filter
            if (rawAngle != 0x0000 && rawAngle != 0xFFFF) {
                rawAngles[selectedEncoder] = rawAngle;
                rawAngle_debug[selectedEncoder] = rawAngle; //debug
            }
            revolutions[selectedEncoder] = revolution;
            revolution_debug[selectedEncoder] = revolution; //debug

            wrappedAngles[selectedEncoder] = getWrappedAngle(selectedEncoder);
            unwrappedAngles[selectedEncoder] = getUnwrappedAngle(selectedEncoder);
            wrappedAngle_debug[selectedEncoder] = wrappedAngles[selectedEncoder]; //debug
            unwrappedAngle_debug[selectedEncoder] = unwrappedAngles[selectedEncoder]; //debug

            //readReady = false;
        //}
    } // void RevEncoder::getData

    // Selects Encoder Through Mux Based on Encoder
    void RevEncoder::selectEncoder(int encoder) {
        (encoder & 0b001) ? MuxS0::set() : MuxS0::reset();
        (encoder & 0b010) ? MuxS1::set() : MuxS1::reset();
        (encoder & 0b100) ? MuxS2::set() : MuxS2::reset();
    } // void RevEncoder::selectEncoder

    // Debug Variables (Velocity)
    float velocity_debug[5];

    // Calculate Velocity of Encoder
    void RevEncoder::readVelocity() 
    {
        // Calculate Change in Time in Milliseconds
        currentTimeMs = tap::arch::clock::getTimeMilliseconds();
        uint32_t dtMs = currentTimeMs - lastTimeMs;

        // Wraps Angle from [0, M_TWOPI] -> [-M_PI, M_PI]
        tap::algorithms::WrappedFloat currentAngle(
            getWrappedAngle(selectedEncoder),
            -M_PI,
            M_PI
        );
        
        // If Not on Startup or No Change in Time
        if (lastTimeMs != 0 && dtMs > 0) {
            // Calculate Velocity (Radians / Seconds)
            float dt = static_cast<float>(dtMs) / 1000.0f;
            float dAngle = currentAngle.minDifference(lastAngle);
            float rawVelocity = dAngle / dt;

            // Raw Velocity Filter
            if (!velocityFilterInitialized) {
                filteredVelocity = rawVelocity;
                velocityFilterInitialized = true;
            } else {
                filteredVelocity += kVelocityAlpha * (rawVelocity - filteredVelocity);
            }

            velocity[selectedEncoder] = filteredVelocity * (30.0f / M_PI);
            velocity_debug[selectedEncoder] = velocity[selectedEncoder]; //debug
        }

        lastAngle = currentAngle;
        lastTimeMs = currentTimeMs;
    } // void RevEncoder::readVelocity

    // Get Raw Encoder Data [0, 65535]
    uint16_t RevEncoder::getRawData(int encoder) const 
    {
        return rawAngles[encoder];
    } // void RevEncoder::getRawData

    // Get Wrapped Angle (M_TWOPI(default) or 360) * [0,1]
    float RevEncoder::getWrappedAngle(int encoder, AngleUnit angleType) const 
    {
        float unitAngle = static_cast<float>(getRawData(encoder)) / 65535.0f;
        if (angleType == AngleUnit::Radians) {
            return (unitAngle * M_TWOPI);
        }
        if (angleType == AngleUnit::Degrees) {
            return (unitAngle * 360.0f);
        }
    } // void RevEncoder::getWrappedAngle

    // Get Unwrapped Angle (M_TWOPI(default) or 360) * [-32768, 32767]
    float RevEncoder::getUnwrappedAngle(int encoder, AngleUnit angleType) const 
    {
        float currentAngle = getWrappedAngle(encoder, angleType);
        float revolution = static_cast<float>(revolutions[encoder]);
        if (angleType == AngleUnit::Radians) {
            return ((revolution * M_TWOPI) + currentAngle);
        }
        if (angleType == AngleUnit::Degrees) {
            return ((revolution * 360.0f) + currentAngle);
        }
    } // void RevEncoder::getUnwrappedAngle

    // Get Velocity (Radian/Second)
    float RevEncoder::getVelocity(int encoder) const 
    {
        return velocity[encoder];
    } // void RevEncoder::getVelocity

} // namespace src::Informants