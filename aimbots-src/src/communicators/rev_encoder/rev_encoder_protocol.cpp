#include "rev_encoder_protocol.hpp"
#include "drivers.hpp"

namespace src::Informants{

    using Spi2Master = modm::platform::SpiMaster2;
    using Spi2Hal = modm::platform::SpiHal2;
    using Spi2Nss = modm::platform::GpioB12;
    using Spi2Sck = modm::platform::GpioB13;
    using Spi2Miso = modm::platform::GpioB14;
    using Spi2Mosi = modm::platform::GpioB15;

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
            Spi2Hal::DataMode::Mode3,
            Spi2Hal::DataOrder::MsbFirst,
            Spi2Hal::DataSize::Bit8
        );
    }

    uint8_t debug_byte[2];
    uint16_t debug_packed = 0;
    uint16_t RevEncoder::readData() {
        uint8_t rx[2]; 

        Spi2Nss::reset();
        Spi2Master::transferBlocking(nullptr, rx, 2);
        Spi2Nss::set();

        debug_byte[0] = rx[0];
        debug_byte[1] = rx[1];

        packedData = (uint16_t(rx[0])<<8) | uint16_t(rx[1]);
        debug_packed = packedData;
        return packedData;
    }

    void RevEncoder::execute() {
        readData();
        getAngle();
    }  
    
    float debug_angle;
    float RevEncoder::getAngle() {
        angle = (float(packedData) / 65535.0f) * 2 * PI;
        debug_angle = angle;
        return angle;
    }
} // namespace src::Informants