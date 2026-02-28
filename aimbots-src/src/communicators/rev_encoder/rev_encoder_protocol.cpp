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
    void RevEncoder::readData() {
        uint8_t rx[2]; 

        Spi2Nss::reset();
        Spi2Master::transferBlocking(nullptr, rx, 2);
        Spi2Nss::set();

        debug_byte[0] = rx[0];
        debug_byte[1] = rx[1];

        data = (uint16_t(rx[0])<<8) | uint16_t(rx[1]);
    }

    uint16_t debug_data = 0;
    void RevEncoder::execute() {
        readData();
        debug_data = data;
    }  
    
    uint16_t RevEncoder::getData() const {
        return data;
    }
} // namespace src::Informants