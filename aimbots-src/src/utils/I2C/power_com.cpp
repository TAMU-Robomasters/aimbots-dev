#include "power_com.hpp"
#include <cstdint>

#include "modm/platform/gpio/base.hpp"
#include <modm/platform/gpio/gpio_B12.hpp>
#include <modm/platform/gpio/gpio_B13.hpp>
#include <modm/platform/gpio/gpio_B14.hpp>
#include <modm/platform/gpio/gpio_B15.hpp>
// #include "modm/platform/gpio/gpio_F0.hpp"
// #include "modm/platform/gpio/gpio_F1.hpp"
#include "tap/board/board.hpp"
//f0 = data f1 = clock
#include "power_com_data.hpp"


namespace utils {


    POWER_COM::POWER_COM()
        : modm::SpiDevice<POWER_COM_DATA::POWER_COM_MASTER>() {}

    void POWER_COM::init(){

        Board::initialize();

        // POWER_COM_DATA::POWER_COM_MASTER::connect<modm::platform::Peripheral::Spi2,modm::platform::GpioOutputB13::Sck,modm::platform::GpioOutputB15::Mosi,modm::platform::GpioInputB14::Miso>();
        POWER_COM_DATA::POWER_COM_MASTER::connect<modm::platform::GpioOutputB13::template Sck,modm::platform::GpioOutputB15::template Mosi,modm::platform::GpioInputB14::template Miso>();
        POWER_COM_DATA::POWER_COM_MASTER::initialize<Board::SystemClock, modm::MBd(20)>();
        POWER_COM_DATA::SPI_CS::setOutput();
        POWER_COM_DATA::SPI_CS::set();
    }

    void POWER_COM::update() {
        POWER_COM_DATA::SPI_CS::reset();
        data = POWER_COM_DATA::POWER_COM_MASTER::transferBlocking(data);
        POWER_COM_DATA::SPI_CS::set();
    }

}