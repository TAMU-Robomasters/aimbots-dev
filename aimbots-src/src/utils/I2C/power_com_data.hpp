#pragma once
#include <cstdint>

#include <modm/platform/gpio/gpio_B12.hpp>
#include <modm/platform/gpio/gpio_B13.hpp>
#include <modm/platform/gpio/gpio_B14.hpp>
#include <modm/platform/gpio/gpio_B15.hpp>

#include <modm/platform/spi/spi_master_2.hpp>

namespace utils::POWER_COM_DATA {

    //using DATA_READY_PIN = modm::platform::GpioF0;
    using SPI_CS = modm::platform::GpioB12;
    using SPI_CLK = modm::platform::GpioB13;
    using SPI_MISO = modm::platform::GpioB14;
    using SPI_MOSI = modm::platform::GPIOB15;
    using POWER_COM_MASTER = modm::platform::SpiMaster2;
    

}