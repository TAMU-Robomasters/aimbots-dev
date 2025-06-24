#pragma once

#include "modm/architecture/interface/spi.hpp"
#include "tap/board/board.hpp"
#include "power_com_data.hpp"


namespace utils {

class POWER_COM : modm::SpiDevice<POWER_COM_DATA::POWER_COM_MASTER>
{
public:
    POWER_COM();

    void init();
    void update();
    uint8_t getData(){return data;};


private:




    uint8_t data;
};
}

