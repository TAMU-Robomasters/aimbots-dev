#pragma once

#include "modm/platform/spi/spi_master_2.hpp"
#include "utils/tools/common_types.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace src::Informants {

class RevEncoder {
public:
    RevEncoder(src::Drivers* drivers);
    ~RevEncoder() = default;

    void initialize();
    uint16_t readData();
    void execute();
    float getAngle();
    
private:
    src::Drivers* drivers;
    float angle;
    uint16_t packedData;

}; // class RevEncoder

} // namespace src::Informants