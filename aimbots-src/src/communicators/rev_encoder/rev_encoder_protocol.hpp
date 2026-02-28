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
    void readData();
    void execute();
    uint16_t getData() const;

private:
    src::Drivers* drivers;
    uint16_t data;

}; // class RevEncoder

} // namespace src::Informants