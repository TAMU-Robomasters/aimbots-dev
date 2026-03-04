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
    void revEncoderVelocity();
    uint16_t getData() const;
    float getVelocity() const;

private:
    src::Drivers* drivers;
    uint16_t data;
    float velocity;

    float lastAngle;
    uint32_t lastTimeMs = 0;
    uint32_t currentTimeMs;
    uint32_t dtMs;

}; // class RevEncoder

} // namespace src::Informants