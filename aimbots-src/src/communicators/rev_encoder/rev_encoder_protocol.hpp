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

    uint16_t data = 0;
    float velocity = 0.0f;

    float lastAngle = 0.0f;
    uint32_t lastTimeMs = 0;
    uint32_t currentTimeMs = 0;
    uint32_t dtMs = 0;

    float filteredAngle = 0.0f;
    bool angleFilterInitialized = false;

    float filteredVelocity = 0.0f;
    bool velocityFilterInitialized = false;

    static constexpr float kPositionAlpha = 0.10f;
    static constexpr float kVelocityAlpha = 0.20f;
}; // class RevEncoder

} // namespace src::Informants