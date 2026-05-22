#pragma once

#include "modm/platform/spi/spi_master_2.hpp"
#include "utils/tools/common_types.hpp"
#include <tap/algorithms/wrapped_float.hpp>
#include <cstdint>

namespace src {
    class Drivers;
}

namespace src::Informants 
{

class RevEncoder
{
public:
    explicit RevEncoder(src::Drivers* drivers);

    void initialize();
    void execute();

    uint16_t getRawData(int encoder) const;
    float getWrappedAngle(int encoder, AngleUnit angleType = AngleUnit::Radians) const;
    float getUnwrappedAngle(int encoder, AngleUnit angleType = AngleUnit::Radians) const;
    float getVelocity(int encoder) const;

private:
    void readEncoders();
    void selectEncoder(int encoder);
    void readVelocity();

    src::Drivers* drivers;

    // RevEncoder
    static constexpr uint8_t NUM_ENCODERS = 5;
    int selectedEncoder = 0;
    uint16_t rawAngles[NUM_ENCODERS];
    int16_t revolutions[NUM_ENCODERS];
    float velocity[NUM_ENCODERS];
    float wrappedAngles[NUM_ENCODERS];
    float unwrappedAngles[NUM_ENCODERS];

    // RevEncoder::getData variables (not needed)
    //int selectedEncoder = 0;
    //bool readReady = false;
    //uint32_t currentTimeMsAngle = 0;
    //uint32_t lastTimeMsAngle = 0;

    // RevEncoder::getVelocity variables
    tap::algorithms::WrappedFloat lastAngle{0.0f, -M_PI, M_PI};
    uint32_t currentTimeMs = 0;
    uint32_t lastTimeMs = 0;
    float filteredVelocity = 0.0f;
    bool velocityFilterInitialized = false;
    static constexpr float kPositionAlpha = 0.10f;
    static constexpr float kVelocityAlpha = 0.20f;

    MilliTimeout startupThreshold;
}; // class RevEncoder

} // namespace src::Informants