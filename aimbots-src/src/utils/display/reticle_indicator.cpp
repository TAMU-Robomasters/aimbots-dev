

#include "reticle_indicator.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"
using namespace tap::communication::serial;

namespace src::utils::display {
ReticleIndicator::ReticleIndicator(tap::Drivers &drivers, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers) {}

modm::ResumableResult<bool> ReticleIndicator::sendInitialGraphics() {
    RF_BEGIN(0);

    // send reticle
    for (reticleIndex = 0; reticleIndex < MODM_ARRAY_SIZE(reticleMsg); reticleIndex++) {
        RF_CALL(refSerialTransmitter.sendGraphic(&reticleMsg[reticleIndex]));
    }

    RF_END();
}

modm::ResumableResult<bool> ReticleIndicator::update() {
    RF_BEGIN(1);
    RF_END();
}

void ReticleIndicator::initialize() {
    uint8_t currLineName[3];
    getUnusedGraphicName(currLineName);

    // Add reticle markers
    uint16_t maxReticleY = 0;
    uint16_t minReticleY = UINT16_MAX;

    // configure all horizontal reticle lines
    for (size_t i = 0; i < NUM_RETICLE_COORDINATES; i++) {
        // reticleMsg is an array of Graphic5Messages, so find the index in the array and in the
        // individual Graphic5Message
        size_t reticleMsgIndex = i / MODM_ARRAY_SIZE(reticleMsg[0].graphicData);
        size_t graphicDataIndex = i % MODM_ARRAY_SIZE(reticleMsg[0].graphicData);

        RefSerialTransmitter::configGraphicGenerics(
            &reticleMsg[reticleMsgIndex].graphicData[graphicDataIndex],
            currLineName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            std::get<2>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]));

        getUnusedGraphicName(currLineName);

        // center of the reticle, in pixels
        uint16_t reticleXCenter = static_cast<int>(SCREEN_WIDTH / 2) + RETICLE_CENTER_X_OFFSET;

        // start and end X pixel coordinates of the current reticle line
        uint16_t startX = reticleXCenter - std::get<0>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);
        uint16_t endX = reticleXCenter + std::get<0>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        // y coordinate of the horizontal reticle line
        uint16_t y = std::get<1>(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        RefSerialTransmitter::configLine(RETICLE_THICKNESS, startX, y, endX, y, &reticleMsg[reticleMsgIndex].graphicData[graphicDataIndex]);

        // update min and max y coordinates to be used when drawing the vertical reticle line that
        // connects horizontal reticle lines
        if (y > maxReticleY) {
            maxReticleY = y;
        }
        if (y < minReticleY) {
            minReticleY = y;
        }
    }

    // Add vertical reticle line to connect reticle markers
    RefSerialTransmitter::configGraphicGenerics(
        &reticleMsg[NUM_RETICLE_COORDINATES / MODM_ARRAY_SIZE(reticleMsg[0].graphicData)]
             .graphicData[NUM_RETICLE_COORDINATES % MODM_ARRAY_SIZE(reticleMsg[0].graphicData)],
        currLineName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        RETICLE_VERTICAL_COLOR);

    RefSerialTransmitter::configLine(
        RETICLE_THICKNESS,
        SCREEN_WIDTH / 2 + RETICLE_CENTER_X_OFFSET,
        minReticleY,
        SCREEN_WIDTH / 2 + RETICLE_CENTER_X_OFFSET,
        maxReticleY,
        &reticleMsg[NUM_RETICLE_COORDINATES / MODM_ARRAY_SIZE(reticleMsg[0].graphicData)]
             .graphicData[NUM_RETICLE_COORDINATES % MODM_ARRAY_SIZE(reticleMsg[0].graphicData)]);
}

}  // namespace src::utils::display
