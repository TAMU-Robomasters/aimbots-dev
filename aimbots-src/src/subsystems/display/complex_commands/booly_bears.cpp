#include "subsystems/display/complex_commands/booly_bears.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "subsystems/display/basic_commands/hud_indicator.hpp"


// #include "subsystems/display/display_constants.hpp"

using namespace tap::communication::serial;
//using namespace src::Utils::HUDClientDisplay;
using namespace src::Utils::ClientDisplay;
using namespace src::Utils::BoolClientDisplay;


//namespace src::Utils::ClientDisplay {
BoolyBear::BoolyBear(
    tap::Drivers &drivers,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers) {}

modm::ResumableResult<bool> BoolyBear::sendInitialGraphics() {
    RF_BEGIN(0);

    // send booly
    for (boolyIndex = 0; boolyIndex < MODM_ARRAY_SIZE(boolyMsg); boolyIndex++) {
        RF_CALL(refSerialTransmitter.sendGraphic(&boolyMsg[boolyIndex]));
    }

    RF_END();
}

modm::ResumableResult<bool> BoolyBear::update() {
    RF_BEGIN(1);
    // send booly
    for (boolyIndex = 0; boolyIndex < MODM_ARRAY_SIZE(boolyMsg); boolyIndex++) {
        RF_CALL(refSerialTransmitter.sendGraphic(&boolyMsg[boolyIndex]));
    }
    RF_END();
}

void BoolyBear::initialize() {
    uint8_t currLineName[3];
    getUnusedGraphicName(currLineName);

    // Add booly markers
    uint16_t maxBoolyY = 0;
    uint16_t minBoolyY = UINT16_MAX;

    // configure all horizontal booly lines
    for (size_t i = 0; i < NUM_BOOLY_COORDINATES; i++) {
        // boolyMsg is an array of Graphic5Messages, so find the index in the array and in the
        // individual Graphic5Message
        size_t boolyMsgIndex = i / MODM_ARRAY_SIZE(boolyMsg[0].graphicData);
        size_t graphicDataIndex = i % MODM_ARRAY_SIZE(boolyMsg[0].graphicData);

        RefSerialTransmitter::configGraphicGenerics(
            &boolyMsg[boolyMsgIndex].graphicData[graphicDataIndex],
            currLineName,
            Tx::GRAPHIC_ADD,
            DEFAULT_GRAPHIC_LAYER,
            std::get<2>(TURRET_BOOLY_X_WIDTH_AND_Y_POS_COORDINATES[i]));

        getUnusedGraphicName(currLineName);

        // center of the booly, in pixels
        uint16_t boolyXCenter = static_cast<int>(SCREEN_WIDTH / 2) + BOOLY_CENTER_X_OFFSET;

        // start and end X pixel coordinates of the current booly line
        uint16_t startX = boolyXCenter - std::get<0>(TURRET_BOOLY_X_WIDTH_AND_Y_POS_COORDINATES[i]);
        uint16_t endX = boolyXCenter + std::get<0>(TURRET_BOOLY_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        // y coordinate of the horizontal booly line
        uint16_t y = std::get<1>(TURRET_BOOLY_X_WIDTH_AND_Y_POS_COORDINATES[i]);

        RefSerialTransmitter::configLine(
            BOOLY_THICKNESS,
            startX,
            y,
            endX,
            y,
            &boolyMsg[boolyMsgIndex].graphicData[graphicDataIndex]);

        // update min and max y coordinates to be used when drawing the vertical booly line that
        // connects horizontal booly lines
        if (y > maxBoolyY) {
            maxBoolyY = y;
        }
        if (y < minBoolyY) {
            minBoolyY = y;
        }
    }

    // Add vertical booly line to connect booly markers
    RefSerialTransmitter::configGraphicGenerics(
        &boolyMsg[NUM_BOOLY_COORDINATES / MODM_ARRAY_SIZE(boolyMsg[0].graphicData)]
             .graphicData[NUM_BOOLY_COORDINATES % MODM_ARRAY_SIZE(boolyMsg[0].graphicData)],
        currLineName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        BOOLY_VERTICAL_COLOR);

    RefSerialTransmitter::configLine(
        BOOLY_THICKNESS,
        SCREEN_WIDTH / 2 + BOOLY_CENTER_X_OFFSET,
        minBoolyY,
        SCREEN_WIDTH / 2 + BOOLY_CENTER_X_OFFSET,
        maxBoolyY,
        &boolyMsg[NUM_BOOLY_COORDINATES / MODM_ARRAY_SIZE(boolyMsg[0].graphicData)]
             .graphicData[NUM_BOOLY_COORDINATES % MODM_ARRAY_SIZE(boolyMsg[0].graphicData)]);
}

//}  // namespace src::Utils::ClientDisplay
