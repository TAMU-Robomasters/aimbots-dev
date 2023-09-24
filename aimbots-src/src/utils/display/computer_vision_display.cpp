#include "computer_vision_display.hpp"

#include <tuple>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_scheduler.hpp"

#include "reticle_indicator.hpp"

using namespace tap::communication::serial;
using namespace tap::communication::referee;

namespace src::Utils::ClientDisplay {
CVDisplay::CVDisplay(
    tap::communication::serial::RefSerialTransmitter &RefSerialTransmitter,
    BallisticsSolver &ballisticsSolver)
    : HudIndicator(refSerialTransmitter),
      balasticSolver(ballisticsSolver) {}

modm::ResumableResult<bool> CVDisplay::sendInitialGraphics() {
    RF_BEGIN(0);

    RF_END();
}

modm::ResumableResult<bool> CVDisplay::update() {
    RF_BEGIN(1)
    RF_END();
}

void CVDisplay::initialize() {
    prevVisionIndicatorColor = std::nullopt;

    static constexpr int CENTER_X_OFFSET = SCREEN_WIDTH / 2 + ReticleIndicator::RETICLE_CENTER_X_OFFSET;

    initializeVisionHudIndicator(
        &visionTargetFoundGraphics.graphicData[0],
        CENTER_X_OFFSET - VISION_TARGET_FOUND_X_DISTANCE_FROM_CENTER);
    initializeVisionHudIndicator(
        &visionTargetFoundGraphics.graphicData[1],
        CENTER_X_OFFSET + VISION_TARGET_FOUND_X_DISTANCE_FROM_CENTER);
}

void CVDisplay::initializeVisionHudIndicator(Tx::GraphicData *graphicData, int xBoxLocation) {
    uint8_t hudIndicatorName[3] = {};

    getUnusedGraphicName(hudIndicatorName);

    RefSerialTransmitter::configGraphicGenerics(
        graphicData,
        hudIndicatorName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        VISION_TARGET_FOUND_COLOR);

    RefSerialTransmitter::configLine(
        VISION_TARGET_FOUND_SQUARE_WIDTH,
        xBoxLocation,
        VISION_TARGET_FOUND_Y_LOCATION - VISION_TARGET_FOUND_SQUARE_WIDTH / 2,
        xBoxLocation,
        VISION_TARGET_FOUND_Y_LOCATION + VISION_TARGET_FOUND_SQUARE_WIDTH / 2,
        graphicData);
}

modm::ResumableResult<bool> CVDisplay::updateVisionTargetStatus() {
    RF_BEGIN(2);

    {
        bool hasTarget = balasticSolver.solve() == std::nullopt ? false : true;

        if (hasTarget) {
            bool shotTimingMode = true;  // visionCoprocessor.getSomeTurretUsingTimedShots();
            newVisionIndicatorColor = shotTimingMode ? Tx::GraphicColor::ORANGE : Tx::GraphicColor::GREEN;
        } else {
            newVisionIndicatorColor = std::nullopt;
        }
    }

    if (newVisionIndicatorColor.has_value()) {
        visionTargetFoundGraphics.graphicData[0].color = static_cast<uint8_t>(*newVisionIndicatorColor);
        visionTargetFoundGraphics.graphicData[1].color = static_cast<uint8_t>(*newVisionIndicatorColor);
    }

    if ((updateVisionTargetFoundTimeout.isExpired() || updateVisionTargetFoundTimeout.isStopped()) &&
        prevVisionIndicatorColor != newVisionIndicatorColor) {
        {
            bool wasPresent = prevVisionIndicatorColor.has_value();
            bool isPresent = newVisionIndicatorColor.has_value();
            auto presenceChanged = !wasPresent || !isPresent;
            auto operation = presenceChanged ? (isPresent ? Tx::GRAPHIC_ADD : Tx::GRAPHIC_DELETE) : Tx::GRAPHIC_MODIFY;
            visionTargetFoundGraphics.graphicData[0].operation = operation;
            visionTargetFoundGraphics.graphicData[1].operation = operation;
        }

        RF_CALL(refSerialTransmitter.sendGraphic(&visionTargetFoundGraphics));

        updateVisionTargetFoundTimeout.restart(VISION_TARGET_FOUND_MAX_REFRESH_RATE);
        prevVisionIndicatorColor = newVisionIndicatorColor;
    }

    RF_END();
}

}  // namespace src::Utils::ClientDisplay