#pragma once

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace src::utils::display {
class ReticleIndicator : public HudIndicator, protected modm::Resumable<2> {
public:
    static constexpr int RETICLE_CENTER_X_OFFSET = 15;

    ReticleIndicator(tap::Drivers &drivers, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);
    modm::ResumableResult<bool> update() override;

    modm::ResumableResult<bool> sendInitialGraphics() override;

    void initialize() override;

private:
    /** Line thickness of the reticle, in pixels. */
    static constexpr uint16_t RETICLE_THICKNESS = 1;

    /** Tuple representing a possible horizontal reticle line. The first element is the pixel width
     * of the line, second is Y location of the line (in pixels), third is the color of the reticle
     * line. */
    using ReticleTuple = std::tuple<int16_t, int16_t, Tx::GraphicColor>;

    static constexpr ReticleTuple TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[]{
        ReticleTuple(50, 435, Tx::GraphicColor::YELLOW),  // 1 m
        ReticleTuple(50, 430, Tx::GraphicColor::YELLOW),
        ReticleTuple(70, 425, Tx::GraphicColor::YELLOW),
        ReticleTuple(50, 420, Tx::GraphicColor::YELLOW),
        ReticleTuple(50, 415, Tx::GraphicColor::YELLOW),
        ReticleTuple(40, 410, Tx::GraphicColor::ORANGE),  // 3 m
        ReticleTuple(40, 405, Tx::GraphicColor::ORANGE),
        ReticleTuple(60, 400, Tx::GraphicColor::ORANGE),
        ReticleTuple(40, 395, Tx::GraphicColor::ORANGE),
        ReticleTuple(40, 390, Tx::GraphicColor::ORANGE),
        ReticleTuple(10, 370, Tx::GraphicColor::YELLOW),  // 5 m
        ReticleTuple(10, 365, Tx::GraphicColor::YELLOW),
        ReticleTuple(30, 360, Tx::GraphicColor::YELLOW),
        ReticleTuple(10, 355, Tx::GraphicColor::YELLOW),
        ReticleTuple(10, 350, Tx::GraphicColor::YELLOW),
    };
    /** Size of TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES (so its easier to understand when used
     * in context). */
    static constexpr size_t NUM_RETICLE_COORDINATES = MODM_ARRAY_SIZE(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES);
    /** The color of the verticle line that connects the horizontal reticle lines. */
    static constexpr Tx::GraphicColor RETICLE_VERTICAL_COLOR = Tx::GraphicColor::YELLOW;

    tap::Drivers &drivers;

    /**
     * Array of `Graphic5Message`s that will be used to send all of the reticle related graphics.
     * This includes all of the reticle markers from `TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES`
     * plus a verticle line to connect the reticle markers.
     *
     * This is an array of `Graphic5Message`s. There are NUM_RETICLE_COORDINATES + 1 reticle
     * graphics that must be fit into N Graphic5Messages. Each Graphic5Message can hold 5 reticle
     * graphics. Thus there are (NUM_RETICLE_COORDINATES + 1)/5 Graphic5Message structs rounded up
     * to the nearest whole number required to fit the reticle graphics.
     */
    Tx::Graphic5Message reticleMsg[tap::algorithms::ceil(static_cast<float>(NUM_RETICLE_COORDINATES + 1) / 5.0f)];

    /** Index used when iterating through the reticleMsg in protothreads. */
    size_t reticleIndex = 0;
};

}  // namespace src::utils::display
