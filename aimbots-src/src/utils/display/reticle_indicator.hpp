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
        ReticleTuple(40, 500, Tx::GraphicColor::CYAN), // 2 m
        ReticleTuple(60, 495, Tx::GraphicColor::CYAN),
        ReticleTuple(40, 490, Tx::GraphicColor::CYAN),
        ReticleTuple(10, 455, Tx::GraphicColor::CYAN), // 4 m
        ReticleTuple(30, 450, Tx::GraphicColor::CYAN),
        ReticleTuple(10, 445, Tx::GraphicColor::CYAN),
        ReticleTuple(10, 485, Tx::GraphicColor::ORANGE),  // 8 m
        ReticleTuple(30, 480, Tx::GraphicColor::ORANGE),
        ReticleTuple(10, 475, Tx::GraphicColor::ORANGE),
    };
    /** Size of TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES (so its easier to understand when used
     * in context). */
    static constexpr size_t NUM_RETICLE_COORDINATES = MODM_ARRAY_SIZE(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES);
    /** The color of the verticle line that connects the horizontal reticle lines. */
    static constexpr Tx::GraphicColor RETICLE_VERTICAL_COLOR = Tx::GraphicColor::CYAN;

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
