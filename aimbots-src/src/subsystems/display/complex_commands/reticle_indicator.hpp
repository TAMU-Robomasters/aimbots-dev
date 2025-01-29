#pragma once

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "modm/processing/resumable.hpp"

#include "subsystems/display/basic_commands/hud_indicator.hpp"

#include "subsystems/display/display_constants.hpp"

namespace tap {
class Drivers;
}

namespace src::Utils::ClientDisplay {
class ReticleIndicator : public HudIndicator, protected modm::Resumable<2> {
public:
    static constexpr int RETICLE_CENTER_X_OFFSET = 1;

    ReticleIndicator(tap::Drivers &drivers, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);
    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    /** Line thickness of the reticle, in pixels. */
    static constexpr uint16_t RETICLE_THICKNESS = 3;

    /** Tuple representing a possible horizontal reticle line. The first element is the pixel width
     * of the line, second is Y location of the line (in pixels), third is the color of the reticle
     * line. */
    using ReticleTuple = std::tuple<int16_t, int16_t, Tx::GraphicColor>;

    // static constexpr ReticleTuple TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[]{
    //     ReticleTuple(40, 545, Tx::GraphicColor::BLACK),  // 2 m
    //     ReticleTuple(60, 540, Tx::GraphicColor::BLACK),
    //     ReticleTuple(40, 535, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 500, Tx::GraphicColor::BLACK),  // 4 m
    //     ReticleTuple(40, 495, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 490, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 530, Tx::GraphicColor::BLACK),  // 8 m
    //     ReticleTuple(30, 525, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 520, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 475, Tx::GraphicColor::BLACK),  // next
    //     ReticleTuple(40, 470, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 465, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 445, Tx::GraphicColor::BLACK),  // next
    //     ReticleTuple(40, 440, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 435, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 415, Tx::GraphicColor::BLACK),  // next
    //     ReticleTuple(40, 410, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 405, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 385, Tx::GraphicColor::BLACK),  // next
    //     ReticleTuple(40, 380, Tx::GraphicColor::BLACK),
    //     ReticleTuple(10, 375, Tx::GraphicColor::BLACK)
    // };
    /** Size of TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES (so its easier to understand when used
     * in context). */
    static constexpr size_t NUM_RETICLE_COORDINATES = MODM_ARRAY_SIZE(TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES);
    /** The color of the verticle line that connects the horizontal reticle lines. */
    static constexpr Tx::GraphicColor RETICLE_VERTICAL_COLOR = SECONDARY_COLOR; //RED_AND_BLUE is assigned by team color

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

}  // namespace src::Utils::ClientDisplay
