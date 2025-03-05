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

namespace src::Utils::HUDClientDisplay {
class BoolyBear : public HudIndicator, protected modm::Resumable<2> {
public:
    static constexpr int BOOLY_CENTER_X_OFFSET = 1;

    BoolyBear(tap::Drivers &drivers, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);
    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    /** Line thickness of the booly, in pixels. */
    static constexpr uint16_t BOOLY_THICKNESS = 3;

    /** Tuple representing a possible horizontal booly line. The first element is the pixel width
     * of the line, second is Y location of the line (in pixels), third is the color of the booly
     * line. */
    using ReticleTuple = std::tuple<int16_t, int16_t, Tx::GraphicColor>;

    // static constexpr BoolyTuple TURRET_BOOLY_X_WIDTH_AND_Y_POS_COORDINATES[]{
    //     BoolyTuple(40, 545, Tx::GraphicColor::BLACK),  // 2 m
    //     BoolyTuple(60, 540, Tx::GraphicColor::BLACK),
    //     BoolyTuple(40, 535, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 500, Tx::GraphicColor::BLACK),  // 4 m
    //     BoolyTuple(40, 495, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 490, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 530, Tx::GraphicColor::BLACK),  // 8 m
    //     BoolyTuple(30, 525, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 520, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 475, Tx::GraphicColor::BLACK),  // next
    //     BoolyTuple(40, 470, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 465, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 445, Tx::GraphicColor::BLACK),  // next
    //     BoolyTuple(40, 440, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 435, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 415, Tx::GraphicColor::BLACK),  // next
    //     BoolyTuple(40, 410, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 405, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 385, Tx::GraphicColor::BLACK),  // next
    //     BoolyTuple(40, 380, Tx::GraphicColor::BLACK),
    //     BoolyTuple(10, 375, Tx::GraphicColor::BLACK)
    // };
    /** Size of TURRET_BOOLY_X_WIDTH_AND_Y_POS_COORDINATES (so its easier to understand when used
     * in context). */
    static constexpr size_t NUM_BOOLY_COORDINATES = MODM_ARRAY_SIZE(TURRET_BOOLY_X_WIDTH_AND_Y_POS_COORDINATES);
    /** The color of the verticle line that connects the horizontal booly lines. */
    static constexpr Tx::GraphicColor BOOLY_VERTICAL_COLOR = SECONDARY_COLOR; //RED_AND_BLUE is assigned by team color

    tap::Drivers &drivers; 

    /**
     * Array of `Graphic5Message`s that will be used to send all of the booly related graphics.
     * This includes all of the booly markers from `TURRET_BOOLY_X_WIDTH_AND_Y_POS_COORDINATES`
     * plus a verticle line to connect the booly markers.
     *
     * This is an array of `Graphic5Message`s. There are NUM_BOOLY_COORDINATES + 1 booly
     * graphics that must be fit into N Graphic5Messages. Each Graphic5Message can hold 5 booly
     * graphics. Thus there are (NUM_BOOLY_COORDINATES + 1)/5 Graphic5Message structs rounded up
     * to the nearest whole number required to fit the booly graphics.
     */
    Tx::Graphic5Message boolyMsg[tap::algorithms::ceil(static_cast<float>(NUM_BOOLY_COORDINATES + 1) / 5.0f)];

    /** Index used when iterating through the boolyMsg in protothreads. */
    size_t boolyIndex = 0;
};

}  // namespace src::Utils::ClientDisplay
