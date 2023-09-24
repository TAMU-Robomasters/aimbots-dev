#pragma once
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_scheduler.hpp"
// #include "tap/drivers.hpp"
#include <tuple>

#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"

#include "modm/processing/resumable.hpp"
#include "utils/ballistics_solver.hpp"

#include "hud_indicator.hpp"

using namespace src::Utils::Ballistics;

namespace src::Utils::ClientDisplay {
class CVDisplay : public HudIndicator, protected modm::Resumable<3> {
public:
    // TODO: need to add the cv pipe line when i get it and the actual darwings
    CVDisplay(tap::communication::serial::RefSerialTransmitter &refSerialTransmitter, BallisticsSolver &balasticSolver);
    modm::ResumableResult<bool> sendInitialGraphics() override;

    modm::ResumableResult<bool> update() override;
    void initialize() override;

private:
    static constexpr int VISION_TARGET_FOUND_SQUARE_WIDTH = 15;
    /** The x location (in pixels) from the center of the reticle where the vision target found
     * squares are located. */
    static constexpr int VISION_TARGET_FOUND_X_DISTANCE_FROM_CENTER = 100;
    /** The y location (in pixels) where the vision target found squares are located. */
    static constexpr int VISION_TARGET_FOUND_Y_LOCATION = 425;
    /** The color of the vision target found squares. */
    static constexpr Tx::GraphicColor VISION_TARGET_FOUND_COLOR = Tx::GraphicColor::GREEN;
    /** The maximum refresh rate of the vision target found squares. */
    static constexpr uint32_t VISION_TARGET_FOUND_MAX_REFRESH_RATE = 250;

    BallisticsSolver &balasticSolver;

    modm::ResumableResult<bool> updateVisionTargetStatus();

    tap::arch::MilliTimeout updateVisionTargetFoundTimeout;

    std::optional<Tx::GraphicColor> prevVisionIndicatorColor = std::nullopt;
    std::optional<Tx::GraphicColor> newVisionIndicatorColor = std::nullopt;

    Tx::Graphic2Message visionTargetFoundGraphics;

    /**
     * Initialize some vision hud indicator (a little square) with some x pixel location
     * (xBoxLocation), relative to the leftmost side of the screen.
     */
    void initializeVisionHudIndicator(Tx::GraphicData *graphicData, int xBoxLocation);
};

}  // namespace src::Utils::ClientDisplay