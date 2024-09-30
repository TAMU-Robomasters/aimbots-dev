#pragma once

#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "modm/math/geometry/vector2.hpp"
#include "modm/processing/resumable.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"

#include "subsystems/display/basic_commands/hud_indicator.hpp"

namespace tap {
class Drivers;
}

namespace src::Utils::ClientDisplay {
class ChassisOrientationIndicator : public HudIndicator, protected modm::Resumable<2> {
public:
    ChassisOrientationIndicator(
        tap::Drivers &drivers,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const src::Gimbal::GimbalSubsystem &gimbal);

    modm::ResumableResult<bool> sendInitialGraphics() override;

    modm::ResumableResult<bool> update() override;

    void initialize() override;

private:
    /** The X location of the center of the animated chassis on the screen, in pixels. */
    static constexpr uint16_t CHASSIS_CENTER_X = 1300;
    /** The Y location of the center of the animated chassis on the screen, in pixels. */
    static constexpr uint16_t CHASSIS_CENTER_Y = 100;
    /** The length of the animated chassis, in pixels. */
    static constexpr uint16_t CHASSIS_LENGTH = 100;
    /** The width of the animated chassis, in pixels. */
    static constexpr uint16_t CHASSIS_WIDTH = 70;
    /** The color of the animated chassis. */
    static constexpr Tx::GraphicColor CHASSIS_ORIENTATION_COLOR = Tx::GraphicColor::YELLOW;
    /** The color of the animated turret barrel in the chassis orientation graphic. */
    static constexpr Tx::GraphicColor CHASSIS_BARREL_COLOR = Tx::GraphicColor::WHITE;
    /** The width of the animated turret barrel, in pixels. */
    static constexpr uint16_t CHASSIS_BARREL_WIDTH = 10;
    /** The length of the animated turret barrel, in pixels. */
    static constexpr uint16_t CHASSIS_BARREL_LENGTH = 90;

    tap::Drivers &drivers;

    const src::Gimbal::GimbalSubsystem &gimbal;
    /**
     * Vector with origin `(0, 0)` and length CHASSIS_LENGTH / 2. The turret drawn on the screen is
     * considered to be pointing up in the y axis (of the screen). This vector can be rotated around
     * the origin by some amount to represent the rotation of the chassis orientation. A clockwise
     * rotation of the turret results in a counterclockwise rotation of the graphic (since the
     * graphic is the chassis relative to the turret).
     */
    modm::Vector2i chassisOrientation;
    /** Previous chassis orientation. Should be a local variable but cannot since it is in a
     * protothread. */
    modm::Vector2i chassisOrientationPrev;
    /**
     * Two graphics that represent chassis orientation. The first graphic is the line representing
     * the turret, and the second graphic is a thick line that represents the chassis and is rotated
     * some amount to represent chassis orientation.
     */
    Tx::Graphic2Message chassisOrientationGraphics;
};

}  // namespace src::Utils::ClientDisplay