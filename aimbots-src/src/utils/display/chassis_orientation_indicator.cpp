#include "chassis_orientation_indicator.hpp"

#include <tuple>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_scheduler.hpp"

#include "modm/math/geometry/vector2.hpp"
#include "modm/processing/resumable.hpp"
#include "subsystems/gimbal/gimbal.hpp"

#include "drivers.hpp"

using namespace src::Gimbal;
using namespace tap::communication::serial;

namespace src::Utils::ClientDisplay {
ChassisOrientationIndicator::ChassisOrientationIndicator(
    tap::Drivers &drivers,
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
    const GimbalSubsystem &gimbal)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers),
      gimbal(gimbal) {}

modm::ResumableResult<bool> ChassisOrientationIndicator::sendInitialGraphics() {
    RF_BEGIN(0)

    // send initial chassis orientation graphics
    RF_CALL(refSerialTransmitter.sendGraphic(&chassisOrientationGraphics));
    chassisOrientationGraphics.graphicData[0].operation = Tx::GRAPHIC_MODIFY;
    chassisOrientationGraphics.graphicData[1].operation = Tx::GRAPHIC_MODIFY;

    RF_END();
}

modm::ResumableResult<bool> ChassisOrientationIndicator::update() {
    RF_BEGIN(1);

    // update chassisOrientation if turret is online
    // otherwise don't rotate chassis
    chassisOrientation.rotate(-gimbal.getCurrentYawAxisAngle(AngleUnit::Radians));

    // if chassis orientation has changed, send new graphic with updated orientation
    if (chassisOrientation != chassisOrientationPrev) {
        // since chassisOrientation is a pixel coordinate centered around
        // `CHASSIS_CENTER_X/Y`, center the line about these coordinates during configuration
        RefSerialTransmitter::configLine(
            CHASSIS_WIDTH,
            CHASSIS_CENTER_X + chassisOrientation.x,
            CHASSIS_CENTER_Y + chassisOrientation.y,
            CHASSIS_CENTER_X - chassisOrientation.x,
            CHASSIS_CENTER_Y - chassisOrientation.y,
            &chassisOrientationGraphics.graphicData[0]);
        RF_CALL(refSerialTransmitter.sendGraphic(&chassisOrientationGraphics));

        chassisOrientationPrev = chassisOrientation;
    }

    // reset rotated orientation back to forward orientation so next time chassisOrientation
    // is rotated by `getYawAngleFromCenter` the rotation is relative to the forward.
    chassisOrientation.set(0, CHASSIS_LENGTH / 2);

    RF_END();
}

void ChassisOrientationIndicator::initialize() {
    // chassis orientation starts forward facing
    chassisOrientation.set(0, CHASSIS_LENGTH / 2);
    chassisOrientationPrev = chassisOrientation;

    uint8_t chassisOrientationName[3];
    getUnusedGraphicName(chassisOrientationName);

    // config the chassis graphic

    RefSerialTransmitter::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[0],
        chassisOrientationName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        CHASSIS_ORIENTATION_COLOR);

    RefSerialTransmitter::configLine(
        CHASSIS_WIDTH,
        CHASSIS_CENTER_X + chassisOrientation.x,
        CHASSIS_CENTER_Y + chassisOrientation.y,
        CHASSIS_CENTER_X - chassisOrientation.x,
        CHASSIS_CENTER_Y - chassisOrientation.y,
        &chassisOrientationGraphics.graphicData[0]);

    getUnusedGraphicName(chassisOrientationName);

    // config the turret graphic

    RefSerialTransmitter::configGraphicGenerics(
        &chassisOrientationGraphics.graphicData[1],
        chassisOrientationName,
        Tx::GRAPHIC_ADD,
        DEFAULT_GRAPHIC_LAYER,
        CHASSIS_BARREL_COLOR);

    RefSerialTransmitter::configLine(
        CHASSIS_BARREL_WIDTH,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y,
        CHASSIS_CENTER_X,
        CHASSIS_CENTER_Y + CHASSIS_BARREL_LENGTH,
        &chassisOrientationGraphics.graphicData[1]);
}

}  // namespace src::Utils::ClientDisplay
