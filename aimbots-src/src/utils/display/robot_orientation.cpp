#include "robot_orientation.hpp"

#include <tuple>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_scheduler.hpp"

#include "modm/math/geometry/vector2.hpp"
#include "modm/processing/resumable.hpp"
#include "subsystems/gimbal/gimbal.hpp"

#include "drivers.hpp"

using namespace tap::communication::serial;
using namespace tap::communication::referee;
using namespace src::Gimbal;

namespace src::utils::display {
RobotOrientation::RobotOrientation(
    tap::Drivers &drivers,
    tap::communication::serial::RefSerialTransmitter &RefSerialTransmitterm,
    const GimbalSubsystem &gimbal)
    : HudIndicator(refSerialTransmitter),
      drivers(drivers),
      gimbal(gimbal) {}

modm::ResumableResult<bool> RobotOrientation::sendInitialGraphics() {
    RF_BEGIN(0);

    RF_CALL(refSerialTransmitter.sendGraphic(&chassisOrientationGraphics));
    chassisOrientationGraphics.graphicData[0].operation = Tx::GRAPHIC_MODIFY;
    chassisOrientationGraphics.graphicData[1].operation = Tx::GRAPHIC_MODIFY;

    RF_END();
}

modm::ResumableResult<bool> RobotOrientation::update() {
    RF_BEGIN(1)

    chassisOrientation.rotate(-gimbal.getCurrentYawAngleFromChassisCenter(AngleUnit::Degrees));

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

    chassisOrientation.set(0, CHASSIS_LENGTH / 2);

    RF_END();
}

void RobotOrientation::initialize() {  // chassis orientation starts forward facing
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

}  // namespace src::utils::display
