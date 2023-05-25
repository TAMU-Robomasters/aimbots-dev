#include "robot_orientation.hpp"

#include <tuple>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_scheduler.hpp"

#include "subsystems/gimbal/gimbal.hpp"

using namespace tap::communication::serial;
using namespace tap::communication::referee;
using namespace src::Gimbal;

namespace src::utils::display {
RobotOrientation::RobotOrientation(
    tap::control::CommandScheduler &commandScheduler,
    tap::communication::serial::RefSerialTransmitter &RefSerialTransmitterm,
    GimbalSubsystem *gimbal)
    : HudIndicator(refSerialTransmitter),
      commandScheduler(commandScheduler),
      gimbal(gimbal) {}

modm::ResumableResult<bool> RobotOrientation::sendInitialGraphics() {
    RF_BEGIN(0);

    RF_END();
}

modm::ResumableResult<bool> RobotOrientation::update() {
    RF_BEGIN(1)
    RF_END();
}

void RobotOrientation::initialize() {}
}  // namespace src::utils::display
