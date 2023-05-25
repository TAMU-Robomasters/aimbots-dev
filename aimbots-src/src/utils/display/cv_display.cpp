#include "cv_display.hpp"

#include <tuple>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command_scheduler.hpp"

using namespace tap::communication::serial;
using namespace tap::communication::referee;

namespace src::utils::display {
CVDisplay::CVDisplay(tap::control::CommandScheduler &commandScheduler, tap::communication::serial::RefSerialTransmitter &RefSerialTransmitter)
    : HudIndicator(refSerialTransmitter),
      commandScheduler(commandScheduler) {}

modm::ResumableResult<bool> CVDisplay::sendInitialGraphics() {
    RF_BEGIN(0);

    RF_END();
}

modm::ResumableResult<bool> CVDisplay::update() {
    RF_BEGIN(1)
    RF_END();
}

void CVDisplay::initialize() {}

}  // namespace src::utils::display