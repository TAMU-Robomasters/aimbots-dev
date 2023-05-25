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
#include "subsystems/gimbal/gimbal.hpp"

#include "hud_indicator.hpp"
using namespace src::Gimbal;

namespace src::utils::display {
class RobotOrientation : public HudIndicator, protected modm::Resumable<2> {
public:
    RobotOrientation(
        tap::control::CommandScheduler &commandScheduler,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        GimbalSubsystem *gimbal);
    modm::ResumableResult<bool> sendInitialGraphics() override;

    modm::ResumableResult<bool> update() override;
    void initialize() override;

private:
    GimbalSubsystem *gimbal;
    tap::control::CommandScheduler &commandScheduler;
};

}  // namespace src::utils::display