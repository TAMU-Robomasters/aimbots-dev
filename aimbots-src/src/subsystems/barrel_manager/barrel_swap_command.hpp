#pragma once

#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#include "barrel_manager.hpp"
#include "drivers.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::BarrelManager {

class BarrelSwapCommand : public TapCommand {
public:
    BarrelSwapCommand(
        src::Drivers* drivers,
        BarrelManagerSubsystem* barrelSwap,
        src::Utils::RefereeHelperTurreted* RefHelper,
        bool& barrelMovingFlag,
        bool& barrelCaliDoneFlag,
        float ACCEPTABLE_HEAT_PERCENTAGE);

    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "barrel swap command"; }

private:
    src::Drivers* drivers;
    BarrelManagerSubsystem* barrelManager;
    src::Utils::RefereeHelperTurreted* refHelper;
    SmoothPID swapMotorPID;

    MilliTimeout gPressedTimeout;
    bool isGPressed = false;

    bool& barrelMovingFlag;
    bool& barrelCaliDoneFlag;
    bool barrelCalibratingFlag = true;

    bool wasRPressed = false;
    bool wasLogicSwitchRequested = false;

    float ACCEPTABLE_HEAT_PERCENTAGE;

    barrelSide currentCalibratingBarrel = barrelSide::LEFT;
};

}  // namespace src::BarrelManager

#endif