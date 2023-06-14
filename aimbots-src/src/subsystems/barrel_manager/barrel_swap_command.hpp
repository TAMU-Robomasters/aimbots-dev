#pragma once

#include "barrel_manager.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "utils/ref_helper.hpp"

#include "drivers.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::Barrel_Manager {

class BarrelSwapCommand : public TapCommand {
public:

    BarrelSwapCommand(src::Drivers* drivers, BarrelManagerSubsystem* barrelSwap, src::Utils::RefereeHelper* RefHelper, bool &barrelMovingFlag, float ACCEPTABLE_HEAT_PERCENTAGE);

    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "barrel swap command"; }

private:
    src::Drivers* drivers;
    BarrelManagerSubsystem* barrelManager;
    src::Utils::RefereeHelper* refHelper;
    SmoothPID swapMotorPID;

    MilliTimeout logicSwapTimeout;

    bool &barrelMovingFlag;
    bool barrelCalibratingFlag = false;

    bool wasRPressed = false;
    bool wasLogicSwitchRequested = false;

    float ACCEPTABLE_HEAT_PERCENTAGE;
    
    barrelSide currentCalibratingBarrel = barrelSide::LEFT;


};

}  // namespace src::Barrel_Manager

#endif