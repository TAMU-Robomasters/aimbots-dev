#pragma once

#include "barrel_manager.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"


#include "drivers.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::Barrel_Manager {

class BarrelSwapCommand : public TapCommand {
public:

    //Add a pointer to a boolean flag that can be changed to alert the feeder that a transition is happening
    BarrelSwapCommand(src::Drivers* drivers, BarrelManagerSubsystem* barrelSwap);

    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "barrel swap command"; }

private:
    src::Drivers* drivers;
    BarrelManagerSubsystem* barrelManager;
    SmoothPID swapMotorPID;

    bool wasRPressed = false;


};

}  // namespace src::Barrel_Manager

#endif