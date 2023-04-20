#include "barrel_swap_command.hpp"
#include "utils/robot_constants.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::Barrel_Manager {

BarrelSwapCommand::BarrelSwapCommand(src::Drivers* drivers, BarrelManagerSubsystem* barrelManager, bool &barrelMovingFlag) : drivers(drivers),
    barrelManager(barrelManager),
    swapMotorPID(BARREL_SWAP_POSITION_PID_CONFIG),
    barrelMovingFlag(barrelMovingFlag) 
    {
    
       addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(barrelManager));
}

void BarrelSwapCommand::initialize() {
    barrelCalibratingFlag = true;
    currentCalibratingBarrel = LEFT;
    // barrelManager->findZeroPosition();
    //barrelManager->setMotorOutput(0);

}

void BarrelSwapCommand::execute() {
    if (!barrelCalibratingFlag) {
        float positionControllerError = barrelManager->getMotorPosition() - barrelManager->getSideInMM(barrelManager->getSide());

        float swapPositionPIDOutput = swapMotorPID.runController(positionControllerError, barrelManager->getMotorOutput());

        barrelManager->setMotorOutput(swapPositionPIDOutput);

        if (drivers->remote.keyPressed(Remote::Key::R)) wasRPressed = true;

        if (wasRPressed && !drivers->remote.keyPressed(Remote::Key::R)) {
            wasRPressed = false;
            barrelManager->toggleSide();
        }

        barrelMovingFlag = barrelManager->isBarrelAligned();
    }
    else {
        if (barrelManager->findZeroPosition(currentCalibratingBarrel)) {
            if (currentCalibratingBarrel == LEFT) { // LEFT first
                currentCalibratingBarrel = RIGHT;
            }
            else {
                barrelCalibratingFlag = false; // done calibrating
            }
        } 
    }
    
}

void BarrelSwapCommand::end(bool) {}

bool BarrelSwapCommand::isReady() { return true; }

bool BarrelSwapCommand::isFinished() const { return false; }

}  // namespace src::Barrel_Manager

#endif
