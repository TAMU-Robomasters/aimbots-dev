#include "barrel_swap_command.hpp"
#include "utils/robot_constants.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::Barrel_Manager {

BarrelSwapCommand::BarrelSwapCommand(src::Drivers* drivers, BarrelManagerSubsystem* barrelManager) : drivers(drivers),
    barrelManager(barrelManager),
    swapMotorPID(BARREL_SWAP_POSITION_PID_CONFIG) 
    {
    
       addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(barrelManager));
}

void BarrelSwapCommand::initialize() {
    barrelManager->findZeroPosition();
    barrelManager->setMotorOutput(0);

}

void BarrelSwapCommand::execute() {
    float positionControllerError = barrelManager->getMotorPosition() - barrelManager->getSideInMM(barrelManager->getSide());

    float swapPositionPIDOutput = swapMotorPID.runController(positionControllerError, barrelManager->getMotorOutput());

    barrelManager->setMotorOutput(swapPositionPIDOutput);

    if (drivers->remote.keyPressed(Remote::Key::R)) wasRPressed = true;

    if (wasRPressed && !drivers->remote.keyPressed(Remote::Key::R)) {
        wasRPressed = false;
        barrelManager->toggleSide();
    }

    

}

void BarrelSwapCommand::end(bool) {}

bool BarrelSwapCommand::isReady() { return true; }

bool BarrelSwapCommand::isFinished() const { return false; }

}  // namespace src::Barrel_Manager

#endif
