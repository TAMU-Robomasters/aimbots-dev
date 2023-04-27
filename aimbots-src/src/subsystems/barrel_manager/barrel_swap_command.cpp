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

//DEBUG VARIABLES
float PIDDisplay = 0;
float positionErrorDisplay = 0;
float sideInMMDisplay = 0;
float motorPositionDisplay = 0;

bool isCommandRunning = false;
bool calFlag = false;

float errorDisplay = 0;
float deriDisplay = 0;

//-----------

void BarrelSwapCommand::initialize() {
    barrelCalibratingFlag = true;

    //currentCalibratingBarrel = barrelSide::LEFT;
    // barrelManager->findZeroPosition();
    //barrelManager->setMotorOutput(0);

}

void BarrelSwapCommand::execute() {
    isCommandRunning = true;
    calFlag = barrelCalibratingFlag;
    if (!barrelCalibratingFlag) {
        sideInMMDisplay = barrelManager->getSideInMM(barrelManager->getSide());
        motorPositionDisplay = barrelManager->getMotorPosition();
        float positionControllerError = barrelManager->getSideInMM(barrelManager->getSide()) - barrelManager->getMotorPosition();
        swapMotorPID.runController(positionControllerError, barrelManager->getMotorOutput());
        float swapPositionPIDOutput = swapMotorPID.getOutput();

        errorDisplay = swapMotorPID.getError();
        deriDisplay = swapMotorPID.getDerivative();

        PIDDisplay = swapPositionPIDOutput;
        positionErrorDisplay = positionControllerError;

        barrelManager->setMotorOutput(swapPositionPIDOutput);

        if (drivers->remote.keyPressed(Remote::Key::R)) wasRPressed = true;

        if (wasRPressed && !drivers->remote.keyPressed(Remote::Key::R)) {
            wasRPressed = false;
            barrelManager->toggleSide();
        }

        if (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP) {
            barrelManager->setSide(barrelSide::LEFT);
        }
        else {
            barrelManager->setSide(barrelSide::RIGHT);
        }

        barrelMovingFlag = barrelManager->isBarrelAligned();
    }
    else {
        if (barrelManager->findZeroPosition(currentCalibratingBarrel)) {
            if (currentCalibratingBarrel == barrelSide::LEFT) { // LEFT first
                currentCalibratingBarrel = barrelSide::RIGHT;
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
