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
bool isBarAlignedDisplay = false;
bool wasSwapDisplay = false;

float errorDisplay = 0;
float deriDisplay = 0;

int16_t heatRemainDisplay = 0;

//-----------

void BarrelSwapCommand::initialize() {
    barrelCalibratingFlag = true;

}

void BarrelSwapCommand::execute() {
    isCommandRunning = true;
    calFlag = barrelCalibratingFlag;

    barrelMovingFlag = barrelCalibratingFlag || !barrelManager->isBarrelAligned();

    isBarAlignedDisplay = barrelManager->isBarrelAligned();

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

        //---------------------- LOGIC FOR BARREL SWAPPING GOES HERE --------------------------------------------------------

        if (drivers->remote.keyPressed(Remote::Key::R)) wasRPressed = true;

        if (wasRPressed && !drivers->remote.keyPressed(Remote::Key::R)) {
            wasRPressed = false;
            barrelManager->toggleSide();
        }
        //
        //drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP || drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::MID
        float stickSwitchThres = 0.1;
        if (abs(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) - 1) >= stickSwitchThres && drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) > 0) {
            barrelManager->setSide(barrelSide::LEFT);
        }
        if (abs(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) - 1) >= stickSwitchThres && drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) < 0) {
            barrelManager->setSide(barrelSide::RIGHT);
        }

        if (wasLogicSwitchRequested && barrelManager->isBarrelAligned()) {wasLogicSwitchRequested = false;}
        wasSwapDisplay = wasLogicSwitchRequested;
        heatRemainDisplay = barrelManager->getRemainingBarrelHeat(barrelSide::CURRENT);
        if (barrelManager->getRemainingBarrelHeat(barrelSide::CURRENT) <= 5 && !wasLogicSwitchRequested) {
            barrelManager->toggleSide();
            //barrelManager->setSide(barrelSide::RIGHT);
            wasLogicSwitchRequested = true;
        }
        


    }
    else {
        barrelCalibratingFlag = !barrelManager->findZeroPosition(barrelSide::LEFT);
        /*if (barrelManager->findZeroPosition(currentCalibratingBarrel)) {
            if (currentCalibratingBarrel == barrelSide::LEFT) { // LEFT first
                currentCalibratingBarrel = barrelSide::RIGHT;
            }
            else {
                barrelCalibratingFlag = false; // done calibrating
            }
        }*/ 
    }
    
}

void BarrelSwapCommand::end(bool) {
    barrelManager->setMotorOutput(0);
}

bool BarrelSwapCommand::isReady() { return true; }

bool BarrelSwapCommand::isFinished() const { return false; }

}  // namespace src::Barrel_Manager

#endif
