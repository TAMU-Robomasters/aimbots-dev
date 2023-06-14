#include "barrel_swap_command.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::Barrel_Manager {

BarrelSwapCommand::BarrelSwapCommand(src::Drivers* drivers, BarrelManagerSubsystem* barrelManager, src::Utils::RefereeHelper* RefHelper, bool &barrelMovingFlag) : drivers(drivers),
    barrelManager(barrelManager),
    refHelper(RefHelper),
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

bool isCommandRunningDisplay = false;
bool calFlagDisplay = false;
bool isBarAlignedDisplay = false;
bool wasSwapDisplay = false;

float errorDisplay = 0;
float deriDisplay = 0;

int16_t heatRemainDisplay = 0;
int16_t currentBarrelDisplay = 0;

//-----------

void BarrelSwapCommand::initialize() {
    barrelCalibratingFlag = true;
    logicSwapTimeout.restart(0);

}

void BarrelSwapCommand::execute() {
    isCommandRunningDisplay = true;
    calFlagDisplay = barrelCalibratingFlag;

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


        float stickSwitchThres = 0.1;
        if (abs(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) - 1) >= stickSwitchThres && drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) > 0) {
            //barrelManager->setSide(barrelSide::RIGHT);
        }
        if (abs(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) - 1) >= stickSwitchThres && drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) < 0) {
            //barrelManager->setSide(barrelSide::LEFT);
        }

        if (wasLogicSwitchRequested && barrelManager->isBarrelAligned()) {
            wasLogicSwitchRequested = false;
            /*logicSwapTimeout.restart(300);*/
        }

        wasSwapDisplay = wasLogicSwitchRequested;
        heatRemainDisplay = refHelper->isBarrelHeatUnderLimit(0.80f);
        currentBarrelDisplay = refHelper->getCurrentBarrel();

        if (/*logicSwapTimeout.isExpired() && */!refHelper->isBarrelHeatUnderLimit(0.80f) && !wasLogicSwitchRequested) {
            wasLogicSwitchRequested = true;
            if (refHelper->getCurrentBarrel() == barrelSide::LEFT) {
                barrelManager->setSide(barrelSide::RIGHT);
            }
            else {
                barrelManager->setSide(barrelSide::LEFT);
            }
           
        }
        


    }
    else {
        barrelCalibratingFlag = !barrelManager->findZeroPosition(barrelSide::LEFT);
    }
    
}

void BarrelSwapCommand::end(bool) {
    barrelManager->setMotorOutput(0);
}

bool BarrelSwapCommand::isReady() { return true; }

bool BarrelSwapCommand::isFinished() const { return false; }

}  // namespace src::Barrel_Manager

#endif
