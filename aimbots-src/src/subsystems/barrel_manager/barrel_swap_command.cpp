#include "barrel_swap_command.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::BarrelManager {

BarrelSwapCommand::BarrelSwapCommand(
    src::Drivers* drivers,
    BarrelManagerSubsystem* barrelManager,
    src::Utils::RefereeHelperTurreted* RefHelper,
    bool& barrelMovingFlag,
    bool& barrelCaliDoneFlag)
    : drivers(drivers),
      barrelManager(barrelManager),
      refHelper(RefHelper),
      swapMotorPID(BARREL_SWAP_POSITION_PID_CONFIG),
      barrelMovingFlag(barrelMovingFlag),
      barrelCaliDoneFlag(barrelCaliDoneFlag) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(barrelManager));
}

// DEBUG VARIABLES
float PIDDisplay = 0;
float positionErrorDisplay = 0;
float sideInMMDisplay = 0;
float motorPositionDisplay = 0;

bool isCommandRunningDisplay = false;
bool calFlagDisplay = false;
bool isBarAlignedDisplay = false;
bool wasSwapDisplay = false;

float errorDisplay = 0;

int16_t heatRemainDisplay = 0;


bool currBHeatDisplay = false;
BarrelID currentBarrelDisplay;
//-----------

void BarrelSwapCommand::initialize() {}

void BarrelSwapCommand::execute() {
    if (drivers->remote.keyPressed(Remote::Key::G) && !isGPressed) {
        gPressedTimeout.restart(1000);
    }

    isGPressed = drivers->remote.keyPressed(Remote::Key::G);

    if (gPressedTimeout.execute() && isGPressed) {
        barrelCalibratingFlag = true;
    }

    if (!barrelCaliDoneFlag) {
        barrelCalibratingFlag = true;
    }

    isCommandRunningDisplay = true;
    calFlagDisplay = barrelCalibratingFlag;

    barrelMovingFlag = barrelCalibratingFlag || !barrelManager->isBarrelAligned();

    isBarAlignedDisplay = barrelManager->isBarrelAligned();

    if (!barrelCalibratingFlag) {
        sideInMMDisplay = barrelManager->getSideInMM(barrelManager->getSide());
        motorPositionDisplay = barrelManager->getMotorPosition();
        float positionControllerError =
            barrelManager->getSideInMM(barrelManager->getSide()) - barrelManager->getMotorPosition();
        swapMotorPID.runController(positionControllerError, barrelManager->getMotorOutput());
        float swapPositionPIDOutput = swapMotorPID.getOutput();

        errorDisplay = swapMotorPID.getError();

        PIDDisplay = swapPositionPIDOutput;
        positionErrorDisplay = positionControllerError;

        barrelManager->setMotorOutput(swapPositionPIDOutput);

        //---------------------- LOGIC FOR BARREL SWAPPING GOES HERE --------------------------------------------------------

        if (drivers->remote.keyPressed(Remote::Key::R)) wasRPressed = true;

        if (wasRPressed && !drivers->remote.keyPressed(Remote::Key::R)) {
            wasRPressed = false;
            barrelManager->toggleSide();
        }

        /*if (abs(drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID)) {
            barrelManager->setSide(barrelSide::RIGHT);
        }
        if (abs(drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP)) {
            barrelManager->setSide(barrelSide::LEFT);
        }*/

        if (wasLogicSwitchRequested && barrelManager->isBarrelAligned()) {
            wasLogicSwitchRequested = false;
        }

        wasSwapDisplay = wasLogicSwitchRequested;

        if (!refHelper->canCurrBarrelShootSafely() && !wasLogicSwitchRequested) {
            wasLogicSwitchRequested = true;
            barrelManager->toggleSide();
        }

    } else {
        barrelCalibratingFlag = !barrelManager->findZeroPosition(barrelSide::LEFT);
    }

    // Check this at the very end of the loop, after barrelCalibratingFlag has been updated
    barrelCaliDoneFlag = !barrelCalibratingFlag;


    currBHeatDisplay = refHelper->isCurrBarrelHeatUnderLimit(0.5f);
    currentBarrelDisplay = refHelper->getCurrentBarrel();
}

void BarrelSwapCommand::end(bool) { barrelManager->setMotorOutput(0); }

bool BarrelSwapCommand::isReady() { return true; }

bool BarrelSwapCommand::isFinished() const { return false; }

}  // namespace src::BarrelManager

#endif
