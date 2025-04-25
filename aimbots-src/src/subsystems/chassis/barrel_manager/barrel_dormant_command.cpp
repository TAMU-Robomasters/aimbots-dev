#include "barrel_dormant_command.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace src::BarrelManager {

//This command has no logic or buttons for swapping, 
//it should calibrate barrels, request re-calibration, and do nothing else otherwise


BarrelDormantCommand::BarrelDormantCommand(
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

//-----------

void BarrelDormantCommand::initialize() {}

void BarrelDormantCommand::execute() {
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

    barrelMovingFlag = barrelCalibratingFlag || !barrelManager->isBarrelAligned();

    if (!barrelCalibratingFlag) {
        float positionControllerError =
            barrelManager->getSideInMM(barrelManager->getSide()) - barrelManager->getMotorPosition();
        swapMotorPID.runController(positionControllerError, barrelManager->getMotorOutput());
        float swapPositionPIDOutput = swapMotorPID.getOutput();

        barrelManager->setMotorOutput(swapPositionPIDOutput);

    } else {
        barrelCalibratingFlag = !barrelManager->findZeroPosition(barrelSide::LEFT);
    }

    // Check this at the very end of the loop, after barrelCalibratingFlag has been updated
    barrelCaliDoneFlag = !barrelCalibratingFlag;
}

void BarrelDormantCommand::end(bool) { barrelManager->setMotorOutput(0); }

bool BarrelDormantCommand::isReady() { return true; }

bool BarrelDormantCommand::isFinished() const { return false; }

}  // namespace src::BarrelManager

#endif
