#include "barrel_manager.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace tap::communication::serial {

}

namespace src::Barrel_Manager {

BarrelManagerSubsystem::BarrelManagerSubsystem(tap::Drivers* drivers) : tap::control::Subsystem(drivers), swapMotor(drivers,
                   SWAP_MOTOR_ID,
                   BARREL_BUS,
                   SWAP_DIRECTION,
                   "Swap Motor") {
    
}

//DEBUG VARIABLES
int16_t currentTorqueDisplay = 0;
float swapMotorPositionDisplay = 0;
bool isSwapOnlineDisplay = false;
float swapOutputDisplay = 0;
float currentSwapDesiredOutputDisplay = 0;
int calStepProg = 0;
bool currentTimer = 0;

//----------------------

void BarrelManagerSubsystem::initialize() {
    swapMotor.initialize();
    swapMotor.setDesiredOutput(0);
    currentSpikeTimer.execute();
}

void BarrelManagerSubsystem::refresh() {
    isSwapOnlineDisplay = swapMotor.isMotorOnline();

    if (swapMotor.isMotorOnline()) {
        int64_t swapMotorUnwrapedEncoder = swapMotor.getEncoderUnwrapped();
        currentSwapMotorPosition = swapMotorUnwrapedEncoder / LEAD_SCREW_TICKS_PER_MM;
        swapMotor.setDesiredOutput(desiredSwapMotorOutput);

        currentSwapDesiredOutputDisplay = desiredSwapMotorOutput;

        currentTorqueDisplay = swapMotor.getTorque();
        swapMotorPositionDisplay = swapMotor.getEncoderUnwrapped();
        swapOutputDisplay = swapMotor.getShaftRPM();
    }
}

void BarrelManagerSubsystem::setMotorOutput(float output) {
    desiredSwapMotorOutput = output;
}

float BarrelManagerSubsystem::getMotorOutput() {
    return swapMotor.isMotorOnline() ? swapMotor.getShaftRPM() : 0;
}

float BarrelManagerSubsystem::getMotorPosition() {
    return currentSwapMotorPosition; // in mm
}

bool BarrelManagerSubsystem::findZeroPosition(barrelSide stopSideToFind) {
    /*if (currentSpikeTimer.isStopped()) {
        currentSpikeTimer.restart(0);
        currentSpikeTimer.execute();
    }*/
    currentTimer = currentSpikeTimer.isExpired();
    //Slam into each wall and find current spike.  Save position at each wall to limitLRPositions
    //find limit
    setMotorOutput((stopSideToFind == barrelSide::LEFT) ? -500 : 500);// TODO: Confirm direction of stop sides
    calStepProg = 0;

    if (currentSpikeTimer.execute() && abs(swapMotor.getTorque()) >= LEAD_SCREW_CURRENT_SPIKE_TORQUE) {
            setMotorOutput(0);

            //when finished
            //This just determines whether values needed to be added or subtracted based on the direction of calibration
            float addSubDirection = (stopSideToFind == barrelSide::LEFT ? 1:-1);

            limitLRPositions[stopSideToFind] = getMotorPosition() + (addSubDirection*HARD_STOP_OFFSET);
            limitLRPositions[1-stopSideToFind] = getMotorPosition() + (addSubDirection*HARD_STOP_OFFSET) +(addSubDirection*BARREL_SWAP_DISTANCE_MM);

            calStepProg = 10;

            return true; //Return true if current spikes
    } 

    if(abs(swapMotor.getTorque()) >= LEAD_SCREW_CURRENT_SPIKE_TORQUE && (currentSpikeTimer.isExpired() || currentSpikeTimer.isStopped())) {
        currentSpikeTimer.restart(500);
        calStepProg = 5;
    }
    
    return false;
}

barrelSide BarrelManagerSubsystem::getSide() {
    return currentBarrelSide; // LEFT or RIGHT
}
void BarrelManagerSubsystem::setSide(barrelSide side) {
    switch (side)
    {
    default:
    case barrelSide::CURRENT:
        /* do nothing */
        // currentBarrelSide = currentBarrelSide;
        break;
    
    case barrelSide::LEFT:
        currentBarrelSide=barrelSide::LEFT;
        break;
    
    case barrelSide::RIGHT:
        currentBarrelSide=barrelSide::RIGHT;
        break;
    }
}

void BarrelManagerSubsystem::toggleSide() {
    currentBarrelSide = (currentBarrelSide == barrelSide::LEFT) ? barrelSide::RIGHT : barrelSide::LEFT;
}

float BarrelManagerSubsystem::getBarrelHeat(barrelSide side = CURRENT) {
    auto turretData = drivers->refSerial.getRobotData().turret;
    if (side == barrelSide::CURRENT){
        side = currentBarrelSide;
    }
    return(side == barrelSide::LEFT) ? turretData.heat17ID1 /*LEFT*/ : turretData.heat17ID2; //TODO: Check that left is ID1 and right is ID2  

}

bool BarrelManagerSubsystem::isBarrelAligned() {
    return abs(currentSwapMotorPosition - getSideInMM(currentBarrelSide)) <= 1.0; //TODO: Find an actually useful constant number
}

}  // namespace src::Shooter

#endif