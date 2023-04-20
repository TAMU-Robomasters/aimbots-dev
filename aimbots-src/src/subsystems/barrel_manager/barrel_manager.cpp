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


void BarrelManagerSubsystem::initialize() {
    swapMotor.initialize();
    swapMotor.setDesiredOutput(0);

}

void BarrelManagerSubsystem::refresh() {
    if (swapMotor.isMotorOnline()) {
        uint64_t swapMotorUnwrapedEncoder = swapMotor.getEncoderUnwrapped();
        currentSwapMotorPosition = swapMotorUnwrapedEncoder / LEAD_SCREW_TICKS_PER_MM;
        
        swapMotor.setDesiredOutput(desiredSwapMotorOutput);
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
    //Slam into each wall and find current spike.  Save position at each wall to limitLRPositions
    //find limit
    setMotorOutput((stopSideToFind == barrelSide::LEFT) ? -1 : 1);// TODO: Confirm direction of stop sides
    
    if(abs(swapMotor.getTorque()) >= LEAD_SCREW_CURRENT_SPIKE_TORQUE) {
        setMotorOutput(0);
        //when finished
        limitLRPositions[stopSideToFind] = currentSwapMotorPosition;
        return true; //Return true if current spikes
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
        currentBarrelSide=LEFT;
        break;
    
    case barrelSide::RIGHT:
        currentBarrelSide=RIGHT;
        break;
    }
}

void BarrelManagerSubsystem::toggleSide() {
    currentBarrelSide = (currentBarrelSide == LEFT) ? RIGHT : LEFT;
}

float BarrelManagerSubsystem::getBarrelHeat(barrelSide side = CURRENT) {
    auto turretData = drivers->refSerial.getRobotData().turret;
    if (side == CURRENT){
        side = currentBarrelSide;
    }
    return(side == LEFT) ? turretData.heat17ID1 /*LEFT*/ : turretData.heat17ID2; //TODO: Check that left is ID1 and right is ID2  

}

bool BarrelManagerSubsystem::isBarrelAligned() {
    return abs(currentSwapMotorPosition - getSideInMM(currentBarrelSide)) <= 1.0; //TODO: Find an actually useful constant number
}

}  // namespace src::Shooter

#endif