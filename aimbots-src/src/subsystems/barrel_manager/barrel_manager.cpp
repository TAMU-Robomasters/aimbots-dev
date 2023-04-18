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
        switch (barrelState) // finding zero or not
        {
        default:
        case 2: // default operation
            // based on barrel position decide whether to swap or not
            if (!isBarrelAligned()) {
                //TODO: Drive motor towards position using PID
                //Should not drive motor once close to the correct position

            }


            break;
        case 0: // LEFT
            // find left limit
            setMotorOutput(1/* TODO: HOMING SPEED */);// move left at homing speed

            if(true/* TODO: Current Spikes */) { 
                setMotorOutput(0);// stop motor
                //when finished
                limitLRPositions[LEFT] = currentSwapMotorPosition;
                barrelState = 1;// go to RIGHT
            }
            break;
        case 1: // RIGHT
            // find right limit
            setMotorOutput(-1/* HOMING SPEED */);// move right at homing speed
            if(true/* Current Spikes */) {
                setMotorOutput(0);
                //when finished
                limitLRPositions[RIGHT] = currentSwapMotorPosition;
                barrelState = 2;//Return to default
            }
            break;
        }


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

bool BarrelManagerSubsystem::findZeroPosition() {
    //Slam into each wall and find current spike.  Save position at each wall to limitLRPositions
    barrelState = 0;
    //TODO
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