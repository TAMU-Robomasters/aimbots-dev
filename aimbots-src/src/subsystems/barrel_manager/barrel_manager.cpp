#include "barrel_manager.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace tap::communication::serial {

}

namespace src::Barrel_Manager {

BarrelManagerSubsystem::BarrelManagerSubsystem(tap::Drivers* drivers,
    float HARD_STOP_OFFSET,
    float BARREL_SWAP_DISTANCE_MM,
    float BARRELS_ALIGNED_TOLERANCE,
    float LEAD_SCREW_TICKS_PER_MM,
    int16_t LEAD_SCREW_CURRENT_SPIKE_TORQUE,
    int16_t LEAD_SCREW_CALI_OUTPUT,
    SmoothPIDConfig BARREL_SWAP_POSITION_PID_CONFIG) : tap::control::Subsystem(drivers), 
                swapMotor(drivers, SWAP_MOTOR_ID, BARREL_BUS, BARREL_SWAP_DIRECTION, "Swap Motor"),
                HARD_STOP_OFFSET(HARD_STOP_OFFSET),
                BARREL_SWAP_DISTANCE_MM(BARREL_SWAP_DISTANCE_MM),
                BARRELS_ALIGNED_TOLERANCE(BARRELS_ALIGNED_TOLERANCE),
                LEAD_SCREW_TICKS_PER_MM(LEAD_SCREW_TICKS_PER_MM),
                LEAD_SCREW_CURRENT_SPIKE_TORQUE(LEAD_SCREW_CURRENT_SPIKE_TORQUE),
                LEAD_SCREW_CALI_OUTPUT(LEAD_SCREW_CALI_OUTPUT),
                BARREL_SWAP_POSITION_PID_CONFIG(BARREL_SWAP_POSITION_PID_CONFIG) {
    
}

//DEBUG VARIABLES
int16_t currentTorqueDisplay = 0;
float swapMotorPositionDisplay = 0;
bool isSwapOnlineDisplay = false;
float swapOutputDisplay = 0;
float currentSwapDesiredOutputDisplay = 0;
int calStepProgressDisplay = 0;
bool currentTimer = 0;

tap::communication::serial::RefSerialData::Rx::RobotData structDisplay;

int16_t barrelHeat = 0;
int16_t barrelMax = 0;
float barrelID = 0;

//----------------------

void BarrelManagerSubsystem::initialize() {
    swapMotor.initialize();
    swapMotor.setDesiredOutput(0);
    currentSpikeTimer.execute();
    fakeHeatGainTimer.restart(500);
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
    currentTimer = currentSpikeTimer.isExpired();
    //Slam into each wall and find current spike.  Save position at each wall to limitLRPositions
    //find limit
    setMotorOutput((stopSideToFind == barrelSide::LEFT) ? -LEAD_SCREW_CALI_OUTPUT : LEAD_SCREW_CALI_OUTPUT);
    calStepProgressDisplay = 0;

    if (currentSpikeTimer.execute() && abs(swapMotor.getTorque()) >= LEAD_SCREW_CURRENT_SPIKE_TORQUE) {
            setMotorOutput(0);

            //when finished
            //This just determines whether values needed to be added or subtracted based on the direction of calibration
            float addSubDirection = (stopSideToFind == barrelSide::LEFT ? 1:-1);

            limitLRPositions[stopSideToFind] = getMotorPosition() + (addSubDirection*HARD_STOP_OFFSET);
            limitLRPositions[1-stopSideToFind] = getMotorPosition() + (addSubDirection*HARD_STOP_OFFSET) +(addSubDirection*BARREL_SWAP_DISTANCE_MM);

            calStepProgressDisplay = 10;

            return true; //Return true if current spikes
    } 

    if(abs(swapMotor.getTorque()) >= LEAD_SCREW_CURRENT_SPIKE_TORQUE && (currentSpikeTimer.isExpired() || currentSpikeTimer.isStopped())) {
        currentSpikeTimer.restart(500);
        calStepProgressDisplay = 5;
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

int16_t BarrelManagerSubsystem::getRemainingBarrelHeat(barrelSide side = CURRENT) {
    using RefSerialRxData = tap::communication::serial::RefSerial::Rx;
    auto turretData = drivers->refSerial.getRobotData().turret;
    structDisplay = drivers->refSerial.getRobotData();
    if (side == barrelSide::CURRENT) {
        side = currentBarrelSide;
    }

    int16_t lastHeat = 0;
    int16_t heatLimit = 0;
    barrelID = 0;

    auto launcherID = turretData.launchMechanismID;
    switch (launcherID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
            lastHeat = turretData.heat17ID1;
            heatLimit = turretData.heatLimit17ID1;
            barrelID = 2;  //Don't worry about it
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
            lastHeat = turretData.heat17ID2;
            heatLimit = turretData.heatLimit17ID2;
            barrelID = 1;  //Don't worry about it
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM: {
            lastHeat = turretData.heat42;
            heatLimit = turretData.heatLimit42;
            barrelID = 3;
            break;
        }
        default:
            break;
    }


    //barrelHeat = (side == barrelSide::RIGHT) ? turretData.heat17ID1 : turretData.heat17ID2;
    //barrelMax = (side == barrelSide::RIGHT) ? turretData.heatLimit17ID1 : turretData.heatLimit17ID2;
    heatLimit = 100; //TODO: Remove this while using the server
    /*if (fakeHeatGainTimer.isExpired()) {
        barrel1_fakeHeat += (side == barrelSide::RIGHT) ? 10 : -10;
        barrel2_fakeHeat += (side == barrelSide::RIGHT) ? -10 : 10;
        fakeHeatGainTimer.restart(500);
    }*/

    //if (barrel1_fakeHeat < 0) {barrel1_fakeHeat = 0;}
    //if (barrel2_fakeHeat < 0) {barrel2_fakeHeat = 0;}

    //lastHeat = (side == barrelSide::RIGHT) ? barrel1_fakeHeat : barrel2_fakeHeat;
    barrelHeat = lastHeat;

    //barrelHeat = lastHeat;
    //barrelMax = heatLimit;
    //barrelHeat = turretData.heat17ID1;
    //barrelHeat = turretData.heat17ID2;
    return heatLimit - lastHeat;
    //return(side == barrelSide::RIGHT) ? turretData.heatLimit17ID1 - turretData.heat17ID1 /*LEFT*/ : turretData.heatLimit17ID2 - turretData.heat17ID2; //TODO: Check that left is ID1 and right is ID2  

}

bool BarrelManagerSubsystem::isBarrelAligned() {
    return abs(currentSwapMotorPosition - getSideInMM(currentBarrelSide)) <= BARRELS_ALIGNED_TOLERANCE; //TODO: Find an actually useful constant number
}

}  // namespace src::Shooter

#endif