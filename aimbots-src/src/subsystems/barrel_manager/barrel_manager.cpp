#include "barrel_manager.hpp"

#ifdef BARREL_SWAP_COMPATIBLE

namespace tap::communication::serial {}

namespace src::BarrelManager {

BarrelManagerSubsystem::BarrelManagerSubsystem(
    tap::Drivers* drivers,
    float HARD_STOP_OFFSET,
    float BARREL_SWAP_DISTANCE_MM,
    float BARRELS_ALIGNED_TOLERANCE,
    float LEAD_SCREW_TICKS_PER_MM,
    int16_t LEAD_SCREW_CURRENT_SPIKE_TORQUE,
    int16_t LEAD_SCREW_CALI_OUTPUT,
    SmoothPIDConfig BARREL_SWAP_POSITION_PID_CONFIG,
    std::array<BarrelID, 2> BARREL_ARRAY,
    BarrelID& currentBarrel)
    : tap::control::Subsystem(drivers),
      swapMotor(drivers, SWAP_MOTOR_ID, BARREL_BUS, BARREL_SWAP_DIRECTION, "Barrel Swap Motor"),
      HARD_STOP_OFFSET(HARD_STOP_OFFSET),
      BARREL_SWAP_DISTANCE_MM(BARREL_SWAP_DISTANCE_MM),
      BARRELS_ALIGNED_TOLERANCE(BARRELS_ALIGNED_TOLERANCE),
      LEAD_SCREW_TICKS_PER_MM(LEAD_SCREW_TICKS_PER_MM),
      LEAD_SCREW_CURRENT_SPIKE_TORQUE(LEAD_SCREW_CURRENT_SPIKE_TORQUE),
      LEAD_SCREW_CALI_OUTPUT(LEAD_SCREW_CALI_OUTPUT),
      BARREL_SWAP_POSITION_PID_CONFIG(BARREL_SWAP_POSITION_PID_CONFIG),
      BARREL_ARRAY(BARREL_ARRAY),
      currentBarrel(currentBarrel) {}

// DEBUG VARIABLES
int16_t currentTorqueDisplay = 0;
float swapMotorPositionDisplay = 0;
bool isSwapOnlineDisplay = false;
float swapOutputDisplay = 0;
float currentSwapDesiredOutputDisplay = 0;
int calStepProgressDisplay = 0;
bool currentTimer = 0;

int16_t barrelHeat = 0;
int16_t barrelMax = 0;
float barrelID = 0;

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

        if (isBarrelAligned() && currentBarrelSide !=
            barrelSide::CURRENT ) {
            //currentBarrel = BARREL_ARRAY[currentBarrelSide];
        }

        currentTorqueDisplay = swapMotor.getTorque();
        swapMotorPositionDisplay = swapMotor.getEncoderUnwrapped();
        swapOutputDisplay = swapMotor.getShaftRPM();
    }

}

void BarrelManagerSubsystem::setMotorOutput(float output) { desiredSwapMotorOutput = output; }

float BarrelManagerSubsystem::getMotorOutput() { return swapMotor.isMotorOnline() ? swapMotor.getShaftRPM() : 0; }

float BarrelManagerSubsystem::getMotorPosition() {
    return currentSwapMotorPosition;  // in mm
}

bool BarrelManagerSubsystem::findZeroPosition(barrelSide stopSideToFind) {
    currentTimer = currentSpikeTimer.isExpired();
    // Slam into each wall and find current spike.  Save position at each wall to limitLRPositions
    setMotorOutput((stopSideToFind == barrelSide::LEFT) ? -LEAD_SCREW_CALI_OUTPUT : LEAD_SCREW_CALI_OUTPUT);
    calStepProgressDisplay = 0;

    if (currentSpikeTimer.execute() && abs(swapMotor.getTorque()) >= LEAD_SCREW_CURRENT_SPIKE_TORQUE) {
        setMotorOutput(0);

        // when finished
        // This just determines whether values needed to be added or subtracted based on the direction of calibration
        float addSubDirection = (stopSideToFind == barrelSide::LEFT ? 1 : -1);

        limitLRPositions[stopSideToFind] = getMotorPosition() + (addSubDirection * HARD_STOP_OFFSET);
        limitLRPositions[1 - stopSideToFind] =
            getMotorPosition() + (addSubDirection * HARD_STOP_OFFSET) + (addSubDirection * BARREL_SWAP_DISTANCE_MM);

        calStepProgressDisplay = 10;

        return true;  // Return true if current spikes
    }

    if (abs(swapMotor.getTorque()) >= LEAD_SCREW_CURRENT_SPIKE_TORQUE &&
        (currentSpikeTimer.isExpired() || currentSpikeTimer.isStopped())) {
        currentSpikeTimer.restart(500);
        calStepProgressDisplay = 5;
    }

    return false;
}

barrelSide BarrelManagerSubsystem::getSide() {
    return currentBarrelSide;  // LEFT or RIGHT
}

void BarrelManagerSubsystem::setSide(barrelSide side) {
    switch (side) {
        default:
        case barrelSide::CURRENT:
            /* do nothing */
            break;

        case barrelSide::LEFT:
            currentBarrelSide = barrelSide::LEFT;
            break;

        case barrelSide::RIGHT:
            currentBarrelSide = barrelSide::RIGHT;
            break;
    }
}

void BarrelManagerSubsystem::toggleSide() {
    currentBarrelSide = (currentBarrelSide == barrelSide::LEFT) ? barrelSide::RIGHT : barrelSide::LEFT;
}

bool BarrelManagerSubsystem::isBarrelAligned() {
    return abs(currentSwapMotorPosition - getSideInMM(currentBarrelSide)) <= BARRELS_ALIGNED_TOLERANCE;
}

}  // namespace src::BarrelManager

#endif