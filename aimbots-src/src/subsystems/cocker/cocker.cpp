#include "cocker.hpp"

#ifdef COCKER_COMPATIBLE

namespace src::Cocker {

CockerSubsystem::CockerSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
    drivers(drivers),
    cockerMotors{
        buildMotor(ONE),
        buildMotor(TWO)
    },
    motorPIDs{
        SmoothPID(COCKER_POSITION_PID_CONFIG),
        SmoothPID(COCKER_POSITION_PID_CONFIG)
    }
{
}

void CockerSubsystem::initialize() {
    ForAllCockerMotors(&DJIMotor::initialize);
}

// refreshes the velocity PID given the target RPM and the current RPM
void CockerSubsystem::refresh() {
    ForAllCockerMotors(&CockerSubsystem::setDesiredOutputToMotor);
}


void CockerSubsystem::updateAllPIDs() {
    ForAllCockerMotors(&CockerSubsystem::updateMotorPID);
}
void CockerSubsystem::updateMotorPID(MotorIndex motorIndex){
    // fix / figure out !!
    float errorMeters = targetPositionsMeters[motorIndex] - getCurrentPositionMeters(motorIndex);
    float errorMeters = 0.0f;
    float errorDerivative = getMotorRPM(motorIndex) * COCKER_MOTOR_RADII_METERS[motorIndex];
    float output = motorPIDs[motorIndex].runController(errorMeters, errorDerivative);
    desiredOutputs[motorIndex] = output;
}

/*
void WristSubsystem::updateAllPIDs() {
    // if (motor_control_setting == VELOCITY) {
    //     ForAllWristMotors(&WristSubsystem::updateMotorPID_velocity);
    // } else if (motor_control_setting == POSITION) {
    //     ForAllWristMotors(&WristSubsystem::updateMotorPID);
    // }
    ForAllWristMotors(&WristSubsystem::updateMotorPID);
}

void WristSubsystem::updateMotorPID(MotorIndex idx) {
    float errorRadians = targetAnglesRads[idx] - getScaledUnwrappedRadiansOffset(idx);
    float errorDerivative = getMotorRPM(idx);
    float output = positionPIDs[idx].runController(errorRadians, errorDerivative);

    desiredMotorOutputs[idx] = output;
}

*/

float CockerSubsystem::getCurrentPositionMeters(MotorIndex motorIndex) const
{
    //todo: this function
    return 0.0f; 
}


}  // namespace src::Indexer

#endif // #ifdef COCKER_COMPATIBLE