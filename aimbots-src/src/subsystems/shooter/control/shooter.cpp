#include "subsystems/shooter/control/shooter.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "utils/tools/common_types.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter {

    //watch variables
    int shooter1rpm = 0;
    int shooter2rpm = 0;

    ShooterSubsystem::ShooterSubsystem(tap::Drivers* drivers) :
    Subsystem(drivers),
    // drivers(drivers),
    flywheel1(drivers, SHOOTER_1_ID, SHOOTER_BUS, SHOOTER_1_DIRECTION, "shooter1"),
    flywheel2(drivers, SHOOTER_2_ID, SHOOTER_BUS, SHOOTER_2_DIRECTION, "shooter2"),
    flywheel1PID(SHOOTER_VELOCITY_PID_CONFIG),
    flywheel2PID(SHOOTER_VELOCITY_PID_CONFIG),
    targetRPMs({0.0f,0.0f}),
    desiredOutputs({0,0})
    {
        motors.at(0) = &flywheel1;
        motors.at(1) = &flywheel2;
        velocityPIDs.at(0) = &flywheel1PID;
        velocityPIDs.at(1) = &flywheel2PID;
        motors.at(0)->initialize();
        motors.at(1)->initialize();
    }


void ShooterSubsystem::initialize(){}

void ShooterSubsystem::refresh(){
    updateMotorVelocityPID(0);
    updateMotorVelocityPID(1);
    setDesiredOutput(0, desiredOutputs.at(0));
    setDesiredOutput(1, desiredOutputs.at(1));
    setDesiredOutputToMotor(0);
    setDesiredOutputToMotor(1);
}

float ShooterSubsystem::getMotorRPM(uint8_t motorIdx) const{
    return 0;
}

void ShooterSubsystem::updateMotorVelocityPID(uint8_t motorIdx){
    velocityPIDs.at(motorIdx)->runControllerDerivateError(targetRPMs.at(motorIdx) - motors.at(motorIdx)->getShaftRPM());
    desiredOutputs.at(motorIdx) = velocityPIDs.at(motorIdx)->getOutput();
}

void ShooterSubsystem::setTargetRPM(uint8_t motorIdx, float targetRPM){
    targetRPMs.at(motorIdx) = targetRPM;
    if(motorIdx == 0){
        shooter1rpm = targetRPM;
    }else{
        shooter2rpm = targetRPM;
    }
}

void ShooterSubsystem::setDesiredOutput(uint8_t motorIdx, float desiredOutput){
    desiredOutputs.at(motorIdx) = desiredOutput;
}

void ShooterSubsystem::setDesiredOutputToMotor(uint8_t motorIdx){
   motors.at(motorIdx)->setDesiredOutput(desiredOutputs.at(motorIdx));
}


    // DJIMotor flywheel1, flywheel2;
    // SmoothPID flywheel1PID, flywheel2PID;

    // std::array<float, SHOOTER_MOTOR_COUNT> targetRPMs;
    // std::array<int32_t, SHOOTER_MOTOR_COUNT> desiredOutputs;
    // std::array<DJIMotor*, SHOOTER_MOTOR_COUNT> motors;

    // std::array<SmoothPID*, SHOOTER_MOTOR_COUNT> velocityPIDs;

}
#endif  //#ifdef SHOOTER_COMPATIBLE