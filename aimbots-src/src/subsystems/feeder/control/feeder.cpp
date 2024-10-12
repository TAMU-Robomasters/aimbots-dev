#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "utils/tools/common_types.hpp"


#ifdef FEEDER_COMPATIBLE



namespace src::Feeder {

// watch varables
    int desiredFeederMotorOutputDisplay = 0;
    int inputDisplay = 0;

FeederSubsystem::FeederSubsystem(src::Drivers* drivers) :
    Subsystem(drivers),
    drivers(drivers),
    feederMotor(drivers, FEEDER_MOTOR_ID, FEEDER_BUS, FEEDER_DIRECTION, "feederMotor"),
    feederVelocityPID(FEEDER_VELOCITY_PID_CONFIG),
    feederTargetRPM(0),
    desiredFeederMotorOutput(0) {
        feederMotor.initialize();
    }




void FeederSubsystem::initialize() {
    
}

void FeederSubsystem::refresh() {
    updateMotorVelocityPID();
    setDesiredFeederMotorOutput();
    desiredFeederMotorOutputDisplay = desiredFeederMotorOutput;
}


void FeederSubsystem::updateMotorVelocityPID(){
    feederVelocityPID.runControllerDerivateError(feederTargetRPM - feederMotor.getShaftRPM());
    desiredFeederMotorOutput = feederVelocityPID.getOutput();
}

void FeederSubsystem::setDesiredFeederMotorOutput() {

    feederMotor.setDesiredOutput(desiredFeederMotorOutput);
}

void FeederSubsystem::setTargetRPM(int input){
    feederTargetRPM = input;
    inputDisplay = input;
}









}
#endif  // #ifdef FEEDER_COMPATIBLE

// src::Drivers* drivers;

// float desiredFeederMotorOutput;

// float feederTargetRPM;

// SmoothPID feederVelocityPID;

// DJIMotor feederMotor;