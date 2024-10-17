#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"

namespace src::Feeder {

    FeeerSubsystem::FeederSubsystem(src::Drivers* drivers) {} // , SmoothPID(SmoothPID())

    void FeederSubsystem::initialize() {
        desiredFeederMotorOutput = 0;
        setTargetRPM(0);
        feederMotor.initialize();
    }

    void FeederSubsystem::refresh() { //cannot have parameter
        updateMotorVelocityPID(feederTargetRPM);
        setDesiredFeederMotorOutput();

    }

    float FeederSubsystem::getCurrentRPM() {
        return feederMotor.getShaftRPM();
    }

    void FeederSubsystem::setTargetRPM(float rpm) { //looking for current speed
        feederTargetRPM = rpm;
        // feederMotor.setDesiredOutputToMotor(feederTargetRPM);
    }

    void FeederSubsystem::updateMotorVelocityPID(){ //pid.runControllerDerivateError(targetRPm - currentRPM)
        feederVelocityPID.runControllerDerivateError(feederTargetRPM - getCurrentRPM());    // Smoothing 
    }

    //targetRPM is determined by commands 
    void FeederSubsystem::setDesiredFeederMotorOutput() {  //motor.setOutput(pid.getOutput())
        feederMotor.setOutput(feederVelocityPID.getOutput());
    }
}
