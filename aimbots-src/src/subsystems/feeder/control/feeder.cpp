#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#ifdef SHOOTER_COMPATIBLE

namespace src::Feeder {
    FeederSubsystem::FeederSubsystem(src::Drivers* drivers) : 
        Subsystem(drivers),
        drivers(drivers),
        feederMotor(drivers, FEEDER_MOTOR_ID, FEEDER_BUS, FEEDER_DIRECTION, "feederMotor"),
        feederVelocityPID(FEEDER_VELOCITY_PID_CONFIG)
    {
        desiredFeederMotorOutput = 0;

        feederTargetRPM = 0;
    }

    void FeederSubsystem::initialize() {
        feederMotor.initialize();
    }

    void FeederSubsystem::refresh() {
        updateMotorVelocityPID();
        setDesiredFeederMotorOutput();
    }

    void FeederSubsystem::updateMotorVelocityPID() {
        feederVelocityPID.runControllerDerivateError(feederTargetRPM - feederMotor.getShaftRPM());
        desiredFeederMotorOutput = feederVelocityPID.getOutput();
    }

    void FeederSubsystem::setTargetRPM(float targetRPM) {
        feederTargetRPM = targetRPM;
    }

    void FeederSubsystem::setDesiredFeederMotorOutput(){
        feederMotor.setDesiredOutput(desiredFeederMotorOutput);
    }
}

#endif