#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {
    static constexpr SmoothPIDConfig tempFeederPID = {
        .kp = 3.5f,
        .ki = 0.0f,
        .kd = 0.0f,
        .maxICumulative = 10.0f,
        .maxOutput = M2006_MAX_OUTPUT,
        .tQDerivativeKalman = 1.0f,
        .tRDerivativeKalman = 1.0f,
        .tQProportionalKalman = 1.0f,
        .tRProportionalKalman = 1.0f,
        .errDeadzone = 0.0f,
        .errorDerivativeFloor = 0.0f,
    };

    FeederSubsystem::FeederSubsystem(src::Drivers* drivers) : 
        Subsystem(drivers),
        drivers(drivers),
        feederMotor(drivers, FEEDER_MOTOR_ID, FEEDER_BUS, FEEDER_DIRECTION, "feederMotor"),
        feederVelocityPID(tempFeederPID)
    {
        desiredFeederMotorOutput = 0;

        feederTargetRPM = 0;
    }

    float feederTargetRPMDisplay = 0;
    float feederMotorRPMDisplay = 0;

    void FeederSubsystem::initialize() {
        feederMotor.initialize();
    }

    void FeederSubsystem::refresh() {
        updateMotorVelocityPID();
        setDesiredFeederMotorOutput();
        feederTargetRPMDisplay = feederTargetRPM;
        feederMotorRPMDisplay = feederMotor.getShaftRPM();
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