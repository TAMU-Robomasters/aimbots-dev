   #include "subsystems/feeder/feeder.hpp"

    //static float pidOut;
namespace src::Feeder{

    tap::algorithms::SmoothPid pidController(20, 0, 0, 0, 8000, 1, 0, 1, 0);

    FeederSubsystem::FeederSubsystem(tap::Drivers* drivers) : Subsystem(drivers),
        feederMotor(drivers, FEEDER_ID, FEED_BUS, false, "Feeder Motor"), targetRPM(0) { 
            
        }


    void FeederSubsystem::initialize() {                            
        feederMotor.initialize();
    }

    void FeederSubsystem::refresh() {                               // unjam stuff
        pidController.runController(targetRPM - feederMotor.getShaftRPM(), pidController.runControllerDerivateError(targetRPM - feederMotor.getShaftRPM(), 1), 1);
        //pidOut = pidController.getOutput();
    }

    int32_t FeederSubsystem::updateRPM(int32_t rpm) {
        targetRPM = rpm;
        return targetRPM;
    }

    void FeederSubsystem::setDesiredOutput(){       // future things: limit switch cool things wow
        
        feederMotor.setDesiredOutput(pidController.getOutput());
    }
}
