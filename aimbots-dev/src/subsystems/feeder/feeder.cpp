#include "subsystems/feeder/feeder.hpp"

namespace src::Feeder{

    FeederSubsystem::FeederSubsystem(tap::Drivers* drivers) : Subsystem(drivers),
        feederMotor(drivers, FEEDER_ID, FEED_BUS, false, "Feeder Motor"), targetRPM(0) { //figure out too many initializer values

            
        }


    void FeederSubsystem::initialize() {
        feederMotor.initialize();
        setDesiredOutput(0);
    }

    void FeederSubsystem::refresh() {
        //update rpms
    }

    int32_t FeederSubsystem::setDesiredOutput(int32_t speed){
        feederMotor.setDesiredOutput(speed);
        return speed;
    }
}
