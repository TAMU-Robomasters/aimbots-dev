#include "subsystems/feeder/feeder.hpp"

namespace src::Feeder{

    FeederSubsystem::FeederSubsystem(tap::Drivers* drivers) : Subsystem(drivers), 
        feederMotor(drivers, FEEDER_ID, FEED_BUS, false, "Feeder Motor"), targetRPM(0) { //figure out too many initializer values

            
        }


    template <class... Args>
    void FeederSubsystem::initialize() {
        feederMotor->&DJIMotor::initialize(args...);
        setDesiredOutputs(0);
    }

    void FeederSubsystem::refresh() {
        //update rpms
    }

    void FeederSubsystem::setDesiredOutput(float speed){
        //run kekw
    }
}