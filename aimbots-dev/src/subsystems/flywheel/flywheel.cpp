#include "subsystems/flywheel/flywheel.hpp"

namespace src::Flywheel{

FlywheelSubsystem::FlywheelSubsystem(tap::Drivers* drivers) : Subsystem(drivers),
    topWheel(drivers, TOP_FLYWHEEL_ID, FLY_BUS, false, "Flywheel One"),
    bottomWheel(drivers, BOT_FLYWHEEL_ID, FLY_BUS, false, "Flywheel Two")
{
    motors[TOP] = &topWheel;
    motors[BOT] = &bottomWheel;
}

void FlywheelSubsystem::initialize(){

    topWheel.initialize();
    bottomWheel.initialize();
    setDesiredOutputs(0);
}

void FlywheelSubsystem::refresh(){
//update motor rpms
}

void FlywheelSubsystem::calculateFlywheel(float r){

}

}; //namespace src::Flywheel