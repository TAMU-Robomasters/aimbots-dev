#include "subsystems/shooter/shooter.hpp"
#include "utils/common_types.hpp"
#include <tap/architecture/clock.hpp>


namespace src::Shooter{

ShooterSubsystem::ShooterSubsystem(tap::Drivers* drivers) : Subsystem(drivers),
    topWheel(drivers, TOP_SHOOTER_ID, FLY_BUS, false, "Flywheel One"),
    bottomWheel(drivers, BOT_SHOOTER_ID, FLY_BUS, false, "Flywheel Two"),
    PID(
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    )
{
    
    motors[TOP] = &topWheel;
    motors[BOT] = &bottomWheel;
}

void ShooterSubsystem::initialize(){
    lastTime = (float)tap::arch::clock::getTimeMilliseconds();
    topWheel.initialize();
    bottomWheel.initialize();
}

void ShooterSubsystem::refresh(){
//update motor rpms
}

std::vector<float>  ShooterSubsystem::calculateShooter(float RPM_Target){
    //calculate rpm
    float time = (float)tap::arch::clock::getTimeMilliseconds();
    float dt  = time - lastTime;
    float topError = RPM_Target - float(topWheel.getShaftRPM());
    float botError = RPM_Target - float(bottomWheel.getShaftRPM());
    float topRPM = PID.runController(topError,0, dt)+RPM_Target;
    float bottomRPM = PID.runController(botError,0, dt)+RPM_Target;

    lastTime = time;
    std::vector<float> rpm = {topRPM, bottomRPM};
    return rpm;
}


void ShooterSubsystem::setDesiredOutputs(float RPM){
    topWheel.setDesiredOutput(RPM);
    bottomWheel.setDesiredOutput(RPM);
}

}; //namespace src::Shooter