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
    lastTime = tap::arch::clock::getTimeMilliseconds();
    topWheel.initialize();
    bottomWheel.initialize();
}

void ShooterSubsystem::refresh(){
//update motor rpms
}

std::vector<float>  ShooterSubsystem::calculateShooter(float RPM_Target){
    //calculate rpm
    uint32_t time = tap::arch::clock::getTimeMilliseconds();
    float dt =  ShooterSubsystem::ieee_float(time) -  ShooterSubsystem::ieee_float(lastTime);
    // float dt  = std::bit_cast<float>(time) - std::bit_cast<float>(lastTime);
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


//this needs to be moved to a math class if it works. if not welp it can be deleted.
float ShooterSubsystem::ieee_float(uint32_t f){
    static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");
    float ret;
    std::memcpy(&ret, &f, sizeof(float));
    return ret;
}

}; //namespace src::Shooter