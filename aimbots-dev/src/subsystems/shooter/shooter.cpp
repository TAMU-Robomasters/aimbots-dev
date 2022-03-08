#include "subsystems/shooter/shooter.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>
#include "utils/common_types.hpp"

// #ifndef TARGET_ENGINEER

namespace src::Shooter {


ShooterSubsystem::ShooterSubsystem(tap::Drivers* drivers) : Subsystem(drivers),            
                                                            topWheel(drivers, TOP_SHOOTER_ID, SHOOTER_BUS, true, "Flywheel One"),
                                                            bottomWheel(drivers, BOT_SHOOTER_ID, SHOOTER_BUS, false, "Flywheel Two"),
                                                            topWheelPID(10.0f,0,0,10,1000,1,1,1,0),
                                                            bottomWheelPID(10.0f,0,0,10,1000,1,1,1,0) 
{
    motors[TOP] = &topWheel;
    motors[BOT] = &bottomWheel;
}

void ShooterSubsystem::initialize() {
    lastTime = static_cast<float>(tap::arch::clock::getTimeMilliseconds());
    topWheel.initialize();
    bottomWheel.initialize();
    //tap::Drivers driver = *drivers; -- no clue what this was intended to do
}

//Update the actual RPMs of the motors; the calculation is called from ShooterCommand
void ShooterSubsystem::refresh() {
    setDesiredOutputs();
}

float PIDout = 0.0f;
float displayShaftSpeed = 0.0f;
//TODO: need to tune PID

/**
 * @brief Calculates shooter RPM values using PIDs and stores values in a list
 * 
 * @param RPM_Target target RPM for the PIDs
 */
void ShooterSubsystem::calculateShooter(float RPM_Target) {
    // calculate rpm
    float time = static_cast<float>(tap::arch::clock::getTimeMilliseconds());
    float dt = time - lastTime;
    // float dt = 1; // only for moc testing.
    float topError = RPM_Target - static_cast<float>(topWheel.getShaftRPM());
    float botError = RPM_Target - static_cast<float>(bottomWheel.getShaftRPM());
    this->targetRPMs[0] = topWheelPID.runController(topError, 0, dt);// + RPM_Target;
    this->targetRPMs[1] = bottomWheelPID.runController(botError, 0, dt);// + RPM_Target;
    PIDout = this->targetRPMs[0];
    displayShaftSpeed = static_cast<float>(topWheel.getShaftRPM());
    lastTime = time;
}

void ShooterSubsystem::setDesiredOutputs() {
    topWheel.setDesiredOutput(targetRPMs[0]);
    bottomWheel.setDesiredOutput(targetRPMs[1]);
    //emergency test lines
    //can be very violent!!1!
    // topWheel.setDesiredOutput(1000.0f);
    // bottomWheel.setDesiredOutput(1000.0f);
}

void ShooterSubsystem::setZeroOutput(){
    topWheel.setDesiredOutput(0.0f);
    bottomWheel.setDesiredOutput(0.0f);
}

};  // namespace src::Shooter

// #endif //#ifndef TARGET_ENGINEER