#include "subsystems/wrist/wrist_control_command.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {
WristControlCommand::WristControlCommand(
    src::Drivers* drivers, WristSubsystem* wrist) 
    : drivers(drivers), wrist(wrist)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(wrist));
}

void WristControlCommand::initialize() {
    joystickReadDelay.restart(500);
}

//Adjust this to change stick sensitivity
float JOYSTICK_TO_DEGREES = 0.2;

void WristControlCommand::execute() {

    if(joystickReadDelay.execute()) {
        float yaw_dTheta = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) * JOYSTICK_TO_DEGREES;
        yawTarget += yaw_dTheta;

        float pitch_dTheta = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) * JOYSTICK_TO_DEGREES;
        pitchTarget += pitch_dTheta;

        joystickReadDelay.restart(500); //Adjust this number if motion stutters too much, might need to be lower
    }

    wrist->setTargetAngle(YAW, modm::toRadian(yawTarget));
    wrist->setTargetAngle(PITCH, modm::toRadian(pitchTarget));
    wrist->setTargetAngle(ROLL, modm::toRadian(rollTarget));

    wrist->updateAllPIDs();
}

void WristControlCommand::end(bool) {
    // wrist->setTargetRPMs(0.0f, 0.0f, 0.0f); // stop moving
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE