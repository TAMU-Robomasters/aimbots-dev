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
    joystickReadDelay.restart(50);
}

//Adjust this to change stick sensitivity
float JOYSTICK_TO_DEGREES = 3;

float yawDeltaDisplay = 0.0f;

void WristControlCommand::execute() {
    float yawChangeDegrees = 0;
    float pitchChangeDegrees = 0;
    float rollChangeDegrees = 0;

    if(joystickReadDelay.execute()) {
        yawChangeDegrees = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) * JOYSTICK_TO_DEGREES;
        pitchChangeDegrees = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) * JOYSTICK_TO_DEGREES;
        joystickReadDelay.restart(50);
    }

    wrist->setTargetAngle(YAW, wrist->getTargetAngle(YAW) + modm::toRadian(yawChangeDegrees));
    wrist->setTargetAngle(PITCH, wrist->getTargetAngle(PITCH) + modm::toRadian(pitchChangeDegrees));
    wrist->setTargetAngle(ROLL, wrist->getTargetAngle(ROLL) + modm::toRadian(rollChangeDegrees));

    wrist->updateAllPIDs();
}

void WristControlCommand::end(bool) {
    wrist->idle();
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE