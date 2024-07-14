#include "subsystems/wrist/wrist_control_command.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {
WristControlCommand::WristControlCommand(src::Drivers* drivers, WristSubsystem* wrist) : drivers(drivers), wrist(wrist) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(wrist));
}

void WristControlCommand::initialize() { joystickReadDelay.restart(50); }

// Adjust this to change stick sensitivity
float JOYSTICK_TO_DEGREES = 3;

float yawDeltaDisplay = 0.0f;

void WristControlCommand::execute() {
    wrist->setTargetAngle(YAW, wrist->getTargetAngle(YAW) + modm::toRadian(drivers->controlOperatorInterface.getWristYawInput()));
    wrist->setTargetAngle(PITCH, wrist->getTargetAngle(PITCH) + modm::toRadian(drivers->controlOperatorInterface.getWristPitchInput()));
    wrist->setTargetAngle(ROLL, wrist->getTargetAngle(ROLL) + modm::toRadian(drivers->controlOperatorInterface.getWristRollInput()));
    
    wrist->updateAllPIDs();
}

void WristControlCommand::end(bool) { wrist->idle(); }

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE