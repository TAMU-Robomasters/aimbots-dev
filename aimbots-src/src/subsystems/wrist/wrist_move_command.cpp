#include "subsystems/wrist/wrist_move_command.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {
WristMoveCommand::WristMoveCommand(src::Drivers* drivers, WristSubsystem* wrist, float yaw, float pitch, float roll) 
    : drivers(drivers), wrist(wrist), yaw(yaw), pitch(pitch), roll(roll) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(wrist));
}

void WristMoveCommand::initialize() {
    // wrist->ForAllMotors(&WristSubsystem::setTargetAngle());
    wrist->setTargetAngle(YAW, yaw);
    wrist->setTargetAngle(PITCH, pitch);
    wrist->setTargetAngle(ROLL, roll);
}

void WristMoveCommand::execute() {
    // wrist->setArmAngles();
    // wrist->ForAllMotors(&WristSubsystem::setDesiredOutputToMotor);
}

void WristMoveCommand::end(bool) {}

bool WristMoveCommand::isReady() { return true; }

bool WristMoveCommand::isFinished() const { return false; }

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE