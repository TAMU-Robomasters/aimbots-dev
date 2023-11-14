#include "subsystems/wrist/wrist_move_command.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist  {
WristMoveCommand::WristMoveCommand(src::Drivers* drivers, WristSubsystem* wrist) : drivers(drivers), wrist(wrist) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(wrist));
}

void WristMoveCommand::initialize() {
    // wrist->ForAllMotors(&WristSubsystem::setTargetAngle());
    wrist->setTargetAngle(YAW, 45.0f);
    wrist->setTargetAngle(PITCH, 45.0f);
    wrist->setTargetAngle(ROLL, 45.0f);
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