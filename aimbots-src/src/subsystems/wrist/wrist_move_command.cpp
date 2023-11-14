#include "subsystems/wrist/wrist_move_command.hpp"
#include "subsystems/wrist/wrist.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist  {
WristMoveCommand::WristMoveCommand(src::Drivers* drivers, WristSubsystem* wrist) : drivers(drivers), wrist(wrist) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(wrist));
}

void WristMoveCommand::initialize() {
    // wrist->ForAllMotors(&WristSubsystem::setTargetAngle());
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