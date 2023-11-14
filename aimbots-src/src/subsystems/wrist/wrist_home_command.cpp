#include "subsystems/wrist/wrist_home_command.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist  {
WristHomeCommand::WristHomeCommand(src::Drivers* drivers, WristSubsystem* wrist) : drivers(drivers), wrist(wrist) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(wrist));
}

void WristHomeCommand::initialize() {
    // wrist->ForAllMotors(&WristSubsystem::setTargetAngle());
    wrist->setTargetAngle(YAW, 0.0f);
    wrist->setTargetAngle(PITCH, 0.0f);
    wrist->setTargetAngle(ROLL, 0.0f);
}

void WristHomeCommand::execute() {
    // wrist->setArmAngles();
    // wrist->ForAllMotors(&WristSubsystem::setDesiredOutputToMotor);
}

void WristHomeCommand::end(bool) {}

bool WristHomeCommand::isReady() { return true; }

bool WristHomeCommand::isFinished() const { return false; }

};  // namespace src::Wrist 


#endif  // #ifdef WRIST_COMPATIBLE