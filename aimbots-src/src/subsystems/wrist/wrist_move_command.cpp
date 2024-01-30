#include "subsystems/wrist/wrist_move_command.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {
WristMoveCommand::WristMoveCommand(
    src::Drivers* drivers, WristSubsystem* wrist, float yaw, float pitch, float roll) 
    : drivers(drivers), wrist(wrist), 
      yaw(yaw), pitch(pitch), roll(roll),
      profileConstraints(
          {1.0f,     // max velocity
           2.0f,     // max acceleration
           10.0f}),  // max jerk
      yawProfile(nullptr)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(wrist));
}

void WristMoveCommand::initialize() {
    wrist->setTargetAngle(YAW, yaw);
    wrist->setTargetAngle(PITCH, pitch);
    wrist->setTargetAngle(ROLL, roll);
}

void WristMoveCommand::execute() {}

void WristMoveCommand::end(bool) {}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE