#include "wrist_aim_command.hpp"

namespace src::Wrist {

WristAimCommand::WristAimCommand(Drivers* drivers, WristSubsystem* wrist)
    : remote(&drivers->remote)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(wrist));
}

void WristAimCommand::initialize() {}

void WristAimCommand::execute() {
    float x = remote->getChannel(Remote::Channel::RIGHT_HORIZONTAL);
    float y = remote->getChannel(Remote::Channel::RIGHT_VERTICAL);

    float radians = atan2(y, x);

    wrist->setTargetAngle(YAW, radians);
}

void WristAimCommand::end(bool interrupted) {}

};