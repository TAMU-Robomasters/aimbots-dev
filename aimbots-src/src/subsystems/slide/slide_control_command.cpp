#include "slide_control_command.hpp"

namespace src::Slide {

SlideControlCommand::SlideControlCommand(Drivers* drivers, SlideSubsystem* slide)
    : drivers(drivers), slide(slide),
      targetX(targetX), targetZ(targetZ)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(slide));
};

void SlideControlCommand::initialize() {
    joystickReadDelay.restart(500);
}

float JOYSTICK_TO_METERS = 1.0 / 1000.0f; //Adjust sensitivity

float deltaZDisplay = 0.0f;

void SlideControlCommand::execute() {

    if (joystickReadDelay.execute()) {
        float z_dMeters = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) * JOYSTICK_TO_METERS;
        targetZ += z_dMeters;

        deltaZDisplay = z_dMeters;

        float x_dMeters = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) * JOYSTICK_TO_METERS;
        targetX += x_dMeters;

        joystickReadDelay.restart(500); //May need to be lowered, if movement is choppy/lagging
    }

    slide->setTargetPositionMeters(targetX, targetZ);
    slide->updateAllPIDs();
}

void SlideControlCommand::end(bool interrupted) {

}

bool SlideControlCommand::isReady() { return true; }

bool SlideControlCommand::isFinished() const { return false; }

};