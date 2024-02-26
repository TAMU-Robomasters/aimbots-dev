#include "slide_control_command.hpp"

namespace src::Slide {

static constexpr float JOYSTICK_TO_METERS = 5 / 1000.0f; //Adjust sensitivity

static constexpr int JOYSTICK_READS_PER_SECOND = 20;
static constexpr int JOYSTICK_READ_DELAY_MS = 1000 / JOYSTICK_READS_PER_SECOND;

SlideControlCommand::SlideControlCommand(Drivers* drivers, SlideSubsystem* slide)
    : drivers(drivers), slide(slide)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(slide));
};

void SlideControlCommand::initialize() {
    joystickReadDelay.restart(JOYSTICK_READ_DELAY_MS);
}

void SlideControlCommand::execute() {
    float targetX = slide->getTargetXMeters();
    float targetZ = slide->getTargetZMeters();

    if (joystickReadDelay.execute()) {
        float x_dMeters = drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL) * JOYSTICK_TO_METERS;
        float z_dMeters = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) * JOYSTICK_TO_METERS;
        targetX += x_dMeters;
        targetZ += z_dMeters;
        joystickReadDelay.restart(JOYSTICK_READ_DELAY_MS); //May need to be lowered, if movement is choppy/lagging
    }

    slide->setTargetPositionMeters(targetX, targetZ);
    slide->updateAllPIDs();
}

void SlideControlCommand::end(bool interrupted) {

}

bool SlideControlCommand::isReady() { return true; }

bool SlideControlCommand::isFinished() const { return false; }

};