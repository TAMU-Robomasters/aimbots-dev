#include "slide_go_to_command.hpp"

namespace src::Slide {

SlideGoToCommand::SlideGoToCommand(Drivers* drivers, SlideSubsystem* slide, float targetX, float targetY, float targetZ)
    : drivers(drivers), slide(slide),
      targetX(targetX), targetY(targetY), targetZ(targetZ)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(slide));
};

void SlideGoToCommand::initialize() {}
 
void SlideGoToCommand::end(bool interrupted) {}

void SlideGoToCommand::execute() {
    slide->setTargetPosition(targetX, targetY, targetZ);
    slide->updateSlidePositionPID();
}

bool SlideGoToCommand::isReady() { return true; }

bool SlideGoToCommand::isFinished() const { return false; }

};