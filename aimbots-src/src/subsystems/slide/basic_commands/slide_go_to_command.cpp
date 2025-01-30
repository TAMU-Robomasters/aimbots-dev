#include "slide_go_to_command.hpp"

#ifdef SLIDE_COMPATIBLE

namespace src::Slide {

SlideGoToCommand::SlideGoToCommand(Drivers* drivers, SlideSubsystem* slide, float targetX, float targetZ)
    : drivers(drivers),
      slide(slide),
      targetX(targetX),
      targetZ(targetZ) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(slide));
};

void SlideGoToCommand::initialize() {}

void SlideGoToCommand::end(bool interrupted) {}

void SlideGoToCommand::execute() {
    double targetVals[] = {targetX, targetZ};
    slide->setAllTargetPositionsMeters( targetVals );
    slide->updateAllPIDs();
}

bool SlideGoToCommand::isReady() { return true; }

bool SlideGoToCommand::isFinished() const { return false; }

};  // namespace src::Slide

#endif