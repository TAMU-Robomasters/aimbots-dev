#include "slide_hold_command.hpp"

#ifdef SLIDE_COMPATIBLE

namespace src::Slide {

SlideHoldCommand::SlideHoldCommand(Drivers* drivers, SlideSubsystem* slide) : drivers(drivers), slide(slide) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(slide));
};

void SlideHoldCommand::initialize() {}

void SlideHoldCommand::execute() { slide->updateAllPIDs(); }

void SlideHoldCommand::end(bool interrupted) { 
        UNUSED(interrupted);
    slide->idle(); }

bool SlideHoldCommand::isReady() { return true; }

bool SlideHoldCommand::isFinished() const { return false; }

};  // namespace src::Slide

#endif