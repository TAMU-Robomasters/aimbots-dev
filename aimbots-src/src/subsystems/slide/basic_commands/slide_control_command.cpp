#include "slide_control_command.hpp"

#ifdef SLIDE_COMPATIBLE

namespace src::Slide {

static constexpr float JOYSTICK_TO_METERS = 3 / 1000.0f;  // Adjust sensitivity

SlideControlCommand::SlideControlCommand(Drivers* drivers, SlideSubsystem* slide) : drivers(drivers), slide(slide) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(slide));
};

void SlideControlCommand::initialize() {}

float xTargetMeterDisplay = 0.0f;
float zTargetMeterDisplay = 0.0f;

void SlideControlCommand::execute() {
    float targetX = slide->getTargetXMeters() + drivers->controlOperatorInterface.getSlideFrontBackInput();
    float targetZ = slide->getTargetZMeters() + drivers->controlOperatorInterface.getSlideUpDownInput();

    xTargetMeterDisplay = targetX;
    zTargetMeterDisplay = targetZ;

    slide->setTargetPositionMeters(targetX, targetZ);
    slide->updateAllPIDs();
}

void SlideControlCommand::end(bool interrupted) {
    UNUSED(interrupted);
    slide->idle();
}

bool SlideControlCommand::isReady() { return true; }

bool SlideControlCommand::isFinished() const { return false; }

};  // namespace src::Slide

#endif