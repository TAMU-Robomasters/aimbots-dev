#include "cv_detection_update_command.hpp"

namespace src::Informants::Vision::CVDetectionController {

CVDectionUpdateCommand::CVDectionUpdateCommand(src::Drivers* drivers, CVDectionSubystem* cvDetectionSubystem)
    : drivers(drivers),
      cvDetectionSubystem(cvDetectionSubystem),
      detectionType(CVDetectionType::ROBOT) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(cvDetectionSubystem));
}

void CVDectionUpdateCommand::initialize() {
    detectionType = drivers->cvCommunicator.getCVDectionType();
    cvDetectionSubystem->setCVDectionType(detectionType);
}

void CVDectionUpdateCommand::execute() {
    detectionType = drivers->cvCommunicator.getCVDectionType();
    switch (detectionType) {
        case CVDetectionType::ROBOT:
            drivers->cvCommunicator.setCVDectionType(CVDetectionType::RUNE);
            break;
        case CVDetectionType::RUNE:
            drivers->cvCommunicator.setCVDectionType(CVDetectionType::ROBOT);
            break;
    }
}

void CVDectionUpdateCommand::end(bool interrupted) {}

bool CVDectionUpdateCommand::isReady() { return true; }

bool CVDectionUpdateCommand::isFinished() const { return false; }


}  // namespace src::Informants::Vision::CVDetectionController