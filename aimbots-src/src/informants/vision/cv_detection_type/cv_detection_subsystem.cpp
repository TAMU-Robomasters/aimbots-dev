#include "cv_detection_subsystem.hpp"

namespace src::Informants::Vision::CVDetectionController {

CVDectionSubystem::CVDectionSubystem(src::Drivers* drivers)
    : Subsystem(drivers),
      drivers(drivers),
      detectionType(CVDetectionType::ROBOT) {}

void CVDectionSubystem::initialize() {}

void CVDectionSubystem::refresh() { drivers->cvCommunicator.setCVDectionType(detectionType); }

}  // namespace src::Informants::Vision::CVDetectionController
