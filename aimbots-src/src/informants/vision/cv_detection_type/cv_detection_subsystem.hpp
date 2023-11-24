#pragma once

#include "tap/control/subsystem.hpp"

#include "drivers.hpp"
// #include "src/vision/jetson_protocol.hpp"
// #include "src/vision/jetson_communicator.hpp"
// #include "src/informants/vision/jetson_protocol.hpp"

// using namespace src::Informants::Vision;
namespace src::Informants::Vision::CVDetectionController {

class CVDectionSubystem : public tap::control::Subsystem {
public:
    CVDectionSubystem(src::Drivers* drivers);

    void initialize() override;
    void refresh() override;

    inline void setCVDectionType(CVDetectionType detectionType) { this->detectionType = detectionType; }

private:
    /* data */
    src::Drivers* drivers;
    CVDetectionType detectionType;
};

}  // namespace src::Informants::Vision::CVDetectionController
