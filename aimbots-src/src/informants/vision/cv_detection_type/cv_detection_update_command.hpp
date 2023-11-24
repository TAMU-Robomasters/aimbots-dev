#pragma once

#include "tap/control/command.hpp"

#include "utils/common_types.hpp"

#include "cv_detection_subsystem.hpp"
#include "drivers.hpp"

namespace src::Informants::Vision::CVDetectionController {

class CVDectionUpdateCommand : public tap::control::Command {
public:
    CVDectionUpdateCommand(src::Drivers* drivers, CVDectionSubystem* cvDetectionSubystem);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "cv detection update command"; }

private:
    src::Drivers* drivers;
    CVDectionSubystem* cvDetectionSubystem;

    CVDetectionType detectionType;
};
}  // namespace src::Informants::Vision::CVDetectionController