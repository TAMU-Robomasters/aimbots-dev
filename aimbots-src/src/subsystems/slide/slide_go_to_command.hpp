#pragma once

#include "slide.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "tap/control/command.hpp"
#include "drivers.hpp"

namespace src::Slide {

class SlideGoToCommand : public TapCommand {
public:
    SlideGoToCommand(Drivers*, SlideSubsystem*, float targetX, float targetY, float targetZ);

    void initialize() override;
    void end(bool interrupted) override;

    void execute() override;
    
    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "slide go to command"; };

private:
    Drivers* drivers;
    SlideSubsystem* slide;
    float targetX, targetY, targetZ;
};

};