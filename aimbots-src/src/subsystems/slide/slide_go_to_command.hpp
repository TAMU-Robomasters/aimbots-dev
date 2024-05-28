#pragma once

#include "tap/control/command.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"
#include "slide.hpp"

#ifdef SLIDE_COMPATIBLE

namespace src::Slide {

class SlideGoToCommand : public TapCommand {
public:
    SlideGoToCommand(Drivers*, SlideSubsystem*, float targetX, float targetZ);

    void initialize() override;
    void end(bool interrupted) override;

    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "slide go to command"; };

private:
    Drivers* drivers;
    SlideSubsystem* slide;
    float targetX, targetZ;
};

};  // namespace src::Slide

#endif