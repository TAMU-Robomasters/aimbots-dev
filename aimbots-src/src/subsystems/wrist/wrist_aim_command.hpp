/*
Temporary command for pointing in the direction of the joystick
*/

#pragma once

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "wrist.hpp"

#include "drivers.hpp"

namespace src::Wrist {

class WristAimCommand : public tap::control::Command {
public:
    WristAimCommand(Drivers* drivers, WristSubsystem* wrist);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override { return true; };
    bool isFinished() const override { return false; };
    const char* getName() const override { return "aim wrist command"; }

private:
    Remote* remote;
    WristSubsystem* wrist;
};

};