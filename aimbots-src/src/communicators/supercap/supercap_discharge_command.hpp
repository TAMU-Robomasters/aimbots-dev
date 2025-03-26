#pragma once

#include "tap/control/command.hpp"

#include "supercap_subsystem.hpp"
#include "drivers.hpp"

namespace src::Communicators::SuperCap {

class SuperCapDischargeCommand : public TapCommand {
public:
    SuperCapDischargeCommand(src::Drivers*, SuperCapSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "charge super cap"; }

private:
    src::Drivers* drivers;
    SuperCapSubsystem* superCap;
};



}