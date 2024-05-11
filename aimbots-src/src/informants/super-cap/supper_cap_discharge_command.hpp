#pragma once

#include "tap/control/command.hpp"

#include "supper_cap_subsystem.hpp"
#include "drivers.hpp"

namespace src::Informants::SupperCap {

class SupperCapDischargeCommand : public TapCommand {
public:
    SupperCapDischargeCommand(src::Drivers*, SupperCapSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "charge supper cap"; }

private:
    src::Drivers* drivers;
    SupperCapSubsystem* supperCap;
};



}