#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/wrist/wrist.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {
class WristMoveCommand : public TapCommand {
public:
    WristMoveCommand(src::Drivers* drivers, WristSubsystem* wirst);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override { return "move wrist command"; }

private:
    src::Drivers* drivers;
    WristSubsystem* wrist;
};

};  // namespace src::Wrist
#endif  // #ifdef WRIST_COMPATIBLE
