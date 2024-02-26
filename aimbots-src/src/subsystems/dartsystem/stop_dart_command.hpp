#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/dartsystem/dart.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef DART_COMPATIBLE

namespace src::Dart {

class StopDartCommand : public TapCommand {
public:
    StopDartCommand(src::Drivers* drivers, DartSubsystem* dart);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "run dart command"; }

private:
    src::Drivers* drivers;
    DartSubsystem* dart;

};

}  // namespace src::Shooter

#endif //#ifdef SHOOTER_COMPATIBLE