#pragma once

#ifdef TARGET_SENTRY

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/chassis_auto_nav_tokyo_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

// enum ChassisMatchStates { SETUP = 0, HEAL, GUARD, AGGRO, CAPTURE, EVADE };

enum ChassisMatchStates { START = 0, RESUPPLYING, GUARDING, RETREAT, AGGRO, CAPTURE, EVADE };

class SentryMatchChassisControlCommand : public TapComprisedCommand {
public:
    SentryMatchChassisControlCommand(
        src::Drivers*,
        ChassisSubsystem*,
        src::Gimbal::GimbalSubsystem*,
        ChassisMatchStates& chassisState,
        src::Utils::RefereeHelperTurreted* refHelper,
        const SnapSymmetryConfig& snapSymmetryConfig,
        const TokyoConfig& tokyoConfig,
        bool randomizeSpinRate,
        const SpinRandomizerConfig& randomizerConfig);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Chassis Control Command"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    src::Chassis::ChassisMatchStates& chassisState;

    src::Utils::RefereeHelperTurreted* refHelper;

    modm::Location2D<float> waypointTarget;

    src::Chassis::ChassisMatchStates lastChassisState;

    ChassisAutoNavCommand autoNavCommand;
    ChassisAutoNavTokyoCommand autoNavTokyoCommand;

    ChassisTokyoCommand tokyoCommand;

    MilliTimeout evadeTimeout;

    int MATCH_TIME_LENGTH = 300;  // in seconds
    int CENTRAL_BUFF_OPEN = 75;
    int matchTimer = 0;  // in seconds

    MilliTimeout delayTimer;
    MilliTimeout lockoutTimer;

    static constexpr std::array<int, 6> STATE_LOCKOUT_TIMES = {0, 3000, 0, 0, 5000, 10000};

    int BUFF_POINT_REFRESH_TIME = 7500;  // in milliseconds

    bool activeMovement = false;

    void inline updateChassisState(ChassisMatchStates newState) {
        if (chassisState != newState) {
            lastChassisState = chassisState;
            chassisState = newState;
            activeMovement = true;
        }
    }

    bool inline isNavSettled() {
        if (comprisedCommandScheduler.isCommandScheduled(&autoNavCommand)) {
            return autoNavCommand.isSettled();
        } else {
            return autoNavTokyoCommand.isSettled();
        }
    }
};

}  // namespace src::Chassis

#endif  //#ifdef CHASSIS_COMPATIBLE

#endif