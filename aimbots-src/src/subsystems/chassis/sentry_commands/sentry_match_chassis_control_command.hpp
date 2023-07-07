#pragma once

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/chassis_auto_nav_tokyo_command.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"

#include "drivers.hpp"

namespace src::Chassis {

enum ChassisMatchStates { NONE = 0, PATROL = 1, EVADE = 2, HEAL = 3 };

class SentryMatchChassisControlCommand : public TapComprisedCommand {
public:
    SentryMatchChassisControlCommand(
        src::Drivers*,
        ChassisSubsystem*,
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
    src::Utils::RefereeHelperTurreted* refHelper;

    Vector<float, 2> TARGET_A = {-5.300f, -0.176f};    // Point A near base
    Vector<float, 2> TARGET_B = {-5.300f, 2.000f};     // Point B near heal
    Vector<float, 2> TARGET_HEAL = {-5.300f, 3.250f};  // Point in heal
    Vector<float, 2> TARGET_START = {
        CHASSIS_START_POSITION_RELATIVE_TO_WORLD[0],
        CHASSIS_START_POSITION_RELATIVE_TO_WORLD[1]};  // starting position

    src::Chassis::ChassisMatchStates& chassisState;

    ChassisAutoNavCommand autoNavCommand;
    ChassisAutoNavTokyoCommand autoNavTokyoCommand;

    MilliTimeout evadeTimeout;
};

}  // namespace src::Chassis