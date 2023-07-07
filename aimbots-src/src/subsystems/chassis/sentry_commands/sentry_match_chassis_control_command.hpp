#pragma once

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/chassis_auto_nav_tokyo_command.hpp"
#include "subsystems/chassis/sentry_commands/sentry_chassis_auto_nav_evade_command.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

namespace src::Chassis {

enum ChassisMatchStates { NONE = 0, PATROL = 1, EVADE = 2, HEAL = 3 };

class SentryMatchChassisControlCommand : public TapComprisedCommand {
public:
    SentryMatchChassisControlCommand(
        src::Drivers*,
        ChassisSubsystem*,
        ChassisMatchStates& chassisState,
        src::Utils::RefereeHelperTurreted* refHelper);

    void initialize() override;
    void execute() override;

    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Sentry Match Chassis Control Command"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    static Vector2f TARGET_A = {-5.300f, -0.176f};    // Point A near base
    static Vector2f TARGET_B = {-5.300f, 2.000f};     // Point B near heal
    static Vector2f TARGET_HEAL = {-5.300f, 3.250f};  // Point in heal
    static Vector2f TARGET_START = {
        CHASSIS_START_POSITION_RELATIVE_TO_WORLD[0],
        CHASSIS_START_POSITION_RELATIVE_TO_WORLD[1]};  // starting position

    modm::Location2D<float> locationStart({-5.300f, -0.176f}, modm::toRadian(0.0f));
    modm::Location2D<float> locationA(TARGET_A, modm::toRadian(0.0f));
    modm::Location2D<float> locationB(TARGET_B, modm::toRadian(0.0f));
    modm::Location2D<float> locationHeal(TARGET_HEAL, modm::toRadian(0.0f));

    src::Chassis::ChassisMatchStates& chassisState;

    ChassisAutoNavCommand autoNavCommand;
    ChassisAutoNavTokyoCommand autoNavTokyoCommand;
    SentryChassisAutoNavEvadeCommand sentryChassisAutoNavEvadeCommand;

    MilliTimeout evadeTimeout;
};

}  // namespace src::Chassis