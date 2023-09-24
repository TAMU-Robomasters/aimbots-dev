#pragma once

#ifdef TARGET_SENTRY


#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_auto_nav_command.hpp"
#include "subsystems/chassis/chassis_auto_nav_tokyo_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"

#include "subsystems/gimbal/gimbal.hpp"


#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

enum ChassisMatchStates { 
    SETUP = 0,
    HEAL,
    GUARD,
    AGGRO,
    CAPTURE,
    EVADE
};

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

    Vector<float, 2> TARGET_A = {-5.300f, -0.176f};    // Point A near base
    Vector<float, 2> TARGET_B = {-5.300f, 2.000f};     // Point B near heal
    Vector<float, 2> TARGET_HEAL = {-5.300f, 3.250f};  // Point in heal
    Vector<float, 2> TARGET_START = {
        CHASSIS_START_POSITION_RELATIVE_TO_WORLD[0],
        CHASSIS_START_POSITION_RELATIVE_TO_WORLD[1]};  // starting position

    modm::Location2D<float> waypointTarget;

    
    src::Chassis::ChassisMatchStates lastChassisState;

    ChassisAutoNavCommand autoNavCommand;
    ChassisAutoNavTokyoCommand autoNavTokyoCommand;

    ChassisTokyoCommand tokyoCommand;

    MilliTimeout evadeTimeout;

    int MATCH_TIME_LENGTH = 300; //in seconds
    int CENTRAL_BUFF_OPEN = 75;
    int matchTimer = 0; //in seconds

    int pathingStep = 0;

    MilliTimeout aggroTimer;

    int BUFF_POINT_REFRESH_TIME = 7500; //in milliseconds
    MilliTimeout buffPointTimer;


    bool engageTokyo = false;

    int currentPathLength = 0;
    int currPatrolIndex = 0;

    void inline updateChassisState(ChassisMatchStates newState) {
        currPatrolIndex = 0;
        lastChassisState = chassisState;
        chassisState = newState;
    }

    bool inline isNavSettled() {
        if (comprisedCommandScheduler.isCommandScheduled(&autoNavCommand)) {
            return autoNavCommand.isSettled();
        }
        else {
            return autoNavTokyoCommand.isSettled();
        }
    }

};

}  // namespace src::Chassis


#endif //#ifdef CHASSIS_COMPATIBLE
#endif