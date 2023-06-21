#pragma once
#ifdef ULTRASONIC

#include "tap/control/command.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "utils/common_types.hpp"
#include "utils/motion/s_curve_acceleration.hpp"
#include "utils/motion/s_curve_motion_profile.hpp"
#include "utils/motion/settled_util.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

using namespace src::Utils::motion;

namespace src::Chassis {

class ChassisRailBounceCommand : public TapCommand {
public:
    ChassisRailBounceCommand(src::Drivers*, ChassisSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis rail bounce"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    SCurveMotionProfile::Constraints profileConstraints;  // m/s, m/s/s, m/s/s/s

    static constexpr float RAIL_SAFETY_BUFFER = 0.10f;  // meters

    static constexpr float leftRailBound = (WHEELBASE_WIDTH + RAIL_POLE_DIAMETER) / 2 + RAIL_SAFETY_BUFFER;

    static constexpr float rightRailBound =
        FULL_RAIL_LENGTH - (WHEELBASE_WIDTH + RAIL_POLE_DIAMETER) / 2 - RAIL_SAFETY_BUFFER;

    Matrix<float, 2, 1> railTargets;
    int railTargetIndex;

    float movementStartTime;
    SCurveMotionProfile* railTraverseProfile;
    int profilerDirection = 1;

    int patrolDelay = 250;

    SettledUtil chassisProfile;
};

}  // namespace src::Chassis

#endif // ULTRASONIC