#pragma once

#include "tap/control/command.hpp"

#include "subsystems/chassis/chassis.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/motion/s_curve_acceleration.hpp"
#include "utils/motion/s_curve_motion_profile.hpp"
#include "utils/motion/settled_util.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

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

    Matrix<float, 2, 1> railTargets;
    int railTargetIndex;

    float movementStartTime;
    SCurveMotionProfile* railTraverseProfile;
    int profilerDirection = 1;

    int patrolDelay = 250;

    SettledUtil chassisProfile;
};

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE