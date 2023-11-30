#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/wrist/wrist.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "utils/motion/s_curve_acceleration.hpp"
#include "utils/motion/s_curve_motion_profile.hpp"


#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

using namespace src::Utils::motion;

namespace src::Wrist {
class WristMoveCommand : public TapCommand {
public:
    WristMoveCommand(src::Drivers* drivers, WristSubsystem* wrist, float yaw, float pitch, float roll);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override { return true; };
    bool isFinished() const override { return false; };
    const char* getName() const override { return "move wrist command"; }

private:
    src::Drivers* drivers;
    WristSubsystem* wrist;
    float yaw, pitch, roll;

    //S curve stuff
    SCurveMotionProfile::Constraints profileConstraints;
    float movementStartTime;
    SCurveMotionProfile* yawProfile;// = new SCurveMotionProfile(profileConstraints, 0.0f);    //(profileConstraints, 0.0f);
    int profilerDirection = 1;
};

};      // namespace src::Wrist
#endif  // #ifdef WRIST_COMPATIBLE
