#pragma once

#include "drivers.hpp"
#include "subsystems/chassis/chassis.hpp"
#include "tap/control/command.hpp"
#include "utils/common_types.hpp"
#include "utils/motion/SCurveAcceleration.hpp"
#include "utils/motion/SCurveMotionProfile.hpp"
#include "utils/robot_specific_inc.hpp"

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

#ifdef TARGET_SENTRY
        SCurveMotionProfile::Constraints profileConstraints;  // m/s, m/s/s, m/s/s/s

        static constexpr float RAIL_SAFETY_BUFFER = 0.05f;  // meters

        Matrix<float, 1, 3> leftRailBound;
        Matrix<float, 1, 3> rightRailBound;

        static constexpr float leftRailBoundArray[3] = {
            (WHEELBASE_WIDTH + RAIL_POLE_DIAMETER / 2) + RAIL_SAFETY_BUFFER,
            0.0f,
            0.0f,
        };

        static constexpr float rightRailBoundArray[3] = {
            FULL_RAIL_LENGTH - (WHEELBASE_WIDTH + RAIL_POLE_DIAMETER / 2) - RAIL_SAFETY_BUFFER,
            0.0f,
            0.0f,
        };

        float currTraverseTarget;
        float movementStartTime;

        SCurveMotionProfile* railTraverseProfile;
#endif
    };

}  // namespace src::Chassis