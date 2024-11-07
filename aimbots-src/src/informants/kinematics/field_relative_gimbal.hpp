#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "communicators/jetson/jetson_communicator.hpp"
#include "informants/odometry/chassis_kf_odometry.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "utils/tools/common_types.hpp"

#include "utils/tools/robot_specific_defines.hpp"
#include "subsystems/gimbal/gimbal_constants.hpp"
#include "subsystems/chassis/chassis_constants.hpp"

#include "robot_frames.hpp"
#include "turret_frames.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace src::Gimbal {
class GimbalSubsystem;
}

namespace src::Chassis {
class ChassisSubsystem;
}  // namespace src::Chassis

using namespace src::Utils;

namespace src::Informants {
    inline std::pair<float, float>& getGimbalFieldOrientation(int index) { return gimbalFieldOrientationBuffer[index]; }

    // put in your time, we get the closest orientation entry at that time.
    inline std::pair<float, float>& getGimbalFieldOrientationAtTime(uint32_t time_ms) {
        // assume 2 ms delay between gimbal updates
        int index = std::min(time_ms / 2, GIMBAL_BUFFER_SIZE - 1);
        return gimbalFieldOrientationBuffer[index];
    }
    // just in case?
    inline void clearGimbalFieldOrientationBuffer() { gimbalFieldOrientationBuffer.clear(); }

        tap::algorithms::WrappedFloat getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat();
    tap::algorithms::WrappedFloat getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat();

    // rad
    float getChassisPitchAngleInGimbalDirection();
    // rad/s
    float getChassisPitchVelocityInGimbalDirection();
    // m/s
    float getChassisLinearAccelerationInGimbalDirection();

    void mirrorPastRobotFrame(uint32_t frameDelay_ms);
}