#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "communicators/jetson/jetson_communicator.hpp"
#include "informants/odometry/chassis_kf_odometry.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "utils/tools/common_types.hpp"

#include "utils/tools/robot_specific_defines.hpp"
#include "subsystems/gimbal/gimbal_constants.hpp"

#include "robot_frames.hpp"
#include "turret_frames.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace src::Gimbal {
class GimbalSubsystem;
}

using namespace src::Utils;

namespace src::Informants {
class RobotRelativeGimbal {
public:
    RobotRelativeGimbal(src::Drivers* drivers);
    ~RobotRelativeGimbal() = default;

    src::Informants::Transformers::RobotFrames& getRobotFrames() { return robotFrames; }

    src::Informants::Transformers::TurretFrames& getTurretFrames() { return turretFrames; }


private:

};
} //namespace Informants