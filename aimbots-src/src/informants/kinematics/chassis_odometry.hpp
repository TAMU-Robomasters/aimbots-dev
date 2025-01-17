#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "utils/kinematics/kinematic_state_vector.hpp"
#include "informants/odometry/chassis_kf_odometry.hpp"
#include "utils/tools/robot_specific_defines.hpp"
#include "utils/tools/common_types.hpp"
#include "informants/kinematics/imu_data.hpp"


namespace src {
class Drivers;
}  // namespace src

namespace src::Chassis {
class ChassisSubsystem;
}  // namespace src::Chassis

using namespace src::Utils;

namespace src::Informants {

class ChassisOdometry { 
public:
    ChassisOdometry(src::Drivers* drivers); 
    ~ChassisOdometry() = default;

    void registerSubsystems(src::Chassis::ChassisSubsystem* chassisSubsystem) 
        { this->chassisSubsystem = chassisSubsystem; }

    void updateChassisAcceleration();

    Vector3f removeFalseAcceleration(
        Vector<KinematicStateVector, 3> imuLKSV,
        Vector<KinematicStateVector, 3> imuAKSV,
        Vector3f r);

    modm::Location2D<float> getRobotLocation2D() { return chassisKFOdometry->getCurrentLocation2D(); }

    modm::Vector2f getRobotVelocity2D() { return chassisKFOdometry->getCurrentVelocity2D(); }

    void updateRobotFrames();

private:
    src::Drivers* drivers;
    src::Chassis::ChassisSubsystem* chassisSubsystem;
    Informants::Odometry::ChassisKFOdometry* chassisKFOdometry;

    static const uint32_t CHASSIS_IMU_BUFFER_SIZE = 50;
    Deque<Vector3f, CHASSIS_IMU_BUFFER_SIZE> chassisIMUHistoryBuffer;  // Buffer for turret orientation data
};  // class ChassisOdometry

}  // namespace src::Informants