#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "utils/kinematics/kinematic_state_vector.hpp"
#include "informants/odometry/chassis_kf_odometry.hpp"
#include "utils/tools/robot_specific_defines.hpp"
#include "utils/tools/common_types.hpp"

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

    void registerSubsystems(tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem) 
        { this->chassisSubsystem = chassisSubsystem; }

    void updateChassisAcceleration();

    Vector3f removeFalseAcceleration(
        Vector<KinematicStateVector, 3> imuLKSV,
        Vector<KinematicStateVector, 3> imuAKSV,
        Vector3f r);

    modm::Location2D<float> getRobotLocation2D() { return chassisKFOdometry->getCurrentLocation2D(); }

    modm::Vector2f getRobotVelocity2D() { return chassisKFOdometry->getCurrentVelocity2D(); }

private:
    src::Drivers* drivers;
    tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem;
    Informants::Odometry::ChassisKFOdometry* chassisKFOdometry;

};  // class ChassisOdometry

}  // namespace src::Informants