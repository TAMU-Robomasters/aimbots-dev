#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "communicators/jetson/jetson_communicator.hpp"
#include "informants/odometry/chassis_kf_odometry.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "utils/tools/common_types.hpp"

#include "utils/tools/robot_specific_defines.hpp"
#include "subsystems/chassis/chassis_constants.hpp"

#include "robot_frames.hpp"
#include "turret_frames.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace src::Chassis {
class ChassisSubsystem;
}  // namespace src::Chassis

using namespace src::Utils;

namespace src::Informants::Kinematics {

class ChassisOdometry { 
public:

    ChassisOdometry(src::Drivers* drivers);  //move to kinematics informant instead of drivers
    ~ChassisOdometry() = default;

    void updateChassisAcceleration();

    Vector3f removeFalseAcceleration(
        Vector<KinematicStateVector, 3> imuLKSV,
        Vector<KinematicStateVector, 3> imuAKSV,
        Vector3f r);

    modm::Location2D<float> getRobotLocation2D() { return chassisKFOdometry->getCurrentLocation2D(); }

    modm::Vector2f getRobotVelocity2D() { return chassisKFOdometry->getCurrentVelocity2D(); }
        
private:
    src::Informants::Odometry::ChassisKFOdometry* chassisKFOdometry;

    KinematicStateVector imuLinearXState;
    KinematicStateVector imuLinearYState;
    KinematicStateVector imuLinearZState;

    KinematicStateVector imuAngularXState;
    KinematicStateVector imuAngularYState;
    KinematicStateVector imuAngularZState;

    KinematicStateVector chassisLinearXState;
    KinematicStateVector chassisLinearYState;
    KinematicStateVector chassisLinearZState;

    KinematicStateVector chassisAngularXState;
    KinematicStateVector chassisAngularYState;
    KinematicStateVector chassisAngularZState;

    modm::Vector<KinematicStateVector, 3> imuLinearState = {imuLinearXState, imuLinearYState, imuLinearZState};
    modm::Vector<KinematicStateVector, 3> imuAngularState = {imuAngularXState, imuAngularYState, imuAngularZState};

    modm::Vector<KinematicStateVector, 3> chassisLinearState = {
        chassisLinearXState,
        chassisLinearYState,
        chassisLinearZState};

    modm::Vector<KinematicStateVector, 3> chassisAngularState = {
        chassisAngularXState,
        chassisAngularYState,
        chassisAngularZState};

    };

} // namespace src::Informants