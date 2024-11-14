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

enum AngularAxis { PITCH_AXIS = 0, ROLL_AXIS = 1, YAW_AXIS = 2 };

class KinematicInformant {
public:
    KinematicInformant(src::Drivers* drivers);
    ~KinematicInformant() = default;

    void registerSubsystems(
        src::Gimbal::GimbalSubsystem* gimbalSubsystem,
        tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem) {
        this->gimbalSubsystem = gimbalSubsystem;
        this->chassisSubsystem = chassisSubsystem;

        chassisKFOdometry->registerChassisSubsystem(chassisSubsystem);
    }

    tap::communication::sensors::imu::ImuInterface::ImuState getIMUState();

    void initialize(float imuFrequency, float imukP, float imukI);
    
private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbalSubsystem;
    tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem;
    src::Informants::Odometry::ChassisKFOdometry* chassisKFOdometry;

    src::Informants::Transformers::RobotFrames robotFrames;
    src::Informants::Transformers::TurretFrames turretFrames;

    // KinematicStateVector turretIMULinearXState;
    // KinematicStateVector turretIMULinearYState;
    // KinematicStateVector turretIMULinearZState;

    // KinematicStateVector turretIMUAngularXState;
    // KinematicStateVector turretIMUAngularYState;
    // KinematicStateVector turretIMUAngularZState;

    // modm::Vector<KinematicStateVector, 3> turretIMULinearState = {
    //     turretIMULinearXState,
    //     turretIMULinearYState,
    //     turretIMULinearZState};
    // modm::Vector<KinematicStateVector, 3> turretIMUAngularState = {
    //     turretIMUAngularXState,
    //     turretIMUAngularYState,
    //     turretIMUAngularZState};

    
};

}  // namespace src::Informants
