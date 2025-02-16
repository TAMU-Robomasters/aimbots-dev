#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "informants/kinematics/chassis_odometry.hpp"
#include "informants/kinematics/field_relative_gimbal.hpp"
#include "informants/kinematics/hitTracker.hpp"
#include "informants/kinematics/imu_data.hpp"

#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_defines.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace src::Gimbal {
class GimbalSubsystem;
}  // namespace src::Gimbal

namespace src::Chassis {
class ChassisSubsystem;
}  // namespace src::Chassis

namespace src::Informants {

class KinematicInformant {
public:
    KinematicInformant(src::Drivers* drivers);
    ~KinematicInformant() = default;

    src::Informants::FieldRelativeGimbal fieldRelativeGimbal;
    src::Informants::ChassisOdometry chassisOdometry;
    src::Informants::IMUData imuData;    
    src::Informants::HitTracker hitTracker;

    void initialize(float imuFrequency, float imukP, float imukI);

    void registerSubsystems(src::Gimbal::GimbalSubsystem* gimbalSubsystem,
                            src::Chassis::ChassisSubsystem* chassisSubsystem);

    void registerSubsystems(src::Chassis::ChassisSubsystem* chassisSubsystem);

    void registerSubsystems(src::Gimbal::GimbalSubsystem* gimbalSubsystem);

    void registerSubsystems();

    // updates all kinematic_informant related files' update functions
    void updateData();
    
private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbalSubsystem;
    src::Chassis::ChassisSubsystem* chassisSubsystem;

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

    
};  // class KinematicInformant

}  // namespace src::Informants