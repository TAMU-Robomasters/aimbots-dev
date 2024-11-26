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

#ifdef CHASSIS_COMPATIBLE
    #ifdef GIMBAL_COMPATIBLE
        void registerSubsystems(
            src::Gimbal::GimbalSubsystem* gimbalSubsystem,
            tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem) {
            this->gimbalSubsystem = gimbalSubsystem;
            this->chassisSubsystem = chassisSubsystem;

            drivers->kinematicInformant.fieldRelativeGimbal.registerSubsystems(gimbalSubsystem, chassisSubsystem);
        }
        #endif
    #endif

#ifdef CHASSIS_COMPATIBLE 
    void registerSubsystems(
        tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem) {
        this->chassisSubsystem = chassisSubsystem;

        drivers->kinematicInformant.chassisOdometry.registerSubsystems(chassisSubsystem);
    } 
    #endif

#ifdef GIMBAL_COMPATIBLE
    void registerSubsystems(
        src::Gimbal::GimbalSubsystem* gimbalSubsystem) {
        this->gimbalSubsystem = gimbalSubsystem;

        drivers->kinematicInformant.hitTracker.registerSubsystems(gimbalSubsystem);
    }
    #endif

    void initialize(float imuFrequency, float imukP, float imukI);

private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbalSubsystem;
    tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem;

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