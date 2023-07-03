#pragma once

#include <tap/algorithms/contiguous_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "informants/odometry/chassis_kf_odometry.hpp"
#include "transformers/robot_frames.hpp"
#include "utils/common_types.hpp"
#include "utils/kinematic_state_vector.hpp"

namespace src {
class Drivers;
}  // namespace src
namespace src::Gimbal {
class GimbalSubsystem;
}

namespace src::Chassis {
class ChassisSubsystem;
}

using namespace src::Utils;

namespace src::Informants {

enum AngularAxis { PITCH_AXIS = 0, ROLL_AXIS = 1, YAW_AXIS = 2 };

class KinematicInformant {
public:
    KinematicInformant(src::Drivers* drivers);
    ~KinematicInformant() = default;

    src::Informants::Transformers::RobotFrames& getRobotFrames() { return robotFrames; }

    void registerSubsystems(
        src::Gimbal::GimbalSubsystem* gimbalSubsystem,
        tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem) {
        this->gimbalSubsystem = gimbalSubsystem;
        this->chassisSubsystem = chassisSubsystem;

        chassisKFOdometry.registerChassisSubsystem(chassisSubsystem);
    }

    void initialize(float imuFrequency, float imukP, float imukI);
    void recalibrateIMU();
    tap::communication::sensors::imu::ImuInterface::ImuState getIMUState();

    float getIMUAngle(AngularAxis axis);
    Vector3f getIMUAngles();

    Vector3f getIMUAngularVelocities();
    float getIMUAngularVelocity(AngularAxis axis);

    Vector3f getIMULinearAccelerations();
    float getIMULinearAcceleration(LinearAxis axis);

    Vector3f removeFalseAcceleration(
        Vector<KinematicStateVector, 3> imuLKSV,
        Vector<KinematicStateVector, 3> imuAKSV,
        Vector3f r);

    void updateIMUKinematicStateVector();

    void updateChassisIMUAngles();

    // Returns angle in rad or deg
    float getChassisIMUAngle(AngularAxis axis, AngleUnit unit);

    // Returns angular velocity in rad/s or deg/s
    float getChassisIMUAngularVelocity(AngularAxis axis, AngleUnit unit);

    Vector3f getIMUAngularAccelerations();
    float getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit);
    // Returns lnothing!!!
    void updateChassisAcceleration();

    void updateRobotFrames();

    tap::algorithms::ContiguousFloat getCurrentFieldRelativeGimbalYawAngleAsContiguousFloat();
    tap::algorithms::ContiguousFloat getCurrentFieldRelativeGimbalPitchAngleAsContiguousFloat();

    // rad
    float getChassisPitchAngleInGimbalDirection();
    // rad/s
    float getChassisPitchVelocityInGimbalDirection();
    // m/s
    float getChassisLinearAccelerationInGimbalDirection();

    void mirrorPastRobotFrame(uint32_t frameDelay_ms);

    modm::Location2D<float> getRobotLocation() { return chassisKFOdometry.getCurrentLocation2D(); }

private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbalSubsystem;
    tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem;

    src::Informants::Transformers::RobotFrames robotFrames;

    KinematicStateVector imuLinearXState;
    KinematicStateVector imuLinearYState;
    KinematicStateVector imuLinearZState;

    KinematicStateVector imuAngularXState;
    KinematicStateVector imuAngularYState;
    KinematicStateVector imuAngularZState;

#ifndef TARGET_TURRET
    KinematicStateVector chassisLinearXState;
    KinematicStateVector chassisLinearYState;
    KinematicStateVector chassisLinearZState;

    KinematicStateVector chassisAngularXState;
    KinematicStateVector chassisAngularYState;
    KinematicStateVector chassisAngularZState;

    KinematicStateVector turretIMULinearXState;
    KinematicStateVector turretIMULinearYState;
    KinematicStateVector turretIMULinearZState;

    KinematicStateVector turretIMUAngularXState;
    KinematicStateVector turretIMUAngularYState;
    KinematicStateVector turretIMUAngularZState;
#endif

    modm::Vector<KinematicStateVector, 3> imuLinearState = {imuLinearXState, imuLinearYState, imuLinearZState};
    modm::Vector<KinematicStateVector, 3> imuAngularState = {imuAngularXState, imuAngularYState, imuAngularZState};

#ifndef TARGET_TURRET
    modm::Vector<KinematicStateVector, 3> chassisLinearState = {
        chassisLinearXState,
        chassisLinearYState,
        chassisLinearZState};
    modm::Vector<KinematicStateVector, 3> chassisAngularState = {
        chassisAngularXState,
        chassisAngularYState,
        chassisAngularZState};

    modm::Vector<KinematicStateVector, 3> turretIMULinearState = {
        turretIMULinearXState,
        turretIMULinearYState,
        turretIMULinearZState};
    modm::Vector<KinematicStateVector, 3> turretIMUAngularState = {
        turretIMUAngularXState,
        turretIMUAngularYState,
        turretIMUAngularZState};
#endif

    src::Informants::Odometry::ChassisKFOdometry chassisKFOdometry;
};

}  // namespace src::Informants
