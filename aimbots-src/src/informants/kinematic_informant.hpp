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

    float getIMUAngularVelocity(AngularAxis axis);
    Vector3f getIMUAngularVelocities();

    float getIMULinearAcceleration(LinearAxis axis);
    Vector3f getIMUAngularAccelerations();

    Vector3f removeFalseAcceleration(Vector3f imuLKSV, Vector3f imuAKSV, Vector3f r);

    void updateIMUKinematicStateVector();

    void updateChassisIMUAngles();

    // Returns angle in rad or deg
    float getChassisIMUAngle(AngularAxis axis, AngleUnit unit);

    // Returns angular velocity in rad/s or deg/s
    float getChassisIMUAngularVelocity(AngularAxis axis, AngleUnit unit);

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

    src::Utils::KinematicStateVector imuLinearXState;
    src::Utils::KinematicStateVector imuLinearYState;
    src::Utils::KinematicStateVector imuLinearZState;

    src::Utils::KinematicStateVector imuAngularXState;
    src::Utils::KinematicStateVector imuAngularYState;
    src::Utils::KinematicStateVector imuAngularZState;

#ifndef TARGET_TURRET
    src::Utils::KinematicStateVector chassisLinearXState;
    src::Utils::KinematicStateVector chassisLinearYState;
    src::Utils::KinematicStateVector chassisLinearZState;

    src::Utils::KinematicStateVector chassisAngularXState;
    src::Utils::KinematicStateVector chassisAngularYState;
    src::Utils::KinematicStateVector chassisAngularZState;

    src::Utils::KinematicStateVector turretIMULinearXState;
    src::Utils::KinematicStateVector turretIMULinearYState;
    src::Utils::KinematicStateVector turretIMULinearZState;

    src::Utils::KinematicStateVector turretIMUAngularXState;
    src::Utils::KinematicStateVector turretIMUAngularYState;
    src::Utils::KinematicStateVector turretIMUAngularZState;
#endif

    modm::Vector<src::Utils::KinematicStateVector, 3> imuLinearState = {imuLinearXState, imuLinearYState, imuLinearZState};
    modm::Vector<src::Utils::KinematicStateVector, 3> imuAngularState = {
        imuAngularXState,
        imuAngularYState,
        imuAngularZState};

#ifndef TARGET_TURRET
    modm::Vector<src::Utils::KinematicStateVector, 3> chassisLinearState = {
        chassisLinearXState,
        chassisLinearYState,
        chassisLinearZState};
    modm::Vector<src::Utils::KinematicStateVector, 3> chassisAngularState = {
        chassisAngularXState,
        chassisAngularYState,
        chassisAngularZState};

    modm::Vector<src::Utils::KinematicStateVector, 3> turretIMULinearState = {
        turretIMULinearXState,
        turretIMULinearYState,
        turretIMULinearZState};
    modm::Vector<src::Utils::KinematicStateVector, 3> turretIMUAngularState = {
        turretIMUAngularXState,
        turretIMUAngularYState,
        turretIMUAngularZState};
#endif

    src::Informants::Odometry::ChassisKFOdometry chassisKFOdometry;
};

}  // namespace src::Informants
