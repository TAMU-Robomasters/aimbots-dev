#pragma once

#include <tap/algorithms/contiguous_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "informants/odometry/chassis_kf_odometry.hpp"
#include "informants/vision/jetson_communicator.hpp"
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
}  // namespace src::Chassis

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

    tap::communication::sensors::imu::ImuInterface::ImuState getIMUState();

    void initialize(float imuFrequency, float imukP, float imukI);

    void recalibrateIMU(Vector3f imuCalibrationEuler = {0.0f, 0.0f, 0.0f});

    // Gets raw IMU values, in our coordinate system XYZ (Pitch, Roll, Yaw)
    Vector3f getLocalIMUAngles();
    float getLocalIMUAngle(AngularAxis axis);

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

    inline Vector3f getChassisIMUOrientationAtTime(uint32_t time_ms) {
        // assume 2 ms delay between gimbal updates
        int index = std::min(time_ms / KINEMATIC_REFRESH_RATE, CHASSIS_IMU_BUFFER_SIZE - 1);
        return chassisIMUHistoryBuffer[index];
    }

    modm::Location2D<float> getRobotLocation2D() { return chassisKFOdometry.getCurrentLocation2D(); }

    modm::Vector2f getRobotVelocity2D() { return chassisKFOdometry.getCurrentVelocity2D(); }

private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbalSubsystem;
    tap::control::chassis::ChassisSubsystemInterface* chassisSubsystem;

    src::Informants::Transformers::RobotFrames robotFrames;

    static const uint32_t CHASSIS_IMU_BUFFER_SIZE = 50;
    static const uint8_t KINEMATIC_REFRESH_RATE = 1;  // ms

    Deque<Vector3f, CHASSIS_IMU_BUFFER_SIZE> chassisIMUHistoryBuffer;

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

    KinematicStateVector turretIMULinearXState;
    KinematicStateVector turretIMULinearYState;
    KinematicStateVector turretIMULinearZState;

    KinematicStateVector turretIMUAngularXState;
    KinematicStateVector turretIMUAngularYState;
    KinematicStateVector turretIMUAngularZState;

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

    modm::Vector<KinematicStateVector, 3> turretIMULinearState = {
        turretIMULinearXState,
        turretIMULinearYState,
        turretIMULinearZState};
    modm::Vector<KinematicStateVector, 3> turretIMUAngularState = {
        turretIMUAngularXState,
        turretIMUAngularYState,
        turretIMUAngularZState};

    src::Informants::Odometry::ChassisKFOdometry chassisKFOdometry;
};

}  // namespace src::Informants