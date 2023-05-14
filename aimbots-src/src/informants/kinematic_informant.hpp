#pragma once

#include <tap/algorithms/contiguous_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "transformers/robot_frames.hpp"
#include "utils/common_types.hpp"
#include "utils/kinematic_state_vector.hpp"

namespace src {
class Drivers;
}  // namespace src
namespace src::Gimbal {
class GimbalSubsystem;
}

namespace src::Informants {

enum AngularAxis { YAW_AXIS = 0, PITCH_AXIS = 1, ROLL_AXIS = 2 };

class KinematicInformant {
public:
    KinematicInformant(src::Drivers* drivers);
    ~KinematicInformant() = default;

    src::Informants::Transformers::RobotFrames& getRobotFrames() { return robotFrames; }

    void registerGimbalSubsystem(src::Gimbal::GimbalSubsystem* gimbalSubsystem) { this->gimbalSubsystem = gimbalSubsystem; }

    void initialize(float imuFrequency, float imukP, float imukI);
    void recalibrateIMU();
    tap::communication::sensors::imu::ImuInterface::ImuState getIMUState();

    void updateChassisIMUAngles();

    // Returns angle in rad or deg
    float getChassisIMUAngle(AngularAxis axis, AngleUnit unit);

    // Returns angular velocity in rad/s or deg/s
    float getIMUAngularVelocity(AngularAxis axis, AngleUnit unit);
    // Returns linear velocity in rad/s^2 or deg/s^2 if you are a loser
    float getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit);
    // Returns lnothing!!!
    void updateChassisAcceleration();

    void updateRobotFrames();

    tap::algorithms::ContiguousFloat getCurrentFieldRelativeYawAngleAsContiguousFloat();
    tap::algorithms::ContiguousFloat getCurrentFieldRelativePitchAngleAsContiguousFloat();
    void mirrorPastRobotFrame(uint32_t frameDelay_ms);

private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbalSubsystem;

    src::Informants::Transformers::RobotFrames robotFrames;
    src::Utils::KinematicStateVector imuLinearXState;
    src::Utils::KinematicStateVector imuLinearYState;
    src::Utils::KinematicStateVector imuLinearZState;

    src::Utils::KinematicStateVector imuAngularXState;
    src::Utils::KinematicStateVector imuAngularYState;
    src::Utils::KinematicStateVector imuAngularZState;

    src::Utils::KinematicStateVector chassisLinearXState;
    src::Utils::KinematicStateVector chassisLinearYState;
    src::Utils::KinematicStateVector chassisLinearZState;

    Vector3f chassisIMUAngles = {0.0f, 0.0f, 0.0f};
    Vector3f chassisIMUAngularVelocities = {0.0f, 0.0f, 0.0f};

    modm::Vector<src::Utils::KinematicStateVector, 3> imuLinearState = {imuLinearXState, imuLinearYState, imuLinearZState};
    modm::Vector<src::Utils::KinematicStateVector, 3> imuAngularState = {
        imuAngularXState,
        imuAngularYState,
        imuAngularZState};
    modm::Vector<src::Utils::KinematicStateVector, 3> chassisLinearState = {
        chassisLinearXState,
        chassisLinearYState,
        chassisLinearZState};
};

}  // namespace src::Informants
