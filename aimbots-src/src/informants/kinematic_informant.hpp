#pragma once

#include <tap/algorithms/contiguous_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "transformers/robot_frames.hpp"
#include "utils/common_types.hpp"

namespace src {

class Drivers;

namespace Gimbal {
class GimbalSubsystem;
}

}  // namespace src

namespace src::Informants {

enum AngularAxis { YAW_AXIS = 0, PITCH_AXIS = 1, ROLL_AXIS = 2 };

class KinematicInformant {
public:
    KinematicInformant(src::Drivers* drivers);
    ~KinematicInformant() = default;

    src::Informants::Transformers::RobotFrames& getRobotFrames() { return robotFrames; }

    void attachGimbalSubsystem(src::Gimbal::GimbalSubsystem* gimbalSubsystem) { this->gimbalSubsystem = gimbalSubsystem; }

    void initialize(float imuFrequency, float imukP, float imukI);
    void recalibrateIMU();
    tap::communication::sensors::imu::ImuInterface::ImuState getIMUState();

    // Returns angle in rad or deg
    float getIMUAngle(AngularAxis axis, AngleUnit unit);
    // Returns angular velocity in rad/s or deg/s
    float getIMUAngularVelocity(AngularAxis axis, AngleUnit unit);
    // Returns linear velocity in rad/s^2 or deg/s^2 if you are a loser
    float getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit);
    // Returns linear acceleration in m/s^2
    float getIMULinearAcceleration(LinearAxis axis);

    void updateRobotFrames();

    tap::algorithms::ContiguousFloat getCurrentFieldRelativeYawAngleAsContiguousFloat();

private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem* gimbalSubsystem;

    src::Informants::Transformers::RobotFrames robotFrames;
};

}  // namespace src::Informants
