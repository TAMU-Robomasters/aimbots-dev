#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "communicators/jetson/jetson_communicator.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "utils/tools/common_types.hpp"

#include "utils/tools/robot_specific_defines.hpp"

#include "robot_frames.hpp"
#include "turret_frames.hpp"

namespace src {
class Drivers;
}  // namespace src

using namespace src::Utils;

namespace src::Informants {

enum AngularAxis { PITCH_AXIS = 0, ROLL_AXIS = 1, YAW_AXIS = 2 };

class IMUData {
public:
    IMUData(src::Drivers* drivers);
    ~IMUData() = default;

    // Gets raw IMU values, in our coordinate system XYZ (Pitch, Roll, Yaw)
    Vector3f getRawLocalIMUAngles();
    float getRawLocalIMUAngle(AngularAxis axis);

    Vector3f getRawIMUAngularVelocities();
    float getRawIMUAngularVelocity(AngularAxis axis);

    Vector3f getRawIMULinearAccelerations();
    float getRawIMULinearAcceleration(LinearAxis axis);

    void updateIMUKinematicStateVector();

    void updateIMUAngles();

    // Returns angle in rad or deg
    float getIMUAngle(AngularAxis axis, AngleUnit unit);

    // Returns angular velocity in rad/s or deg/s
    float getIMUAngularVelocity(AngularAxis axis, AngleUnit unit);

    Vector3f getIMUAngularAccelerations();
    float getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit);
    // Returns lnothing!!!
    void recalibrateIMU(Vector3f imuCalibrationEuler = {0.0f, 0.0f, 0.0f});
private:

};

}  // namespace src::Informants