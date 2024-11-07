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

class KinematicInformant {
public:
    KinematicInformant(src::Drivers* drivers);
    ~KinematicInformant() = default;

    // Gets raw IMU values, in our coordinate system XYZ (Pitch, Roll, Yaw)
    Vector3f getLocalIMUAngles();
    float getLocalIMUAngle(AngularAxis axis);

    Vector3f getIMUAngularVelocities();
    float getIMUAngularVelocity(AngularAxis axis);

    Vector3f getIMULinearAccelerations();
    float getIMULinearAcceleration(LinearAxis axis);

    void updateIMUKinematicStateVector();

    void updateIMUAngles();

    // Returns angle in rad or deg
    float getIMUAngle(AngularAxis axis, AngleUnit unit);

    // Returns angular velocity in rad/s or deg/s
    float getIMUAngularVelocity(AngularAxis axis, AngleUnit unit);

    Vector3f getIMUAngularAccelerations();
    float getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit);
    // Returns lnothing!!!
private:
    src::Drivers* drivers;

    src::Informants::Transformers::RobotFrames robotFrames;
    src::Informants::Transformers::TurretFrames turretFrames;

    static const uint32_t CHASSIS_IMU_BUFFER_SIZE = 50;
    static const uint8_t KINEMATIC_REFRESH_RATE = 1;  // ms

    Deque<Vector3f, CHASSIS_IMU_BUFFER_SIZE> chassisIMUHistoryBuffer;

    KinematicStateVector imuLinearXState;
    KinematicStateVector imuLinearYState;
    KinematicStateVector imuLinearZState;

    KinematicStateVector imuAngularXState;
    KinematicStateVector imuAngularYState;
    KinematicStateVector imuAngularZState;

    KinematicStateVector turretIMULinearXState;
    KinematicStateVector turretIMULinearYState;
    KinematicStateVector turretIMULinearZState;

    KinematicStateVector turretIMUAngularXState;
    KinematicStateVector turretIMUAngularYState;
    KinematicStateVector turretIMUAngularZState;

    
    modm::Vector<KinematicStateVector, 3> imuLinearState = {imuLinearXState, imuLinearYState, imuLinearZState};
    modm::Vector<KinematicStateVector, 3> imuAngularState = {imuAngularXState, imuAngularYState, imuAngularZState};

    
    modm::Vector<KinematicStateVector, 3> turretIMULinearState = {
        turretIMULinearXState,
        turretIMULinearYState,
        turretIMULinearZState};
    modm::Vector<KinematicStateVector, 3> turretIMUAngularState = {
        turretIMUAngularXState,
        turretIMUAngularYState,
        turretIMUAngularZState};
};

}  // namespace src::Informants