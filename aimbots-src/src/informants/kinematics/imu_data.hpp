#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "communicators/jetson/jetson_communicator.hpp"

#include "utils/tools/common_types.hpp"

#include "utils/tools/robot_specific_defines.hpp"


namespace src {
class Drivers;
}  // namespace src

using namespace src::Utils;

namespace src::Informants {

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

    inline Vector3f getIMUOrientationAtTime(uint32_t time_ms) {
        // assume 2 ms delay between gimbal updates
        int index = std::min(time_ms / KINEMATIC_REFRESH_RATE, CHASSIS_IMU_BUFFER_SIZE - 1);
        return chassisIMUHistoryBuffer[index];
    }

    tap::communication::sensors::imu::ImuInterface::ImuState getIMUState();

    KinematicStateVector getImuLinearXState() {
        return imuLinearXState;
    }
    KinematicStateVector getImuLinearYState() {
        return imuLinearYState;
    }
    KinematicStateVector getCmuLinearZState() {
        return imuLinearZState;
    }
    
    KinematicStateVector getImuAngularXState() {
        return imuAngularXState;
    }
    KinematicStateVector getImuAngularYState() {
        return imuAngularYState;
    }
    KinematicStateVector getImuAngularZState() {
        return imuAngularZState;
    }
    
    KinematicStateVector getChassisLinearXState() {
        return chassisLinearXState;
    }
    KinematicStateVector getChassisLinearYState() {
        return chassisLinearYState;
    }
    KinematicStateVector getChassisLinearZState() {
        return chassisLinearZState;
    }
    
    KinematicStateVector getChassisAngularXState() {
        return chassisAngularXState;
    }
    KinematicStateVector getChassisAngularYState() {
        return chassisAngularYState;
    }
    KinematicStateVector getChassisAngularZState() {
        return chassisAngularZState;
    }

    modm::Vector<KinematicStateVector, 3> getImuLinearState() {
        return imuLinearState;
    }
    modm::Vector<KinematicStateVector, 3> getImuAngularState() {
        return imuLinearState;
    }

    modm::Vector<KinematicStateVector, 3> getChassisLinearState() {
        return chassisLinearState; 
    }

    modm::Vector<KinematicStateVector, 3> getChassisAngularState() {
        return chassisAngularState; 
    }

private:
    src::Drivers* drivers;

    static const uint8_t KINEMATIC_REFRESH_RATE = 1;  // ms
    static const uint32_t CHASSIS_IMU_BUFFER_SIZE = 50; 
    Deque<Vector3f, CHASSIS_IMU_BUFFER_SIZE> chassisIMUHistoryBuffer;  // Buffer for turret orientation data

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

};  // class IMUData

}  // namespace Informants