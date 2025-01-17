#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "utils/tools/common_types.hpp"
#include "informants/kinematics/imu_data.hpp"

#include "robot_frames.hpp"
#include "turret_frames.hpp"

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

class FieldRelativeGimbal{

public:
    FieldRelativeGimbal(src::Drivers* drivers);
    ~FieldRelativeGimbal() = default;

    void registerSubsystems(
        src::Gimbal::GimbalSubsystem* gimbalSubsystem,
        src::Chassis::ChassisSubsystem* chassisSubsystem) 
        { this->chassisSubsystem = chassisSubsystem;
          this->gimbalSubsystem = gimbalSubsystem; }

    inline std::pair<float, float>& getGimbalFieldOrientation(int index) { return gimbalFieldOrientationBuffer[index]; }

    // put in your time, we get the closest orientation entry at that time.
    inline std::pair<float, float>& getGimbalFieldOrientationAtTime(uint32_t time_ms) {
        // assume 2 ms delay between gimbal updates
        int index = std::min(time_ms / 2, GIMBAL_BUFFER_SIZE - 1);
        return gimbalFieldOrientationBuffer[index];
    }

    // just in case?
    inline void clearGimbalFieldOrientationBuffer() { gimbalFieldOrientationBuffer.clear(); }

    tap::algorithms::WrappedFloat getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat();
    tap::algorithms::WrappedFloat getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat();

    // rad
    float getChassisPitchAngleInGimbalDirection();

    // rad/s
    float getChassisPitchVelocityInGimbalDirection();

    // m/s
    float getChassisLinearAccelerationInGimbalDirection();

    void mirrorPastRobotFrame(uint32_t frameDelay_ms);

    void updateRobotFrames();

    src::Informants::Transformers::RobotFrames& getRobotFrames() { return robotFrames; }

    src::Informants::Transformers::TurretFrames& getTurretFrames() { return turretFrames; }

private:
    src::Drivers* drivers;

    src::Gimbal::GimbalSubsystem* gimbalSubsystem;
    src::Chassis::ChassisSubsystem* chassisSubsystem;

    src::Informants::Transformers::RobotFrames robotFrames;
    src::Informants::Transformers::TurretFrames turretFrames;

    static const uint32_t GIMBAL_BUFFER_SIZE = 40;
    Deque<std::pair<float, float>, GIMBAL_BUFFER_SIZE> gimbalFieldOrientationBuffer;

};  // class FieldRelativeGimbal

}  // namespace Informants