#pragma once
#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "informants/ultrasonic_distance_sensor.hpp"
#include "utils/common_types.hpp"

namespace src {

class Drivers;

namespace Gimbal {
class GimbalSubsystem;
}

}  // namespace src

namespace src::Informants {

class FieldRelativeInformant {
public:
    FieldRelativeInformant(src::Drivers* drivers);
    ~FieldRelativeInformant() = default;

    void initialize(float imuFrequency, float imukP, float imukI);
    inline void attachGimbalSubsystem(src::Gimbal::GimbalSubsystem const* gimbal) { this->gimbal = gimbal; }

    void recalibrateIMU();

    inline float getCurrentFieldRelativeGimbalYaw(AngleUnit unit) const;

    float getChassisYaw();
    float getChassisPitch();
    float getChassisRoll();
    tap::communication::sensors::imu::ImuInterface::ImuState getImuState();

    float getGz();  // yaw axis
    float getGy();  // pitch axis
    float getGx();  // roll axis
    // get ax, ay, az haven't been converted to robot-relative yet
    float getAx();
    float getAy();
    float getAz();

    void updateFieldRelativeRobotPosition();

    Matrix<float, 1, 3> getFieldRelativeRobotPosition() { return fieldRelativeRobotPosition; }

    float getXYAngleToFieldCoordinate(AngleUnit unit, Matrix<float, 1, 3> fieldCoordinate);

private:
    src::Drivers* drivers;
    src::Gimbal::GimbalSubsystem const* gimbal;

    Matrix<float, 1, 3> robotStartingPosition;

    Matrix<float, 1, 3> fieldRelativeRobotPosition;
};

}  // namespace src::Informants
