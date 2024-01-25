#pragma once

#include "informants/kinematic_informant.hpp"
#include "informants/turret-comms/turret_can_communicator.hpp"
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "utils/common_types.hpp"
#if defined(GIMBAL_COMPATIBLE) && defined(CHASSIS_COMPATIBLE)

namespace src::Informants {

class IMUCalibrateCommand : public TapCommand {
public:
    enum class CalibrationState {
        WAITING_FOR_SYSTEMS_ONLINE,
        LOCKING_CHASSIS,
        LOCKING_TURRET,
        CALIBRATING_IMU,
        WAITING_CALIBRATION_COMPLETE,
        DONE
    };

    IMUCalibrateCommand(src::Drivers*, src::Chassis::ChassisSubsystem*, src::Gimbal::GimbalSubsystem*);
    ~IMUCalibrateCommand() = default;

    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "IMU Calibrate"; }

private:
    src::Drivers* drivers;

    CalibrationState calibrationState;

    src::Chassis::ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    src::Gimbal::GimbalChassisRelativeController gimbalController;

    static const uint32_t MAXIMUM_CALIBRATION_TIME = 5000;
    static const uint32_t WAIT_EXTRA_TIME = 1000;

    MilliTimeout extraCalibrationWaitTimer;
    MilliTimeout maximumCalibrationTimer;
};

}  // namespace src::Informants
#endif