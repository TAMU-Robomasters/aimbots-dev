#pragma once

#include "tap/control/command.hpp"

#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#include "subsystems/chassis/control/chassis_helper.hpp"
#include "drivers.hpp"

#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

// field relative chassis locking command
class ChassisIgnoreGimbalCommand : public TapCommand {
public:
    ChassisIgnoreGimbalCommand(
        src::Drivers*,
        ChassisSubsystem*,
        src::Gimbal::GimbalSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Chassis Ignore Gimbal"; }

    /**
     * Allows a parent scheduler command to set the wheel RPM ceiling dynamically.
     * This is used by ChassisToggleDriveCustomControllerCommand for power-aware
     * acceleration limiting.
     */
    void setMaxWheelSpeed(float maxWheelSpeed);

private:
    static constexpr float DEFAULT_CUSTOM_CONTROLLER_MAX_WHEEL_SPEED = 6500.0f;
    static constexpr float CUSTOM_CONTROLLER_TRANSLATION_DEADBAND = 0.05f;

    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    SmoothPID rotationController;

    float maxWheelSpeed = DEFAULT_CUSTOM_CONTROLLER_MAX_WHEEL_SPEED;
};

}  // namespace src::Chassis

#endif  // #ifdef CHASSIS_COMPATIBLE

#endif  // #ifdef GIMBAL_UNTETHERED