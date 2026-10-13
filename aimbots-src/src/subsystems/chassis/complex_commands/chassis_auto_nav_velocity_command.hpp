#pragma once

#include "tap/control/command.hpp"

#include "subsystems/chassis/control/chassis.hpp"
#include "subsystems/chassis/control/chassis_helper.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

#if defined(CHASSIS_COMPATIBLE) && defined(GIMBAL_COMPATIBLE)

namespace src::Chassis {

/**
 * Drives the chassis from the turret-relative velocity command (m/s) that nav2 sends over the
 * Jetson link. The command is scaled by a tunable global multiplier, power-limited, 
 * then rotated from the turret frame into the chassis frame
 * using the gimbal's yaw-from-chassis-center. Rotation is left at 0 (translation only).
 */
class ChassisAutoNavVelocityCommand : public TapCommand {
public:
    ChassisAutoNavVelocityCommand(
        src::Drivers*,
        ChassisSubsystem*,
        src::Gimbal::GimbalSubsystem*);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;
    bool isFinished() const override;

    const char* getName() const override { return "Chassis Auto Nav Velocity"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;
};

}  // namespace src::Chassis

#endif  // defined(CHASSIS_COMPATIBLE) && defined(GIMBAL_COMPATIBLE)
