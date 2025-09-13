#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/control/gimbal.hpp>
#include <tap/control/command.hpp>

#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal {

struct MortarCommandConfig {
    // /** @brief everything is relative to where the gimbal origin is which is define
    //  * as the point where the the yaw axis of rotation and the pitch axis of rotation
    //  * intersect
    //  */
    // float xFinalProjectilePosition = 0.0f; // when yaw angle is zero this is right (+) or left (-)
    // float yFinalProjectilePosition = 0.0f; // when yaw angle is zero this is forward (+) or backwards (-)
    // float zFinalProjectilePosition = 0.0f; // when yaw angle is zero this is up (+) or down (-)

    float pitchAngleDegrees = 0.0f;
    float yawPositionAmplitudeDegrees = 0.0f;
    float yawFrequencyPeriodSeconds = 0.0f;
};

class GimbalMortarCommand : public tap::control::Command {
public:
    GimbalMortarCommand(
        src::Drivers* drivers,
        GimbalSubsystem* gimbalSubsystem,
        GimbalFieldRelativeController* controller,
        // src::Utils::Ballistics::BallisticsSolver* ballisticsSolver,
        // src::Utils::RefereeHelperTurreted* refHelper,
        MortarCommandConfig config
        /*float defaultLaunchSpeed*/);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;
    bool isReady() override;

    const char* getName() const { return "gimbal mortar command"; }

private:
    src::Drivers* drivers;
    GimbalSubsystem* gimbal;
    GimbalFieldRelativeController* controller;
    MortarCommandConfig config;
    uint32_t initTime;

    float getYawTargetPosition();
    float getRelativeTime() const;
};

} // namespace src::Gimbal 
#endif // #ifdef GIMBAL_COMPATIBLE