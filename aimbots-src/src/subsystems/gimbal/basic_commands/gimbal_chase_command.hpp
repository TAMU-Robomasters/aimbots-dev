#pragma once

#include "tap/control/command.hpp"

#include "informants/kinematics/enemy_data_conversion.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "subsystems/gimbal/control/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/filters/ema.hpp"
#include "utils/filters/fourth_order_low_pass.hpp"

#include "drivers.hpp"
#ifdef GIMBAL_COMPATIBLE

namespace src::Utils::Ballistics {
class BallisticsSolver;

const modm::Pair<float, float> TARGET_DISTANCE_TO_YAW_VELOCITY_LIMITS[2] = {{0.5f, 3.0f}, {5.0f, 10.0f}};  // m, rad/s

const modm::interpolation::Linear<modm::Pair<float, float>> YAW_VELOCITY_LIMITER(TARGET_DISTANCE_TO_YAW_VELOCITY_LIMITS, 2);
}  // namespace src::Utils::Ballistics

namespace src::Gimbal {

class GimbalChaseCommand : public tap::control::Command {
public:
    GimbalChaseCommand(
        src::Drivers*,
        GimbalSubsystem*,
        GimbalFieldRelativeController*,
        GimbalFieldRelativeController*,
        src::Utils::RefereeHelperTurreted*,
        src::Utils::Ballistics::BallisticsSolver*,
        float defaultLaunchSpeed);

    char const* getName() const override { return "Gimbal Chase Command"; }

    void initialize() override;
    void execute() override;

    void setIgnoreQuickTurn(bool ignore) { ignoreQuickTurns = ignore; }

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

private:
    float calcDerivative(float x0, float x1, float dt) { return (x1 - x0) / (dt); }

    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalFieldRelativeController* controller;
    GimbalFieldRelativeController* cvController;

    src::Utils::RefereeHelperTurreted* refHelper;

    src::Utils::Ballistics::BallisticsSolver* ballisticsSolver;

    float defaultLaunchSpeed = 0.0f;
    
    float previousTargetYawAngle = -1000; // shouldn't every be -1000
    float previousYawVelocity = -1E6; // shouldn't ever be -1E6
    uint32_t currTime_uS = 0;
    float dt = 0.0f;
    float yawVelocity = 0.0f;
    float yawAcceleration = 0.0f;

    float previousTargetPitchAngle = -1000; // shouldn't every be -1000
    float previousPitchVelocity = -1E6; // shouldn't ever be -1E6
    float pitchVelocity = 0.0f;
    float pitchAcceleration = 0.0f;

    uint32_t lastBallisticsSolutionTimeStamp_uS = 0;

    src::Informants::Vision::PlateKinematicState data;

    bool wasQPressed = false;
    bool wasEPressed = false;
    bool ignoreQuickTurns = false;

    src::Utils::Filters::YawVelocityFourthOrderLPF yawVelocityFilter;
    src::Utils::Filters::YawAccelerationFourthOrderLPF yawAccelerationFilter;
    src::Utils::Filters::EMAFilter yawBallisticsFilter;

    src::Utils::Filters::YawVelocityFourthOrderLPF pitchVelocityFilter;
    src::Utils::Filters::YawAccelerationFourthOrderLPF pitchAccelerationFilter;
    src::Utils::Filters::EMAFilter pitchBallisticsFilter;

    bool isTargetBeingTracked = false;

};

}  // namespace src::Gimbal

#endif