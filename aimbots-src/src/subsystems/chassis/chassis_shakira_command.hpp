#pragma once

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/gimbal.hpp"

#include "chassis_helper.hpp"
#include "drivers.hpp"


using namespace src::Utils::Ballistics;

namespace src::Utils::Ballistics {
class BallisticsSolver;
}

namespace src::Chassis {

class ChassisShakiraCommand : public TapCommand {
public:
    ChassisShakiraCommand(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        src::Gimbal::GimbalSubsystem* gimbal,
        SmoothPIDConfig* rotationControllerConfig,
        BallisticsSolver* ballisticsSolver,
        const TokyoConfig& tokyoConfig,
        const SnapSymmetryConfig& snapSymmetryConfig = SnapSymmetryConfig(),
        float angularMagnitude = modm::toRadian(10.0f),
        uint32_t timePeriod = 3000);
    ~ChassisShakiraCommand() override = default;

    void initialize() override;

    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    const char* getName() const override { return "These hips do lie"; }

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;
    src::Gimbal::GimbalSubsystem* gimbal;

    SmoothPID rotationController;
    BallisticsSolver* ballisticsSolver;

    uint16_t startingTimestamp = 0.0f;

    const SnapSymmetryConfig& snapSymmetryConfig;

    const TokyoConfig& tokyoConfig;

    float angularMagnitude;  // rad
    uint32_t timePeriod;     // in ms
};

}  // namespace src::Chassis