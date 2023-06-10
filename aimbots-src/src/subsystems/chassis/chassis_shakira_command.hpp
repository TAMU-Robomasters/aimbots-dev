#pragma once

#include "subsystems/chassis/chassis.hpp"
#include "subsystems/gimbal/gimbal.hpp"

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
        int numCorners = 4,
        float starterAngle = modm::toRadian(45.0f),
        float angularMagnitude = modm::toRadian(10.0f),
        uint32_t timePeriod = 3000);
    ~ChassisShakiraCommand() override = default;

    void initialize() override;

    void execute() override;

    float findNearestCornerAngle(float targetAngle);

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

    int numCorners;
    float starterAngle;  // rad

    float angularMagnitude;  // rad
    uint32_t timePeriod;     // in ms
};

}  // namespace src::Chassis