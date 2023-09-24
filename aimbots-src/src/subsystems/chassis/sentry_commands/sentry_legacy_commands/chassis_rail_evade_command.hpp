#pragma once

#include "tap/control/command.hpp"

#include "subsystems/chassis/chassis.hpp"

#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

class ChassisRailEvadeCommand : public TapCommand {
public:
    ChassisRailEvadeCommand(src::Drivers*, ChassisSubsystem*, float velocityRampValue = 25.0f);

    char const* getName() const override { return "Sentry Evade Command"; }

    void initialize() override;
    void execute() override;

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    inline bool hasTraveledDriveDistance(float currentPosition) const {
        return fabs(lastPositionWhenDirectionChanged - currentPosition) >= distanceToDrive;
    }

    void changeDirectionForRandomDistance(int32_t minimumDistance, int32_t maximumDistance);
    void changeDirectionIfCloseToEnd(float currentRailPosition);

    uint32_t getRandomInteger();
    int32_t getRandomIntegerInBounds(int32_t min, int32_t max);

    int32_t getNewRPM();

private:
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    float velocityRampValue = 0.0f;
    tap::algorithms::Ramp velocityRamp;

    float distanceToDrive = 0.0f;
    float lastPositionWhenDirectionChanged = 0.0f;
    float currentDesiredRPM = 0.0f;
};

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE