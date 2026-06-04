#pragma once

#include <drivers.hpp>
#include <subsystems/feeder/control/feeder.hpp>
#include <tap/control/command.hpp>

#include "subsystems/feeder/control/feeder.hpp"

#ifdef FEEDER_COMPATIBLE
namespace src::Feeder {

struct FeederVelocityTunningConfig {
    float VelocityAmplitudeRPM = 0.0f;
    float frequencyHz = 0.0f;
};

class FeederVelocityTunningCommand : public tap::control::Command {
public:
    FeederVelocityTunningCommand(
        src::Drivers* drivers,
        FeederSubsystem* feederSubsystem,
        FeederVelocityTunningConfig feederVelocityConfig);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;
    bool isReady() override;
        
    const char* getName() const { return "feeder velocity tunning"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    FeederVelocityTunningConfig feederVelocityConfig;
    uint32_t initTime;

    float getFeederTargetVelocity();
    float getRelativeTime() const;
};

} // namespace src::Feeder
#endif // #ifdef FEEDER_COMPATIBLE