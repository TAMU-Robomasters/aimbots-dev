#pragma once
#include <cstdint>
#include "subsystems/feeder/basic_commands/dual_barrel_feeder_command.hpp"
#include "subsystems/feeder/complex_commands/feeder_shot_timing_command.hpp"
#include "subsystems/feeder/control/feeder.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
#include "utils/tools/common_types.hpp"
#include "tap/architecture/timeout.hpp"
#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

static constexpr uint32_t NO_TARGET_IDLE_TIMEOUT_MS = 500;

 enum AutoAimFeederState : uint8_t { IDLE, CONTINUOUS_FIRE, SHOT_TIMING };

class AutoAimFeederCommand : public TapComprisedCommand {
public:
    AutoAimFeederCommand(
	src::Drivers* drivers,
	FeederSubsystem* feeder,
	src::Utils::RefereeHelperTurreted*,
    std::array<BarrelID, 2> BARREL_IDS,
    uint8_t projectileBuffer,
	int UMJAM_TIMER_MS = 300);


    void initialize() override;
    void execute() override;

    void end(bool interupted) override;
    bool isReady() override;
    bool isFinished() const override;

    char const* getName() const override { return "Auto-Aim Feeder Command"; }

private:

    src::Drivers* drivers;
    DualBarrelFeederCommand dualBarrelFeederCommand;
    FeederShotTimingCommand feederShotTimingCommand;
    AutoAimFeederState currentState = AutoAimFeederState::IDLE;
    tap::arch::MilliTimeout noTargetTimer;
};

} // namespace src::Feeder

#endif // #ifdef FEEDER_COMPATIBLE
