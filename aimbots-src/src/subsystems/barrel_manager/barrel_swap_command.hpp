#pragma once
#ifdef TARGET_STANDARD
#include "barrel_manager.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

#include "drivers.hpp"

namespace src::Shooter {
class BarrelSwapCommand : public TapCommand {
public:
    BarrelSwapCommand(src::Drivers* drivers, BarrelSwapSubsytem* barrelSwap);

    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "barrel swap command"; }

private:
    src::Drivers* drivers;
    BarrelSwapSubsytem* barrelSwap;
};

}  // namespace src::Shooter

#endif