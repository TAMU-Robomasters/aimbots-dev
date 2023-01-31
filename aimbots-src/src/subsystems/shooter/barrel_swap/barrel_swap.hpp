#pragma once
#ifdef TARGET_STANDARD

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

enum location {
    LEFT = 0,
    RIGHT = 1,
};

namespace src::Shooter {

class BarrelSwapSubsytem : public tap::control::Subsystem {
public:
    BarrelSwapSubsytem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;

private:
    tap::Drivers* drivers;
    location position;
};

}  // namespace src::Shooter
#endif