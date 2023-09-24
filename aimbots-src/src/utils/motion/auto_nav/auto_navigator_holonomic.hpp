#pragma once

#include "informants/odometry/chassis_kf_odometry.hpp"
#include "modm/math/geometry/location_2d.hpp"
#include "subsystems/chassis/chassis_helper.hpp"
#include "utils/common_types.hpp"

namespace src::Chassis::AutoNav {

class AutoNavigatorHolonomic {
public:
    AutoNavigatorHolonomic();
    ~AutoNavigatorHolonomic() = default;

    void setTargetLocation(const modm::Location2D<float>& targetLocation) { this->targetLocation = targetLocation; }

    void update(modm::Location2D<float> currentWorldLocation);

    void getDesiredInput(float* worldXError, float* worldYError, float* worldRotationError);

private:
    src::Drivers* drivers;

    modm::Location2D<float> targetLocation;

    float worldXError;
    float worldYError;
    float worldRotationError;

    float linearTolerance;
    float angularTolerance;
};

};  // namespace src::Chassis::AutoNav