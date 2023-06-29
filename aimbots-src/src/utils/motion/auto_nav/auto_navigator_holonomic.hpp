#pragma once

#include "informants/odometry/chassis_kf_odometry.hpp"
#include "modm/math/geometry/location_2d.hpp"
#include "subsystems/chassis/chassis_helper.hpp"
#include "utils/common_types.hpp"

namespace src::Chassis::AutoNav {

class AutoNavigatorHolonomic {
public:
    AutoNavigatorHolonomic(
        src::Drivers* drivers,
        SmoothPIDConfig linearPIDConfig,
        SmoothPIDConfig rotationPIDConfig,
        const src::Informants::Odometry::ChassisKFOdometry& odometry,
        const SnapSymmetryConfig& snapSymmetryConfig = SnapSymmetryConfig(),
        float linearTolerance = 0.1,  // meters
        float angularTolerance = modm::toRadian(2.0f));
    ~AutoNavigatorHolonomic() = default;

    void setTargetLocation(const modm::Location2D<float>& targetLocation) { this->targetLocation = targetLocation; }

    void update();

    float getXOutput() { return xController.getOutput(); }
    float getYOutput() { return yController.getOutput(); }
    float getRotationOutput() { return rotationController.getOutput(); }

    bool isSettled();

private:
    src::Drivers* drivers;

    modm::Location2D<float> targetLocation;

    SmoothPID xController;
    SmoothPID yController;
    SmoothPID rotationController;

    const SnapSymmetryConfig& snapSymmetryConfig;

    const src::Informants::Odometry::ChassisKFOdometry& odometry;

    float linearTolerance;
    float angularTolerance;
};

};  // namespace src::Chassis::AutoNav