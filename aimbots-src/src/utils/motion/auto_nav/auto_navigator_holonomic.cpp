#include "auto_navigator_holonomic.hpp"

#include "src/subsystems/chassis/chassis_helper.hpp"

namespace src::Chassis::AutoNav {

AutoNavigatorHolonomic::AutoNavigatorHolonomic() {}

void AutoNavigatorHolonomic::update(modm::Location2D<float> currentWorldLocation) {
    this->worldXError = targetLocation.getX() - currentWorldLocation.getX();
    this->worldYError = targetLocation.getY() - currentWorldLocation.getY();
    this->worldRotationError = targetLocation.getOrientation() - currentWorldLocation.getOrientation();
}

void AutoNavigatorHolonomic::getDesiredInput(float* worldXError, float* worldYError, float* worldRotationError) {
    if (worldXError == nullptr || worldYError == nullptr || worldRotationError == nullptr) return;

    *worldXError = this->worldXError;
    *worldYError = this->worldYError;
    *worldRotationError = this->worldRotationError;
}

};  // namespace src::Chassis::AutoNav