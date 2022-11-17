#pragma once

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/util_macros.hpp"
#include "utils/robot_specific_inc.hpp"

using namespace tap::algorithms;

namespace src::Control {

class OperatorInterface {
   private:
    tap::Drivers *drivers;

    uint32_t prevUpdateCounterX = 0;
    uint32_t prevUpdateCounterY = 0;
    uint32_t prevUpdateCounterRotation = 0;

    LinearInterpolationPredictor chassisXInput;
    LinearInterpolationPredictor chassisYInput;
    LinearInterpolationPredictor chassisRotationInput;

   public:
    OperatorInterface(tap::Drivers *drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(OperatorInterface)
    mockable ~OperatorInterface() = default;

    mockable float getChassisXInput();
    mockable float getChassisYInput();
    mockable float getChassisRotationInput();

    // mockable float getGimbalYawInput();
    // mockable float getGimbalPitchInput();
};

}  // namespace src::Control