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

    uint32_t lastXInputCallTime = 0;
    uint32_t lastYInputCallTime = 0;
    uint32_t lastRInputCallTime = 0;

    LinearInterpolationPredictor chassisXInput;
    LinearInterpolationPredictor chassisYInput;
    LinearInterpolationPredictor chassisRotationInput;

    tap::algorithms::Ramp chassisXRamp;
    tap::algorithms::Ramp chassisYRamp;
    tap::algorithms::Ramp chassisRotationRamp;

   public:
    OperatorInterface(tap::Drivers *drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(OperatorInterface)
    mockable ~OperatorInterface() = default;

    mockable float getChassisXInput();
    mockable float getChassisYInput();
    mockable float getChassisRotationInput();

    mockable float getGimbalYawInput();
    mockable float getGimbalPitchInput();

    // Think of the maxes like setting a sensitivity for the mouse
    // The max defines an operational range of the mouse velocity
    // The scalar is used in the calculation to determine what percentage of that range is currently being used
    //(Or so I believe, this is techincally straight copied from UW's code and has yet to be field tested)
    static constexpr int16_t USER_MOUSE_YAW_MAX = 250;
    static constexpr float USER_MOUSE_YAW_SCALAR =16.0f / USER_MOUSE_YAW_MAX;

    static constexpr int16_t USER_MOUSE_PITCH_MAX = 100;
    static constexpr float USER_MOUSE_PITCH_SCALAR = 5.0f / USER_MOUSE_PITCH_MAX;
};

}  // namespace src::Control