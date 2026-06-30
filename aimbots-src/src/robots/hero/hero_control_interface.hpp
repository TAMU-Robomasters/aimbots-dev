#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/util_macros.hpp"

#include "utils/filters/ema.hpp"

using namespace tap::algorithms;

namespace tap {
class Drivers;
}

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

    src::Utils::Filters::EMAFilter mouseXFilter;
    src::Utils::Filters::EMAFilter mouseYFilter;

    tap::algorithms::Ramp chassisXRamp;
    tap::algorithms::Ramp chassisYRamp;
    tap::algorithms::Ramp chassisRotationRamp;

    // Custom controller values are decoded in drivers->customController at ~30 Hz.
    // These predictors/ramp objects make the values usable in the faster control loop.
    uint32_t prevCustomControllerUpdateCounter = 0;
    uint32_t lastCustomControllerInputCallTime = 0;
    bool customControllerWasConnected = false;

    LinearInterpolationPredictor customControllerJoystick1XInput;
    LinearInterpolationPredictor customControllerJoystick1YInput;
    LinearInterpolationPredictor customControllerJoystick2XInput;
    LinearInterpolationPredictor customControllerJoystick2YInput;
    LinearInterpolationPredictor customControllerAnalog1Input;
    LinearInterpolationPredictor customControllerAnalog2Input;

    tap::algorithms::Ramp customControllerChassisXRamp;
    tap::algorithms::Ramp customControllerChassisYRamp;

    int16_t customControllerJoystick1XRaw = 0;
    int16_t customControllerJoystick1YRaw = 0;
    int16_t customControllerJoystick2XRaw = 0;
    int16_t customControllerJoystick2YRaw = 0;
    uint16_t customControllerAnalog1Raw = 0;
    uint16_t customControllerAnalog2Raw = 0;
    bool customControllerButton1 = false;
    bool customControllerButton2 = false;
    bool customControllerButton3 = false;
    bool customControllerButton4 = false;
    bool customControllerSwitch1 = false;
    bool customControllerSwitch2 = false;
    std::array<int16_t, 6> customControllerArmJoints{{0, 0, 0, 0, 0, 0}};

    void updateCustomControllerInputs();

   public:
    OperatorInterface(tap::Drivers *drivers)
        :
        drivers(drivers),
        mouseXFilter(0.5f),
        mouseYFilter(0.5f)  {}
    DISALLOW_COPY_AND_ASSIGN(OperatorInterface)
    mockable ~OperatorInterface() = default;

    mockable float getChassisXInput();
    mockable float getChassisYInput();
    mockable float getChassisRotationInput();

    mockable float getGimbalYawInput();
    mockable float getGimbalPitchInput();

    // Custom controller chassis/control getters. Values are normalized to [-1, 1]
    // for signed joystick axes and [0, 1] for unsigned analog axes.
    mockable bool isCustomControllerConnected();
    mockable float getCustomControllerChassisXInput();
    mockable float getCustomControllerChassisYInput();
    mockable float getCustomControllerManualSpinInput();
    mockable float getCustomControllerJoystick2YInput();
    mockable float getCustomControllerAnalog1Input();
    mockable float getCustomControllerAnalog2Input();

    mockable int16_t getCustomControllerJoystick1XRaw();
    mockable int16_t getCustomControllerJoystick1YRaw();
    mockable int16_t getCustomControllerJoystick2XRaw();
    mockable int16_t getCustomControllerJoystick2YRaw();
    mockable uint16_t getCustomControllerAnalog1Raw();
    mockable uint16_t getCustomControllerAnalog2Raw();
    mockable int16_t getCustomControllerArmJointRaw(std::size_t idx);

    mockable bool customControllerButton1Pressed();
    mockable bool customControllerButton2Pressed();
    mockable bool customControllerButton3Pressed();
    mockable bool customControllerButton4Pressed();
    mockable bool customControllerSwitch1On();
    mockable bool customControllerSwitch2On();

    // Think of the maxes like setting a sensitivity for the mouse
    // The max defines an operational range of the mouse velocity
    // The scalar is used in the calculation to determine what percentage of that range is currently being used
    //(Or so I believe, this is techincally straight copied from UW's code and has yet to be field tested)
    static constexpr int16_t USER_MOUSE_YAW_MAX = 250;
    static constexpr float USER_MOUSE_YAW_SCALAR = 16.0f / USER_MOUSE_YAW_MAX;

    static constexpr int16_t USER_MOUSE_PITCH_MAX = 100;
    static constexpr float USER_MOUSE_PITCH_SCALAR = 5.0f / USER_MOUSE_PITCH_MAX;
};

}  // namespace src::Control