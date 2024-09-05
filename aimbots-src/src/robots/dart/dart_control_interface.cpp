#if defined(TARGET_DART)
#include "aerial_control_interface.hpp"

#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;
using namespace tap::algorithms;

float finalXWatch = 0;
uint32_t timeCtr = 0;

static constexpr float INPUT_X_INC = 0.003f;
static constexpr float INPUT_Y_INC = 0.003f;
static constexpr float INPUT_XY_STOP_INC = 0.03f;
static constexpr float INPUT_R_INC = 0.003f;

static constexpr float YAW_JOYSTICK_INPUT_SENSITIVITY = 0.008f;
static constexpr float PITCH_JOYSTICK_INPUT_SENSITIVITY = 0.015f;

static constexpr int16_t MOUSE_YAW_MAX = 1000;
static constexpr int16_t MOUSE_PITCH_MAX = 1000;
static constexpr float YAW_MOUSE_INPUT_SENSITIVITY = (0.15f / MOUSE_YAW_MAX);
static constexpr float PITCH_MOUSE_INPUT_SENSITIVITY = (0.1f / MOUSE_PITCH_MAX);

static constexpr float CTRL_SCALAR = (1.0f / 4);
static constexpr float SHIFT_SCALAR = 0.6f;

namespace src::Control {

/**
 * @brief Gets the current X input from the operator.
 * As the remote only returns user input every 17ms, analog input is interpolated to get a smoother usable value.
 *
 * @return float The current X input from the operator.
 */
float OperatorInterface::getChassisXInput() {
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    lastXInputCallTime = currTime;

    if (prevUpdateCounterX != updateCounter) {
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), currTime);
        prevUpdateCounterX = updateCounter;
    }

    float digitalX = drivers->remote.keyPressed(Remote::Key::D) - drivers->remote.keyPressed(Remote::Key::A);

    float finalX = limitVal<float>(chassisXInput.getInterpolatedValue(currTime) + digitalX, -1.0f, 1.0f);

    // Scales analog values by values defined in hero_constants.hpp to speedshift input
    finalX *= drivers->remote.keyPressed(Remote::Key::CTRL) ? CTRL_SCALAR : 1.0f;
    finalX *= drivers->remote.keyPressed(Remote::Key::SHIFT) ? SHIFT_SCALAR : 1.0f;

    chassisXRamp.setTarget(finalX);

    finalXWatch = digitalX;

    if (chassisXRamp.getTarget() == 0.0f)
        chassisXRamp.update(INPUT_XY_STOP_INC);
    else
        chassisXRamp.update(INPUT_X_INC);
    return chassisXRamp.getValue();
}

/**
 * @brief Gets the current Y input from the operator.
 * As the remote only returns user input every 17ms, analog input is interpolated to get a smoother usable value.
 *
 * @return float The current Y input from the operator.
 */
float OperatorInterface::getChassisYInput() {
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    lastYInputCallTime = currTime;

    if (prevUpdateCounterY != updateCounter) {
        chassisYInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterY = updateCounter;
    }

    float digitalY = drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);

    float finalY = limitVal<float>(chassisYInput.getInterpolatedValue(currTime) + digitalY, -1.0f, 1.0f);

    // Scales analog values by values defined in hero_constants.hpp to speedshift input
    finalY *= drivers->remote.keyPressed(Remote::Key::CTRL) ? CTRL_SCALAR : 1.0f;
    finalY *= drivers->remote.keyPressed(Remote::Key::SHIFT) ? SHIFT_SCALAR : 1.0f;

    chassisYRamp.setTarget(finalY);

    if (chassisYRamp.getTarget() == 0.0f)
        chassisYRamp.update(INPUT_XY_STOP_INC);
    else
        chassisYRamp.update(INPUT_Y_INC);
    return chassisYRamp.getValue();
}

/**
 * @brief Gets the current rotation input from the operator.
 * As the remote only returns user input every 17ms, analog input is interpolated to get a smoother usable value.
 *
 * @return float The current rotation input from the operator.
 */
float OperatorInterface::getChassisRotationInput() {
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    lastRInputCallTime = currTime;

    if (prevUpdateCounterRotation != updateCounter) {
        chassisRotationInput.update(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), currTime);
        prevUpdateCounterRotation = updateCounter;
    }

    float digitalRotation = drivers->remote.keyPressed(Remote::Key::Z) - drivers->remote.keyPressed(Remote::Key::X);

    float finalRotation =
        limitVal<float>(chassisRotationInput.getInterpolatedValue(currTime) + digitalRotation, -1.0f, 1.0f);
    finalRotation *= drivers->remote.keyPressed(Remote::Key::CTRL) ? CTRL_SCALAR : 1.0f;

    chassisRotationRamp.setTarget(finalRotation);

    chassisRotationRamp.update(INPUT_R_INC);
    return chassisRotationRamp.getValue();
}

int16_t mouseXDisplay = 0;
int16_t mouseYDisplay = 0;
float OperatorInterface::getGimbalYawInput() {
    mouseXFilter.update(drivers->remote.getMouseX());
    mouseXDisplay = mouseXFilter.getValue();

    // mouseXDisplay = drivers->remote.getMouseX();
    return drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) * YAW_JOYSTICK_INPUT_SENSITIVITY +
           static_cast<float>(limitVal<int16_t>(mouseXFilter.getValue(), -MOUSE_YAW_MAX, MOUSE_YAW_MAX)) *
               YAW_MOUSE_INPUT_SENSITIVITY;
}

float OperatorInterface::getGimbalPitchInput() {
    mouseYFilter.update(-drivers->remote.getMouseY());
    mouseYDisplay = mouseYFilter.getValue();

    // mouseYDisplay = drivers->remote.getMouseY();
    return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) * PITCH_JOYSTICK_INPUT_SENSITIVITY +
           static_cast<float>(limitVal<int16_t>(mouseYFilter.getValue(), -MOUSE_PITCH_MAX, MOUSE_PITCH_MAX)) *
               PITCH_MOUSE_INPUT_SENSITIVITY;
}

}  // namespace src::Control

#endif