#include "utils/robot_specific_defines.hpp"

#if defined(ALL_SENTRIES)

#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "subsystems/gimbal/gimbal.hpp"

#include "sentry_control_interface.hpp"

using namespace tap::communication::serial;
using namespace tap::algorithms;

int8_t finalXWatch = 0;
uint32_t timeCtr = 0;

static constexpr float YAW_JOYSTICK_INPUT_SENSITIVITY = 0.015f;
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

    if (prevUpdateCounterX != updateCounter) {
        chassisXInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_HORIZONTAL), currTime);
        prevUpdateCounterX = updateCounter;
    }

    float analogX = limitVal<float>(chassisXInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    float finalX = analogX;  // TODO: Add digital values from keyboard as well

    // Scales analog values by values defined in standard_constants.hpp to speedshift input
    finalX *= drivers->remote.keyPressed(Remote::Key::CTRL) ? CTRL_SCALAR : 1.0f;
    finalX *= drivers->remote.keyPressed(Remote::Key::SHIFT) ? SHIFT_SCALAR : 1.0f;

    finalXWatch = (int8_t)(finalX * 127.0f);

    return finalX;
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

    if (prevUpdateCounterY != updateCounter) {
        chassisYInput.update(drivers->remote.getChannel(Remote::Channel::LEFT_VERTICAL), currTime);
        prevUpdateCounterY = updateCounter;
    }

    float analogY = limitVal<float>(chassisYInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    float finalY = analogY;  // TODO: Add digital values from keyboard as well

    // Scales analog values by values defined in standard_constants.hpp to speedshift input
    finalY *= drivers->remote.keyPressed(Remote::Key::CTRL) ? CTRL_SCALAR : 1.0f;
    finalY *= drivers->remote.keyPressed(Remote::Key::SHIFT) ? SHIFT_SCALAR : 1.0f;

    return finalY;
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

    if (prevUpdateCounterRotation != updateCounter) {
        chassisRotationInput.update(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), currTime);
        prevUpdateCounterRotation = updateCounter;
    }

    float analogRotation = limitVal<float>(chassisRotationInput.getInterpolatedValue(currTime), -1.0f, 1.0f);

    float finalRotation = analogRotation;  // TODO: Add digital values from keyboard as well

    // Scales analog values by values defined in standard_constants.hpp to speedshift input
    finalRotation *= drivers->remote.keyPressed(Remote::Key::CTRL) ? CTRL_SCALAR : 1.0f;
    finalRotation *= drivers->remote.keyPressed(Remote::Key::SHIFT) ? SHIFT_SCALAR : 1.0f;

    return finalRotation;
}

float OperatorInterface::getGimbalYawInput() {
    mouseXFilter.update(drivers->remote.getMouseX());
    // mouseXDisplay = drivers->remote.getMouseX();
    // mouseXDisplay = mouseXFilter.getValue();

    return drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) * YAW_JOYSTICK_INPUT_SENSITIVITY +
           static_cast<float>(limitVal<int16_t>(mouseXFilter.getValue(), -MOUSE_YAW_MAX, MOUSE_YAW_MAX)) *
               YAW_MOUSE_INPUT_SENSITIVITY;
}

float OperatorInterface::getGimbalPitchInput() {
    mouseYFilter.update(-drivers->remote.getMouseY());
    // mouseYDisplay = mouseYFilter.getValue();

    return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) * PITCH_JOYSTICK_INPUT_SENSITIVITY +
           static_cast<float>(limitVal<int16_t>(mouseYFilter.getValue(), -MOUSE_PITCH_MAX, MOUSE_PITCH_MAX)) *
               PITCH_MOUSE_INPUT_SENSITIVITY;
}

}  // namespace src::Control

#endif