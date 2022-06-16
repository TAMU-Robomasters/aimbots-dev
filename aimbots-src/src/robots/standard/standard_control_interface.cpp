#ifdef TARGET_STANDARD
#include "standard_control_interface.hpp"

#include "subsystems/gimbal/gimbal.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;
using namespace tap::algorithms;

int8_t finalXWatch = 0;
uint32_t timeCtr = 0;

float mouseRotation = 0.0f;
float remoteX = 0.0f;

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

    float digitalX = drivers->remote.keyPressed(Remote::Key::D) - drivers->remote.keyPressed(Remote::Key::A);

    float analogX = chassisXInput.getInterpolatedValue(currTime);

    float finalX = limitVal<float>(digitalX + analogX, -1.0f, 1.0f);

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

    float digitalY = drivers->remote.keyPressed(Remote::Key::W) - drivers->remote.keyPressed(Remote::Key::S);

    float analogY = chassisYInput.getInterpolatedValue(currTime);

    float finalY = limitVal<float>(digitalY + analogY, -1.0f, 1.0f);

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

    float digitalRotation = drivers->remote.keyPressed(Remote::Key::Q) - drivers->remote.keyPressed(Remote::Key::E);

    digitalRotation = static_cast<float>(limitVal<int16_t>(drivers->remote.getMouseX(), -USER_MOUSE_YAW_MAX, USER_MOUSE_YAW_MAX)) * USER_MOUSE_YAW_SCALAR;

    float analogRotation = chassisRotationInput.getInterpolatedValue(currTime);

    float finalRotation = limitVal(analogRotation + digitalRotation, -1.0f, 1.0f);  // TODO: Add digital values from keyboard as well

    // Scales analog values by values defined in standard_constants.hpp to speedshift input
    finalRotation *= drivers->remote.keyPressed(Remote::Key::CTRL) ? CTRL_SCALAR : 1.0f;
    finalRotation *= drivers->remote.keyPressed(Remote::Key::SHIFT) ? SHIFT_SCALAR : 1.0f;

    return finalRotation;
}

float OperatorInterface::getGimbalYawInput() {
    float analogYaw = drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);

    float mouseYaw = static_cast<float>(limitVal<int16_t>(drivers->remote.getMouseX(), -USER_MOUSE_YAW_MAX, USER_MOUSE_YAW_MAX)) * USER_MOUSE_YAW_SCALAR;

    float finalYaw = analogYaw + mouseYaw;

    return finalYaw;
}

float OperatorInterface::getGimbalPitchInput() {
    float analogYaw = drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL);

    // float mouseYaw = static_cast<float>(limitVal<int16_t>(drivers->remote.getMouseY(), -USER_MOUSE_PITCH_MAX, USER_MOUSE_PITCH_MAX)) * USER_MOUSE_PITCH_SCALAR*;

    // float finalYaw = analogYaw + mouseYaw;

    return ((drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) + (static_cast<float>(limitVal<int16_t>(-drivers->remote.getMouseY(), -USER_MOUSE_PITCH_MAX, USER_MOUSE_PITCH_MAX)) * USER_MOUSE_PITCH_SCALAR)) * src::Gimbal::getPitchMotorDirection());
}

}  // namespace src::Control

#endif