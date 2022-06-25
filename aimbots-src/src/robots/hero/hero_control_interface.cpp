#ifdef TARGET_HERO
#include "hero_control_interface.hpp"

#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

using namespace tap::communication::serial;
using namespace tap::algorithms;

int8_t finalXWatch = 0;
uint32_t timeCtr = 0;

static constexpr float INPUT_X_MAX_ACCEL = 1000.0f;
static constexpr float INPUT_X_MAX_DECEL = 20000.0f;

static constexpr float INPUT_Y_MAX_ACCEL = 500.0f;
static constexpr float INPUT_Y_MAX_DECEL = 20000.0f;

static constexpr float INPUT_R_MAX_ACCEL = 7000.0f;
static constexpr float INPUT_R_MAX_DECEL = 20000.0f;

// static constexpr float INPUT_MOUSE_FACTOR = 1.5f;

namespace src::Control {

static float inputAccelerationTranslation(float value){
    float x = getSign(value);
    value = abs(value);
    if(value > 0){
        value = 2.0f * powf(value,2.0f);
    } else {
        value = -(2.0f)*(2.0f-(4.0f/3.0f)*((1.0f/18.0f)*powf((powf(value,-1.0f)-1.8),2.0f)))-3.1f;  
    }
    return value*x;
}

static inline void applyAccelerationToRamp(
    tap::algorithms::Ramp &ramp,
    float maxAcceleration,
    float maxDeceleration,
    float dt) {
    if (getSign(ramp.getTarget()) == getSign(ramp.getValue()) &&
        abs(ramp.getTarget()) > abs(ramp.getValue())) {
        // we are trying to speed up
        ramp.update(maxAcceleration * dt);
    } else {
        // we are trying to slow down
        ramp.update(maxDeceleration * dt);
    }
}

/**
 * @brief Gets the current X input from the operator.
 * As the remote only returns user input every 17ms, analog input is interpolated to get a smoother usable value.
 *
 * @return float The current X input from the operator.
 */
float OperatorInterface::getChassisXInput() {
    uint32_t updateCounter = drivers->remote.getUpdateCounter();
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = lastXInputCallTime - currTime;
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

    // float xValue = inputAcce(finalX);
    chassisXRamp.setTarget(finalX);

    finalXWatch = (int8_t)(finalX * 127.0f);

    applyAccelerationToRamp(
        chassisXRamp,
        INPUT_X_MAX_ACCEL,
        INPUT_X_MAX_DECEL,
        static_cast<float>(dt) / 1E3);
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
    uint32_t dt = lastYInputCallTime - currTime;
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

    applyAccelerationToRamp(
        chassisYRamp,
        INPUT_Y_MAX_ACCEL,
        INPUT_Y_MAX_DECEL,
        static_cast<float>(dt) / 1E3);
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
    uint32_t dt = lastRInputCallTime - currTime;
    lastRInputCallTime = currTime;

    if (prevUpdateCounterRotation != updateCounter) {
        chassisRotationInput.update(drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), currTime);
        prevUpdateCounterRotation = updateCounter;
    }

    float digitalRotation = drivers->remote.keyPressed(Remote::Key::Z) - drivers->remote.keyPressed(Remote::Key::X);

    float finalRotation = limitVal<float>(chassisRotationInput.getInterpolatedValue(currTime) + digitalRotation, -1.0f, 1.0f)*10.0f;
    finalRotation *= drivers->remote.keyPressed(Remote::Key::CTRL) ? CTRL_SCALAR : 1.0f;

    chassisRotationRamp.setTarget(finalRotation);

    applyAccelerationToRamp(
        chassisRotationRamp,
        INPUT_R_MAX_ACCEL,
        INPUT_R_MAX_DECEL,
        static_cast<float>(dt) / 1E3);
    return chassisRotationRamp.getValue();
}

float OperatorInterface::getGimbalYawInput() {
    return drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) +
           static_cast<float>(limitVal<int16_t>(
               drivers->remote.getMouseX(),
               -USER_MOUSE_YAW_MAX,
               USER_MOUSE_YAW_MAX)) *
               USER_MOUSE_YAW_SCALAR;
}

float OperatorInterface::getGimbalPitchInput() {
    return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) +
           static_cast<float>(limitVal<int16_t>(
               -drivers->remote.getMouseY(),
               -USER_MOUSE_PITCH_MAX,
               USER_MOUSE_PITCH_MAX)) *
               USER_MOUSE_PITCH_SCALAR;
}

}  // namespace src::Control

#endif