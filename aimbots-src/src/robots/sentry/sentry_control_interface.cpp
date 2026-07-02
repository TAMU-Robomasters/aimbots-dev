#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_SENTRIES)
#include <algorithm>
#include <cstddef>

#include "tap/algorithms/ramp.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "drivers.hpp"
#include "sentry_control_interface.hpp"

using namespace tap::communication::serial;
using namespace tap::algorithms;

float finalXWatch = 0;
uint32_t timeCtr = 0;

static constexpr float INPUT_X_INC = 0.01f;
static constexpr float INPUT_Y_INC = 0.01f;
static constexpr float INPUT_XY_STOP_INC = 0.03f;
static constexpr float INPUT_R_INC = 0.01f;

// Custom controller is updated at about 30 Hz, so ramp it a little more slowly than keyboard release,
// but still fast enough to feel responsive in the 2 ms command loop.
static constexpr float CUSTOM_CONTROLLER_INPUT_X_INC = 0.01f;
static constexpr float CUSTOM_CONTROLLER_INPUT_Y_INC = 0.01f;
static constexpr float CUSTOM_CONTROLLER_INPUT_STOP_INC = 0.035f;

static constexpr float YAW_JOYSTICK_INPUT_SENSITIVITY = 0.015f;
static constexpr float PITCH_JOYSTICK_INPUT_SENSITIVITY = 0.015f;

static constexpr int16_t MOUSE_YAW_MAX = 1000;
static constexpr int16_t MOUSE_PITCH_MAX = 1000;
static constexpr float YAW_MOUSE_INPUT_SENSITIVITY = (0.15f / MOUSE_YAW_MAX);
static constexpr float PITCH_MOUSE_INPUT_SENSITIVITY = (0.1f / MOUSE_PITCH_MAX);

static constexpr float CTRL_SCALAR = (1.0f / 4);
static constexpr float SHIFT_SCALAR = 0.6f;

static constexpr float CUSTOM_CONTROLLER_SIGNED_AXIS_SCALE = 1.0f / 1024.0f;
static constexpr float CUSTOM_CONTROLLER_ANALOG_AXIS_SCALE = 1.0f / 2048.0f;

namespace src::Control {

static inline float normalizeSignedCustomControllerAxis(int16_t value) {
    return limitVal<float>(static_cast<float>(value) * CUSTOM_CONTROLLER_SIGNED_AXIS_SCALE, -1.0f, 1.0f);
}

static inline float normalizeUnsignedCustomControllerAxis(uint16_t value) {
    return limitVal<float>(static_cast<float>(value) * CUSTOM_CONTROLLER_ANALOG_AXIS_SCALE, 0.0f, 1.0f);
}

void OperatorInterface::updateCustomControllerInputs() {
    const uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    lastCustomControllerInputCallTime = currTime;

    src::Drivers *srcDrivers = static_cast<src::Drivers *>(drivers);

    // This is safe even though main/updateIo already calls customController.read();
    // vtmCan only sends a poll when its timer is due.
    srcDrivers->customController.read();

    const bool connected = srcDrivers->customController.isConnected();
    if (!connected) {
        customControllerButton1 = false;
        customControllerButton2 = false;
        customControllerButton3 = false;
        customControllerButton4 = false;
        customControllerSwitch1 = false;
        customControllerSwitch2 = false;

        customControllerJoystick1XRaw = 0;
        customControllerJoystick1YRaw = 0;
        customControllerJoystick2XRaw = 0;
        customControllerJoystick2YRaw = 0;
        customControllerAnalog1Raw = 0;
        customControllerAnalog2Raw = 0;
        customControllerArmJoints = {{0, 0, 0, 0, 0, 0}};

        if (customControllerWasConnected) {
            customControllerJoystick1XInput.update(0.0f, currTime);
            customControllerJoystick1YInput.update(0.0f, currTime);
            customControllerJoystick2XInput.update(0.0f, currTime);
            customControllerJoystick2YInput.update(0.0f, currTime);
            customControllerAnalog1Input.update(0.0f, currTime);
            customControllerAnalog2Input.update(0.0f, currTime);
        }

        customControllerWasConnected = false;
        return;
    }

    customControllerWasConnected = true;

    const uint32_t updateCounter = srcDrivers->customController.getUpdateCounter();
    if (prevCustomControllerUpdateCounter == updateCounter) {
        return;
    }
    prevCustomControllerUpdateCounter = updateCounter;

    customControllerJoystick1XRaw = srcDrivers->customController.joystick1X();
    customControllerJoystick1YRaw = srcDrivers->customController.joystick1Y();
    customControllerJoystick2XRaw = srcDrivers->customController.joystick2X();
    customControllerJoystick2YRaw = srcDrivers->customController.joystick2Y();
    customControllerAnalog1Raw = srcDrivers->customController.analogInput1();
    customControllerAnalog2Raw = srcDrivers->customController.analogInput2();

    customControllerButton1 = srcDrivers->customController.button1Pressed();
    customControllerButton2 = srcDrivers->customController.button2Pressed();
    customControllerButton3 = srcDrivers->customController.button3Pressed();
    customControllerButton4 = srcDrivers->customController.button4Pressed();
    customControllerSwitch1 = srcDrivers->customController.switch1On();
    customControllerSwitch2 = srcDrivers->customController.switch2On();

    for (std::size_t i = 0; i < customControllerArmJoints.size(); i++) {
        customControllerArmJoints[i] = srcDrivers->customController.armJoint(i);
    }

    customControllerJoystick1XInput.update(
        normalizeSignedCustomControllerAxis(customControllerJoystick1XRaw),
        currTime);
    customControllerJoystick1YInput.update(
        normalizeSignedCustomControllerAxis(customControllerJoystick1YRaw),
        currTime);
    customControllerJoystick2XInput.update(
        normalizeSignedCustomControllerAxis(customControllerJoystick2XRaw),
        currTime);
    customControllerJoystick2YInput.update(
        normalizeSignedCustomControllerAxis(customControllerJoystick2YRaw),
        currTime);
    customControllerAnalog1Input.update(
        normalizeUnsignedCustomControllerAxis(customControllerAnalog1Raw),
        currTime);
    customControllerAnalog2Input.update(
        normalizeUnsignedCustomControllerAxis(customControllerAnalog2Raw),
        currTime);
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
    // mouseXDisplay = drivers->remote.getMouseX();
    mouseXDisplay = mouseXFilter.getValue();

    return drivers->remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL) * YAW_JOYSTICK_INPUT_SENSITIVITY +
           static_cast<float>(limitVal<int16_t>(mouseXFilter.getValue(), -MOUSE_YAW_MAX, MOUSE_YAW_MAX)) *
               YAW_MOUSE_INPUT_SENSITIVITY;
}

float OperatorInterface::getGimbalPitchInput() {
    mouseYFilter.update(drivers->remote.getMouseY());
    mouseYDisplay = mouseYFilter.getValue();

    return drivers->remote.getChannel(Remote::Channel::RIGHT_VERTICAL) * PITCH_JOYSTICK_INPUT_SENSITIVITY +
           static_cast<float>(limitVal<int16_t>(mouseYFilter.getValue(), -MOUSE_PITCH_MAX, MOUSE_PITCH_MAX)) *
               PITCH_MOUSE_INPUT_SENSITIVITY;
}

bool OperatorInterface::isCustomControllerConnected() {
    updateCustomControllerInputs();
    return customControllerWasConnected;
}

float OperatorInterface::getCustomControllerChassisXInput() {
    updateCustomControllerInputs();
    const uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    const float input = customControllerJoystick1XInput.getInterpolatedValue(currTime);
    customControllerChassisXRamp.setTarget(limitVal<float>(input, -1.0f, 1.0f));

    if (customControllerChassisXRamp.getTarget() == 0.0f) {
        customControllerChassisXRamp.update(CUSTOM_CONTROLLER_INPUT_STOP_INC);
    } else {
        customControllerChassisXRamp.update(CUSTOM_CONTROLLER_INPUT_X_INC);
    }

    return customControllerChassisXRamp.getValue();
}

float OperatorInterface::getCustomControllerChassisYInput() {
    updateCustomControllerInputs();
    const uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    const float input = customControllerJoystick1YInput.getInterpolatedValue(currTime);
    customControllerChassisYRamp.setTarget(limitVal<float>(input, -1.0f, 1.0f));

    if (customControllerChassisYRamp.getTarget() == 0.0f) {
        customControllerChassisYRamp.update(CUSTOM_CONTROLLER_INPUT_STOP_INC);
    } else {
        customControllerChassisYRamp.update(CUSTOM_CONTROLLER_INPUT_Y_INC);
    }

    return customControllerChassisYRamp.getValue();
}

float OperatorInterface::getCustomControllerManualSpinInput() {
    updateCustomControllerInputs();
    return limitVal<float>(
        customControllerJoystick2XInput.getInterpolatedValue(tap::arch::clock::getTimeMilliseconds()),
        -1.0f,
        1.0f);
}

float OperatorInterface::getCustomControllerJoystick2YInput() {
    updateCustomControllerInputs();
    return limitVal<float>(
        customControllerJoystick2YInput.getInterpolatedValue(tap::arch::clock::getTimeMilliseconds()),
        -1.0f,
        1.0f);
}

float OperatorInterface::getCustomControllerAnalog1Input() {
    updateCustomControllerInputs();
    return limitVal<float>(
        customControllerAnalog1Input.getInterpolatedValue(tap::arch::clock::getTimeMilliseconds()),
        0.0f,
        1.0f);
}

float OperatorInterface::getCustomControllerAnalog2Input() {
    updateCustomControllerInputs();
    return limitVal<float>(
        customControllerAnalog2Input.getInterpolatedValue(tap::arch::clock::getTimeMilliseconds()),
        0.0f,
        1.0f);
}

int16_t OperatorInterface::getCustomControllerJoystick1XRaw() {
    updateCustomControllerInputs();
    return customControllerJoystick1XRaw;
}

int16_t OperatorInterface::getCustomControllerJoystick1YRaw() {
    updateCustomControllerInputs();
    return customControllerJoystick1YRaw;
}

int16_t OperatorInterface::getCustomControllerJoystick2XRaw() {
    updateCustomControllerInputs();
    return customControllerJoystick2XRaw;
}

int16_t OperatorInterface::getCustomControllerJoystick2YRaw() {
    updateCustomControllerInputs();
    return customControllerJoystick2YRaw;
}

uint16_t OperatorInterface::getCustomControllerAnalog1Raw() {
    updateCustomControllerInputs();
    return customControllerAnalog1Raw;
}

uint16_t OperatorInterface::getCustomControllerAnalog2Raw() {
    updateCustomControllerInputs();
    return customControllerAnalog2Raw;
}

int16_t OperatorInterface::getCustomControllerArmJointRaw(std::size_t idx) {
    updateCustomControllerInputs();
    if (idx >= customControllerArmJoints.size()) {
        return 0;
    }
    return customControllerArmJoints[idx];
}

bool OperatorInterface::customControllerButton1Pressed() {
    updateCustomControllerInputs();
    return customControllerButton1;
}

bool OperatorInterface::customControllerButton2Pressed() {
    updateCustomControllerInputs();
    return customControllerButton2;
}

bool OperatorInterface::customControllerButton3Pressed() {
    updateCustomControllerInputs();
    return customControllerButton3;
}

bool OperatorInterface::customControllerButton4Pressed() {
    updateCustomControllerInputs();
    return customControllerButton4;
}

bool OperatorInterface::customControllerSwitch1On() {
    updateCustomControllerInputs();
    return customControllerSwitch1;
}

bool OperatorInterface::customControllerSwitch2On() {
    updateCustomControllerInputs();
    return customControllerSwitch2;
}

}  // namespace src::Control

#endif