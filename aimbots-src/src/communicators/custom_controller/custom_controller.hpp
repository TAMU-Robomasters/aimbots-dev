#ifndef CUSTOM_CONTROLLER_HPP_
#define CUSTOM_CONTROLLER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "tap/architecture/timeout.hpp"
#include "tap/util_macros.hpp"

namespace src
{
class Drivers;
}

namespace src::communicators::vtm_can
{
class VtmCan;
}

namespace src::communicators::custom_controller
{
/**
 * Decoded custom controller state carried inside the 30-byte payload sent by CCadapter
 *
 * Bit layout (little-endian bitstream, LSB-first within each byte):
 *   [joystick1 x:12][joystick1 y:12][joystick2 x:12][joystick2 y:12]
 *   [analog1:12][analog2:12]
 *   [btn1:1][btn2:1][btn3:1][btn4:1][sw1:1][sw2:1]
 *   [arm1:12][arm2:12][arm3:12][arm4:12][arm5:12][arm6:12]
 *
 * Signed 12-bit fields are interpreted as two's complement and then clamped
 * to [-1024, +1024] (Dr. Porter my GOAT). General analog inputs are unsigned and clamped to [0, 2048]
 */
struct CustomControllerState
{
    int16_t joystick1X = 0;
    int16_t joystick1Y = 0;
    int16_t joystick2X = 0;
    int16_t joystick2Y = 0;

    uint16_t analog1 = 0;
    uint16_t analog2 = 0;

    bool button1 = false;
    bool button2 = false;
    bool button3 = false;
    bool button4 = false;
    
    bool switch1 = false;
    bool switch2 = false;

    std::array<int16_t, 6> armJoints{{0, 0, 0, 0, 0, 0}};
};

class CustomController
{
public:
    explicit CustomController(src::Drivers* drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(CustomController)
    ~CustomController() = default;

    void setVtmCan(src::communicators::vtm_can::VtmCan* vtmCan) { this->vtmCan = vtmCan; }

    void initialize();
    void read();

    bool isConnected() const;

    const std::array<uint8_t, 30>& getData() const { return data; }
    uint8_t getByte(uint8_t index) const { return data[index]; }
    uint32_t getUpdateCounter() const { return updateCounter; }

    int16_t joystick1X() const { return state.joystick1X; }
    int16_t joystick1Y() const { return state.joystick1Y; }
    int16_t joystick2X() const { return state.joystick2X; }
    int16_t joystick2Y() const { return state.joystick2Y; }

    uint16_t analogInput1() const { return state.analog1; }
    uint16_t analogInput2() const { return state.analog2; }

    bool button1Pressed() const { return state.button1; }
    bool button2Pressed() const { return state.button2; }
    bool button3Pressed() const { return state.button3; }
    bool button4Pressed() const { return state.button4; }

    bool switch1On() const { return state.switch1; }
    bool switch2On() const { return state.switch2; }

    int16_t armJoint(std::size_t idx) const { return state.armJoints[idx]; }
    int16_t armJoint1() const { return state.armJoints[0]; }
    int16_t armJoint2() const { return state.armJoints[1]; }
    int16_t armJoint3() const { return state.armJoints[2]; }
    int16_t armJoint4() const { return state.armJoints[3]; }
    int16_t armJoint5() const { return state.armJoints[4]; }
    int16_t armJoint6() const { return state.armJoints[5]; }

    const CustomControllerState& getState() const { return state; }

private:
    static constexpr uint32_t CUSTOM_CONTROLLER_DISCONNECT_TIMEOUT_MS = 1000;

    void decodeStateFromData();

    src::Drivers* drivers;
    src::communicators::vtm_can::VtmCan* vtmCan = nullptr;

    std::array<uint8_t, 30> data{{0}};
    CustomControllerState state{};
    uint32_t updateCounter = 0;
    uint32_t lastSeenVtmCounter = 0;
    tap::arch::MilliTimeout disconnectTimeout;
};

}  // namespace src::communicators::custom_controller

#endif