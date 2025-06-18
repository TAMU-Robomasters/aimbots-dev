/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_REMOTE_HPP_
#define TAPROOT_REMOTE_HPP_

#include <cstdint>

#ifndef PLATFORM_HOSTED
#include "modm/platform.hpp"
#endif

#include "tap/util_macros.hpp"

namespace tap
{
class Drivers;
}

namespace tap::communication::serial
{
/**
 * A unique UART handler that uses timing in leu of DBUS communication (modm does not
 * support DBUS) to interact with the DR16 receiver.
 *
 * Information for implementation was translated from a user manual for the DR16 that was
 * only available in Chinese. AI-Translated version of document available here:
 * https://drive.google.com/file/d/1-ZGe4mXVhxP4IEmHccphnzKzYWJyw3C3/view?usp=sharing
 */
class Remote
{
public:
    Remote(Drivers *drivers) : drivers(drivers) {}
    DISALLOW_COPY_AND_ASSIGN(Remote)
    mockable ~Remote() = default;

    /**
     * Specifies a particular joystick.
     */
    enum class Channel
    {
        RIGHT_HORIZONTAL,
        RIGHT_VERTICAL,
        LEFT_HORIZONTAL,
        LEFT_VERTICAL
    };

    /**
     * Specifies a particular switch.
     */
    enum class Switch
    {
        LEFT_SWITCH,
        RIGHT_SWITCH
    };

    /**
     * Different switch orientations.
     */
    enum class SwitchState
    {
        UNKNOWN = 0,
        UP = 1,
        DOWN = 2,
        MID = 3,
    };

    /**
     * A list of the particular keys to interact with, in bit order.
     */
    enum class Key
    {
        W = 0,
        S,
        A,
        D,
        SHIFT,
        CTRL,
        Q,
        E,
        R,
        F,
        G,
        Z,
        X,
        C,
        V,
        B
    };

    /**
     * Enables and initializes `bound_ports::REMOTE_SERIAL_UART_PORT`.
     */
    mockable void initialize();

    /**
     * Reads/parses the current buffer and updates the current remote info state
     * and `CommandMapper` state.
     */
    mockable void read();

    /**
     * @return `true` if the remote is connected, `false` otherwise.
     * @note A timer is used to determine if the remote is disconnected, so expect a
     *      second or so of delay from disconnecting the remote to this function saying
     *      the remote is disconnected.
     */
    mockable bool isConnected() const;

    /**
     * @return The value of the given channel, between [-1, 1].
     */
    mockable float getChannel(Channel ch) const;

    /**
     * @return The state of the given switch.
     */
    mockable SwitchState getSwitch(Switch sw) const;

    /**
     * @return The current mouse x value.
     */
    mockable inline int16_t getMouseX() const { return remote.mouse.x; }

    /**
     * @return The current mouse y value.
     */
    mockable inline int16_t getMouseY() const { return remote.mouse.y; }

    /**
     * @return The current mouse z value.
     */
    mockable inline int16_t getMouseZ() const { return remote.mouse.z; }

    /**
     * @return The current mouse l value.
     */
    mockable inline bool getMouseL() const { return remote.mouse.l; }

    /**
     * @return The current mouse r value.
     */
    mockable inline bool getMouseR() const { return remote.mouse.r; }

    /**
     * @return `true` if the given `key` is pressed, `false` otherwise.
     */
    mockable inline bool keyPressed(Key key) const
    {
        return (remote.key & (1 << static_cast<uint8_t>(key))) != 0;
    }

    /**
     * @return the value of the wheel, between `[-STICK_MAX_VALUE, STICK_MAX_VALUE]`.
     */
    mockable inline int16_t getWheel() const { return remote.wheel; }

    /**
     * @return the number of times remote info has been received.
     */
    mockable uint32_t getUpdateCounter() const;

private:
    static const int REMOTE_BUF_LEN = 18;              ///< Length of the remote recieve buffer.
    static const int REMOTE_READ_TIMEOUT = 6;          ///< Timeout delay between valid packets.
    static const int REMOTE_DISCONNECT_TIMEOUT = 100;  ///< Timeout delay for remote disconnect.
    static const int REMOTE_INT_PRI = 12;              ///< Interrupt priority.
    static constexpr float STICK_MAX_VALUE = 660.0f;   ///< Max value received by one of the sticks.

    /// The current remote information
    struct RemoteInfo
    {
        uint32_t updateCounter = 0;
        int16_t rightHorizontal = 0;
        int16_t rightVertical = 0;
        int16_t leftHorizontal = 0;
        int16_t leftVertical = 0;
        SwitchState leftSwitch = SwitchState::UNKNOWN;
        SwitchState rightSwitch = SwitchState::UNKNOWN;
        /// Mouse information
        struct
        {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
            bool l = false;
            bool r = false;
        } mouse;
        uint16_t key = 0;   ///< Keyboard information
        int16_t wheel = 0;  ///< Remote wheel information
    };

    Drivers *drivers;

    RemoteInfo remote;

    /// Remote connection state.
    bool connected = false;

    /// UART recieve buffer.
    uint8_t rxBuffer[REMOTE_BUF_LEN]{0};

    /// Timestamp when last byte was read (milliseconds).
    uint32_t lastRead = 0;

    /// Current count of bytes read.
    uint8_t currentBufferIndex = 0;

    /// Parses the current rxBuffer.
    void parseBuffer();

    /// Clears the current rxBuffer.
    void clearRxBuffer();

    /// Resets the current remote info.
    void reset();
};  // class Remote

}  // namespace tap::communication::serial

#endif  // TAPROOT_REMOTE_HPP_
