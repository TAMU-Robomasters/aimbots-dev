/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TERMINAL_SERIAL_HPP_
#define TERMINAL_SERIAL_HPP_

#include <cstring>
#include <map>

#ifdef PLATFORM_HOSTED
#include "hosted_terminal_device.hpp"
#else
#include "uart_terminal_device.hpp"
#endif

#include "tap/architecture/periodic_timer.hpp"
#include "tap/board/board.hpp"
#include "tap/communication/serial/uart.hpp"
#include "tap/util_macros.hpp"

#include "modm/io.hpp"

namespace tap
{
class Drivers;
namespace communication
{
namespace serial
{
/**
 * If you would like to interact with the terminal, extend this class and implement
 * the callback.
 */
class ITerminalSerialCallback
{
public:
    /**
     * @param[in] inputLine The user input to be processed.
     * @param[out] outputStream The stream to write information to.
     * @param[in] streamingEnabled Set to `true` when the streaming is initially enabled.
     *      Subsequent interactions with the callback handler will be via
     *      terminalSerialStreamCallback until streaming has been disabled.
     * @return `true` if the inputLine was valid and was parsed correctly, `false` otherwise.
     */
    virtual bool terminalSerialCallback(
        char *inputLine,
        modm::IOStream &outputStream,
        bool streamingEnabled) = 0;

    /**
     * Called repeatedly by the TerminalSerial when in streaming mode.
     */
    virtual void terminalSerialStreamCallback(modm::IOStream &outputStream) = 0;
};  // class ITerminalSerialCallback

/**
 * Handles incoming requests from the "terminal". The "terminal" is either
 * a uart line connected to a computer with a serial connection open or
 * when runing on the simulator, stdin/stdout.
 *
 * To add a handler to the terminal, extend the ITerminalSerialCallback
 * and add it to an instance of the TerminalSerial class via `addHeader`.
 * Whenever the header is received on the terminal line, the contents of
 * the message (minus the header and any flags specified for the TerminalSerial)
 * will be passed on to the registered callback via terminalSerialCallback.
 *
 * @note If the "-S" flag is specified directly after the header, the terminal
 *      enters streaming mode. In this mode, so long as terminalSerialCallback
 *      returns true when streaming mode is initially enabled, the
 *      ITerminalSerialCallback's `terminalSerialStreamCallback` function will
 *      be called repeatedly until the user enters any new key presses. If you
 *      design an ITerminalSerialCallback that does not need/handle streaming,
 *      simply check the argument in `terminalSerialCallback` called `streamingEnabled`
 *      `return false` and write to the `outputStream` to notify the user that
 *      streaming is not enabled. An example of where you would use streaming
 *      mode is if you would like to have a mode that prints out motor information
 *      constantly without having to retype a command into the terminal.
 */
class TerminalSerial
{
public:
    static constexpr char DELIMITERS[] = " \t";

    explicit TerminalSerial(Drivers *drivers);

    DISALLOW_COPY_AND_ASSIGN(TerminalSerial);

    virtual ~TerminalSerial() = default;

    mockable void initialize();

    mockable void update();

    mockable void addHeader(const char *header, ITerminalSerialCallback *callback);

private:
    static constexpr int MAX_LINE_LENGTH = 256;
    static constexpr int STREAMING_PERIOD = 500;

    // Use either an IO device that interacts with UART or with stdin/stdout.
#ifdef PLATFORM_HOSTED
    HostedTerminalDevice device;
#else
    UartTerminalDevice device;
#endif

    ///< Hardware abstraction of a stream that provides utilities for tx/rx.
    modm::IOStream stream;

    ///< A single line is parsed into this buffer.
    char rxBuff[MAX_LINE_LENGTH];

    ///< Index into the rxBuff.
    uint8_t currLineSize = 0;

    /**
     * Used when in streaming mode to signify the serial callback that has taken
     * control of writing to the IOStream
     */
    ITerminalSerialCallback *currStreamer;

    tap::arch::PeriodicMilliTimer streamingTimer;

    struct cmpByStringEquality
    {
        bool operator()(const char *c1, const char *c2) const { return strcmp(c1, c2) < 0; }
    };

    std::map<const char *, ITerminalSerialCallback *, cmpByStringEquality> headerCallbackMap;

    Drivers *drivers;

    bool prevCharSpace = false;

    void printUsage();
};  // class TerminalSerial
}  // namespace serial
}  // namespace communication
}  // namespace tap

#endif  // TERMINAL_SERIAL_HPP_
