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

#include "terminal_serial.hpp"

#include "tap/algorithms/strtok.hpp"
#include "tap/drivers.hpp"

namespace tap::communication::serial
{
constexpr char TerminalSerial::DELIMITERS[];

TerminalSerial::TerminalSerial(Drivers *drivers)
    : device(drivers),
      stream(device),
      streamingTimer(STREAMING_PERIOD),
      drivers(drivers)
{
}

void TerminalSerial::initialize() { device.initialize(); }

void TerminalSerial::update()
{
    char nextC;
    stream.get(nextC);
    if (nextC == modm::IOStream::eof)
    {
        if (currStreamer != nullptr && streamingTimer.execute())
        {
            currStreamer->terminalSerialStreamCallback(stream);
        }
        return;
    }
    if (currStreamer != nullptr)
    {
        currStreamer = nullptr;
    }
    // Some systems use '\r', some '\n' when the user hits enter
    if (nextC == '\r' || nextC == '\n')
    {
        // If the rx buff doesn't contain a string just return
        if (rxBuff[0] == '\0')
        {
            return;
        }
        rxBuff[currLineSize] = '\0';

        char *strtokSavePtr = rxBuff;
        char *headerStr = strtokR(strtokSavePtr, DELIMITERS, &strtokSavePtr);

        if (headerCallbackMap.count(headerStr) == 0)
        {
            stream << "Header \'" << headerStr << "\' not found" << modm::endl;
            printUsage();
        }
        else
        {
            constexpr char STREAMING_ID[] = "-S";
            // strlen("-S"), but strlen isn't guaranteed to be evaluated at runtime
            constexpr int STREAMING_ID_LEN = 2;
            if (strncmp(STREAMING_ID, strtokSavePtr, STREAMING_ID_LEN) == 0)
            {
                currStreamer = headerCallbackMap[headerStr];
                strtokSavePtr += STREAMING_ID_LEN;
            }

            if (!headerCallbackMap[headerStr]
                     ->terminalSerialCallback(strtokSavePtr, stream, currStreamer != nullptr))
            {
                stream << "invalid arguments" << modm::endl;
                currStreamer = nullptr;
            }
        }
        currLineSize = 0;
        rxBuff[0] = '\0';
    }
    else
    {
        if (currLineSize >= MAX_LINE_LENGTH - 1)
        {
            stream << "input line too long, throwing away data" << modm::endl;
            currLineSize = 0;
        }
        else
        {
            // Only put one space between received words
            bool space = isspace(nextC);
            if (!space || !prevCharSpace)
            {
                rxBuff[currLineSize++] = nextC;
            }
            prevCharSpace = space;
        }
    }
}

void TerminalSerial::addHeader(const char *header, TerminalSerialCallbackInterface *callback)
{
    if (callback == nullptr)
    {
        return;
    }
    headerCallbackMap[header] = callback;
}

void TerminalSerial::printUsage()
{
    stream << "Usage: <header> [-S] <args>\n"
              "  Where\n"
              "    -S Enable streaming mode\n"
              "  and <header> is one of\n";
    for (const auto &header : headerCallbackMap)
    {
        stream << "    " << header.first << modm::endl;
    }
    stream << "  and <args> is specific to <header> (query <header> -H for more help)\n";
}
}  // namespace tap::communication::serial
