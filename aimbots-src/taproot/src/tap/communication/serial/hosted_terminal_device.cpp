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

#ifdef PLATFORM_HOSTED

#include "hosted_terminal_device.hpp"

#include <iostream>
#include <thread>

namespace tap::communication::serial
{
HostedTerminalDevice::HostedTerminalDevice(Drivers *drivers)
    : drivers(drivers),
      readStdinThread(nullptr),
      rxBuff(),
      rxBuffMutex()
{
}

HostedTerminalDevice::~HostedTerminalDevice() { delete readStdinThread; }

void HostedTerminalDevice::readCin()
{
    while (true)
    {
        char c;
        ::std::cin >> ::std::noskipws >> c;
        rxBuffMutex.lock();
        rxBuff.appendOverwrite(c);
        rxBuffMutex.unlock();
    }
}

void HostedTerminalDevice::initialize()
{
    readStdinThread = new std::thread(&HostedTerminalDevice::readCin, this);
    readStdinThread->detach();
}

bool HostedTerminalDevice::read(char &c)
{
    rxBuffMutex.lock();
    if (rxBuff.getSize() > 0)
    {
        c = rxBuff.getFront();
        rxBuff.removeFront();

        rxBuffMutex.unlock();
        return true;
    }
    rxBuffMutex.unlock();
    return false;
}

void HostedTerminalDevice::write(char c) { ::std::cout << c; }

void HostedTerminalDevice::flush() { ::std::cout.flush(); }
}  // namespace tap::communication::serial

#endif
