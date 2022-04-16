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

#ifndef TAPROOT_HOSTED_TERMINAL_DEVICE_HPP_
#define TAPROOT_HOSTED_TERMINAL_DEVICE_HPP_

#ifdef PLATFORM_HOSTED

#include <mutex>

#include "tap/util_macros.hpp"

#include "modm/container/deque.hpp"
#include "modm/io/iodevice.hpp"

namespace std
{
class thread;
}

namespace tap
{
class Drivers;
}

namespace tap::communication::serial
{
/**
 * A device that interacts with stdin and stdout to be used
 * on the hosted environment. Identical to `UartTerminalDevice`
 * in functionality except that it is used when running in the
 * hosted environment.
 *
 * @see UartTerminalDevice
 */
class HostedTerminalDevice : public modm::IODevice
{
public:
    HostedTerminalDevice(Drivers *drivers);
    DISALLOW_COPY_AND_ASSIGN(HostedTerminalDevice);
    virtual ~HostedTerminalDevice();

    void initialize();

    bool read(char &c) override;

    using IODevice::write;
    void write(char c) override;

    void flush() override;

private:
    static constexpr int RX_BUFF_SIZE = 256;

    Drivers *drivers;

    std::thread *readStdinThread;

    modm::BoundedDeque<char, RX_BUFF_SIZE> rxBuff;

    std::mutex rxBuffMutex;

    void readCin();
};  // class HostedTerminalDevice
}  // namespace tap::communication::serial

#endif

#endif  // TAPROOT_HOSTED_TERMINAL_DEVICE_HPP_
