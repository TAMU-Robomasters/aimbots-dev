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

#ifndef TAPROOT_SCHEDULER_TERMINAL_HANDLER_HPP_
#define TAPROOT_SCHEDULER_TERMINAL_HANDLER_HPP_

#include "tap/communication/serial/terminal_serial.hpp"
#include "tap/util_macros.hpp"

namespace tap
{
class Drivers;
namespace control
{
class SchedulerTerminalHandler : public communication::serial::TerminalSerialCallbackInterface
{
public:
    static constexpr char HEADER[] = "scheduler";

    SchedulerTerminalHandler(Drivers* drivers);
    DISALLOW_COPY_AND_ASSIGN(SchedulerTerminalHandler);
    mockable ~SchedulerTerminalHandler() = default;

    mockable void init();

    bool terminalSerialCallback(
        char* inputLine,
        modm::IOStream& outputStream,
        bool streamingEnabled) override;

    void terminalSerialStreamCallback(modm::IOStream& outputStream) override;

private:
    Drivers* drivers;

    static constexpr char USAGE[] =
        "Usage: scheduler <target>\n"
        "  Where \"<target>\" is one of:\n"
        "    - \"-H\": displays possible commands.\n"
        "    - \"allsubcmd\" prints all running subsystems and.\n";

    void printInfo(modm::IOStream& outputStream);
};

}  // namespace control

}  // namespace tap

#endif  // TAPROOT_SCHEDULER_TERMINAL_HANDLER_HPP_
