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

#include "scheduler_terminal_handler.hpp"

#include <algorithm>

#include "tap/algorithms/strtok.hpp"
#include "tap/drivers.hpp"

#include "command.hpp"
#include "subsystem.hpp"

namespace tap
{
namespace control
{
constexpr char SchedulerTerminalHandler::HEADER[];
constexpr char SchedulerTerminalHandler::USAGE[];

SchedulerTerminalHandler::SchedulerTerminalHandler(Drivers* driver) : drivers(driver) {}

void SchedulerTerminalHandler::init() { drivers->terminalSerial.addHeader(HEADER, this); }

void SchedulerTerminalHandler::terminalSerialStreamCallback(modm::IOStream& outputStream)
{
    printInfo(outputStream);
}

bool SchedulerTerminalHandler::terminalSerialCallback(
    char* inputLine,
    modm::IOStream& outputStream,
    bool streamingEnabled)
{
    char* arg = strtokR(inputLine, communication::serial::TerminalSerial::DELIMITERS, &inputLine);

    if (arg != nullptr && strcmp(arg, "allsubcmd") == 0)
    {
        printInfo(outputStream);
        return true;
    }
    else
    {
        outputStream << USAGE;
        return (arg != nullptr) && !streamingEnabled && (strcmp(arg, "-H") == 0);
    }
}

void SchedulerTerminalHandler::printInfo(modm::IOStream& outputStream)
{
    outputStream << "Subsystems:" << modm::endl;
    std::for_each(
        drivers->commandScheduler.subMapBegin(),
        drivers->commandScheduler.subMapEnd(),
        [&](Subsystem* sub) { outputStream << " " << sub->getName() << modm::endl; });

    outputStream << "Commands:" << modm::endl;
    std::for_each(
        drivers->commandScheduler.cmdMapBegin(),
        drivers->commandScheduler.cmdMapEnd(),
        [&](Command* cmd) { outputStream << " " << cmd->getName() << modm::endl; });
}
}  // namespace control

}  // namespace tap
