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

#include "command_scheduler.hpp"

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/assert.hpp"

#include "command.hpp"
#include "subsystem.hpp"

using namespace tap::errors;

namespace tap
{
namespace control
{
bool CommandScheduler::masterSchedulerExists = false;
Subsystem *CommandScheduler::globalSubsystemRegistrar[CommandScheduler::MAX_SUBSYSTEM_COUNT];
Command *CommandScheduler::globalCommandRegistrar[CommandScheduler::MAX_COMMAND_COUNT];
int CommandScheduler::maxSubsystemIndex = 0;
int CommandScheduler::maxCommandIndex = 0;
SafeDisconnectFunction CommandScheduler::defaultSafeDisconnectFunction;

int CommandScheduler::constructCommand(Command *command)
{
    modm_assert(command != nullptr, "CommandScheduer::constructCommand", "called with nullptr cmd");

    modm_assert(
        maxCommandIndex < MAX_COMMAND_COUNT,
        "CommandScheduler::constructCommand",
        "Too many commands constructed!");

    // Loop through the globalCommandRegistrar, find the lowest nullptr index
    for (int i = 0; i < MAX_COMMAND_COUNT; i++)
    {
        if (globalCommandRegistrar[i] == nullptr)
        {
            // Update max index if need be
            maxCommandIndex = std::max(maxCommandIndex, i + 1);
            globalCommandRegistrar[i] = command;
            return i;
        }
    }

    // Will never happen
    return -1;
}

int CommandScheduler::constructSubsystem(Subsystem *subsystem)
{
    modm_assert(
        subsystem != nullptr,
        "CommandScheduer::constructSubsystem",
        "called with nullptr sub");

    modm_assert(
        maxSubsystemIndex < MAX_SUBSYSTEM_COUNT,
        "CommandScheduler::constructSubsystem",
        "Too many subsystems constructed!");

    // Loop through the globalSubsystemRegistrar, find the lowest nullptr index
    for (int i = 0; i < MAX_SUBSYSTEM_COUNT; i++)
    {
        if (globalSubsystemRegistrar[i] == nullptr)
        {
            // Update max index if need be
            maxSubsystemIndex = std::max(maxSubsystemIndex, i + 1);
            globalSubsystemRegistrar[i] = subsystem;
            return i;
        }
    }

    // This will never happen
    return -1;
}

void CommandScheduler::destructCommand(Command *command)
{
    modm_assert(command != nullptr, "CommandScheduer::destructCommand", "called with nullptr cmd");

    auto cmdId = command->getGlobalIdentifier();

    modm_assert(
        cmdId >= 0 && cmdId < MAX_COMMAND_COUNT,
        "CommandScheduler::destructCommand",
        "Trying to destruct command with invalid identifier");

    globalCommandRegistrar[cmdId] = nullptr;
    if (cmdId == maxCommandIndex - 1)
    {
        maxCommandIndex--;
    }
}

void CommandScheduler::destructSubsystem(Subsystem *subsystem)
{
    modm_assert(
        subsystem != nullptr,
        "CommandScheduer::destructSubsystem",
        "called with nullptr sub");

    auto subId = subsystem->getGlobalIdentifier();
    modm_assert(
        subId >= 0 && subId < MAX_SUBSYSTEM_COUNT,
        "CommandScheduler::destructSubsystem",
        "Trying to destruct subsystem with invalid identifier");

    globalSubsystemRegistrar[subId] = nullptr;
    if (subId == maxSubsystemIndex - 1)
    {
        maxSubsystemIndex--;
    }
}

CommandScheduler::CommandScheduler(
    Drivers *drivers,
    bool masterScheduler,
    SafeDisconnectFunction *safeDisconnectFunction)
    : drivers(drivers),
      safeDisconnectFunction(safeDisconnectFunction)
{
    if (masterScheduler && masterSchedulerExists)
    {
        RAISE_ERROR(drivers, "master scheduler already exists");
    }
    else
    {
        isMasterScheduler = masterScheduler;
        if (masterScheduler)
        {
            masterSchedulerExists = true;
        }
    }
}

CommandScheduler::~CommandScheduler()
{
    if (isMasterScheduler)
    {
        masterSchedulerExists = false;
    }
}

void CommandScheduler::run()
{
#ifndef PLATFORM_HOSTED
    uint32_t runStart = arch::clock::getTimeMicroseconds();
#endif

    if (runningHardwareTests)
    {
        // Call runHardwareTests on all subsystems in the registeredSubsystemBitmap
        // if a hardware test is not already complete
        for (auto it = subMapBegin(); it != subMapEnd(); it++)
        {
            Subsystem *sub = *it;
            if (!sub->isHardwareTestComplete())
            {
                sub->runHardwareTests();
            }
            sub->refresh();
        }
        return;
    }

    if (safeDisconnected())
    {
        // End all commands running. They were interrupted by the remote disconnecting.
        for (auto it = cmdMapBegin(); it != cmdMapEnd(); it++)
        {
            removeCommand(*it, true);
        }
    }
    else
    {
        // Execute commands in the addedCommandBitmap, remove any that are finished
        for (auto it = cmdMapBegin(); it != cmdMapEnd(); it++)
        {
            (*it)->execute();
            if ((*it)->isFinished())
            {
                removeCommand(*it, false);
            }
        }
    }

    // Only refresh subsystems if this is the master scheduler
    if (isMasterScheduler)
    {
        // Refresh subsystems in the registeredSubsystemBitmap
        for (auto it = subMapBegin(); it != subMapEnd(); it++)
        {
            (*it)->refresh();

            Command *defaultCmd;
            // If the remote is connected given the scheduler is in safe disconnect mode and
            // the current subsystem does not have an associated command and the current
            // subsystem has a default command, add it
            if (!safeDisconnected() &&
                !(subsystemsAssociatedWithCommandBitmap &
                  (LSB_ONE_HOT_SUBSYSTEM_BITMAP << (*it)->getGlobalIdentifier())) &&
                ((defaultCmd = (*it)->getDefaultCommand()) != nullptr))
            {
                addCommand(defaultCmd);
            }
        }
    }

#ifndef PLATFORM_HOSTED
    // make sure we are not going over tolerable runtime, otherwise something is really
    // wrong with the code
    if (arch::clock::getTimeMicroseconds() - runStart > MAX_ALLOWABLE_SCHEDULER_RUNTIME)
    {
        // shouldn't take more than MAX_ALLOWABLE_SCHEDULER_RUNTIME microseconds
        // to complete all this stuff, if it does something
        // is seriously wrong (i.e. you are adding subsystems unchecked or the scheduler
        // itself is broken).
        RAISE_ERROR(drivers, "scheduler took longer than MAX_ALLOWABLE_SCHEDULER_RUNTIME");
    }
#endif
}

void CommandScheduler::addCommand(Command *commandToAdd)
{
    if (safeDisconnected())
    {
        return;
    }
    else if (runningHardwareTests)
    {
        RAISE_ERROR(drivers, "attempting to add command while running tests");
        return;
    }
    else if (commandToAdd == nullptr)
    {
        RAISE_ERROR(drivers, "attempting to add nullptr command");
        return;
    }
    else if (!commandToAdd->isReady())
    {
        // Do not add command if it is not ready to be scheduled.
        return;
    }

    subsystem_scheduler_bitmap_t requirementsBitwise = commandToAdd->getRequirementsBitwise();

    // Check to see if all the requirements are in the subsytemToCommandMap
    if ((requirementsBitwise & registeredSubsystemBitmap) != requirementsBitwise ||
        requirementsBitwise == static_cast<subsystem_scheduler_bitmap_t>(0))
    {
        // the command you are trying to add has a subsystem that is not in the
        // scheduler, so you cannot add it (will lead to undefined control behavior)
        RAISE_ERROR(drivers, "Attempting to add a command without subsystem in the scheduler");
        return;
    }

    // End all commands running that used the subsystem requirements. They were interrupted.
    for (auto it = cmdMapBegin(); it != cmdMapEnd(); it++)
    {
        // Does this command's requierments intersect the new command?
        if (((*it)->getRequirementsBitwise() & requirementsBitwise) !=
            static_cast<subsystem_scheduler_bitmap_t>(0))
        {
            removeCommand(*it, true);
        }
    }

    // Add the subsystem requirements to the subsystems associated with command bitmap
    subsystemsAssociatedWithCommandBitmap |= requirementsBitwise;
    commandToAdd->initialize();
    // Add the command to the command bitmap
    addedCommandBitmap |= LSB_ONE_HOT_COMMAND_BITMAP << commandToAdd->getGlobalIdentifier();
}

bool CommandScheduler::isCommandScheduled(const Command *command) const
{
    return command != nullptr &&
           (addedCommandBitmap & (LSB_ONE_HOT_COMMAND_BITMAP << command->getGlobalIdentifier()));
}

void CommandScheduler::removeCommand(Command *command, bool interrupted)
{
    if (command == nullptr)
    {
        RAISE_ERROR(drivers, "trying to remove nullptr command");
        return;
    }
    else if (!isCommandScheduled(command))
    {
        return;
    }

    command->end(interrupted);

    // Remove all subsystem requirements from the subsystem associated with command bitmap
    subsystemsAssociatedWithCommandBitmap &= ~command->getRequirementsBitwise();

    // Remove the command from the command bitmap
    addedCommandBitmap &= ~(LSB_ONE_HOT_COMMAND_BITMAP << command->getGlobalIdentifier());
}

void CommandScheduler::setSafeDisconnectFunction(SafeDisconnectFunction *func)
{
    this->safeDisconnectFunction = func;
}

bool CommandScheduler::safeDisconnected() { return this->safeDisconnectFunction->operator()(); }

void CommandScheduler::registerSubsystem(Subsystem *subsystem)
{
    if (subsystem == nullptr)
    {
        RAISE_ERROR(drivers, "trying to register nullptr subsystem");
    }
    else if (isSubsystemRegistered(subsystem))
    {
        RAISE_ERROR(drivers, "subsystem is already added");
    }
    else
    {
        // Add the subsystem to the registered subsystem bitmap
        registeredSubsystemBitmap |=
            (LSB_ONE_HOT_SUBSYSTEM_BITMAP << subsystem->getGlobalIdentifier());
    }
}

bool CommandScheduler::isSubsystemRegistered(const Subsystem *subsystem) const
{
    return subsystem != nullptr &&
           ((LSB_ONE_HOT_SUBSYSTEM_BITMAP << subsystem->getGlobalIdentifier()) &
            registeredSubsystemBitmap);
}

void CommandScheduler::startHardwareTests()
{
    // End all commands that are currently being run
    runningHardwareTests = true;
    for (auto it = cmdMapBegin(); it != cmdMapEnd(); it++)
    {
        (*it)->end(true);
    }

    // Clear command bitmap (now all commands are removed)
    addedCommandBitmap = 0;
    // No more subsystems associated with commands, so clear this bitmap as well
    subsystemsAssociatedWithCommandBitmap = 0;

    // Start hardware tests
    for (auto it = subMapBegin(); it != subMapEnd(); it++)
    {
        (*it)->setHardwareTestsIncomplete();
    }
}

void CommandScheduler::stopHardwareTests()
{
    // Stop all hardware tests
    for (auto it = subMapBegin(); it != subMapEnd(); it++)
    {
        (*it)->setHardwareTestsComplete();
    }
    runningHardwareTests = false;
}

int CommandScheduler::subsystemListSize() const
{
    int size = 0;
    for (int i = 0; i < maxSubsystemIndex; i++)
    {
        if (registeredSubsystemBitmap & (LSB_ONE_HOT_SUBSYSTEM_BITMAP << i))
        {
            size++;
        }
    }
    return size;
}

int CommandScheduler::commandListSize() const
{
    int size = 0;
    for (int i = 0; i < maxCommandIndex; i++)
    {
        if (addedCommandBitmap & (LSB_ONE_HOT_COMMAND_BITMAP << i))
        {
            size++;
        }
    }
    return size;
}

CommandScheduler::CommandIterator CommandScheduler::cmdMapBegin()
{
    return CommandIterator(this, 0);
}

CommandScheduler::CommandIterator CommandScheduler::cmdMapEnd()
{
    return CommandIterator(this, INVALID_ITER_INDEX);
}

CommandScheduler::SubsystemIterator CommandScheduler::subMapBegin()
{
    return SubsystemIterator(this, 0);
}

CommandScheduler::SubsystemIterator CommandScheduler::subMapEnd()
{
    return SubsystemIterator(this, INVALID_ITER_INDEX);
}

CommandScheduler::CommandIterator::CommandIterator(CommandScheduler *scheduler, int i)
    : scheduler(scheduler),
      currIndex(i)
{
    // Set to invalid iterator if the index passed in is invalid
    if (i < 0 || i >= maxCommandIndex)
    {
        currIndex = INVALID_ITER_INDEX;
    }
    else
    {
        // If the curr index is pointing somewhere in the valid range of commands but the command
        // associated with the index is not in the current added commands bitmap, increment the
        // iterator to find the next valid index
        if (!(scheduler->addedCommandBitmap & (LSB_ONE_HOT_COMMAND_BITMAP << currIndex)))
        {
            (*this)++;
        }
    }
}

CommandScheduler::CommandIterator::pointer CommandScheduler::CommandIterator::operator*()
{
    return currIndex == INVALID_ITER_INDEX ? nullptr : globalCommandRegistrar[currIndex];
}

CommandScheduler::CommandIterator &CommandScheduler::CommandIterator::operator++()
{
    if (currIndex == INVALID_ITER_INDEX)
    {
        return *this;
    }

    currIndex++;
    while (currIndex < maxCommandIndex)
    {
        // Is the current index in the bitmap of added commands?
        if (scheduler->addedCommandBitmap & (LSB_ONE_HOT_COMMAND_BITMAP << currIndex))
        {
            // We found the correct index
            return *this;
        }
        currIndex++;
    }
    // We didn't find the correct index, set currindex to invalid index
    currIndex = INVALID_ITER_INDEX;
    return *this;
}

CommandScheduler::CommandIterator CommandScheduler::CommandIterator::operator++(int)
{
    CommandIterator tmp = *this;
    ++(*this);
    return tmp;
}

bool operator==(
    const CommandScheduler::CommandIterator &a,
    const CommandScheduler::CommandIterator &b)
{
    return a.currIndex == b.currIndex && a.scheduler == b.scheduler;
};

bool operator!=(
    const CommandScheduler::CommandIterator &a,
    const CommandScheduler::CommandIterator &b)
{
    return !(a == b);
};

CommandScheduler::SubsystemIterator::SubsystemIterator(CommandScheduler *scheduler, int i)
    : scheduler(scheduler),
      currIndex(i)
{
    // Set to invalid iterator if the index passed in is invalid
    if (currIndex < 0 || currIndex >= maxSubsystemIndex)
    {
        currIndex = INVALID_ITER_INDEX;
    }
    else
    {
        // If the curr index is pointing somewhere in the valid range of subsystems but the
        // subsystem associated with the index is not in the current registered subsystem bitmap,
        // increment the iterator to find the next valid index
        if (!(scheduler->registeredSubsystemBitmap & (LSB_ONE_HOT_SUBSYSTEM_BITMAP << currIndex)))
        {
            (*this)++;
        }
    }
}

CommandScheduler::SubsystemIterator::pointer CommandScheduler::SubsystemIterator::operator*()
{
    return currIndex == INVALID_ITER_INDEX ? nullptr : globalSubsystemRegistrar[currIndex];
}

CommandScheduler::SubsystemIterator &CommandScheduler::SubsystemIterator::operator++()
{
    if (currIndex == INVALID_ITER_INDEX)
    {
        return *this;
    }

    currIndex++;
    while (currIndex < maxSubsystemIndex)
    {
        // Is the current index in the bitmap of added commands?
        if (scheduler->registeredSubsystemBitmap & (LSB_ONE_HOT_SUBSYSTEM_BITMAP << currIndex))
        {
            // We found the correct index
            return *this;
        }
        currIndex++;
    }
    // We didn't find the correct index, set currindex to invalid index
    currIndex = INVALID_ITER_INDEX;
    return *this;
}

CommandScheduler::SubsystemIterator CommandScheduler::SubsystemIterator::operator++(int)
{
    CommandScheduler::SubsystemIterator tmp = *this;
    ++(*this);
    return tmp;
}

bool operator==(
    const CommandScheduler::SubsystemIterator &a,
    const CommandScheduler::SubsystemIterator &b)
{
    return a.currIndex == b.currIndex && a.scheduler == b.scheduler;
}

bool operator!=(
    const CommandScheduler::SubsystemIterator &a,
    const CommandScheduler::SubsystemIterator &b)
{
    return !(a == b);
}
}  // namespace control
}  // namespace tap
