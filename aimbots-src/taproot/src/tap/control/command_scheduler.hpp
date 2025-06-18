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

#ifndef TAPROOT_COMMAND_SCHEDULER_HPP_
#define TAPROOT_COMMAND_SCHEDULER_HPP_

#include <iterator>

#include "tap/util_macros.hpp"

#include "command_scheduler_types.hpp"

namespace tap
{
class Drivers;
namespace control
{
class Command;
class Subsystem;

/**
 * Abstract base class for a functor that defines how a robot is considered
 * "disconnected" in the CommandScheduler. When the functor returns true, the
 * robot is then considered disconnected and will end all Commands in the
 * CommandScheduler and disallow new Commands from being added.
 *
 * By default, the SafeDisconnectFunction will always return false, i.e.,
 * allow the CommandScheduler to continue running Commands in a
 * "disconnected" state.
 */
class SafeDisconnectFunction
{
public:
    SafeDisconnectFunction(){};
    virtual bool operator()() { return false; }
};

/**
 * Class for handling all the commands you would like to currently run.
 * Interfaces with the Subsystem and Command classes to provide a means
 * of safely scheduling multiple Commands and Subsystems. Checks are
 * provided while scheduling such that multiple commands that require
 * the same subsystem cannot run at the same time. Suppose for example
 * that you have a Command that moves a mechanical arm Subsystem to some
 * position and another Command that moves the same arm to a different
 * position. Obvious issues arise if one attempts to tell the Subsystem
 * to do two things at once. Using this class will disallow these two
 * Commands from being executed at the same time.
 *
 * This class contains a map of Subsystems -> Commands. The Subsystems
 * will be refreshed each time the CommandScheduler is ran. If there
 * are commands associated with the Subsystem, the CommandScheduler will
 * execute these commands as well. Additional less important features
 * are explained in more detail in the function definitions.
 *
 * The goal of this class is for the user to interace directly as
 * little as possible. Aside from calling `run` each time to update the
 * scheduler, the user should be interacting with the Command,
 * ComprisedCommand, Subsystem, and CommandMapper classes to add
 * and remove commands from the scheduler.
 *
 * The main use case will be to be refreshing all the main subsystems running
 * on the robot. You should access the main scheduler via the global `tap::Drivers *`
 * instance, which should have an instance of a `CommandScheduler` called  `commandScheduler`.
 * The below example code registers a subsystem (`sub`) and adds a command (`cmd`) to
 * the scheduler. Then the scheduler is run over and over, in a loop.
 *
 * ```
 * // A class that has Subsystem as a base class.
 * CoolSubsystem sub;
 * // A class that has Command as a base class that requires
 * // the subsystem above. In the constructor of the ControlCoolCommand,
 * // you must call `addSubsystemRequirement(sub)`, where `sub` is the
 * // `CoolSubsystem` defined above.
 * ControlCoolCommand cmd(&sub);
 *
 * drivers->commandScheduler.registerSubsystem(&sub);
 * drivers->commandScheduler.addCommand(&cmd);
 *
 * while (1)
 * {
 *     // The subsystem will refresh forever and the command will execute until it
 *     // is finished.
 *     drivers->commandScheduler.run();
 * }
 * ```
 *
 * The second use case for a CommandScheduler is in the ComprisedCommand class.
 * Here, you utilize the CommandScheduler to coordinate multiple commands inside a
 * single command. The usage is exactly the same as using the main CommandScheduler.
 */
class CommandScheduler
{
public:
    CommandScheduler(
        Drivers* drivers,
        bool masterScheduler = false,
        SafeDisconnectFunction* safeDisconnectFunction =
            &CommandScheduler::defaultSafeDisconnectFunction);
    DISALLOW_COPY_AND_ASSIGN(CommandScheduler)
    mockable ~CommandScheduler();

    /**
     * Calls the `refresh()` function for all Subsystems and the associated
     * `execute()` function for all Commands. A Subsystem is guarenteed to
     * be refreshed no more than one time each time the mainScheduler's run
     * function is called. The same goes for a Command. This includes even if
     * multiple CommandSchedulers are running in ComprisedCommands that have
     * shared Subsystems.
     *
     * If any Subsystem that is in the scheduler does not have a Command
     * controlling it but does have a default command (via the Subsystem's
     * `getDefaultCommand()`), the default command is added to the scheduler.
     *
     * If any Command is finished after execution, the Command is removed from
     * the scheduler. The Command's `end()` function is called, passing in
     * `isInterrupted = false`.
     *
     * @note checks the run time of the scheduler. An error is added to the
     *      error handler if the time is greater than `MAX_ALLOWABLE_SCHEDULER_RUNTIME`
     *      (in microseconds).
     */
    mockable void run();

    /**
     * Attempts to add a Command to the scheduler. There are a number of ways this
     * function can fail. If failure does occur, an error will be added to the
     * error handler.
     *
     * These are the following reasons why adding a Command fails:
     * - The commandToAdd is `nullptr`
     * - The commandToAdd has no Subsystem requirements.
     * - The commandToAdd has Subsystems not in the CommandScheduler.
     *
     * If a Command is successfully added to the CommandScheduler, any Subsystems
     * that the commandToAdd requires that have Commands running will be ended
     * (and the interrupted flag for that Command set to `true`).
     *
     * If a Command is successfully added, the Command's `initialize()` function will
     * be called.
     *
     * @note If the `commandToAdd` was already scheduled, it will be interrupted (its `end()`
     *      will be called) and then the command will be rescheduled.
     *
     * @param[in] commandToAdd the Command to be added to the scheduler.
     */
    mockable void addCommand(Command* commandToAdd);

    /**
     * Removes the given Command completely from the CommandScheduler. This
     * means removing all instances of the command pointer from the Subsystem ->
     * Command map (since a single Subsystem can map to multiple Commands).
     *
     * @param[in] command the Command to remove. Must not be `nullptr`. If the
     *      Command is not in the scheduler, nothing is removed.
     * @param[in] interrupted an argument passed in to the Command's `end()`
     *      function when removing the desired Command.
     */
    mockable void removeCommand(Command* command, bool interrupted);

    /**
     * @return `true` if the CommandScheduler contains the requrested Command.
     *      `false` otherwise.
     */
    mockable bool isCommandScheduled(const Command* command) const;

    /**
     * Adds the given Subsystem to the CommandScheduler.  The subsystem is
     * added with the currently scheduled Command as `nullptr`.
     *
     * @param[in] subsystem the Subsystem to add. Must be not `nullptr` and not
     *      registered already (check via `isSubsystemRegistered()`), otherwise
     *      an error is added to the error handler.
     */
    mockable void registerSubsystem(Subsystem* subsystem);

    /**
     * @brief Set the SafeDisconnectFunction to the given function.
     *
     * @param[in] func the function that the CommandScheduler will use to
     *      determine what constitutes a "disconnected" state.
     */
    mockable void setSafeDisconnectFunction(SafeDisconnectFunction* func);

    /**
     * @param[in] subsystem the subsystem to check
     * @return `true` if the Subsystem is already scheduled, `false` otherwise.
     */
    mockable bool isSubsystemRegistered(const Subsystem* subsystem) const;

    mockable void startHardwareTests();
    mockable void stopHardwareTests();

    /**
     * @return The number of subsystems registered with the scheduler.
     */
    mockable int subsystemListSize() const;

    /**
     * @return The number of commands added in the scheduler.
     */
    mockable int commandListSize() const;

    /**
     * Iterator used for looking through the commands added to the scheduler
     */
    struct CommandIterator
    {
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Command;
        using pointer = Command*;
        using reference = Command&;

        CommandIterator(CommandScheduler* scheduler, int i);

        pointer operator*();

        // Prefix increment
        CommandIterator& operator++();

        // Postfix increment
        CommandIterator operator++(int);

        friend bool operator==(const CommandIterator& a, const CommandIterator& b);
        friend bool operator!=(const CommandIterator& a, const CommandIterator& b);

    private:
        CommandScheduler* scheduler;
        int currIndex;
    };

    /**
     * Iterator used for looking through the subsystems registered in the scheduler
     */
    struct SubsystemIterator
    {
        using iterator_category = std::forward_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using value_type = Subsystem;
        using pointer = Subsystem*;
        using reference = Subsystem&;

        SubsystemIterator(CommandScheduler* scheduler, int i);

        pointer operator*();

        // Prefix increment
        SubsystemIterator& operator++();

        // Postfix increment
        SubsystemIterator operator++(int);

        friend bool operator==(const SubsystemIterator& a, const SubsystemIterator& b);
        friend bool operator!=(const SubsystemIterator& a, const SubsystemIterator& b);

    private:
        CommandScheduler* scheduler;
        int currIndex;
    };

    mockable CommandIterator cmdMapBegin();
    mockable CommandIterator cmdMapEnd();
    mockable SubsystemIterator subMapBegin();
    mockable SubsystemIterator subMapEnd();

    mockable subsystem_scheduler_bitmap_t getRegisteredSubsystemBitmap() const
    {
        return registeredSubsystemBitmap;
    }
    mockable command_scheduler_bitmap_t getAddedCommandBitmap() const { return addedCommandBitmap; }

    static int constructCommand(Command* command);
    static int constructSubsystem(Subsystem* subsystem);
    static void destructCommand(Command* command);
    static void destructSubsystem(Subsystem* subsystem);

private:
    /// Maximum time before we start erroring, in microseconds.
    static constexpr float MAX_ALLOWABLE_SCHEDULER_RUNTIME = 100;
    static constexpr int MAX_SUBSYSTEM_COUNT = sizeof(subsystem_scheduler_bitmap_t) * 8;
    static constexpr int MAX_COMMAND_COUNT = sizeof(command_scheduler_bitmap_t) * 8;
    static constexpr subsystem_scheduler_bitmap_t LSB_ONE_HOT_SUBSYSTEM_BITMAP = 1;
    static constexpr command_scheduler_bitmap_t LSB_ONE_HOT_COMMAND_BITMAP = 1;
    static constexpr int INVALID_ITER_INDEX = -1;

    /**
     * The smallest index such that all indices in the globalSubsystemRegistrar >= to them are
     * nullptr.
     *
     * To loop through the globalCommandRegistrar, use this value as the max value, i.e.
     * for (int i = 0; i < maxSubsystemIndex; i++) {...}
     */
    static int maxSubsystemIndex;

    /**
     * A global array of all constructed subsystems. When a subsystem is constructed it is given an
     * index in the registrar. When a subsystem is destructed it is removed from the registrar.
     */
    static Subsystem* globalSubsystemRegistrar[MAX_SUBSYSTEM_COUNT];

    /**
     * The smallest index such that all indices in the globalCommandRegistrar >= to them are
     * nullptr.
     */
    static int maxCommandIndex;

    /**
     * A global array of all constructed commands. When a command is constructed it is given an
     * index in the registrar. When a command is destructed it is removed from the registrar.
     */
    static Command* globalCommandRegistrar[MAX_COMMAND_COUNT];

    /**
     * A global flag indicating whether or not a "master" scheduler has been constructed.
     */
    static bool masterSchedulerExists;

    /**
     * Returns true if the remote is disconnected and the safeDisconnectMode flag is
     * enabled.
     */
    bool safeDisconnected();

    Drivers* drivers;

    /**
     * A global SafeDisconnectFunction used by CommandScheduler by default.
     */
    static SafeDisconnectFunction defaultSafeDisconnectFunction;

    /**
     * The SafeDisconnectFunction used by the CommandScheduler to determine
     * the "disconnected" state.
     */
    SafeDisconnectFunction* safeDisconnectFunction;

    /**
     * Each bit in the bitmap represents a unique subsystem that has been constructed
     * in the codebase. If a subsystem is registered, the associated bit in this bitmap
     * will be set to 1.
     */
    subsystem_scheduler_bitmap_t registeredSubsystemBitmap = 0;

    /**
     * Each bit in the bitmap corresponds to an index into the subsystem registrar. If a
     * bit is set, it means that the subsystem in the registrar has a command associated
     * in in this scheduler.
     */
    subsystem_scheduler_bitmap_t subsystemsAssociatedWithCommandBitmap = 0;

    /**
     * If a command has been added and is running, the associated bit in this bitmap will
     * be set to 1.
     */
    command_scheduler_bitmap_t addedCommandBitmap = 0;

    bool isMasterScheduler = false;

    bool runningHardwareTests = false;
};  // class CommandScheduler

}  // namespace control

}  // namespace tap

#endif  // TAPROOT_COMMAND_SCHEDULER_HPP_
