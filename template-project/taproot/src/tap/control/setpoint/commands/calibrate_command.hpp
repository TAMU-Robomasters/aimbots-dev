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

#ifndef AGITATOR_CALIBRATE_COMMAND_HPP_
#define AGITATOR_CALIBRATE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
// Forward declarations
class SetpointSubsystem;

/**
 * Default command that can be used to calibrate the agitator (spam calls
 * `calibrateHere`). By default, the agitator will keep calling `calibrateHere`
 * until the agitator is connected, however this command is for the following:
 *  - A placeholder command initially.
 *  - Allows you to recalibrate an agitator that has already been calibrated if necessary.
 */
class CalibrateCommand : public tap::control::Command
{
public:
    /**
     * @param[in] agitator The subsystem this command is dependent upon.
     */
    explicit CalibrateCommand(
        tap::control::setpoint::SetpointSubsystem* angularPositionalSubsystem);

    const char* getName() const override { return "agitator calibrate"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    tap::control::setpoint::SetpointSubsystem* setpointSubsystem;
};  // class CalibrateCommand

}  // namespace setpoint

}  // namespace control

}  // namespace tap

#endif  // AGITATOR_CALIBRATE_COMMAND_HPP_
