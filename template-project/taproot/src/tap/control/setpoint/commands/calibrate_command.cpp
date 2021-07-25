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

#include "calibrate_command.hpp"

#include "tap/control/setpoint/interfaces/setpoint_subsystem.hpp"
#include "tap/control/subsystem.hpp"

using tap::control::setpoint::SetpointSubsystem;

namespace tap
{
namespace control
{
namespace setpoint
{
CalibrateCommand::CalibrateCommand(SetpointSubsystem* setpointSubsystem)
    : setpointSubsystem(setpointSubsystem)
{
    this->addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(setpointSubsystem));
}

void CalibrateCommand::initialize() { setpointSubsystem->calibrateHere(); }

void CalibrateCommand::execute() { setpointSubsystem->calibrateHere(); }

void CalibrateCommand::end(bool) {}

bool CalibrateCommand::isFinished() const { return setpointSubsystem->isCalibrated(); }

}  // namespace setpoint

}  // namespace control

}  // namespace tap
