/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "turret_setpoint_command.hpp"

#include "aruwlib/drivers.hpp"

using namespace aruwlib::control::turret;

namespace aruwlib::control::turret::commands
{
TurretSetpointCommand::TurretSetpointCommand(
    aruwlib::Drivers *drivers,
    TurretSubsystemInterface *turret,
    const float yawInputScalar,
    const float pitchInputScalar)
    : drivers(drivers),
      turret(turret),
      yawInputScalar(yawInputScalar),
      pitchInputScalar(pitchInputScalar)
{
    addSubsystemRequirement(turret);
}

void TurretSetpointCommand::execute()
{
    turret->setPitchSetpoint(
        turret->getPitchSetpoint() +
        pitchInputScalar * drivers->controlOperatorInterface.getTurretPitchInput());

    turret->setYawSetpoint(
        turret->getYawSetpoint() +
        yawInputScalar * drivers->controlOperatorInterface.getTurretYawInput());
}

}  // namespace aruwlib::control::turret::commands
