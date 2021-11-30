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

#include "turret_setpoint_command.hpp"

#include "tap/drivers.hpp"

using namespace tap::control::turret;

namespace tap::control::turret::commands
{
TurretSetpointCommand::TurretSetpointCommand(
    tap::Drivers *drivers,
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

}  // namespace tap::control::turret::commands
