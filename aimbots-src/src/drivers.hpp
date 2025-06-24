/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aimbots-src.
 *
 * aimbots-src is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aimbots-src is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aimbots-src.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#include "tap/drivers.hpp"

#include "informants/kinematics/hitTracker.hpp"
#include "informants/kinematics/kinematic_informant.hpp"
// #include "informants/ultrasonic_distance_sensor.hpp"
#include "communicators/devboard/turret_can_communicator.hpp"
#include "communicators/jetson/jetson_communicator.hpp"
#include "utils/music/jukebox_player.hpp"
#include "utils/nxp_imu/magnetometer/ist8310.hpp"
#include "utils/I2C/power_com.hpp"
#include "utils/tools/robot_specific_defines.hpp"
// graphing elsewhere lol

namespace src {
class Drivers : public tap::Drivers {
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers()
        : tap::Drivers(),
          controlOperatorInterface(this),
       //   magnetometer(),
          powerSensor(),
          cvCommunicator(this),
          kinematicInformant(this),
          hitTracker(this),
          turretCommunicator(this, CANBus::CAN_BUS1),
          musicPlayer(this, STARTUP_SONG) {}

public:
    Control::OperatorInterface controlOperatorInterface;
    //utils::Ist8310 magnetometer;
    utils::POWER_COM powerSensor;
    Informants::Vision::JetsonCommunicator cvCommunicator;
    Informants::KinematicInformant kinematicInformant;
    Informants::HitTracker hitTracker;
    Informants::TurretComms::TurretCommunicator turretCommunicator;
    utils::Jukebox::JukeboxPlayer musicPlayer;
};  // class Drivers

}  // namespace src

#endif  // DRIVERS_HPP_
