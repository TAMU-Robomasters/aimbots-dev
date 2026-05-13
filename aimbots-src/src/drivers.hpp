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
#include "communicators/custom_controller/custom_controller.hpp"
#include "communicators/vtm_can/vtm_can.hpp"
#include "communicators/vtm_can/can_test_listener.hpp"
#include "communicators/rev_encoder/rev_encoder_protocol.hpp"
//#include "communicators/INA260/INA260_communicator.hpp"
#include "utils/music/jukebox_player.hpp"
#include "utils/nxp_imu/magnetometer/ist8310.hpp"
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
          magnetometer(),
          customController(this),
          vtmCan(this),
          can351Listener(this),
          cvCommunicator(this),
      //  powerCommunicator(this),
      #ifdef YAW_3508
          revEncoder(this),
      #endif
          kinematicInformant(this),
          hitTracker(this),
          turretCommunicator(this, CANBus::CAN_BUS1),
          musicPlayer(this, STARTUP_SONG) {customController.setVtmCan(&vtmCan);}

public:
    Control::OperatorInterface controlOperatorInterface;
    utils::Ist8310 magnetometer;
    src::communicators::custom_controller::CustomController customController;
    src::communicators::vtm_can::VtmCan vtmCan;
    src::communicators::vtm_can::Can351Listener can351Listener;
    Informants::Vision::JetsonCommunicator cvCommunicator;
  //  Informants::INA260::INA260Communicator powerCommunicator;
  #ifdef YAW_3508
    Informants::RevEncoder revEncoder;
  #endif
    Informants::KinematicInformant kinematicInformant;
    Informants::HitTracker hitTracker;
    Informants::TurretComms::TurretCommunicator turretCommunicator;
    utils::Jukebox::JukeboxPlayer musicPlayer;
};  // class Drivers

}  // namespace src

#endif  // DRIVERS_HPP_
