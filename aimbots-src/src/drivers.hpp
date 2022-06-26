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

#include "informants/field_relative_informant.hpp"
#include "informants/ultrasonic_distance_sensor.hpp"
#include "informants/vision/jetson_communicator.hpp"
#include "tap/drivers.hpp"
#include "utils/nxp_imu/magnetometer/ist8310.hpp"
#include "utils/nxp_imu/nxp_bmi088.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src {
class Drivers : public tap::Drivers {
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
   public:
#endif
    Drivers() : tap::Drivers(),
#ifdef TARGET_SENTRY
                railDistanceSensor(this),
#endif
                controlOperatorInterface(this),
                imu(this),
                magnetometer(),
                cvCommunicator(this),
                fieldRelativeInformant(this) {
    }

   public:
#ifdef TARGET_SENTRY
    Informants::UltrasonicDistanceSensor railDistanceSensor;
#endif
    Control::OperatorInterface controlOperatorInterface;
    ::utils::NXPBMI088 imu;
    ::utils::Ist8310 magnetometer;
    Informants::vision::JetsonCommunicator cvCommunicator;
    Informants::FieldRelativeInformant fieldRelativeInformant;
};  // class Drivers

}  // namespace src

#endif  // DRIVERS_HPP_
