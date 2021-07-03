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

#ifdef PLATFORM_HOSTED
#ifndef SIM_HANDLER_HPP_
#define SIM_HANDLER_HPP_

#include <array>

#include "aruwlib/communication/can/can_bus.hpp"
#include "aruwlib/motor/dji_motor_tx_handler.hpp"

#include "motor_sim.hpp"

namespace aruwlib
{
namespace motorsim
{
class SimHandler
{
public:
    SimHandler();
    ~SimHandler();
    /**
     * Reset output stream for SimHandler as well as all of the MotorSim objects.
     */
    static void resetMotorSims();
    /**
     * Registers a new MotorSim object for the given motor type
     * that will respond at the given position on the given CAN bus.
     *
     * Default torque loading for this function is 0 N*m.
     */
    static void registerSim(
        MotorSim::MotorType type,
        aruwlib::can::CanBus bus,
        aruwlib::motor::MotorId id,
        float loading = 0);
    /**
     * Returns whether or not the SimHandler is ready to send another message.
     */
    static bool readyToSend(aruwlib::can::CanBus bus);
    /**
     * Allows the SimHandler to receive a given CAN message
     * and stream input values to the motor sims.
     * Returns true if data is processed (it always should be).
     */
    static bool getMessage(aruwlib::can::CanBus bus, const modm::can::Message& message);
    /**
     * Fills the given pointer with a new motor sim feedback message.
     * Returns true if successful (it always should be).
     */
    static bool sendMessage(aruwlib::can::CanBus bus, modm::can::Message* message);
    /**
     * Updates all MotorSim objects (position, RPM, time values).
     */
    static void updateSims();

private:
    /* Constants */
    static const uint8_t CAN_BUSSES = 2;
    static const uint8_t INDEX_LAST_PORT =
        aruwlib::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN - 1;
    /* Singleton Class Variables */
    static std::array<
        std::array<MotorSim*, aruwlib::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN>,
        CAN_BUSSES>
        sims;
    static std::array<uint8_t, CAN_BUSSES> nextCanSendIndex;
};
}  // namespace motorsim
}  // namespace aruwlib
#endif  // SIM_HANDLER_HPP_
#endif  // PLATFORM_HOSTED
