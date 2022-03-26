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

#ifndef TAPROOT_SIM_HANDLER_HPP_
#define TAPROOT_SIM_HANDLER_HPP_

#ifdef PLATFORM_HOSTED

#include <array>

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor_tx_handler.hpp"

#include "motor_sim.hpp"

namespace tap
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
        tap::can::CanBus bus,
        tap::motor::MotorId id,
        float loading = 0);
    /**
     * Returns whether or not the SimHandler is ready to send another message.
     */
    static bool readyToSend(tap::can::CanBus bus);
    /**
     * Allows the SimHandler to receive a given CAN message
     * and stream input values to the motor sims.
     * Returns true if data is processed (it always should be).
     */
    static bool getMessage(tap::can::CanBus bus, const modm::can::Message& message);
    /**
     * Fills the given pointer with a new motor sim feedback message.
     * Returns true if successful (it always should be).
     */
    static bool sendMessage(tap::can::CanBus bus, modm::can::Message* message);
    /**
     * Updates all MotorSim objects (position, RPM, time values).
     */
    static void updateSims();

private:
    /* Constants */
    static const uint8_t CAN_BUSSES = 2;
    static const uint8_t INDEX_LAST_PORT = tap::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN - 1;
    /* Singleton Class Variables */
    static std::
        array<std::array<MotorSim*, tap::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN>, CAN_BUSSES>
            sims;
    static std::array<uint8_t, CAN_BUSSES> nextCanSendIndex;
};
}  // namespace motorsim
}  // namespace tap

#endif  // PLATFORM_HOSTED

#endif  // TAPROOT_SIM_HANDLER_HPP_
