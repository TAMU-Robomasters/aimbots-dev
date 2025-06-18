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

#ifndef TAPROOT_DJI_MOTOR_TX_HANDLER_HPP_
#define TAPROOT_DJI_MOTOR_TX_HANDLER_HPP_

#include <limits.h>

#include "tap/util_macros.hpp"

#include "dji_motor.hpp"

namespace tap
{
class Drivers;
}

namespace tap::motor
{
/**
 * Converts the dji MotorId to a uint32_t.
 * @param[in] id Some CAN MotorId
 * @return id normalized to be around [0, DJI_MOTORS_PER_CAN), or some value >= DJI_MOTORS_PER_CAN
 * if the id is out of bounds
 */
#define DJI_MOTOR_TO_NORMALIZED_ID(id)                                                  \
    static_cast<uint32_t>(                                                              \
        (id < tap::motor::MOTOR1) ? (tap::motor::DjiMotorTxHandler::DJI_MOTORS_PER_CAN) \
                                  : (id - tap::motor::MOTOR1))

/**
 * Converts the dji MotorId to a uint32_t.
 * @param[in] idx Some index, a motor id index normalized between [0, DJI_MOTORS_PER_CAN)
 * @return idx, converted to a MotorId
 */
#define NORMALIZED_ID_TO_DJI_MOTOR(idx) \
    static_cast<tap::motor::MotorId>(idx + static_cast<int32_t>(tap::motor::MotorId::MOTOR1))

/**
 * Uses modm can interface to send CAN packets to `DjiMotor`'s connected to the two CAN buses.
 *
 * To use this class properly, declare a motor somewhere, then call the initialize method, which
 * allows one to start interacting with a motor connected via CAN bus. When the motor's `initialize`
 * function is called, this object's `addMotorToManager` function is called and the motor is ready
 * to have its control information sent to the motor on the bus.
 *
 * To send messages, call this class's `encodeAndSendCanData` function.
 */
class DjiMotorTxHandler
{
public:
    /** Number of motors on each CAN bus. */
    static constexpr int DJI_MOTORS_PER_CAN = 8;
    /** CAN message length of each motor control message. */
    static constexpr int CAN_DJI_MESSAGE_SEND_LENGTH = 8;
    /** CAN message identifier for "low" segment (low 4 CAN motor IDs) of control message. */
    static constexpr uint32_t CAN_DJI_LOW_IDENTIFIER = 0X200;
    /** CAN message identifier for "high" segment (high 4 CAN motor IDs) of control message. */
    static constexpr uint32_t CAN_DJI_HIGH_IDENTIFIER = 0X1FF;

    DjiMotorTxHandler(Drivers* drivers) : drivers(drivers) {}
    mockable ~DjiMotorTxHandler() = default;
    DISALLOW_COPY_AND_ASSIGN(DjiMotorTxHandler)

    /**
     * Adds the motor to the manager so that it can receive motor messages from the CAN bus. If
     * there is already a motor with the same ID in the manager, the program will abort
     */
    mockable void addMotorToManager(DjiMotor* motor);

    /**
     * Sends motor commands across the CAN bus. Sends up to 4 messages (2 per CAN bus), though it
     * may send less depending on which motors have been registered with the motor manager. Each
     * messages encodes motor controller command information for up to 4 motors.
     */
    mockable void encodeAndSendCanData();

    /**
     * Removes the motor from the motor manager.
     */
    mockable void removeFromMotorManager(const DjiMotor& motor);

    mockable DjiMotor const* getCan1Motor(MotorId motorId);

    mockable DjiMotor const* getCan2Motor(MotorId motorId);

private:
    Drivers* drivers;

    DjiMotor* can1MotorStore[DJI_MOTORS_PER_CAN] = {0};
    DjiMotor* can2MotorStore[DJI_MOTORS_PER_CAN] = {0};

    void addMotorToManager(DjiMotor** canMotorStore, DjiMotor* const motor);

    void serializeMotorStoreSendData(
        DjiMotor** canMotorStore,
        modm::can::Message* messageLow,
        modm::can::Message* messageHigh,
        bool* validMotorMessageLow,
        bool* validMotorMessageHigh);

    void removeFromMotorManager(const DjiMotor& motor, DjiMotor** motorStore);
};

}  // namespace tap::motor

#endif  // TAPROOT_DJI_MOTOR_TX_HANDLER_HPP_
