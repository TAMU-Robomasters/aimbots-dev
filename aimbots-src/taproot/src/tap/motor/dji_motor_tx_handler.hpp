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

/*
 * This file is part of Taproot.
 *
 * Uses modm can interface to send CAN packets to dji motors. Receiving
 * data is done on the motor level in conjunction with
 * can_receive_interface.hpp.
 *
 * When a motor is instantiated, a CAN interface and a motor number is
 * requested. This class checks when a motor is created and kills
 * the board if multiple identical motors are created.
 *
 * To use this class properly, declare a motor somewhere, preferibly
 * in static memory, and then call the initialize method, which allows
 * one to start interacting with a motor connected via CAN bus. When
 * initialize is called, the motor class is added to a list of motors
 * which a handler runs through, sending and receiving messages for all
 * motors.
 *
 * To receive messages, create a new motor and call the initialize
 * function. This adds the motor to a queue of receive message handlers
 * that is updated by calling the pollCanData function.
 *
 * To send messages, call processCanSendMessage.
 *
 * Currently, no error handling is implemented if you attempt to use
 * a motor without initialization. This must be implemented when
 * error handling is complete.
 */

#ifndef TAPROOT_DJI_MOTOR_TX_HANDLER_HPP_
#define TAPROOT_DJI_MOTOR_TX_HANDLER_HPP_

#include <limits.h>

#include "tap/util_macros.hpp"

#include "dji_motor.hpp"

namespace tap
{
class Drivers;
namespace motor
{
#define DJI_MOTOR_NORMALIZED_ID(id) ((int32_t)id - tap::motor::MotorId::MOTOR1)
#define NORMALIZED_ID_TO_DJI_MOTOR(idx) \
    static_cast<tap::motor::MotorId>(idx + static_cast<int32_t>(tap::motor::MotorId::MOTOR1))

class DjiMotorTxHandler
{
public:
    static constexpr int DJI_MOTORS_PER_CAN = 8;

    DjiMotorTxHandler(Drivers* drivers) : drivers(drivers) {}
    mockable ~DjiMotorTxHandler() = default;
    DISALLOW_COPY_AND_ASSIGN(DjiMotorTxHandler)

    // Called when a motor is created, adds to the motor manager
    // and checks to make sure the motor is not already being used.
    mockable void addMotorToManager(DjiMotor* motor);

    mockable void processCanSendData();

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
        modm::can::Message* messageHigh);

    void removeFromMotorManager(const DjiMotor& motor, DjiMotor** motorStore);

    void zeroTxMessage(modm::can::Message* message);
};

}  // namespace motor

}  // namespace tap

#endif  // TAPROOT_DJI_MOTOR_TX_HANDLER_HPP_
