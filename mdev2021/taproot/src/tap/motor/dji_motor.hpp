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

#ifndef __DJI_MOTOR_HPP__
#define __DJI_MOTOR_HPP__

#include <string>

#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"

#include "motor_interface.hpp"

namespace tap::motor
{
// for declaring a new motor, must be one of these motor
// identifiers
enum MotorId : int32_t
{
    MOTOR1 = 0X201,
    MOTOR2 = 0x202,
    MOTOR3 = 0x203,
    MOTOR4 = 0x204,
    MOTOR5 = 0x205,
    MOTOR6 = 0x206,
    MOTOR7 = 0x207,
    MOTOR8 = 0x208,
};

// extend the CanRxListener class, which allows one to connect a
// motor to the receive handler and use the class's built in
// receive handler
class DjiMotor : public can::CanRxListener, public MotorInterface
{
public:
    // 0 - 8191 for dji motors
    static constexpr uint16_t ENC_RESOLUTION = 8192;

    // construct new motor
    DjiMotor(
        Drivers* drivers,
        MotorId desMotorIdentifier,
        tap::can::CanBus motorCanBus,
        bool isInverted,
        const char* name,
        uint16_t encWrapped = ENC_RESOLUTION / 2,
        int64_t encRevolutions = 0);

    mockable ~DjiMotor();

    void initialize() override;

    // formerly encoderstore
    int64_t getEncoderUnwrapped() const override;

    // formerly encoderstore
    uint16_t getEncoderWrapped() const override;

    DISALLOW_COPY_AND_ASSIGN(DjiMotor)

    // overrides virtual method in the can class, called every time a message is
    // received by the can receive handler
    void processMessage(const modm::can::Message& message) override { parseCanRxData(message); }

    // Accept a larger value in case someone is stupid and gave something smaller or greater
    // than 2^16, then limit it.
    // Limiting should typically be done on a motor by motor basis in a wrapper class, this
    // is simply a sanity check.
    void setDesiredOutput(int32_t desiredOutput) override;

    bool isMotorOnline() const override;

    // Serializes send data and deposits it in a message to be sent.
    mockable void serializeCanSendData(modm::can::Message* txMessage) const;

    // getter functions
    int16_t getOutputDesired() const override;

    mockable uint32_t getMotorIdentifier() const;

    int8_t getTemperature() const override;

    int16_t getTorque() const override;

    int16_t getShaftRPM() const override;

    mockable bool isMotorInverted() const;

    mockable tap::can::CanBus getCanBus() const;

    mockable const char* getName() const;

    template <typename T>
    static void assertEncoderType()
    {
        constexpr bool good_type =
            std::is_same<typename std::decay<T>::type, std::int64_t>::value ||
            std::is_same<typename std::decay<T>::type, std::uint16_t>::value;
        static_assert(good_type, "x is not of the correct type");
    }

    template <typename T>
    static T degreesToEncoder(float angle)
    {
        assertEncoderType<T>();
        return static_cast<T>((ENC_RESOLUTION * angle) / 360);
    }

    template <typename T>
    static float encoderToDegrees(T encoder)
    {
        assertEncoderType<T>();
        return (360.0f * static_cast<float>(encoder)) / ENC_RESOLUTION;
    }

private:
    // wait time before the motor is considered disconnected, in milliseconds
    static const uint32_t MOTOR_DISCONNECT_TIME = 100;

    const char* motorName;

    // formerly encoderstore
    void updateEncoderValue(uint16_t newEncWrapped);

    // Parses receive data given message with the correct identifier.
    void parseCanRxData(const modm::can::Message& message);

    Drivers* drivers;

    uint32_t motorIdentifier;

    tap::can::CanBus motorCanBus;

    int16_t desiredOutput;

    int16_t shaftRPM;

    int8_t temperature;

    int16_t torque;

    bool motorInverted;

    // formerly encoderstore
    uint16_t encoderWrapped;

    // formerly encoderstore
    int64_t encoderRevolutions;

    tap::arch::MilliTimeout motorDisconnectTimeout;
};

}  // namespace tap::motor

#endif
