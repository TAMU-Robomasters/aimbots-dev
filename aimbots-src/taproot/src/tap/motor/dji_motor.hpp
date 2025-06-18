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

#ifndef TAPROOT_DJI_MOTOR_HPP_
#define TAPROOT_DJI_MOTOR_HPP_

#include <string>

#include "tap/architecture/timeout.hpp"
#include "tap/communication/can/can_rx_listener.hpp"

#include "motor_interface.hpp"

namespace tap::motor
{
/**
 * CAN IDs for the feedback messages sent by DJI motor controllers. Motor `i` in the set
 * {1, 2,...,8} sends feedback data with in a CAN message with ID 0x200 + `i`.
 * for declaring a new motor, must be one of these motor
 * identifiers
 */
enum MotorId : uint32_t
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

/**
 * A class designed to interface with DJI brand motors and motor controllers over CAN.
 * This includes the C610 and C620 motor controllers and the GM6020 motor (that has a
 * built-in motor controller).
 *
 * @note: the default positive rotation direction (i.e.: when `this->isMotorInverted()
 *      == false`) is counter clockwise when looking at the shaft from the side opposite
 *      the motor. This is specified in the C620 user manual (page 18).
 *
 * DJI motor encoders store a consistent encoding for a given angle across power-cycles.
 * This means the encoder angle reported by the motor can have meaning if the encoding
 * for an angle is unique as it is for the GM6020s. However for geared motors like the
 * M3508 where a full encoder revolution does not correspond 1:1 to a shaft revolution,
 * it is impossible to know the orientation of the shaft given just the encoder value.
 *
 * Extends the CanRxListener class to attach a message handler for feedback data from the
 * motor to the CAN Rx dispatch handler.
 *
 * @note Currently there is no error handling for using a motor without having it be properly
 * initialize. You must call the `initialize` function in order for this class to work properly.
 */
class DjiMotor : public can::CanRxListener, public MotorInterface
{
public:
    // 0 - 8191 for dji motors
    static constexpr uint16_t ENC_RESOLUTION = 8192;

    /**
     * @param drivers a pointer to the drivers struct
     * @param desMotorIdentifier the ID of this motor controller
     * @param motorCanBus the CAN bus the motor is on
     * @param isInverted if `false` the positive rotation direction of the shaft is
     *      counter-clockwise when looking at the shaft from the side opposite the motor.
     *      If `true` then the positive rotation direction will be clockwise.
     * @param name a name to associate with the motor for use in the motor menu
     * @param encoderWrapped the starting encoderValue to store for this motor.
     *      Will be overwritten by the first reported encoder value from the motor
     * @param encoderRevolutions the starting number of encoder revolutions to store.
     *      See comment for DjiMotor::encoderRevolutions for more details.
     */
    DjiMotor(
        Drivers* drivers,
        MotorId desMotorIdentifier,
        tap::can::CanBus motorCanBus,
        bool isInverted,
        const char* name,
        uint16_t encoderWrapped = ENC_RESOLUTION / 2,
        int64_t encoderRevolutions = 0);

    mockable ~DjiMotor();

    void initialize() override;

    int64_t getEncoderUnwrapped() const override;

    uint16_t getEncoderWrapped() const override;

    DISALLOW_COPY_AND_ASSIGN(DjiMotor)

    /**
     * Overrides virtual method in the can class, called every time a message with the
     * CAN message id this class is attached to is received by the can receive handler.
     * Parses the data in the message and updates this class's fields accordingly.
     *
     * @param[in] message the message to be processed.
     */
    void processMessage(const modm::can::Message& message) override { parseCanRxData(message); }

    /**
     * Set the desired output for the motor. The meaning of this value is motor
     * controller specific.
     *
     * @param[in] desiredOutput the desired motor output. Limited to the range of a 16-bit int.
     *
     * @note: `desiredOutput` is cast to an int16_t and limited to an int16_t's range! The
     *      user should make sure their value is in range. The declaration takes an int32_t
     *      in hopes to mitigate overflow.
     */
    void setDesiredOutput(int32_t desiredOutput) override;

    /**
     * @return `true` if a CAN message has been received from the motor within the last
     *      `MOTOR_DISCONNECT_TIME` ms, `false` otherwise.
     */
    bool isMotorOnline() const override;

    /**
     * Serializes send data and deposits it in a message to be sent.
     */
    mockable void serializeCanSendData(modm::can::Message* txMessage) const;

    /**
     * @return the raw `desiredOutput` value which will be sent to the motor controller
     *      (specified via `setDesiredOutput()`)
     */
    int16_t getOutputDesired() const override;

    mockable uint32_t getMotorIdentifier() const;

    /**
     * @return the temperature of the motor as reported by the motor in degrees Celsius
     */
    int8_t getTemperature() const override;

    int16_t getTorque() const override;

    /// For interpreting the sign of return value see class comment
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

    /**
     * Updates the stored encoder value given a newly received encoder value
     * special logic necessary for keeping track of unwrapped encoder value.
     */
    void updateEncoderValue(uint16_t newEncWrapped);

    /**
     * Parses the data from a DJI motor feedback CAN message.
     */
    void parseCanRxData(const modm::can::Message& message);

    Drivers* drivers;

    uint32_t motorIdentifier;

    tap::can::CanBus motorCanBus;

    int16_t desiredOutput;

    int16_t shaftRPM;

    int8_t temperature;

    int16_t torque;

    /**
     * If `false` the positive rotation direction of the shaft is counter-clockwise when
     * looking at the shaft from the side opposite the motor. If `true` then the positive
     * rotation direction will be clockwise.
     */
    bool motorInverted;

    /**
     * The raw encoder value reported by the motor controller. It wraps around from
     * {0..8191}, hence "Wrapped"
     */
    uint16_t encoderWrapped;

    /**
     * Absolute unwrapped encoder position =
     *      encoderRevolutions * ENCODER_RESOLUTION + encoderWrapped
     * This lets us keep track of some sense of absolute position even while
     * raw encoderValue continuosly loops within {0..8191}. Origin value is
     * arbitrary.
     */
    int64_t encoderRevolutions;

    tap::arch::MilliTimeout motorDisconnectTimeout;
};

}  // namespace tap::motor

#endif  // TAPROOT_DJI_MOTOR_HPP_
