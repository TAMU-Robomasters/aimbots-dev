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

#ifndef TAPROOT_MOTOR_SIM_HPP_
#define TAPROOT_MOTOR_SIM_HPP_

#ifdef PLATFORM_HOSTED

#include <cmath>
#include <cstdint>

namespace tap
{
namespace motorsim
{
class MotorSim
{
public:
    /**
     * Enum type representing the different types of motors that can be simulated.
     */
    enum MotorType
    {
        M3508,
        GM6020
    };

    MotorSim(MotorType type, float loading = 0);

    /**
     * Resets the dynamic values in the motor simulator.
     * Should be run before any simulation.
     */
    void reset();

    /**
     * Returns the current (in amps) given to the GM3508 for the given input.
     */
    float getActualCurrent();

    /**
     * Returns the angular position of the motor.
     */
    int16_t getEnc();

    /**
     * Returns the input integer given to the motor by the CAN messages.
     */
    int16_t getInput();

    /**
     * Returns the maximum torque (in N*m) that can be held in equilibrium for the given input.
     */
    float getMaxTorque();

    /**
     * Returns the current rotational speed of the motor in RPM.
     */
    int16_t getRPM();

    /**
     * Sets the input (as an integer) used for simulation.
     * Should be updated every cycle. Default input is 0.
     * Function will do nothing if input is invalid.
     */
    void setInput(int16_t in);

    /**
     * Sets the loading on the motor (in N*m) used for simulation.
     * The default loading value is 0 N*m.
     * Function will do nothing if input is greater than rated torque.
     */
    void setLoading(float t);

    /**
     * Updates the relevant quantities for the motor being simulated.
     * Must be run iteratively in order for getEnc() and getRPM() to work correctly.
     */
    void update();

private:
    /* Constants */
    static constexpr float MINUTES_PER_MILLISECOND = 1.0f / 60000.0f;

    /* Note that these should all be constexpr, but because of
    how they are initialized in construction they cannot be. */
    int16_t MAX_INPUT_MAG = 0;    // Integer
    int16_t MAX_ENCODER_VAL = 0;  // Integer
    float MAX_CURRENT_OUT = 0;    // Amps
    float CURRENT_LIM = 0;        // Amps
    float MAX_W = 0;              // RPM
    float KT = 0;                 // (N*m)/A
    float WT_GRAD = 0;            // RPM/(N*m)

    /// Initializes constant variables based on motor type
    void initConstants(MotorType type);

    /* Class Variables */
    float loading = 0;  // N*m
    float pos = 0;      // Meters
    float rpm = 0;      // RPM
    int16_t input = 0;  // 16-bit Integer
    uint32_t time = 0;  // Milliseconds
};
}  // namespace motorsim

}  // namespace tap

#endif  // PLATFORM_HOSTED

#endif  // TAPROOT_MOTOR_SIM_HPP_
