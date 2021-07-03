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

#include "motor_sim.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/architecture/clock.hpp"

namespace aruwlib
{
namespace motorsim
{
MotorSim::MotorSim(MotorType type, float loading)
{
    initConstants(type);
    reset();
    setLoading(loading);
}

void MotorSim::reset()
{
    pos = 0;
    rpm = 0;
    input = 0;
    time = 0;
}

void MotorSim::setInput(int16_t in)
{
    input = aruwlib::algorithms::limitVal<int16_t>(in, -MAX_INPUT_MAG, MAX_INPUT_MAG);
}

void MotorSim::setLoading(float t)
{
    loading = aruwlib::algorithms::limitVal<float>(t, (-KT * CURRENT_LIM), (KT * CURRENT_LIM));
}

float MotorSim::getActualCurrent()
{
    return static_cast<float>(input / MAX_INPUT_MAG) * MAX_CURRENT_OUT;
}

int16_t MotorSim::getEnc() { return static_cast<int16_t>(pos * MAX_ENCODER_VAL); }

int16_t MotorSim::getInput() { return input; }

float MotorSim::getMaxTorque() { return getActualCurrent() * KT; }

int16_t MotorSim::getRPM() { return static_cast<int16_t>(rpm); }

// TODO: Make sure that position simulation is accurate (potential timer issues, encoder tics?)
void MotorSim::update()
{
    rpm = (MAX_W - (WT_GRAD * loading)) * static_cast<float>(input) /
          (static_cast<float>(MAX_INPUT_MAG) * (CURRENT_LIM / MAX_CURRENT_OUT));
    pos += (static_cast<float>(aruwlib::arch::clock::getTimeMilliseconds() - time) /
            MINUTES_PER_MILLISECOND) *
           rpm;
    pos = fmod(pos, 1);
    time = aruwlib::arch::clock::getTimeMilliseconds();
}

void MotorSim::initConstants(MotorType type)
{
    switch (type)
    {
        case GM6020:
            MAX_INPUT_MAG = 30000;
            MAX_ENCODER_VAL = 0;
            MAX_CURRENT_OUT = 0;
            CURRENT_LIM = 0;
            MAX_W = 0;
            KT = 0;
            WT_GRAD = 0;
            break;

        case M3508:
            MAX_INPUT_MAG = 16384;
            MAX_ENCODER_VAL = 8191;
            MAX_CURRENT_OUT = 20;
            CURRENT_LIM = 10;
            MAX_W = 469;
            KT = 0.3;
            WT_GRAD = 72;
            break;

        default:
            MAX_INPUT_MAG = 0;
            MAX_CURRENT_OUT = 0;
            CURRENT_LIM = 0;
            MAX_W = 0;
            KT = 0;
            WT_GRAD = 0;
            break;
    }
}
}  // namespace motorsim

}  // namespace aruwlib

#endif  // PLATFORM_HOSTED
