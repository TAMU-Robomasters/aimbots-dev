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

#include "pwm.hpp"

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/rm-dev-board-a/board.hpp"

using namespace Board;

namespace aruwlib
{
namespace gpio
{
void Pwm::init()
{
#ifndef PLATFORM_HOSTED
    Timer8::connect<PWMOutPinW::Ch1, PWMOutPinX::Ch2, PWMOutPinY::Ch3, PWMOutPinZ::Ch4>();
    Timer8::enable();
    Timer8::setMode(Timer8::Mode::UpCounter);

    Timer8::setPrescaler(Board::SystemClock::APB2_PRESCALER);
    Timer8::setOverflow(Board::SystemClock::PWM_RESOLUTION);
#endif
    // Set all out pins to 0 duty
    writeAll(0.0f);

#ifndef PLATFORM_HOSTED
    // Start the timer
    Timer8::start();
    Timer8::enableOutput();
#endif
}

void Pwm::writeAll(float duty)
{
#ifndef PLATFORM_HOSTED
    write(duty, Pin::W);
    write(duty, Pin::X);
    write(duty, Pin::Y);
    write(duty, Pin::Z);
#endif
}

void Pwm::write(float duty, Pin pin)
{
#ifndef PLATFORM_HOSTED
    duty = aruwlib::algorithms::limitVal<float>(duty, 0.0f, 1.0f);
    Timer8::configureOutputChannel(
        static_cast<int>(pin),
        Timer8::OutputCompareMode::Pwm,
        Board::SystemClock::PWM_RESOLUTION * duty);
#endif
}
}  // namespace gpio

}  // namespace aruwlib
