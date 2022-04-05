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

#include "pwm.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/board/board.hpp"
#include "tap/util_macros.hpp"

using namespace Board;
using namespace tap::algorithms;

namespace tap
{
namespace gpio
{
void Pwm::init()
{
#ifndef PLATFORM_HOSTED

    Timer8::connect<PWMOutPinW::Ch1, PWMOutPinX::Ch2, PWMOutPinY::Ch3, PWMOutPinZ::Ch4>();
    Timer8::enable();
    Timer8::setMode(Timer8::Mode::UpCounter);
    timer8CalculatedOverflow = Timer8::setPeriod<Board::SystemClock>(1'000'000 / DEFAULT_TIMER8_FREQUENCY);
    Timer8::start();
    Timer8::enableOutput();

    Timer12::connect<PWMOutPinBuzzer::Ch1>();
    Timer12::enable();
    Timer12::setMode(Timer12::Mode::UpCounter);
    timer12CalculatedOverflow = Timer12::setPeriod<Board::SystemClock>(1'000'000 / DEFAULT_TIMER12_FREQUENCY);
    Timer12::start();
    Timer12::enableOutput();

    Timer3::connect<PWMOutPinImuHeater::Ch2>();
    Timer3::enable();
    Timer3::setMode(Timer3::Mode::UpCounter);
    timer3CalculatedOverflow = Timer3::setPeriod<Board::SystemClock>(1'000'000 / DEFAULT_TIMER3_FREQUENCY);
    Timer3::start();
    Timer3::enableOutput();

#endif
    // Set all out pins to 0 duty
    writeAllZeros();
}

void Pwm::writeAllZeros()
{
#ifndef PLATFORM_HOSTED
    write(0.0f, Pin::W);
    write(0.0f, Pin::X);
    write(0.0f, Pin::Y);
    write(0.0f, Pin::Z);
    write(0.0f, Pin::Buzzer);
    write(0.0f, Pin::ImuHeater);
#endif
}

void Pwm::write(float duty, Pin pin)
{
#ifdef PLATFORM_HOSTED
    UNUSED(duty);
    UNUSED(pin);
#else
    duty = limitVal<float>(duty, 0.0f, 1.0f);
    switch (pin)
    {
        case Pin::W:
            Timer8::configureOutputChannel(
                Ch1,
                Timer8::OutputCompareMode::Pwm,
                duty * timer8CalculatedOverflow);
            break;
        case Pin::X:
            Timer8::configureOutputChannel(
                Ch2,
                Timer8::OutputCompareMode::Pwm,
                duty * timer8CalculatedOverflow);
            break;
        case Pin::Y:
            Timer8::configureOutputChannel(
                Ch3,
                Timer8::OutputCompareMode::Pwm,
                duty * timer8CalculatedOverflow);
            break;
        case Pin::Z:
            Timer8::configureOutputChannel(
                Ch4,
                Timer8::OutputCompareMode::Pwm,
                duty * timer8CalculatedOverflow);
            break;
        case Pin::Buzzer:
            Timer12::configureOutputChannel(
                Ch1,
                Timer12::OutputCompareMode::Pwm,
                duty * timer12CalculatedOverflow);
            break;
        case Pin::ImuHeater:
            Timer3::configureOutputChannel(
                Ch2,
                Timer3::OutputCompareMode::Pwm,
                duty * timer3CalculatedOverflow);
            break;
        default:
            break;
    };
#endif
}

void Pwm::setTimerFrequency(Timer timer, uint32_t frequency)
{
#ifdef PLATFORM_HOSTED
    UNUSED(timer);
    UNUSED(frequency);
#else
    switch (timer)
    {
        case TIMER8:
            timer8CalculatedOverflow = Timer8::setPeriod<Board::SystemClock>(1'000'000 / frequency);
            break;
        case TIMER12:
            timer12CalculatedOverflow = Timer12::setPeriod<Board::SystemClock>(1'000'000 / frequency);
            break;
        case TIMER3:
            timer3CalculatedOverflow = Timer3::setPeriod<Board::SystemClock>(1'000'000 / frequency);
            break;
    }
#endif
}

void Pwm::pause(Timer timer)
{
#ifdef PLATFORM_HOSTED
    UNUSED(timer);
#else
    switch (timer)
    {
        case TIMER8:
            Timer8::pause();
            break;
        case TIMER12:
            Timer12::pause();
            break;
        case TIMER3:
            Timer3::pause();
            break;
    }
#endif
}

void Pwm::start(Timer timer)
{
#ifdef PLATFORM_HOSTED
    UNUSED(timer);
#else
    switch (timer)
    {
        case TIMER8:
            Timer8::start();
            break;
        case TIMER12:
            Timer12::start();
            break;
        case TIMER3:
            Timer3::start();
            break;
    }
#endif
}
}  // namespace gpio

}  // namespace tap
