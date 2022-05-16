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

    Timer1::connect<PWMOutPinC1::Ch1, PWMOutPinC2::Ch2, PWMOutPinC3::Ch3, PWMOutPinC4::Ch4>();
    Timer1::enable();
    Timer1::setMode(Timer1::Mode::UpCounter);
    timer1CalculatedOverflow = Timer1::setPeriod<Board::SystemClock>(1'000'000 / DEFAULT_TIMER1_FREQUENCY);
    Timer1::start();
    Timer1::enableOutput();

    Timer8::connect<PWMOutPinC5::Ch1, PWMOutPinC6::Ch2, PWMOutPinC7::Ch3>();
    Timer8::enable();
    Timer8::setMode(Timer8::Mode::UpCounter);
    timer8CalculatedOverflow = Timer8::setPeriod<Board::SystemClock>(1'000'000 / DEFAULT_TIMER8_FREQUENCY);
    Timer8::start();
    Timer8::enableOutput();

    Timer4::connect<PWMOutPinBuzzer::Ch3>();
    Timer4::enable();
    Timer4::setMode(Timer4::Mode::UpCounter);
    timer4CalculatedOverflow = Timer4::setPeriod<Board::SystemClock>(1'000'000 / DEFAULT_TIMER4_FREQUENCY);
    Timer4::start();
    Timer4::enableOutput();

    Timer10::connect<PWMOutPinImuHeater::Ch1>();
    Timer10::enable();
    Timer10::setMode(Timer10::Mode::UpCounter);
    timer10CalculatedOverflow = Timer10::setPeriod<Board::SystemClock>(1'000'000 / DEFAULT_TIMER10_FREQUENCY);
    Timer10::start();
    Timer10::enableOutput();

#endif
    // Set all out pins to 0 duty
    writeAllZeros();
}

void Pwm::writeAllZeros()
{
#ifndef PLATFORM_HOSTED
    write(0.0f, Pin::C1);
    write(0.0f, Pin::C2);
    write(0.0f, Pin::C3);
    write(0.0f, Pin::C4);
    write(0.0f, Pin::C5);
    write(0.0f, Pin::C6);
    write(0.0f, Pin::C7);
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
        case Pin::C1:
            Timer1::configureOutputChannel(
                Ch1,
                Timer1::OutputCompareMode::Pwm,
                duty * timer1CalculatedOverflow);
            break;
        case Pin::C2:
            Timer1::configureOutputChannel(
                Ch2,
                Timer1::OutputCompareMode::Pwm,
                duty * timer1CalculatedOverflow);
            break;
        case Pin::C3:
            Timer1::configureOutputChannel(
                Ch3,
                Timer1::OutputCompareMode::Pwm,
                duty * timer1CalculatedOverflow);
            break;
        case Pin::C4:
            Timer1::configureOutputChannel(
                Ch4,
                Timer1::OutputCompareMode::Pwm,
                duty * timer1CalculatedOverflow);
            break;
        case Pin::C5:
            Timer8::configureOutputChannel(
                Ch1,
                Timer8::OutputCompareMode::Pwm,
                duty * timer8CalculatedOverflow);
            break;
        case Pin::C6:
            Timer8::configureOutputChannel(
                Ch2,
                Timer8::OutputCompareMode::Pwm,
                duty * timer8CalculatedOverflow);
            break;
        case Pin::C7:
            Timer8::configureOutputChannel(
                Ch3,
                Timer8::OutputCompareMode::Pwm,
                duty * timer8CalculatedOverflow);
            break;
        case Pin::Buzzer:
            Timer4::configureOutputChannel(
                Ch3,
                Timer4::OutputCompareMode::Pwm,
                duty * timer4CalculatedOverflow);
            break;
        case Pin::ImuHeater:
            Timer10::configureOutputChannel(
                Ch1,
                Timer10::OutputCompareMode::Pwm,
                duty * timer10CalculatedOverflow);
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
        case TIMER1:
            timer1CalculatedOverflow = Timer1::setPeriod<Board::SystemClock>(1'000'000 / frequency);
            break;
        case TIMER8:
            timer8CalculatedOverflow = Timer8::setPeriod<Board::SystemClock>(1'000'000 / frequency);
            break;
        case TIMER4:
            timer4CalculatedOverflow = Timer4::setPeriod<Board::SystemClock>(1'000'000 / frequency);
            break;
        case TIMER10:
            timer10CalculatedOverflow = Timer10::setPeriod<Board::SystemClock>(1'000'000 / frequency);
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
        case TIMER1:
            Timer1::pause();
            break;
        case TIMER8:
            Timer8::pause();
            break;
        case TIMER4:
            Timer4::pause();
            break;
        case TIMER10:
            Timer10::pause();
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
        case TIMER1:
            Timer1::start();
            break;
        case TIMER8:
            Timer8::start();
            break;
        case TIMER4:
            Timer4::start();
            break;
        case TIMER10:
            Timer10::start();
            break;
    }
#endif
}
}  // namespace gpio

}  // namespace tap
