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
 * Copyright (c) 2015-2018, Niklas Hauser
 * Copyright (c) 2017, Sascha Schade
 * Copyright (c) 2018, Antal Szabó
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_ROBOMASTER_DEV_BOARD_A_HPP
#define MODM_ROBOMASTER_DEV_BOARD_A_HPP

#ifndef PLATFORM_HOSTED
#include "modm/architecture/interface/clock.hpp"
#include "modm/platform.hpp"

using namespace modm::platform;
#else
#include "modm/math/units.hpp"
#endif

/// @ingroup TODO
namespace Board
{
using namespace modm::literals;

/**
 * STM32F427 running at 180MHz from the external 12MHz crystal
 */
struct SystemClock
{
    static constexpr uint32_t Frequency = 180_MHz;
    static constexpr uint32_t Apb1 = Frequency / 2;
    static constexpr uint32_t Apb2 = Frequency;

    static constexpr uint32_t Adc = Apb2;

    static constexpr uint32_t Spi1 = Apb2;
    static constexpr uint32_t Spi2 = Apb1;
    static constexpr uint32_t Spi3 = Apb1;
    static constexpr uint32_t Spi4 = Apb2;
    static constexpr uint32_t Spi5 = Apb2;
    static constexpr uint32_t Spi6 = Apb2;

    static constexpr uint32_t Usart1 = Apb2;
    static constexpr uint32_t Usart2 = Apb1;
    static constexpr uint32_t Usart3 = Apb1;
    static constexpr uint32_t Uart4 = Apb1;
    static constexpr uint32_t Uart5 = Apb1;
    static constexpr uint32_t Usart6 = Apb2;
    static constexpr uint32_t Uart7 = Apb1;
    static constexpr uint32_t Uart8 = Apb1;

    static constexpr uint32_t Can1 = Apb1;
    static constexpr uint32_t Can2 = Apb1;

    static constexpr uint32_t I2c1 = Apb1;
    static constexpr uint32_t I2c2 = Apb1;
    static constexpr uint32_t I2c3 = Apb1;

    static constexpr uint32_t Apb1Timer = 2 * Apb1;
    static constexpr uint32_t Apb2Timer = 2 * Apb2;
    static constexpr uint32_t Timer1 = Apb2Timer;
    static constexpr uint32_t Timer2 = Apb1Timer;
    static constexpr uint32_t Timer3 = Apb1Timer;
    static constexpr uint32_t Timer4 = Apb1Timer;
    static constexpr uint32_t Timer5 = Apb1Timer;
    static constexpr uint32_t Timer6 = Apb1Timer;
    static constexpr uint32_t Timer7 = Apb1Timer;
    static constexpr uint32_t Timer8 = Apb2Timer;
    static constexpr uint32_t Timer9 = Apb2Timer;
    static constexpr uint32_t Timer10 = Apb2Timer;
    static constexpr uint32_t Timer11 = Apb2Timer;
    static constexpr uint32_t Timer12 = Apb1Timer;
    static constexpr uint32_t Timer13 = Apb1Timer;
    static constexpr uint32_t Timer14 = Apb1Timer;

    static constexpr uint32_t PWM_FREQUENCY = 50;
    static constexpr uint32_t PWM_RESOLUTION = 31000;
    static constexpr uint32_t APB1_TIMER_CLOCKS = 48150000;
    static constexpr uint32_t APB2_TIMER_CLOCKS = 92500000;
    static constexpr uint32_t APB1_PRESCALER =
        ((APB1_TIMER_CLOCKS / PWM_FREQUENCY) / PWM_RESOLUTION - 1);
    static constexpr uint32_t APB2_PRESCALER =
        ((APB2_TIMER_CLOCKS / PWM_FREQUENCY) / PWM_RESOLUTION - 1);

    static bool inline enable()
    {
#ifndef PLATFORM_HOSTED
        Rcc::enableExternalCrystal();  // 8 MHz
        Rcc::PllFactors pllF = {
            6,    // 12MHz / N=6 -> 2MHz
            180,  // 2MHz * M=180 -> 360MHz
            2     // 360MHz / P=2 -> 180MHz = F_cpu
        };
        Rcc::enablePll(Rcc::PllSource::ExternalCrystal, pllF);

        Rcc::setFlashLatency<Frequency>();
        Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
        Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div2);
        Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
        Rcc::updateCoreFrequency<Frequency>();
#endif

        return true;
    }
};

#ifndef PLATFORM_HOSTED

// initialize a button built into mcb
using Button = GpioInputB2;

// initialize 9 green Leds and 1 red LED
// leds 1-8 used for error handling codes
// led9 used for error handling error (unrepresentable error)
using LedA = GpioOutputG8;
using LedB = GpioOutputG7;
using LedC = GpioOutputG6;
using LedD = GpioOutputG5;
using LedE = GpioOutputG4;
using LedF = GpioOutputG3;
using LedG = GpioOutputG2;
using LedH = GpioOutputG1;
using LedGreen = GpioOutputF14;
using LedRed = GpioOutputE11;

using LedsPort = SoftwareGpioPort<LedA, LedB, LedC, LedD, LedE, LedF, LedG, LedH, LedGreen, LedRed>;

// initialize 4 24V outputs
using PowerOut1 = GpioOutputH2;
using PowerOut2 = GpioOutputH3;
using PowerOut3 = GpioOutputH4;
using PowerOut4 = GpioOutputH5;

using PowerOuts = SoftwareGpioPort<PowerOut1, PowerOut2, PowerOut3, PowerOut4>;

// initialize analog input pins
using AnalogInPinS = GpioOutputA0;
using AnalogInPinT = GpioOutputA1;
using AnalogInPinU = GpioOutputA2;
using AnalogInPinV = GpioOutputA3;
using AnalogInPinOled = GpioOutputA6;

using AnalogInPins =
    SoftwareGpioPort<AnalogInPinS, AnalogInPinT, AnalogInPinU, AnalogInPinV, AnalogInPinOled>;

// initialize 4 pwm output pins
using PWMOutPinW = GpioInputI5;
using PWMOutPinX = GpioInputI6;
using PWMOutPinY = GpioInputI7;
using PWMOutPinZ = GpioInputI2;

using PWMOutPins = SoftwareGpioPort<PWMOutPinW, PWMOutPinX, PWMOutPinY, PWMOutPinZ>;

// initialize 4 digital input pins
using DigitalInPinA = GpioOutputI0;
using DigitalInPinB = GpioOutputH12;
using DigitalInPinC = GpioOutputH11;
using DigitalInPinD = GpioOutputH10;

using DigitalInPins = SoftwareGpioPort<DigitalInPinA, DigitalInPinB, DigitalInPinC, DigitalInPinD>;

// initialize 4 digital output pins
using DigitalOutPinE = GpioInputD15;
using DigitalOutPinF = GpioInputD14;
using DigitalOutPinG = GpioInputD13;
using DigitalOutPinH = GpioInputD12;

using DigitalOutPins =
    SoftwareGpioPort<DigitalOutPinE, DigitalOutPinF, DigitalOutPinG, DigitalOutPinH>;

// gpio pins used for SPI communication to the onboard MPU6500 IMU
using ImuSck = GpioF7;
using ImuMiso = GpioF8;
using ImuMosi = GpioF9;
using ImuNss = GpioF6;
using ImuSpiMaster = SpiMaster5;

using DisplaySck = GpioB3;
using DisplayMiso = GpioB4;
using DisplayMosi = GpioA7;
using DisplayReset = GpioB10;
using DisplayCommand = GpioB9;
using DisplaySpiMaster = SpiMaster1;

#endif

inline void initialize()
{
    // init clock
    SystemClock::enable();
#ifndef PLATFORM_HOSTED
    SysTickTimer::initialize<SystemClock>();
    // init 24V output
    PowerOuts::setOutput(modm::Gpio::High);
    // init button on board
    Button::setInput();
#endif
}

}  // namespace Board

#endif  // MODM_ROBOMASTER_DEV_BOARD_A_HPP
