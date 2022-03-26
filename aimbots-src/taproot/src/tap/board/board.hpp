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

#ifndef TAPROOT_BOARD_HPP_
#define TAPROOT_BOARD_HPP_

#ifndef PLATFORM_HOSTED
#include "modm/architecture/interface/clock.hpp"
#include "modm/platform.hpp"

using namespace modm::platform;
#else
#include "modm/math/units.hpp"
#endif

namespace Board
{
using namespace modm::literals;

/**
 * STM32F427 running at 180MHz from the external 12MHz crystal
 */
struct SystemClock
{
    static constexpr uint32_t Frequency = 180_MHz;
    static constexpr uint32_t Apb1 = Frequency / 4;
    static constexpr uint32_t Apb2 = Frequency / 2;

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

    static bool inline enable()
    {
#ifndef PLATFORM_HOSTED
        Rcc::enableExternalCrystal();  // 8 MHz
        Rcc::PllFactors pllF = {
            6,    // 12MHz / M=6 -> 2MHz
            180,  // 2MHz * N=180 -> 360MHz
            2     // 360MHz / P=2 -> 180MHz = F_cpu
        };
        Rcc::enablePll(Rcc::PllSource::ExternalCrystal, pllF);

        Rcc::setFlashLatency<Frequency>();
        Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
        Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div4);
        Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div2);
        Rcc::updateCoreFrequency<Frequency>();
#endif

        return true;
    }
};

#ifndef PLATFORM_HOSTED

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

// Initialize analog input pins
        
using AnalogInPinS = GpioA0;
using AnalogInPinT = GpioA1;
using AnalogInPinU = GpioA2;
using AnalogInPinV = GpioA3;
using AnalogInPinOledJoystick = GpioA6;
        
using AnalogInPins = SoftwareGpioPort<AnalogInPinS, AnalogInPinT, AnalogInPinU, AnalogInPinV, AnalogInPinOledJoystick>;

// Initialize PWM pins
        
using PWMOutPinW = GpioI5;
using PWMOutPinX = GpioI6;
using PWMOutPinY = GpioI7;
using PWMOutPinZ = GpioI2;
using PWMOutPinBuzzer = GpioH6;
using PWMOutPinImuHeater = GpioB5;
        
using PWMOutPins = SoftwareGpioPort<PWMOutPinW, PWMOutPinX, PWMOutPinY, PWMOutPinZ, PWMOutPinBuzzer, PWMOutPinImuHeater>;

// Initialize digital input pins
        
using DigitalInPinA = GpioI0;
using DigitalInPinB = GpioH12;
using DigitalInPinC = GpioH11;
using DigitalInPinD = GpioH10;
        
using DigitalInPins = SoftwareGpioPort<DigitalInPinA, DigitalInPinB, DigitalInPinC, DigitalInPinD>;

// Initialize digital output pins
        
using DigitalOutPinE = GpioD15;
using DigitalOutPinF = GpioD14;
using DigitalOutPinG = GpioD13;
using DigitalOutPinH = GpioD12;
using DigitalOutPinLaser = GpioG13;
        
using DigitalOutPins = SoftwareGpioPort<DigitalOutPinE, DigitalOutPinF, DigitalOutPinG, DigitalOutPinH, DigitalOutPinLaser>;

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
#endif
}

}  // namespace Board

#endif  // TAPROOT_BOARD_HPP_
