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
 * STM32F407 running at 168MHz from the external 12MHz crystal
 */
struct SystemClock
{
    static constexpr uint32_t Frequency = 168_MHz;
    static constexpr uint32_t Ahb = Frequency;
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

    static constexpr uint32_t Apb1Timer = Apb1 * 2;
    static constexpr uint32_t Apb2Timer = Apb2 * 2;
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
        Rcc::enableExternalCrystal();  // 12 MHz
        Rcc::PllFactors pllF = {
            6,    // 12MHz / M=6 -> 2MHz
            168,  // 2MHz * N=168 -> 336MHz
            2     // 336MHz / P=2 -> 168MHz = F_cpu
        };
        Rcc::enablePll(Rcc::PllSource::ExternalCrystal, pllF);

        Rcc::setFlashLatency<Frequency>();
        Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
        Rcc::setAhbPrescaler(Rcc::AhbPrescaler::Div1);
        Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div4);
        Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div2);
        Rcc::updateCoreFrequency<Frequency>();
#endif

        return true;
    }
};

#ifndef PLATFORM_HOSTED

// Initialize leds

using LedRed = GpioOutputH12;
using LedGreen = GpioOutputH11;
using LedBlue = GpioOutputH10;
using LedsPort = SoftwareGpioPort<GpioOutputH12, GpioOutputH11, GpioOutputH10>;


// Initialize PWM pins
        
using PWMOutPinC1 = GpioE9;
using PWMOutPinC2 = GpioE11;
using PWMOutPinC3 = GpioE13;
using PWMOutPinC4 = GpioE14;
using PWMOutPinC5 = GpioC6;
using PWMOutPinC6 = GpioI6;
using PWMOutPinC7 = GpioI7;
using PWMOutPinBuzzer = GpioD14;
using PWMOutPinImuHeater = GpioF6;
        
using PWMOutPins = SoftwareGpioPort<PWMOutPinC1, PWMOutPinC2, PWMOutPinC3, PWMOutPinC4, PWMOutPinC5, PWMOutPinC6, PWMOutPinC7, PWMOutPinBuzzer, PWMOutPinImuHeater>;

// Initialize digital input pins
        
using DigitalInPinPF1 = GpioF1;
using DigitalInPinPF0 = GpioF0;
using DigitalInPinB12 = GpioB12;
        
using DigitalInPins = SoftwareGpioPort<DigitalInPinPF1, DigitalInPinPF0, DigitalInPinB12>;

// Initialize digital output pins
        
using DigitalOutPinB13 = GpioB13;
using DigitalOutPinB14 = GpioB14;
using DigitalOutPinB15 = GpioB15;
using DigitalOutPinLaser = GpioC8;
        
using DigitalOutPins = SoftwareGpioPort<DigitalOutPinB13, DigitalOutPinB14, DigitalOutPinB15, DigitalOutPinLaser>;

// gpio pins used for SPI communication to the onboard BMI088 IMU

using ImuSck = GpioB3;
using ImuMiso = GpioB4;
using ImuMosi = GpioA7;
using ImuCS1Accel = GpioA4;
using ImuCS1Gyro = GpioB0;
using ImuInt1Accel = GpioC4;
using ImuInt1Gyro = GpioC5;
using ImuHeater = GpioF6;
using ImuSpiMaster = SpiMaster1;

#endif

inline void initialize()
{
    // init clock
    SystemClock::enable();
#ifndef PLATFORM_HOSTED
    SysTickTimer::initialize<SystemClock>();
#endif
}

}  // namespace Board

#endif  // TAPROOT_BOARD_HPP_
