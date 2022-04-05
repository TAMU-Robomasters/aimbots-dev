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

#ifndef TAPROOT_SH1106_HPP_
#error "Don't include this file directly, use 'sh1106.hpp' instead!"
#endif

#include "sh1106_defines.hpp"

template <
    typename SPI,
    typename A0,
    typename Reset,
    unsigned int Width,
    unsigned int Height,
    bool Flipped>
modm::ResumableResult<bool> tap::display::Sh1106<SPI, A0, Reset, Width, Height, Flipped>::
    updateNonblocking()
{
    RF_BEGIN(0);

    if (!writeToDisplay.testAndSet(false)) RF_RETURN(false);

    for (y = 0; y < (Height / 8); ++y)
    {
        // command mode
        a0.reset();
        RF_CALL(spi.transfer(SH1106_PAGE_ADDRESS | y));  // Row select
        RF_CALL(spi.transfer(SH1106_COL_ADDRESS_MSB));   // Column select high

        if (Flipped)
        {
            RF_CALL(spi.transfer(SH1106_COL_ADDRESS_LSB | 4));  // Column select low
        }
        else
        {
            RF_CALL(spi.transfer(SH1106_COL_ADDRESS_LSB | SH1106_COL_OFFSET));  // Column select low
        }

        // switch to data mode
        a0.set();
        for (x = 0; x < Width; ++x)
        {
            RF_CALL(spi.transfer(this->buffer[y][x]));
        }
    }

    RF_END_RETURN(true);
}

template <
    typename SPI,
    typename A0,
    typename Reset,
    unsigned int Width,
    unsigned int Height,
    bool Flipped>
void tap::display::Sh1106<SPI, A0, Reset, Width, Height, Flipped>::update()
{
    writeToDisplay.testAndSet(true);
}

template <
    typename SPI,
    typename A0,
    typename Reset,
    unsigned int Width,
    unsigned int Height,
    bool Flipped>
void tap::display::Sh1106<SPI, A0, Reset, Width, Height, Flipped>::setInvert(bool invert)
{
    a0.reset();

    if (invert)
    {
        spi.transferBlocking(SH1106_REVERSE);
    }
    else
    {
        spi.transferBlocking(SH1106_NORMAL);
    }
}

// ----------------------------------------------------------------------------
template <
    typename SPI,
    typename A0,
    typename Reset,
    unsigned int Width,
    unsigned int Height,
    bool Flipped>
void tap::display::Sh1106<SPI, A0, Reset, Width, Height, Flipped>::initializeBlocking()
{
    a0.setOutput();
    reset.setOutput();

    // reset the controller
    reset.reset();
    modm::delay_ms(20);
    reset.set();

    a0.reset();

    // View direction
    if (Flipped)
    {
        spi.transferBlocking(SH1106_ADC_NORMAL);
        spi.transferBlocking(SH1106_SCAN_DIR_NORMAL);
    }
    else
    {
        spi.transferBlocking(SH1106_ADC_REVERSE);
        spi.transferBlocking(SH1106_SCAN_DIR_REVERSE);
    }

    spi.transferBlocking(SH1106_ON);

    this->clear();
    this->update();
}
