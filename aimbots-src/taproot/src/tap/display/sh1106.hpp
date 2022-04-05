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
#define TAPROOT_SH1106_HPP_

#include "modm/architecture/driver/atomic/flag.hpp"
#include "modm/architecture/interface/delay.hpp"
#include "modm/processing/resumable.hpp"
#include "modm/ui/display/monochrome_graphic_display_vertical.hpp"

namespace tap
{
namespace display
{
/**
 * Driver for SH1106 based OLED displays
 */
template <
#ifndef PLATFORM_HOSTED
    typename SPI,
    typename A0,
    typename Reset,
#endif
    unsigned int Width,
    unsigned int Height,
    bool Flipped>
class Sh1106 : public modm::MonochromeGraphicDisplayVertical<Width, Height>, modm::Resumable<1>
{
public:
    virtual ~Sh1106() {}

    void initializeBlocking();

    /**
     * Requests that the display begin a re-paint from the in-memory buffer.
     */
    virtual void update();

    /**
     * Update the display with the content of the RAM buffer.
     *
     * @note This function uses protothreads (http://dunkels.com/adam/pt/).
     *      Local variables *do not* necessarily behave correctly and this
     *      function should be edited with care.
     */
    modm::ResumableResult<bool> updateNonblocking();

    /**
     * Invert the display content.
     */
    void setInvert(bool invert);

protected:
#ifndef PLATFORM_HOSTED
    SPI spi;
    A0 a0;
    Reset reset;
#endif

private:
    static constexpr uint8_t SH1106_COL_OFFSET = 2;

    /**
     * Variables used in `updateNonblocking`. Since it is generally not a good idea
     * to have local variables in protothreads these are stored by this class.
     */
    uint8_t x, y;

    modm::atomic::Flag writeToDisplay;
};

}  // namespace display
}  // namespace tap

#ifdef PLATFORM_HOSTED
#include "sh1106_mock_impl.hpp"
#else
#include "sh1106_impl.hpp"
#endif

#endif  // TAPROOT_SH1106_HPP_
