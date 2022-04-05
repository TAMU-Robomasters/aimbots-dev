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

#ifndef TAPROOT_SH1106_DEFINES_HPP_
#define TAPROOT_SH1106_DEFINES_HPP_

namespace tap
{
namespace display
{
// (1) Set Lower Column Address
#define SH1106_COL_ADDRESS_MSB 0x10
// (2) Set Higher Column Address
#define SH1106_COL_ADDRESS_LSB 0x00

// (3) Set Pump voltage value (unused)
// (4) Set Display Start Line (unused)

// (5) Set Contrast Control Register
#define SH1106_CONTRAST_MODE 0x81
#define SH1106_CONTRAST_DATA 0x00

// (6) Set Segment Re-map
#define SH1106_ADC_NORMAL 0xA0
#define SH1106_ADC_REVERSE 0xA1

// (7) Set Entire Display OFF/ON
#define SH1106_FORCE_ON_DEFAULT 0xA4
#define SH1106_FORCE_ON_ENABLED 0xA5

// (8) Set Normal/Reverse Display
#define SH1106_NORMAL 0xA6
#define SH1106_REVERSE 0xA7

// (9) Set Multiplex Ration (unused)
// (10) Set DC-DC OFF/ON (unused)

// (11) Display OFF/ON
#define SH1106_ON 0xAF
#define SH1106_OFF 0xAE

// (12) Set Page Address
#define SH1106_PAGE_ADDRESS 0xB0

// (13) Set Common Output Scan Direction
#define SH1106_SCAN_DIR_NORMAL 0xC0
#define SH1106_SCAN_DIR_REVERSE 0xC8

// (14) Set Display Offset (unused)
// (15) Set Display Clock Divide Ratio/Oscillator Frequency (unused)
// (16) Set Dis-charge/Pre-charge Period (unused)
// (17) Set Common pads hardware configuration (unused)
// (18) Set VCOM Deselect Level (unused)
// (19) Read-Modify-Write (unused)
// (20) End (for Read-Modify-Write, unused)

// (21) NOP
#define SH1106_NOP 0xE3

// (22) Write Display Data
// no specific address

// (23) Read Status (unused)
// (24) Read Display Data (unused)

}  // namespace display
}  // namespace tap

#endif  // TAPROOT_SH1106_DEFINES_HPP_
