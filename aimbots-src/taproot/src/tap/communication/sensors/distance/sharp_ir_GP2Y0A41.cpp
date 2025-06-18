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

#include "sharp_ir_GP2Y0A41.hpp"

namespace tap
{
namespace sensors
{
// Constructor to init Sharp IR analog pin
// Uses preset values for boundary and distance conversion
SharpIrGP2Y0A41::SharpIrGP2Y0A41(Drivers *drivers, gpio::Analog::Pin pin)
    : AnalogDistanceSensor(
          drivers,
          SHARP_IR_MIN,
          SHARP_IR_MAX,
          SHARP_IR_M,
          SHARP_IR_B,
          SHARP_IR_OFFSET,
          pin)
{
}
}  // namespace sensors

}  // namespace tap
