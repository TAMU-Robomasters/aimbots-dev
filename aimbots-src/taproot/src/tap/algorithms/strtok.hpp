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

#ifndef TAPROOT_STRTOK_HPP_
#define TAPROOT_STRTOK_HPP_

#ifndef __DOXYGEN__

/**
 * Source code from glibc/string/strtok_r.c. arm-none-eabi-gcc does not have strtok_r,
 * so we implement it ourselves.
 */
char *strtokR(char *s, const char *delim, char **savePtr);

#endif

#endif  // TAPROOT_STRTOK_HPP_
