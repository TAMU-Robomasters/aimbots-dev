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

#ifndef BUZZER_HPP_
#define BUZZER_HPP_

#include <cstdint>

namespace tap
{
namespace gpio
{
class Pwm;
}

namespace buzzer
{
/**
 * Plays the buzzer at the given frequency.
 *
 * @param[in] pwmController The PWM object that has access to the buzzer.
 * @param[in] frequency The PWM freqneucy that corresponds to a pitch.
 */
void playNote(gpio::Pwm *pwmController, uint32_t frequency);

/**
 * Silences the buzzer.
 *
 * @param[in] pwmController the PWM object that has access to the buzzer.
 */
void silenceBuzzer(gpio::Pwm *pwmController);
}  // namespace buzzer
}  // namespace tap

#endif  // BUZZER_HPP_