/*
 * Copyright (c) 2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_GPIO_SET_HPP
#define MODM_STM32_GPIO_SET_HPP

#include "../device.hpp"
#include "base.hpp"

namespace modm
{

namespace platform
{

/// @ingroup modm_platform_gpio
template< class... Gpios >
class GpioSet : public Gpio
{
protected:
	static constexpr uint16_t inverteds[9] = {
		(((Gpios::port == Port::A and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::B and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::C and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::D and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::E and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::F and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::G and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::H and Gpios::isInverted) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::I and Gpios::isInverted) ? Gpios::mask : 0) | ...),
	};
	static constexpr uint16_t inverted(uint8_t id) { return inverteds[id]; }

	static constexpr uint16_t masks[9] = {
		(((Gpios::port == Port::A) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::B) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::C) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::D) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::E) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::F) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::G) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::H) ? Gpios::mask : 0) | ...),
		(((Gpios::port == Port::I) ? Gpios::mask : 0) | ...),
	};
	static constexpr uint16_t mask(uint8_t id) { return masks[id]; }
	static constexpr uint32_t mask2(uint8_t id, uint8_t value = 0b11) {
		uint32_t r{0};
		for (int ii=0; ii<16; ii++)
			if (masks[id] & (1 << ii)) r |= (uint32_t(value) << (ii * 2));
		return r;
	}
	static constexpr uint8_t numberOfPorts() {
		uint8_t r{0};
		for (const auto &m: masks) r += (m) ? 1 : 0;
		return r;
	}
public:
	static constexpr uint8_t width = sizeof...(Gpios);
	static constexpr uint8_t number_of_ports = numberOfPorts();
public:
	static void setOutput()
	{
		if constexpr (mask(0)) GPIOA->MODER = (GPIOA->MODER & ~mask2(0)) | mask2(0, i(Mode::Output));
		if constexpr (mask(1)) GPIOB->MODER = (GPIOB->MODER & ~mask2(1)) | mask2(1, i(Mode::Output));
		if constexpr (mask(2)) GPIOC->MODER = (GPIOC->MODER & ~mask2(2)) | mask2(2, i(Mode::Output));
		if constexpr (mask(3)) GPIOD->MODER = (GPIOD->MODER & ~mask2(3)) | mask2(3, i(Mode::Output));
		if constexpr (mask(4)) GPIOE->MODER = (GPIOE->MODER & ~mask2(4)) | mask2(4, i(Mode::Output));
		if constexpr (mask(5)) GPIOF->MODER = (GPIOF->MODER & ~mask2(5)) | mask2(5, i(Mode::Output));
		if constexpr (mask(6)) GPIOG->MODER = (GPIOG->MODER & ~mask2(6)) | mask2(6, i(Mode::Output));
		if constexpr (mask(7)) GPIOH->MODER = (GPIOH->MODER & ~mask2(7)) | mask2(7, i(Mode::Output));
		if constexpr (mask(8)) GPIOI->MODER = (GPIOI->MODER & ~mask2(8)) | mask2(8, i(Mode::Output));
	}

	static void setOutput(bool status)
	{
		set(status);
		setOutput();
	}

	static void setOutput(OutputType type, OutputSpeed speed = OutputSpeed::MHz50)
	{
		configure(type, speed);
		setOutput();
	}

	static void configure(OutputType type, OutputSpeed speed = OutputSpeed::MHz50)
	{
		if constexpr (mask(0)) {
			GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~mask2(0)) | (i(speed) * mask2(0, 0b01));
			GPIOA->OTYPER  = (GPIOA->OTYPER  & ~mask(0))  | (i(type) ? mask(0) : 0);
		}
		if constexpr (mask(1)) {
			GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~mask2(1)) | (i(speed) * mask2(1, 0b01));
			GPIOB->OTYPER  = (GPIOB->OTYPER  & ~mask(1))  | (i(type) ? mask(1) : 0);
		}
		if constexpr (mask(2)) {
			GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~mask2(2)) | (i(speed) * mask2(2, 0b01));
			GPIOC->OTYPER  = (GPIOC->OTYPER  & ~mask(2))  | (i(type) ? mask(2) : 0);
		}
		if constexpr (mask(3)) {
			GPIOD->OSPEEDR = (GPIOD->OSPEEDR & ~mask2(3)) | (i(speed) * mask2(3, 0b01));
			GPIOD->OTYPER  = (GPIOD->OTYPER  & ~mask(3))  | (i(type) ? mask(3) : 0);
		}
		if constexpr (mask(4)) {
			GPIOE->OSPEEDR = (GPIOE->OSPEEDR & ~mask2(4)) | (i(speed) * mask2(4, 0b01));
			GPIOE->OTYPER  = (GPIOE->OTYPER  & ~mask(4))  | (i(type) ? mask(4) : 0);
		}
		if constexpr (mask(5)) {
			GPIOF->OSPEEDR = (GPIOF->OSPEEDR & ~mask2(5)) | (i(speed) * mask2(5, 0b01));
			GPIOF->OTYPER  = (GPIOF->OTYPER  & ~mask(5))  | (i(type) ? mask(5) : 0);
		}
		if constexpr (mask(6)) {
			GPIOG->OSPEEDR = (GPIOG->OSPEEDR & ~mask2(6)) | (i(speed) * mask2(6, 0b01));
			GPIOG->OTYPER  = (GPIOG->OTYPER  & ~mask(6))  | (i(type) ? mask(6) : 0);
		}
		if constexpr (mask(7)) {
			GPIOH->OSPEEDR = (GPIOH->OSPEEDR & ~mask2(7)) | (i(speed) * mask2(7, 0b01));
			GPIOH->OTYPER  = (GPIOH->OTYPER  & ~mask(7))  | (i(type) ? mask(7) : 0);
		}
		if constexpr (mask(8)) {
			GPIOI->OSPEEDR = (GPIOI->OSPEEDR & ~mask2(8)) | (i(speed) * mask2(8, 0b01));
			GPIOI->OTYPER  = (GPIOI->OTYPER  & ~mask(8))  | (i(type) ? mask(8) : 0);
		}
	}

	static void setInput()
	{
		if constexpr (mask(0)) {
			GPIOA->MODER &= ~mask2(0);
			GPIOA->OTYPER &= ~mask(0);
			GPIOA->OSPEEDR &= ~mask2(0);
		}
		if constexpr (mask(1)) {
			GPIOB->MODER &= ~mask2(1);
			GPIOB->OTYPER &= ~mask(1);
			GPIOB->OSPEEDR &= ~mask2(1);
		}
		if constexpr (mask(2)) {
			GPIOC->MODER &= ~mask2(2);
			GPIOC->OTYPER &= ~mask(2);
			GPIOC->OSPEEDR &= ~mask2(2);
		}
		if constexpr (mask(3)) {
			GPIOD->MODER &= ~mask2(3);
			GPIOD->OTYPER &= ~mask(3);
			GPIOD->OSPEEDR &= ~mask2(3);
		}
		if constexpr (mask(4)) {
			GPIOE->MODER &= ~mask2(4);
			GPIOE->OTYPER &= ~mask(4);
			GPIOE->OSPEEDR &= ~mask2(4);
		}
		if constexpr (mask(5)) {
			GPIOF->MODER &= ~mask2(5);
			GPIOF->OTYPER &= ~mask(5);
			GPIOF->OSPEEDR &= ~mask2(5);
		}
		if constexpr (mask(6)) {
			GPIOG->MODER &= ~mask2(6);
			GPIOG->OTYPER &= ~mask(6);
			GPIOG->OSPEEDR &= ~mask2(6);
		}
		if constexpr (mask(7)) {
			GPIOH->MODER &= ~mask2(7);
			GPIOH->OTYPER &= ~mask(7);
			GPIOH->OSPEEDR &= ~mask2(7);
		}
		if constexpr (mask(8)) {
			GPIOI->MODER &= ~mask2(8);
			GPIOI->OTYPER &= ~mask(8);
			GPIOI->OSPEEDR &= ~mask2(8);
		}
	}

	static void setInput(InputType type)
	{
		configure(type);
		setInput();
	}

	static void setAnalogInput()
	{
		if constexpr (mask(0)) GPIOA->MODER |= mask2(0, i(Mode::Analog));
		if constexpr (mask(1)) GPIOB->MODER |= mask2(1, i(Mode::Analog));
		if constexpr (mask(2)) GPIOC->MODER |= mask2(2, i(Mode::Analog));
		if constexpr (mask(3)) GPIOD->MODER |= mask2(3, i(Mode::Analog));
		if constexpr (mask(4)) GPIOE->MODER |= mask2(4, i(Mode::Analog));
		if constexpr (mask(5)) GPIOF->MODER |= mask2(5, i(Mode::Analog));
		if constexpr (mask(6)) GPIOG->MODER |= mask2(6, i(Mode::Analog));
		if constexpr (mask(7)) GPIOH->MODER |= mask2(7, i(Mode::Analog));
		if constexpr (mask(8)) GPIOI->MODER |= mask2(8, i(Mode::Analog));
	}

	static void configure(InputType type)
	{
		if constexpr (mask(0)) {
			GPIOA->PUPDR = (GPIOA->PUPDR & ~mask2(0)) | (i(type) * mask2(0, 0b01));
		}
		if constexpr (mask(1)) {
			GPIOB->PUPDR = (GPIOB->PUPDR & ~mask2(1)) | (i(type) * mask2(1, 0b01));
		}
		if constexpr (mask(2)) {
			GPIOC->PUPDR = (GPIOC->PUPDR & ~mask2(2)) | (i(type) * mask2(2, 0b01));
		}
		if constexpr (mask(3)) {
			GPIOD->PUPDR = (GPIOD->PUPDR & ~mask2(3)) | (i(type) * mask2(3, 0b01));
		}
		if constexpr (mask(4)) {
			GPIOE->PUPDR = (GPIOE->PUPDR & ~mask2(4)) | (i(type) * mask2(4, 0b01));
		}
		if constexpr (mask(5)) {
			GPIOF->PUPDR = (GPIOF->PUPDR & ~mask2(5)) | (i(type) * mask2(5, 0b01));
		}
		if constexpr (mask(6)) {
			GPIOG->PUPDR = (GPIOG->PUPDR & ~mask2(6)) | (i(type) * mask2(6, 0b01));
		}
		if constexpr (mask(7)) {
			GPIOH->PUPDR = (GPIOH->PUPDR & ~mask2(7)) | (i(type) * mask2(7, 0b01));
		}
		if constexpr (mask(8)) {
			GPIOI->PUPDR = (GPIOI->PUPDR & ~mask2(8)) | (i(type) * mask2(8, 0b01));
		}
	}

	static void set()
	{
		if constexpr (mask(0)) GPIOA->BSRR = (inverted(0) << 16) | (mask(0) & ~inverted(0));
		if constexpr (mask(1)) GPIOB->BSRR = (inverted(1) << 16) | (mask(1) & ~inverted(1));
		if constexpr (mask(2)) GPIOC->BSRR = (inverted(2) << 16) | (mask(2) & ~inverted(2));
		if constexpr (mask(3)) GPIOD->BSRR = (inverted(3) << 16) | (mask(3) & ~inverted(3));
		if constexpr (mask(4)) GPIOE->BSRR = (inverted(4) << 16) | (mask(4) & ~inverted(4));
		if constexpr (mask(5)) GPIOF->BSRR = (inverted(5) << 16) | (mask(5) & ~inverted(5));
		if constexpr (mask(6)) GPIOG->BSRR = (inverted(6) << 16) | (mask(6) & ~inverted(6));
		if constexpr (mask(7)) GPIOH->BSRR = (inverted(7) << 16) | (mask(7) & ~inverted(7));
		if constexpr (mask(8)) GPIOI->BSRR = (inverted(8) << 16) | (mask(8) & ~inverted(8));
	}

	static void set(bool status)
	{
		if (status) set();
		else        reset();
	}

	static void reset()
	{
		if constexpr (mask(0)) GPIOA->BSRR = ((uint32_t(mask(0)) & ~inverted(0)) << 16) | inverted(0);
		if constexpr (mask(1)) GPIOB->BSRR = ((uint32_t(mask(1)) & ~inverted(1)) << 16) | inverted(1);
		if constexpr (mask(2)) GPIOC->BSRR = ((uint32_t(mask(2)) & ~inverted(2)) << 16) | inverted(2);
		if constexpr (mask(3)) GPIOD->BSRR = ((uint32_t(mask(3)) & ~inverted(3)) << 16) | inverted(3);
		if constexpr (mask(4)) GPIOE->BSRR = ((uint32_t(mask(4)) & ~inverted(4)) << 16) | inverted(4);
		if constexpr (mask(5)) GPIOF->BSRR = ((uint32_t(mask(5)) & ~inverted(5)) << 16) | inverted(5);
		if constexpr (mask(6)) GPIOG->BSRR = ((uint32_t(mask(6)) & ~inverted(6)) << 16) | inverted(6);
		if constexpr (mask(7)) GPIOH->BSRR = ((uint32_t(mask(7)) & ~inverted(7)) << 16) | inverted(7);
		if constexpr (mask(8)) GPIOI->BSRR = ((uint32_t(mask(8)) & ~inverted(8)) << 16) | inverted(8);
	}

	static void toggle()
	{
		if constexpr (mask(0)) {
			uint32_t are_set = (GPIOA->ODR & mask(0));
			uint32_t are_reset = mask(0) ^ are_set;
			GPIOA->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(1)) {
			uint32_t are_set = (GPIOB->ODR & mask(1));
			uint32_t are_reset = mask(1) ^ are_set;
			GPIOB->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(2)) {
			uint32_t are_set = (GPIOC->ODR & mask(2));
			uint32_t are_reset = mask(2) ^ are_set;
			GPIOC->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(3)) {
			uint32_t are_set = (GPIOD->ODR & mask(3));
			uint32_t are_reset = mask(3) ^ are_set;
			GPIOD->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(4)) {
			uint32_t are_set = (GPIOE->ODR & mask(4));
			uint32_t are_reset = mask(4) ^ are_set;
			GPIOE->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(5)) {
			uint32_t are_set = (GPIOF->ODR & mask(5));
			uint32_t are_reset = mask(5) ^ are_set;
			GPIOF->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(6)) {
			uint32_t are_set = (GPIOG->ODR & mask(6));
			uint32_t are_reset = mask(6) ^ are_set;
			GPIOG->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(7)) {
			uint32_t are_set = (GPIOH->ODR & mask(7));
			uint32_t are_reset = mask(7) ^ are_set;
			GPIOH->BSRR = (are_set << 16) | are_reset;
		}
		if constexpr (mask(8)) {
			uint32_t are_set = (GPIOI->ODR & mask(8));
			uint32_t are_reset = mask(8) ^ are_set;
			GPIOI->BSRR = (are_set << 16) | are_reset;
		}
	}

	static void lock()
	{
		if constexpr (mask(0)) {
			GPIOA->LCKR = 0x10000 | mask(0);
			GPIOA->LCKR = 0x00000 | mask(0);
			GPIOA->LCKR = 0x10000 | mask(0);
			(void) GPIOA->LCKR;
		}
		if constexpr (mask(1)) {
			GPIOB->LCKR = 0x10000 | mask(1);
			GPIOB->LCKR = 0x00000 | mask(1);
			GPIOB->LCKR = 0x10000 | mask(1);
			(void) GPIOB->LCKR;
		}
		if constexpr (mask(2)) {
			GPIOC->LCKR = 0x10000 | mask(2);
			GPIOC->LCKR = 0x00000 | mask(2);
			GPIOC->LCKR = 0x10000 | mask(2);
			(void) GPIOC->LCKR;
		}
		if constexpr (mask(3)) {
			GPIOD->LCKR = 0x10000 | mask(3);
			GPIOD->LCKR = 0x00000 | mask(3);
			GPIOD->LCKR = 0x10000 | mask(3);
			(void) GPIOD->LCKR;
		}
		if constexpr (mask(4)) {
			GPIOE->LCKR = 0x10000 | mask(4);
			GPIOE->LCKR = 0x00000 | mask(4);
			GPIOE->LCKR = 0x10000 | mask(4);
			(void) GPIOE->LCKR;
		}
		if constexpr (mask(5)) {
			GPIOF->LCKR = 0x10000 | mask(5);
			GPIOF->LCKR = 0x00000 | mask(5);
			GPIOF->LCKR = 0x10000 | mask(5);
			(void) GPIOF->LCKR;
		}
		if constexpr (mask(6)) {
			GPIOG->LCKR = 0x10000 | mask(6);
			GPIOG->LCKR = 0x00000 | mask(6);
			GPIOG->LCKR = 0x10000 | mask(6);
			(void) GPIOG->LCKR;
		}
		if constexpr (mask(7)) {
			GPIOH->LCKR = 0x10000 | mask(7);
			GPIOH->LCKR = 0x00000 | mask(7);
			GPIOH->LCKR = 0x10000 | mask(7);
			(void) GPIOH->LCKR;
		}
		if constexpr (mask(8)) {
			GPIOI->LCKR = 0x10000 | mask(8);
			GPIOI->LCKR = 0x00000 | mask(8);
			GPIOI->LCKR = 0x10000 | mask(8);
			(void) GPIOI->LCKR;
		}
	}

	static void disconnect()
	{
		(Gpios::disconnect(), ...);
	}
};

} // namespace platform

} // namespace modm

#endif // MODM_STM32_GPIO_SET_HPP