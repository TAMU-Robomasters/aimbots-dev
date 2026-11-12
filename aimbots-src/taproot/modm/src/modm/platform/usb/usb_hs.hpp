/*
* Copyright (c) 2020, Erik Henriksson
* Copyright (c) 2020, 2024, Niklas Hauser
*
* This file is part of the modm project.
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/
// ----------------------------------------------------------------------------

#pragma once

#include <modm/platform/device.hpp>
#include <modm/platform/gpio/connector.hpp>

namespace modm::platform
{

/// @ingroup modm_platform_usb modm_platform_usb_hs
class UsbHs
{
public:
	template< class SystemClock >
	static void
	initialize(uint8_t priority=3)
	{
		static_assert(SystemClock::Usb == 48_MHz, "UsbHs must have a 48MHz clock!");
#ifdef PWR_CR3_USB33DEN
		PWR->CR3 |= PWR_CR3_USB33DEN;
#endif
		Rcc::enable<Peripheral::Usbotghs>();
		NVIC_SetPriority(OTG_HS_IRQn, priority);
		NVIC_SetPriority(OTG_HS_EP1_IN_IRQn, priority);
		NVIC_SetPriority(OTG_HS_EP1_OUT_IRQn, priority);
		NVIC_SetPriority(OTG_HS_WKUP_IRQn, priority);
	}

	template< class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Usbotghs, Signals...>;
		using Dp = typename Connector::template GetSignal< Gpio::Signal::Dp >;
		using Dm = typename Connector::template GetSignal< Gpio::Signal::Dm >;
		using Id = typename Connector::template GetSignal< Gpio::Signal::Id >;
		static constexpr bool all_usb =
			Connector::template IsValid<Dp> and Connector::template IsValid<Dm>;
		static constexpr bool any_usb =
			Connector::template IsValid<Dp> or Connector::template IsValid<Dm>;

		using Ulpick = typename Connector::template GetSignal< Gpio::Signal::Ulpick >;
		using Ulpid0 = typename Connector::template GetSignal< Gpio::Signal::Ulpid0 >;
		using Ulpid1 = typename Connector::template GetSignal< Gpio::Signal::Ulpid1 >;
		using Ulpid2 = typename Connector::template GetSignal< Gpio::Signal::Ulpid2 >;
		using Ulpid3 = typename Connector::template GetSignal< Gpio::Signal::Ulpid3 >;
		using Ulpid4 = typename Connector::template GetSignal< Gpio::Signal::Ulpid4 >;
		using Ulpid5 = typename Connector::template GetSignal< Gpio::Signal::Ulpid5 >;
		using Ulpid6 = typename Connector::template GetSignal< Gpio::Signal::Ulpid6 >;
		using Ulpid7 = typename Connector::template GetSignal< Gpio::Signal::Ulpid7 >;
		using Ulpidir = typename Connector::template GetSignal< Gpio::Signal::Ulpidir >;
		using Ulpinxt = typename Connector::template GetSignal< Gpio::Signal::Ulpinxt >;
		using Ulpistp = typename Connector::template GetSignal< Gpio::Signal::Ulpistp >;

		static constexpr bool all_ulpi =
			Connector::template IsValid< Ulpick > and
			Connector::template IsValid< Ulpid0 > and
			Connector::template IsValid< Ulpid1 > and
			Connector::template IsValid< Ulpid2 > and
			Connector::template IsValid< Ulpid3 > and
			Connector::template IsValid< Ulpid4 > and
			Connector::template IsValid< Ulpid5 > and
			Connector::template IsValid< Ulpid6 > and
			Connector::template IsValid< Ulpid7 > and
			Connector::template IsValid< Ulpidir > and
			Connector::template IsValid< Ulpinxt > and
			Connector::template IsValid< Ulpistp >;

		static constexpr bool any_ulpi =
			Connector::template IsValid< Ulpick > or
			Connector::template IsValid< Ulpid0 > or
			Connector::template IsValid< Ulpid1 > or
			Connector::template IsValid< Ulpid2 > or
			Connector::template IsValid< Ulpid3 > or
			Connector::template IsValid< Ulpid4 > or
			Connector::template IsValid< Ulpid5 > or
			Connector::template IsValid< Ulpid6 > or
			Connector::template IsValid< Ulpid7 > or
			Connector::template IsValid< Ulpidir > or
			Connector::template IsValid< Ulpinxt > or
			Connector::template IsValid< Ulpistp >;

		static_assert((any_ulpi xor any_usb) and (all_ulpi or all_usb),
			"UsbHs::connect() requires at least Dp, Dm (+Id) signals OR 12 ULPI signals:\n"
			"    - CK\n"
			"    - D0\n"
			"    - D1\n"
			"    - D2\n"
			"    - D3\n"
			"    - D4\n"
			"    - D5\n"
			"    - D6\n"
			"    - D7\n"
			"    - DIR\n"
			"    - NXT\n"
			"    - STP");

		if constexpr (all_ulpi and not all_usb)
		{
			Rcc::enable<Peripheral::Usbotghsulpi>();
			GpioSet< Ulpick, Ulpid0, Ulpid1, Ulpid2, Ulpid3, Ulpid4, Ulpid5, Ulpid6, Ulpid7, Ulpidir, Ulpinxt, Ulpistp >::configure(
					Gpio::OutputType::PushPull, Gpio::OutputSpeed::High);
		}
		else if constexpr (not all_ulpi and all_usb)
		{
			Rcc::disable<Peripheral::Usbotghsulpi>();
			GpioSet<Dp, Dm>::configure(Gpio::OutputType::PushPull, Gpio::OutputSpeed::High);
			Id::configure(Gpio::OutputType::OpenDrain, Gpio::OutputSpeed::High);
			Id::configure(Gpio::InputType::PullUp);
		}
		Connector::connect();
	}
};

} // namespace modm::platform