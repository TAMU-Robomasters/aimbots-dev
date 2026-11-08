/*
 * Copyright (c) 2017-2026, Niklas Hauser
 * Copyright (c) 2022, Andrey Kunitsyn
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "unused.hpp"
#include "static.hpp"
#include <type_traits>

/// @cond
namespace modm::platform::detail
{

template< Gpio::Signal signal, class... Signals >
struct GpioGetSignal;
template< Gpio::Signal signal, class SignalT, class... Signals >
struct GpioGetSignal<signal, SignalT, Signals...>
{
	using Gpio = std::conditional_t<
				(SignalT::Signal == signal),
				typename modm::platform::GpioStatic<typename SignalT::Data>,
				typename GpioGetSignal<signal, Signals...>::Gpio
			>;
};
template< Gpio::Signal signal >
struct GpioGetSignal<signal>
{
	using Gpio = GpioUnused;
};

} // namespace modm::platform::detail
/// @endcond

namespace modm::platform
{

/// @ingroup modm_platform_gpio
template< Peripheral peripheral, class... Signals >
struct GpioConnector
{
	template< class GpioQuery >
	static constexpr bool Contains = (
		std::is_same_v<typename Signals::Data, typename GpioQuery::Data> or ...);

	template< class GpioQuery >
	static constexpr bool IsValid = not std::is_same_v<typename GpioQuery::Data, detail::DataUnused>;

	template< Gpio::Signal signal >
	using GetSignal = typename detail::GpioGetSignal<signal, Signals...>::Gpio;

	template< class Signal >
	static void connectSignal()
	{
		using Connection = detail::SignalConnection<Signal, peripheral>;
		using Pin = GpioStatic<typename Signal::Data>;
		if constexpr(Connection::af == -2) {
			Pin::disconnect();
			Pin::setAnalogInput();
		}
		if constexpr (Connection::af >= 0) {
			Pin::setAlternateFunction(Connection::af);
		}
	}

	static inline void connect()
	{
		(connectSignal<Signals>(), ...);
	}

	static inline void disconnect()
	{
		(GpioStatic<typename Signals::Data>::disconnect(), ...);
	}
};

} // namespace modm::platform

