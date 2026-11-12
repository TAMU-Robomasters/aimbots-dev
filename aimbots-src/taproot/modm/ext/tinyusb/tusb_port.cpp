/*
 * Copyright (c) 2020, Erik Henriksson
 * Copyright (c) 2020-2026, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/platform/device.hpp>
#include <modm/architecture/interface/atomic_lock.hpp>
#include <modm/architecture/interface/delay.hpp>
#include <modm/architecture/interface/interrupt.hpp>
#include "tusb.h"

MODM_ISR(OTG_HS)
{
	tusb_int_handler(1, true);
}
MODM_ISR(OTG_HS_EP1_IN)
{
	tusb_int_handler(1, true);
}
MODM_ISR(OTG_HS_EP1_OUT)
{
	tusb_int_handler(1, true);
}
MODM_ISR(OTG_HS_WKUP)
{
	tusb_int_handler(1, true);
}
extern "C" uint8_t
tusb_get_device_serial(uint16_t *serial_str)
{
	uint8_t raw_id[16];

	uint32_t *id_addresses[4] =
	{
		((uint32_t *) UID_BASE), ((uint32_t *) UID_BASE) + 1,
		((uint32_t *) UID_BASE) + 2, ((uint32_t *) UID_BASE) + 3
	};

	for (int i = 0; i < 4; i++)
		for (int k = 0; k < 4; k++)
			raw_id[4 * i + (3 - k)] = (*(id_addresses[i]) >> k * 8) & 0xff;
	const auto fn_nibble = [](uint8_t nibble)
	{
		return nibble + (nibble > 9 ? 'A' - 10 : '0');
	};

	for (unsigned int i = 0; i < sizeof(raw_id); i++)
	{
		serial_str[i * 2 + 1] = fn_nibble(raw_id[i] & 0xf);
		serial_str[i * 2] = fn_nibble(raw_id[i] >> 4 & 0xf);
	}

	return sizeof(raw_id) * 2;
}

extern "C" void
tusb_time_delay_ms_api(uint32_t ms)
{
	modm::delay_ms(ms);
}

#ifdef CFG_TUSB_DEBUG
#include <modm/debug.hpp>

extern "C" int
tinyusb_debug_printf(const char *fmt, ...)
{
	va_list va;
	va_start(va, fmt);
	modm::log::debug.vprintf(fmt, va);
	va_end(va);
	return 0;
}
#endif
