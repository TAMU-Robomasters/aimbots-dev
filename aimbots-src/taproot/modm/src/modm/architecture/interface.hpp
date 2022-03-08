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

#ifndef MODM_INTERFACE_HPP
#define MODM_INTERFACE_HPP

#include <stdint.h>

#include "interface/accessor.hpp"
#include "interface/accessor_flash.hpp"
#include "interface/accessor_ram.hpp"
#include "interface/adc.hpp"
#include "interface/adc_interrupt.hpp"
#include "interface/assert.hpp"
#include "interface/atomic_lock.hpp"
#include "interface/can.hpp"
#include "interface/can_filter.hpp"
#include "interface/can_message.hpp"
#include "interface/clock.hpp"
#include "interface/delay.hpp"
#include "interface/gpio.hpp"
#include "interface/i2c.hpp"
#include "interface/i2c_device.hpp"
#include "interface/i2c_master.hpp"
#include "interface/i2c_transaction.hpp"
#include "interface/interrupt.hpp"
#include "interface/memory.hpp"
#include "interface/register.hpp"
#include "interface/spi.hpp"
#include "interface/spi_device.hpp"
#include "interface/spi_master.hpp"
#include "interface/uart.hpp"
#include "interface/peripheral.hpp"

#endif	// MODM_INTERFACE_HPP