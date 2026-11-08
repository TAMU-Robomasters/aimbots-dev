/*
 * Copyright (c) 2019, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdarg.h>
#include <modm/architecture/interface/accessor.hpp>
#include <cmath>
#include "iostream.hpp"

#include <printf/printf.h>
#include <printf/printf_config.h>

extern "C"
{
#if PRINTF_SUPPORT_LONG_LONG
typedef unsigned long long printf_unsigned_value_t;
#else
typedef unsigned long printf_unsigned_value_t;
#endif

#define FLAGS_SPACE		(1U <<  3U)
#define FLAGS_SHORT		(1U <<  7U)
#define FLAGS_LONG		(1U <<  9U)
#define FLAGS_LONG_LONG	(1U << 10U)
#define FLAGS_PRECISION	(1U << 11U)
#define FLAGS_ADAPT_EXP	(1U << 12U)

extern
void print_integer(printf_output_gadget_t* output, printf_unsigned_value_t value,
                   bool negative, uint8_t base, unsigned int precision,
                   unsigned int width, unsigned int flags);
extern
void print_floating_point(printf_output_gadget_t* output, double value,
                          unsigned int precision, unsigned int width,
                          unsigned int flags, bool prefer_exponential);
}
namespace modm
{

IOStream&
IOStream::printf(const char *fmt, ...)
{
	va_list va;
	va_start(va, fmt);
	this->vprintf(fmt, va);
	va_end(va);
	return *this;
}

IOStream&
IOStream::vprintf(const char *fmt, va_list ap)
{
	vfctprintf(&out_char, this, fmt, ap);
	return *this;
}
void
IOStream::writeInteger(int16_t value)
{
	print_integer(&output_gadget, uint16_t(value < 0 ? -value : value),
	              value < 0, 10, 0, printf_width, (printf_width ? FLAGS_SPACE : 0) | FLAGS_SHORT);
}

void
IOStream::writeInteger(uint16_t value)
{
	print_integer(&output_gadget, value, false, 10,
	              0, printf_width, (printf_width ? FLAGS_SPACE : 0) | FLAGS_SHORT);
}

void
IOStream::writeInteger(int32_t value)
{
	print_integer(&output_gadget, uint32_t(value < 0 ? -value : value),
	              value < 0, 10, 0, printf_width, (printf_width ? FLAGS_SPACE : 0) | FLAGS_LONG);
}

void
IOStream::writeInteger(uint32_t value)
{
	print_integer(&output_gadget, value, false, 10, 0,
	              printf_width, (printf_width ? FLAGS_SPACE : 0) | FLAGS_LONG);
}

void
IOStream::writeInteger(int64_t value)
{
	print_integer(&output_gadget, uint64_t(value < 0 ? -value : value),
	              value < 0, 10, 0, printf_width, (printf_width ? FLAGS_SPACE : 0) | FLAGS_LONG_LONG);
}

void
IOStream::writeInteger(uint64_t value)
{
	print_integer(&output_gadget, value, false, 10, 0,
	              printf_width, (printf_width ? FLAGS_SPACE : 0) | FLAGS_LONG_LONG);
}
void
IOStream::writeDouble(const double& value)
{
	print_floating_point(&output_gadget, value, printf_precision, printf_width,
	                     (printf_width ? FLAGS_SPACE : 0) | FLAGS_PRECISION | FLAGS_ADAPT_EXP, false);
}
} // namespace modm