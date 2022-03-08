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

#define _vsnprintf _vsnprintf_windows
#include <stdio.h>
#undef _vsnprintf
#include <stdarg.h>
#include <modm/architecture/interface/accessor.hpp>
#include <cmath>

#include "iostream.hpp"

extern "C"
{
	// configure printf implementation
	#define PRINTF_DEFAULT_FLOAT_PRECISION 5U

	// include source from modm/ext/printf
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wdouble-promotion"
	#include "printf/printf.source"
	#pragma GCC diagnostic pop
	void _putchar(char) {};

}

namespace
{
void out_char(char character, void* buffer, size_t, size_t)
{
	if (character)
		reinterpret_cast<modm::IOStream*>(buffer)->write(character);
}
}
namespace modm
{

IOStream&
IOStream::printf(const char *fmt, ...)
{
	va_list va;
	va_start(va, fmt);
	vprintf(fmt, va);
	va_end(va);
	return *this;
}

IOStream&
IOStream::vprintf(const char *fmt, va_list ap)
{
	_vsnprintf(out_char, (char*)this, -1, fmt, ap);
	return *this;
}
void
IOStream::writeInteger(int16_t value)
{
	_ntoa_long(out_char, (char*)this,
			   0, -1,
			   uint16_t(value < 0 ? -value : value),
			   value < 0,
			   10, 0, 0,
			   FLAGS_SHORT);
}

void
IOStream::writeInteger(uint16_t value)
{
	_ntoa_long(out_char, (char*)this,
			   0, -1,
			   value,
			   false,
			   10, 0, 0,
			   FLAGS_SHORT);
}

void
IOStream::writeInteger(int32_t value)
{
	_ntoa_long(out_char, (char*)this,
			   0, -1,
			   uint32_t(value < 0 ? -value : value),
			   value < 0,
			   10, 0, 0,
			   FLAGS_LONG);
}

void
IOStream::writeInteger(uint32_t value)
{
	_ntoa_long(out_char, (char*)this,
			   0, -1,
			   value,
			   false,
			   10, 0, 0,
			   FLAGS_LONG);
}

void
IOStream::writeInteger(int64_t value)
{
	_ntoa_long_long(out_char, (char*)this,
					0, -1,
					uint64_t(value < 0 ? -value : value),
					value < 0,
					10, 0, 0,
					FLAGS_LONG_LONG);
}

void
IOStream::writeInteger(uint64_t value)
{
	_ntoa_long_long(out_char, (char*)this,
					0, -1,
					value,
					false,
					10, 0, 0,
					FLAGS_LONG_LONG);
}
void
IOStream::writeDouble(const double& value)
{
	_etoa(out_char, (char*)this,
		  0, -1,
		  value,
		  0, 0, 0);
}
} // namespace modm