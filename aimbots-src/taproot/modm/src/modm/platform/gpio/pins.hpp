/*
 * Copyright (c) 2021-2026, Niklas Hauser
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

#include "data.hpp"
#include "static.hpp"

namespace modm::platform
{

/// @ingroup modm_platform_gpio
/// @{
using GpioA0 = GpioStatic<detail::DataA0>;
using GpioOutputA0 = GpioA0;
using GpioInputA0  = GpioA0;

using GpioA1 = GpioStatic<detail::DataA1>;
using GpioOutputA1 = GpioA1;
using GpioInputA1  = GpioA1;

using GpioA2 = GpioStatic<detail::DataA2>;
using GpioOutputA2 = GpioA2;
using GpioInputA2  = GpioA2;

using GpioA3 = GpioStatic<detail::DataA3>;
using GpioOutputA3 = GpioA3;
using GpioInputA3  = GpioA3;

using GpioA4 = GpioStatic<detail::DataA4>;
using GpioOutputA4 = GpioA4;
using GpioInputA4  = GpioA4;

using GpioA5 = GpioStatic<detail::DataA5>;
using GpioOutputA5 = GpioA5;
using GpioInputA5  = GpioA5;

using GpioA6 = GpioStatic<detail::DataA6>;
using GpioOutputA6 = GpioA6;
using GpioInputA6  = GpioA6;

using GpioA7 = GpioStatic<detail::DataA7>;
using GpioOutputA7 = GpioA7;
using GpioInputA7  = GpioA7;

using GpioA8 = GpioStatic<detail::DataA8>;
using GpioOutputA8 = GpioA8;
using GpioInputA8  = GpioA8;

using GpioA9 = GpioStatic<detail::DataA9>;
using GpioOutputA9 = GpioA9;
using GpioInputA9  = GpioA9;

using GpioA10 = GpioStatic<detail::DataA10>;
using GpioOutputA10 = GpioA10;
using GpioInputA10  = GpioA10;

using GpioA11 = GpioStatic<detail::DataA11>;
using GpioOutputA11 = GpioA11;
using GpioInputA11  = GpioA11;

using GpioA12 = GpioStatic<detail::DataA12>;
using GpioOutputA12 = GpioA12;
using GpioInputA12  = GpioA12;

using GpioA13 = GpioStatic<detail::DataA13>;
using GpioOutputA13 = GpioA13;
using GpioInputA13  = GpioA13;

using GpioA14 = GpioStatic<detail::DataA14>;
using GpioOutputA14 = GpioA14;
using GpioInputA14  = GpioA14;

using GpioA15 = GpioStatic<detail::DataA15>;
using GpioOutputA15 = GpioA15;
using GpioInputA15  = GpioA15;

using GpioB0 = GpioStatic<detail::DataB0>;
using GpioOutputB0 = GpioB0;
using GpioInputB0  = GpioB0;

using GpioB1 = GpioStatic<detail::DataB1>;
using GpioOutputB1 = GpioB1;
using GpioInputB1  = GpioB1;

using GpioB2 = GpioStatic<detail::DataB2>;
using GpioOutputB2 = GpioB2;
using GpioInputB2  = GpioB2;

using GpioB3 = GpioStatic<detail::DataB3>;
using GpioOutputB3 = GpioB3;
using GpioInputB3  = GpioB3;

using GpioB4 = GpioStatic<detail::DataB4>;
using GpioOutputB4 = GpioB4;
using GpioInputB4  = GpioB4;

using GpioB5 = GpioStatic<detail::DataB5>;
using GpioOutputB5 = GpioB5;
using GpioInputB5  = GpioB5;

using GpioB6 = GpioStatic<detail::DataB6>;
using GpioOutputB6 = GpioB6;
using GpioInputB6  = GpioB6;

using GpioB7 = GpioStatic<detail::DataB7>;
using GpioOutputB7 = GpioB7;
using GpioInputB7  = GpioB7;

using GpioB8 = GpioStatic<detail::DataB8>;
using GpioOutputB8 = GpioB8;
using GpioInputB8  = GpioB8;

using GpioB9 = GpioStatic<detail::DataB9>;
using GpioOutputB9 = GpioB9;
using GpioInputB9  = GpioB9;

using GpioB10 = GpioStatic<detail::DataB10>;
using GpioOutputB10 = GpioB10;
using GpioInputB10  = GpioB10;

using GpioB11 = GpioStatic<detail::DataB11>;
using GpioOutputB11 = GpioB11;
using GpioInputB11  = GpioB11;

using GpioB12 = GpioStatic<detail::DataB12>;
using GpioOutputB12 = GpioB12;
using GpioInputB12  = GpioB12;

using GpioB13 = GpioStatic<detail::DataB13>;
using GpioOutputB13 = GpioB13;
using GpioInputB13  = GpioB13;

using GpioB14 = GpioStatic<detail::DataB14>;
using GpioOutputB14 = GpioB14;
using GpioInputB14  = GpioB14;

using GpioB15 = GpioStatic<detail::DataB15>;
using GpioOutputB15 = GpioB15;
using GpioInputB15  = GpioB15;

using GpioC0 = GpioStatic<detail::DataC0>;
using GpioOutputC0 = GpioC0;
using GpioInputC0  = GpioC0;

using GpioC1 = GpioStatic<detail::DataC1>;
using GpioOutputC1 = GpioC1;
using GpioInputC1  = GpioC1;

using GpioC2 = GpioStatic<detail::DataC2>;
using GpioOutputC2 = GpioC2;
using GpioInputC2  = GpioC2;

using GpioC3 = GpioStatic<detail::DataC3>;
using GpioOutputC3 = GpioC3;
using GpioInputC3  = GpioC3;

using GpioC4 = GpioStatic<detail::DataC4>;
using GpioOutputC4 = GpioC4;
using GpioInputC4  = GpioC4;

using GpioC5 = GpioStatic<detail::DataC5>;
using GpioOutputC5 = GpioC5;
using GpioInputC5  = GpioC5;

using GpioC6 = GpioStatic<detail::DataC6>;
using GpioOutputC6 = GpioC6;
using GpioInputC6  = GpioC6;

using GpioC7 = GpioStatic<detail::DataC7>;
using GpioOutputC7 = GpioC7;
using GpioInputC7  = GpioC7;

using GpioC8 = GpioStatic<detail::DataC8>;
using GpioOutputC8 = GpioC8;
using GpioInputC8  = GpioC8;

using GpioC9 = GpioStatic<detail::DataC9>;
using GpioOutputC9 = GpioC9;
using GpioInputC9  = GpioC9;

using GpioC10 = GpioStatic<detail::DataC10>;
using GpioOutputC10 = GpioC10;
using GpioInputC10  = GpioC10;

using GpioC11 = GpioStatic<detail::DataC11>;
using GpioOutputC11 = GpioC11;
using GpioInputC11  = GpioC11;

using GpioC12 = GpioStatic<detail::DataC12>;
using GpioOutputC12 = GpioC12;
using GpioInputC12  = GpioC12;

using GpioC13 = GpioStatic<detail::DataC13>;
using GpioOutputC13 = GpioC13;
using GpioInputC13  = GpioC13;

using GpioC14 = GpioStatic<detail::DataC14>;
using GpioOutputC14 = GpioC14;
using GpioInputC14  = GpioC14;

using GpioC15 = GpioStatic<detail::DataC15>;
using GpioOutputC15 = GpioC15;
using GpioInputC15  = GpioC15;

using GpioD0 = GpioStatic<detail::DataD0>;
using GpioOutputD0 = GpioD0;
using GpioInputD0  = GpioD0;

using GpioD1 = GpioStatic<detail::DataD1>;
using GpioOutputD1 = GpioD1;
using GpioInputD1  = GpioD1;

using GpioD2 = GpioStatic<detail::DataD2>;
using GpioOutputD2 = GpioD2;
using GpioInputD2  = GpioD2;

using GpioD3 = GpioStatic<detail::DataD3>;
using GpioOutputD3 = GpioD3;
using GpioInputD3  = GpioD3;

using GpioD4 = GpioStatic<detail::DataD4>;
using GpioOutputD4 = GpioD4;
using GpioInputD4  = GpioD4;

using GpioD5 = GpioStatic<detail::DataD5>;
using GpioOutputD5 = GpioD5;
using GpioInputD5  = GpioD5;

using GpioD6 = GpioStatic<detail::DataD6>;
using GpioOutputD6 = GpioD6;
using GpioInputD6  = GpioD6;

using GpioD7 = GpioStatic<detail::DataD7>;
using GpioOutputD7 = GpioD7;
using GpioInputD7  = GpioD7;

using GpioD8 = GpioStatic<detail::DataD8>;
using GpioOutputD8 = GpioD8;
using GpioInputD8  = GpioD8;

using GpioD9 = GpioStatic<detail::DataD9>;
using GpioOutputD9 = GpioD9;
using GpioInputD9  = GpioD9;

using GpioD10 = GpioStatic<detail::DataD10>;
using GpioOutputD10 = GpioD10;
using GpioInputD10  = GpioD10;

using GpioD11 = GpioStatic<detail::DataD11>;
using GpioOutputD11 = GpioD11;
using GpioInputD11  = GpioD11;

using GpioD12 = GpioStatic<detail::DataD12>;
using GpioOutputD12 = GpioD12;
using GpioInputD12  = GpioD12;

using GpioD13 = GpioStatic<detail::DataD13>;
using GpioOutputD13 = GpioD13;
using GpioInputD13  = GpioD13;

using GpioD14 = GpioStatic<detail::DataD14>;
using GpioOutputD14 = GpioD14;
using GpioInputD14  = GpioD14;

using GpioD15 = GpioStatic<detail::DataD15>;
using GpioOutputD15 = GpioD15;
using GpioInputD15  = GpioD15;

using GpioE0 = GpioStatic<detail::DataE0>;
using GpioOutputE0 = GpioE0;
using GpioInputE0  = GpioE0;

using GpioE1 = GpioStatic<detail::DataE1>;
using GpioOutputE1 = GpioE1;
using GpioInputE1  = GpioE1;

using GpioE2 = GpioStatic<detail::DataE2>;
using GpioOutputE2 = GpioE2;
using GpioInputE2  = GpioE2;

using GpioE3 = GpioStatic<detail::DataE3>;
using GpioOutputE3 = GpioE3;
using GpioInputE3  = GpioE3;

using GpioE4 = GpioStatic<detail::DataE4>;
using GpioOutputE4 = GpioE4;
using GpioInputE4  = GpioE4;

using GpioE5 = GpioStatic<detail::DataE5>;
using GpioOutputE5 = GpioE5;
using GpioInputE5  = GpioE5;

using GpioE6 = GpioStatic<detail::DataE6>;
using GpioOutputE6 = GpioE6;
using GpioInputE6  = GpioE6;

using GpioE7 = GpioStatic<detail::DataE7>;
using GpioOutputE7 = GpioE7;
using GpioInputE7  = GpioE7;

using GpioE8 = GpioStatic<detail::DataE8>;
using GpioOutputE8 = GpioE8;
using GpioInputE8  = GpioE8;

using GpioE9 = GpioStatic<detail::DataE9>;
using GpioOutputE9 = GpioE9;
using GpioInputE9  = GpioE9;

using GpioE10 = GpioStatic<detail::DataE10>;
using GpioOutputE10 = GpioE10;
using GpioInputE10  = GpioE10;

using GpioE11 = GpioStatic<detail::DataE11>;
using GpioOutputE11 = GpioE11;
using GpioInputE11  = GpioE11;

using GpioE12 = GpioStatic<detail::DataE12>;
using GpioOutputE12 = GpioE12;
using GpioInputE12  = GpioE12;

using GpioE13 = GpioStatic<detail::DataE13>;
using GpioOutputE13 = GpioE13;
using GpioInputE13  = GpioE13;

using GpioE14 = GpioStatic<detail::DataE14>;
using GpioOutputE14 = GpioE14;
using GpioInputE14  = GpioE14;

using GpioE15 = GpioStatic<detail::DataE15>;
using GpioOutputE15 = GpioE15;
using GpioInputE15  = GpioE15;

using GpioH0 = GpioStatic<detail::DataH0>;
using GpioOutputH0 = GpioH0;
using GpioInputH0  = GpioH0;

using GpioH1 = GpioStatic<detail::DataH1>;
using GpioOutputH1 = GpioH1;
using GpioInputH1  = GpioH1;

/// @}

} // namespace modm::platform

