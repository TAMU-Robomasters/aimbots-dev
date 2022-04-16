/*
 * Copyright (c) 2009-2010, Fabian Greif
 * Copyright (c) 2009-2010, Martin Rosekeit
 * Copyright (c) 2012, 2014-2015, 2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "adc_interrupt_3.hpp"
#include <modm/architecture/interface/interrupt.hpp>
// ----------------------------------------------------------------------------
modm::platform::AdcInterrupt3::Handler
modm::platform::AdcInterrupt3::handler(modm::dummy);

MODM_ISR(ADC3)
{
    if (modm::platform::AdcInterrupt3::getInterruptFlags()) {
        modm::platform::AdcInterrupt3::handler();
    }
}
