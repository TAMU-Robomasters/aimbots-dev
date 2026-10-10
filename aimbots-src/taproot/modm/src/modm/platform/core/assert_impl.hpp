/*
 * Copyright (c) 2020, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

/// @cond

#define MODM_ASSERTION_HANDLER(handler) \
	__attribute__((section(".assertion"), used)) \
	const modm::AssertionHandler \
	handler ## _assertion_handler_ptr = handler
/// @endcond