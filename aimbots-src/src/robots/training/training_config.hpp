#pragma once

#ifdef TARGET_TRAINING

/*
 * WEEK 2: which code reads the DT7 remote?
 *
 *   0 -> taproot's built-in tap::communication::serial::Remote (drivers->remote). Week 1 uses this.
 *   1 -> YOUR parser, src::Training::Dt7Remote (drivers->dt7Remote).
 *
 * Only one of them can own the remote's UART, so main.cpp checks this flag with `#if` and calls
 * exactly one of them. Flip it to 1 once you have filled in the TODO(week2) blocks in drivers.hpp
 * and main.cpp. See training/week2/README.md, section 5.
 */
#define TRAINING_USE_MY_REMOTE 1

#endif  // TARGET_TRAINING
