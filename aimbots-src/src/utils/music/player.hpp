#pragma once

#include "drivers.hpp"

namespace utils::Music {

void continuePlayingXPStartupTune(src::Drivers* drivers);

void enableTokyoDriftTune(bool value);
void continuePlayingTokyoDriftTune(src::Drivers* drivers);

}