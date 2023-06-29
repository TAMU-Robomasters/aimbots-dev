#pragma once
#include "modm/platform/random/random_number_generator.hpp"

namespace src::Utils::Random {

// modm::platform::RandomNumberGenerator::enable() called in main.cpp's initializeIO()

static uint32_t getRandomInteger() {
    if (modm::platform::RandomNumberGenerator::isReady()) return modm::platform::RandomNumberGenerator::getValue();
    return 0;
}

static int32_t getRandomIntegerInBounds(int32_t min, int32_t max) {
    uint32_t range = max - min;
    uint32_t random = getRandomInteger();
    uint32_t base = random % range;

    return static_cast<int32_t>(base) + min;
}

static float getRandomFloatInBounds(float min, float max) {
    float range = max - min;
    float random = getRandomInteger() / static_cast<float>(UINT32_MAX);
    float base = random * range;

    return base + min;
}
};  // namespace src::Utils::Random