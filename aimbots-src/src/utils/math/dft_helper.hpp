#pragma once

#include "sliding_dft.hpp"

namespace src::Utils::DFTHelper {

template <class NumberFormat, size_t N>
size_t getDominantFrequency(std::complex<NumberFormat> dft[N]) {
    size_t highestMagIndex = 0;
    NumberFormat highestMag = 0;
    for (size_t i = 0; i < N; i++) {
        if (std::abs(dft[i]) > highestMag) {
            highestMag = std::abs(dft[i]);
            highestMagIndex = i;
        }
    }
    return highestMagIndex;
}

};  // namespace src::Utils::DFTHelper