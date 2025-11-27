#pragma once
#include <stddef.h>
#include <array>

namespace src::Utils::Filters {

template <size_t Order> class FirstOrderButterworthLPF{
    std::array<float, Order + 1> b; // nominator values in Z-domain
    std::array<float, Order + 1> a; // denominator values in Z-domain
    std::array<float, Order + 1> w; // intermediate values in difference equation
public:
    FirstOrderButterworthLPF(
        std::array<float, Order + 1> b, 
        std::array<float, Order + 1> a) 
        : b(b), a(a), w({0}) {}
    
    float update(float value) { // implement filter with Direct Form II structure
        float result = b[0] * value + w[0];

        for (size_t i = 0; i < Order; i++) {
            w[i] = b[i+1] * value - a[i+1] * result + w[i+1];
        }

        return result;
    }
};

} // namespace src::Utils::Filters

