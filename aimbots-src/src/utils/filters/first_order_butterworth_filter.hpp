#pragma once


namespace src::Utils::Filters {

class FirstOrderButterworthLPF {
    float b0 = 0.0f;
    float b1 = 0.0f;
    float a1 = 0.0f;
    float perviousResult = 0.0f;
    float perviousValue = 0.0f;
public:
    FirstOrderButterworthLPF(float b0, float b1, float a1) 
        : b0(b0), b1(b1), a1(a1) {}
    
    float update(float value) {
        float result = b0*value + b1*perviousValue - a1*perviousResult;
        perviousValue = value;
        perviousResult = result;

        return result;
    }
};

} // namespace src::Utils::Filters

