#pragma once

namespace src::Utils::Filters {

class EMAFilter {
private:
    float alpha;
    float lastValue;

public:
    EMAFilter(float alpha) {
        this->alpha = alpha;
        this->lastValue = 0;
    }

    float update(float value) {
        float filteredValue = alpha * value + (1 - alpha) * lastValue;
        lastValue = filteredValue;
        return filteredValue;
    }

    float getValue() { return lastValue; }

    void setAlpha(float alpha) { this->alpha = alpha; }
};

}  // namespace src::Utils::Filters