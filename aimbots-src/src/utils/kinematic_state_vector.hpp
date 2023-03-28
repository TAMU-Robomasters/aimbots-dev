#pragma once

#include "utils/common_types.hpp"

namespace src::Utils {
class KinematicStateVector {
public:
    KinematicStateVector();
    ~KinematicStateVector();

    Vector3f getKinematicStateVector();

    inline float getPosition() { return kinematicStateVector.getX(); }
    inline float getVelocity() { return kinematicStateVector.getY(); }
    inline float getAcceleration() { return kinematicStateVector.getZ(); }

    inline void setPosition(float x) { kinematicStateVector.setX(x); }
    inline void setVelocity(float v) { kinematicStateVector.setY(v); }
    inline void setAcceleration(float a) { kinematicStateVector.setZ(a); }

    void updateFromPosition(float x) {
        uint16_t currTime = tap::arch::clock::getTimeMilliseconds();

        // In this use case getPosition actually gets the last position
        float v1 = calcDerivative(getPosition(), lastTime, x, currTime);
        float a1 = calcDerivative(getVelocity(), lastTime, v1, currTime);

        setPosition(x);
        setVelocity(v1);
        setAcceleration(a1);

        lastTime = currTime;
    }

    void updateFromVelocity(float v, bool updateLowerOrderTerms = true) {
        uint16_t currTime = tap::arch::clock::getTimeMilliseconds();

        float x1 = calcIntegral(getPosition(), lastTime, v, currTime);
        float a1 = calcDerivative(getVelocity(), lastTime, v, currTime);

        if (updateLowerOrderTerms) {
            setPosition(x1);
        }
        setVelocity(v);
        setAcceleration(a1);

        lastTime = currTime;
    }

    void updateFromAcceleration(float a) {
        uint16_t currTime = tap::arch::clock::getTimeMilliseconds();

        float v1 = calcIntegral(getVelocity(), lastTime, a, currTime);
        float x1 = calcIntegral(getPosition(), lastTime, v1, currTime);

        setPosition(x1);
        setVelocity(v1);
        setAcceleration(a);

        lastTime = currTime;
    }

private:
    float calcIntegral(float x0, uint16_t t0, float v1, uint16_t t1) { return x0 + v1 * static_cast<float>(t1 - t0); }
    float calcDerivative(float x0, uint16_t t0, float x1, uint16_t t1) { return (x1 - x0) / static_cast<float>(t1 - t0); }

    Vector3f kinematicStateVector;
    uint16_t lastTime;
};
}  // namespace src::Utils