#pragma once

#include <optional>

#include "utils/common_types.hpp"

namespace src::Utils {
class KinematicStateVector {
public:
    KinematicStateVector()
        : stateVector(Vector3f(0, 0, 0)),
          lastTime(0)  //
    {}
    ~KinematicStateVector() = default;

    Vector3f getStateVector() { return stateVector; }

    inline float getPosition() { return stateVector.getX(); }
    inline float getVelocity() { return stateVector.getY(); }
    inline float getAcceleration() { return stateVector.getZ(); }

    inline void setPosition(float x) { stateVector.setX(x); }
    inline void setVelocity(float v) { stateVector.setY(v); }
    inline void setAcceleration(float a) { stateVector.setZ(a); }

    void updateFromPosition(float x, std::optional<float> dt = std::nullopt) {
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

        // In this use case getPosition actually gets the last position
        float v1 = calcDerivative(getPosition(), x, dt.value_or(currTime - lastTime));
        float a1 = calcDerivative(getVelocity(), v1, dt.value_or(currTime - lastTime));

        setPosition(x);
        setVelocity(v1);
        setAcceleration(a1);

        lastTime = currTime;
    }

    void updateFromVelocity(float v, bool updateLowerOrderTerms = true, std::optional<float> dt = std::nullopt) {
        float currTime = tap::arch::clock::getTimeMilliseconds();

        float x1 = calcIntegral(getPosition(), v, dt.value_or(currTime - lastTime));
        float a1 = calcDerivative(getVelocity(), v, dt.value_or(currTime - lastTime));

        if (updateLowerOrderTerms) {
            setPosition(x1);
        }
        setVelocity(v);
        setAcceleration(a1);

        lastTime = currTime;
    }
    float dTime = 0;
    float V = 0;
    float S = 0;
    float count = 0;
    void updateFromAcceleration(float a, bool updateLowerOrderTerms = true, std::optional<float> dt = std::nullopt) {
        float currTime = tap::arch::clock::getTimeMilliseconds();
        if (updateLowerOrderTerms) {
            // float v1 = calcIntegral(getVelocity(), a, dt.value_or(currTime - lastTime));
            // float x1 = calcIntegral(getPosition(), v1, dt.value_or(currTime - lastTime));
            float v2 = a * (currTime - lastTime);
            if (abs(a) > 0.5) {
                float v1 = getVelocity() + v2;
            //     //float v1 = (getVelocity()-
            //     // float v1 = calcIntegral(getVelocity(), a, dt.value_or(currTime - lastTime));
                setVelocity(v1);
            } else {
                a = 0;
            }
            setPosition(v2);
            //float x1 = getPosition() + 2;
            // float x1 = getPosition() + (v1) / 2 * (currTime - lastTime);
            // float x1 = (v1 * (currTime - lastTime)) + (1/2) * a * ((currTime - lastTime) * (currTime - lastTime))
            //float x1 = currTime;
            //setPosition(x1);
        }
        setAcceleration(a);

        lastTime = currTime;
    }

    void updatePositionOnly(float a, std::optional<float> dt = std::nullopt) {
        float currTime = tap::arch::clock::getTimeMilliseconds();
        float deltaTime = dt.value_or(currTime - lastTime);
        float v = getVelocity();
        float s = getPosition();
        float newVelocity = v + a * deltaTime;
        float newPosition = s + (v + newVelocity) / 2 * deltaTime;
        setVelocity(newVelocity);
        setPosition(newPosition);
        lastTime = currTime;
    }

    uint32_t getDeltaTime() const {
        float currTime = tap::arch::clock::getTimeMilliseconds();

        float time = currTime - lastTime;
        return time;
    }

    // void updatePostion(float newPosition, bool updateLowerOrderTerms = true, std::optional<float> dt = std::nullopt) {
    //     float currTime = tap::arch::clock::getTimeMilliseconds();
    //     float deltaTime = currTime - lastTime;
    //     newPosition = 
    //     setPosition()
    // }

private:
    float calcIntegral(float x0, float v1, float dt) { return x0 + v1 * (dt); }
    float calcDerivative(float x0, float x1, float dt) { return (x1 - x0) / (dt); }

    Vector3f stateVector;
    uint32_t lastTime;
};
}  // namespace src::Utils