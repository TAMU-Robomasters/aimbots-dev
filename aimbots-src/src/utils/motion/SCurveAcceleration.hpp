#pragma once

#include <cmath>

class SCurveAcceleration {
   public:
    struct Step {
        float velocity;
        float acceleration;
        float jerk;
        float dist;

        Step(float dist = 0.0f, float velocity = 0.0f, float acceleration = 0.0f, float jerk = 0.0f)
            : velocity(velocity), acceleration(acceleration), jerk(jerk), dist(dist) {}
    };

    struct Constraints {
        float velocity;
        float acceleration;
        float jerk;

        Constraints(float velocity, float acceleration, float jerk)
            : velocity(velocity), acceleration(acceleration), jerk(jerk) {}
    };

    float t1, v1, d1, t2, v2, d2, t3, v3, d3;
    float distInterpA, distInterpB;

   private:
    Constraints constraints;

   public:
    SCurveAcceleration(Constraints constraints) : constraints(constraints) {
        // time to jerk up to max acceleration
        t1 = constraints.jerk <= 0.0f ? 0.0 : constraints.acceleration / constraints.jerk;
        // velocity at end of jerk up period
        v1 = constraints.jerk <= 0.0f ? 0.0 : constraints.jerk * (t1 * t1) / 2.0f;
        // displacement from jerk up period
        d1 = constraints.jerk <= 0.0f ? 0.0 : constraints.jerk * (t1 * t1 * t1) / 6.0f;

        // velocity at end of constant acceleration period
        v2 = constraints.velocity - v1;
        // time of constant acceleration period
        t2 = (v2 - v1) / constraints.acceleration;
        // displacement from constant acceleration period
        // d2 = v1 * t2 + constraints.acceleration * (t2 * t2) / 2.0f;
        // d2 = t2 * (v1 + constraints.acceleration * t2 / 2.0f);
        d2 = v1 * t2 + constraints.acceleration * (t2 * t2) / 2.0f;

        t3 = t1;
        v3 = constraints.velocity;
        d3 = v2 * t3 + constraints.acceleration * (t3 * t3) / 2.0f - constraints.jerk * (t3 * t3 * t3) / 6.0f;

        // no closed form solution for dispalcement paramaterization of the third segment, so quadratic interpolation works really nicely
        float t3_mid = t3 / 2.0f;
        float d3_mid = -constraints.jerk * (t3_mid * t3_mid * t3_mid) / 6.0f + constraints.acceleration * (t3_mid * t3_mid) / 2.0f + v2 * t3_mid;
        float denominator = d3_mid * d3 * (d3_mid - d3);

        distInterpA = (d3 * t3_mid - d3_mid * t3) / denominator;
        distInterpB = ((d3_mid * d3_mid) * t3 - (d3 * d3) * t3_mid) / denominator;
    }

    Step stepAtT(float t) {
        if (t <= 0.0f) {
            return Step(0.0f, 0.0f, 0.0f);
        }

        if (t < t1) {
            return Step(constraints.jerk * (t * t * t) / 6.0f, constraints.jerk * (t * t) / 2.0, constraints.jerk * t, constraints.jerk);
        }

        t -= t1;

        if (t < t2) {
            return Step(d1 + v1 * t + constraints.acceleration * (t * t) / 2.0f, v1 + constraints.acceleration * t, constraints.acceleration, 0.0f);
        }

        t -= t2;

        if (t < t3) {
            return Step(d1 + d2 - constraints.jerk * (t * t * t) / 6.0f + constraints.acceleration * (t * t) / 2.0f + v2 * t, v2 + constraints.acceleration * t - (constraints.jerk * (t * t)) / 2.0f, constraints.acceleration - constraints.jerk * t, -constraints.jerk);
        }

        return Step(constraints.velocity, 0.0f, 0.0f);
    }

    float timeAtDisplacement(float displacement) {
        if (displacement < d1) {
            float t = std::cbrt(displacement * 6.0f / constraints.jerk);
            return (t);
        }

        displacement -= d1;

        if (displacement < d2) {
            float t = (-v1 + std::sqrt(v1 * v1 - 4.0f * constraints.acceleration / 2.0f * -displacement)) / (constraints.acceleration);
            return (t + t1);
        }

        displacement -= d2;

        if (displacement < d3) {
            float t = distInterpA * (displacement * displacement) + distInterpB * displacement;
            return (t + t1 + t2);
        }

        return t1 + t2 + t3;
    }

    Step stepAtDisplacement(float displacement) {
        if (displacement <= 0.0f) {
            return Step();
        }

        if (displacement < d1) {
            float t = std::cbrt(displacement * 6.0f / constraints.jerk);
            return Step(constraints.jerk * (t * t * t) / 6.0f, constraints.jerk * (t * t) / 2.0, constraints.jerk * t, constraints.jerk);
        }

        displacement -= d1;

        if (displacement < d2) {
            float t = (-v1 + std::sqrt(v1 * v1 - 4.0f * constraints.acceleration / 2.0f * -displacement)) / (constraints.acceleration);
            return Step(d1 + v1 * t + constraints.acceleration * (t * t) / 2.0f, v1 + constraints.acceleration * t, constraints.acceleration, 0.0f);
        }

        displacement -= d2;

        if (displacement < d3) {
            float t = distInterpA * (displacement * displacement) + distInterpB * displacement;
            return Step(d1 + d2 - constraints.jerk * (t * t * t) / 6.0f + constraints.acceleration * (t * t) / 2.0f + v2 * t, v2 + constraints.acceleration * t - (constraints.jerk * (t * t)) / 2.0f, constraints.acceleration - constraints.jerk * t, -constraints.jerk);
        }

        return Step(constraints.velocity, 0.0f, 0.0f);
    }

    float totalTime() {
        return t1 + t2 + t3;
    }

    float totalDisplacement() {
        return d1 + d2 + d3;
    }
};