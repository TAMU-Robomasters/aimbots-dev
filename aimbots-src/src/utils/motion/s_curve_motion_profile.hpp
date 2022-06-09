#pragma once

#include "s_curve_acceleration.hpp"

namespace src::utils::motion {

    class SCurveMotionProfile {
       public:
        using Constraints = SCurveAcceleration::Constraints;
        using Step = SCurveAcceleration::Step;

       private:
        Constraints constraints;
        SCurveAcceleration accelerationProfile;

        float displacement;
        float cruiseDisplacement;
        float cruiseTime;
        float accelerationTime;
        float accelerationDisplacement;

       public:
        float constrainVelocity() {
            float maxAccel = constraints.acceleration;
            float maxAccelSquared = maxAccel * maxAccel;
            float maxJerk = constraints.jerk;
            float maxJerkSquared = maxJerk * maxJerk;

            return (sqrt(fabs(pow(maxAccel, 4.0) + 4.0 * displacement * maxAccel * maxJerkSquared)) - maxAccelSquared) / (maxJerk * 2.0);
        }

        SCurveMotionProfile(Constraints constraints, float displacement) : constraints(constraints), accelerationProfile(constraints), displacement(displacement) {
            accelerationDisplacement = accelerationProfile.totalDisplacement();
            if (accelerationDisplacement * 2.0f > displacement) {
                constraints.velocity = constrainVelocity();
                accelerationProfile = SCurveAcceleration(constraints);
            }

            accelerationTime = accelerationProfile.totalTime();
            accelerationDisplacement = accelerationProfile.totalDisplacement();

            cruiseDisplacement = displacement - accelerationDisplacement * 2.0f;
            cruiseTime = cruiseDisplacement / constraints.velocity;
        }

        Step stepAtTime(float t) {
            if (t < accelerationTime) {
                return accelerationProfile.stepAtT(t);
            }

            t -= accelerationTime;

            if (t < cruiseTime) {
                return Step(accelerationDisplacement + constraints.velocity * t, constraints.velocity);
            }

            t -= cruiseTime;

            Step step = accelerationProfile.stepAtT(accelerationTime - t);

            step.dist = accelerationDisplacement + cruiseDisplacement + (accelerationDisplacement - step.dist);
            step.acceleration *= -1;
            step.jerk *= -1;

            return step;
        }

        Step stepAtDisplacement(float s) {
            if (s < accelerationDisplacement) {
                return accelerationProfile.stepAtDisplacement(s);
            }

            s -= accelerationDisplacement;

            if (s < cruiseDisplacement) {
                return Step(accelerationDisplacement + s, constraints.velocity);
            }

            s -= cruiseDisplacement;

            Step step = accelerationProfile.stepAtDisplacement(accelerationDisplacement - s);

            step.dist = accelerationDisplacement + cruiseDisplacement + (accelerationDisplacement - step.dist);
            step.acceleration *= -1;
            step.jerk *= -1;

            return step;
        }

        float timeAtDisplacement(float s) {
            if (s < accelerationDisplacement) {
                return accelerationProfile.timeAtDisplacement(s);
            }

            s -= accelerationDisplacement;

            if (s < cruiseDisplacement) {
                return accelerationTime + s / constraints.velocity;
            }

            s -= cruiseDisplacement;

            return accelerationTime + cruiseTime + accelerationTime - accelerationProfile.timeAtDisplacement(accelerationDisplacement - s);
        }

        float totalTime() {
            return accelerationTime + accelerationTime + cruiseTime;
        }
    };

}  // namespace src::utils::motion