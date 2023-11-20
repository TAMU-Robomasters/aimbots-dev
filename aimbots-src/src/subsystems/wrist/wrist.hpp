#pragma once

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

// static inline float DJIEncoderValueToRadians(int64_t encoderValue) {
//     return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
// }

namespace src::Wrist {

enum MotorIndex {YAW, PITCH, ROLL};

class WristSubsystem : public tap::control::Subsystem {
public:
    WristSubsystem(src::Drivers*);

    void initialize() override;
    void refresh() override;

    bool isOnline() const {
        for (auto&& motor : motors)
            if (!motor->isMotorOnline()) return false;

        return true;
    }

    /**
     * @brief calculates angles based on desired arm grabber position
     * reference frame unknown
     * @param x
     * @param y
     * @param z
     */
    void calculateArmAngles(uint16_t x, uint16_t y, uint16_t z);

    void setDesiredOutputToMotor(MotorIndex idx);
    void updateCurrentMotorAngles();
    void updateMotorPositionPID(MotorIndex idx);

    int16_t getMotorRPM(MotorIndex idx) const {
        return isMotorOnline(idx) ? motors[idx]->getShaftRPM() : 0;
    }

    int16_t getMotorTorque(MotorIndex idx) const {
        return isMotorOnline(idx) ? motors[idx]->getTorque() : 0;
    }

    void setTargetAngle(MotorIndex idx, float angle) {
        targetAngles[idx]->setValue(angle);
    }

private:
    src::Drivers* drivers;

    std::array<DJIMotor*, 3> motors;
    std::array<SmoothPID*, 3> positionPIDs;

    /** The target desired angles of each motor AFTER scaling for the gear boxes */
    std::array<ContiguousFloat*, 3> targetAngles;

    /** The current angles of each motor AFTER scaling for the gear boxes */
    std::array<ContiguousFloat*, 3> currentAngles;

    std::array<float, 3> desiredMotorOutputs;

    float getCurrentAngleUnwrappedRadians(MotorIndex idx) const {
        return DJIEncoderValueToRadians(motors[idx]->getEncoderUnwrapped());
    }

    bool isMotorOnline(MotorIndex idx) const {
        return motors[idx]->isMotorOnline();
    }
};
};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE