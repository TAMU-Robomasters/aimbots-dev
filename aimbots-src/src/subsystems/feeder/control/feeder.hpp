#pragma once
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "informants/sensors/limit_switch.hpp"
#include "subsystems/feeder/feeder_constants.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {
class FeederSubsystem : public tap::control::Subsystem {
public:
    FeederSubsystem(src::Drivers* drivers);
    // tagging on building the target RPM array here
    void BuildFeederMotors() {
        for (auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
            feederMotors[i] =
                new DJIMotor(drivers, FEEDER_MOTOR_IDS[i], FEEDER_BUS, FEEDER_DIRECTION[i], FEEDER_MOTOR_NAMES[i]);
            feederTargetRPMs[i] = 0.0f;
            feederCustomSpeedActive[i] = false;
        }
    }

    template <class... Args>
    void ForAllFeederMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& feederMotor : feederMotors) {
            (feederMotor->*func)(args...);
        }
    }

    template <class... Args>
    void ForAllFeederMotors(void (FeederSubsystem::*func)(uint8_t FeederIdx, Args...), Args... args) {
        for (uint8_t i = 0; i < FEEDER_MOTOR_COUNT; i++) {
            (this->*func)(i, args...);
        }
    }

    template <class... Args>
    void ForFeederMotorGroup(FeederGroup groupID, void (FeederSubsystem::*func)(uint8_t FeederIdx, Args...), Args... args) {
        for (uint8_t i = 0; i < FEEDER_MOTOR_COUNT; i++) {
            if (groupID == ALL || FEEDER_MOTOR_GROUPS[i] == groupID) {
                (this->*func)(i, args...);
            }
        }
    }

    void BuildPIDControllers() {
        for (auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
            feederVelocityPIDs[i] = new SmoothPID(FEEDER_VELOCITY_PID_CONFIG);
        }
    }

    mockable void initialize() override;
    mockable void refresh() override;

    //  float getMotorSpeed(FeederIdx feederIdx) const;

    inline bool isOnline() const {
        bool feederOnline = false;

        for (auto& feederMotor : feederMotors) {
            if (feederMotor->isMotorOnline()) {
                feederOnline = true;
            }
        }

        return feederOnline;
    }

    inline bool isFeederMotorOnline(uint8_t FeederIdx) const {
        if (FeederIdx >= FEEDER_MOTOR_COUNT) {
            return false;
        }
        return feederMotors[FeederIdx]->isMotorOnline();
    }

    void updateMotorVelocityPID(uint8_t FeederIdxIdx);

    void setDesiredFeederMotorOutput(uint8_t FeederIdx, float output) { desiredFeederMotorOutputs[FeederIdx] = output; }

    void setAllDesiredFeederMotorOutputs(uint16_t output) { desiredFeederMotorOutputs.fill(output); }

    void setTargetRPM(uint8_t FeederIdx, float rpm);

    float getCurrentRPM(uint8_t FeederIdx = 0) const {
        return (feederMotors[FeederIdx]->isMotorOnline()) ? -feederMotors[FeederIdx]->getShaftRPM() : 0;
    }

    inline int16_t getFeederMotorTorque(uint8_t feederIdx = 0) const {
        return feederMotors[feederIdx]->isMotorOnline() ? feederMotors[feederIdx]->getTorque() : 0;
    }

    bool getPressed();

    // float getCurrentRPM() const { return feederMotor.getShaftRPM(); }

    int64_t getEncoderUnwrapped(uint8_t feederIdx = 0) const {
        return feederMotors[feederIdx]->getEncoderUnwrapped() / FEEDER_GEAR_RATIOS[feederIdx];
    }

    int getTotalLimitCount() const;

    // Group commands: These are to be used with ForFeederMotorGroup
    void activateFeederMotor(uint8_t feederIdx = 0) {
        if (!feederCustomSpeedActive[feederIdx]) {  // Ignore the default RPM for a custom speed setting
            setTargetRPM(feederIdx, FEEDER_NORMAL_RPMS[feederIdx]);
        }
    }

    inline void setFeederCustomStatus(uint8_t feederIdx, bool customStatus) {
        feederCustomSpeedActive[feederIdx] = customStatus;
    }

    void setFeederCustomMulti(uint8_t feederIdx, float mul) {
        if (mul == 1.0) {
            setFeederCustomStatus(feederIdx, false);
        } else {
            setFeederCustomStatus(feederIdx, true);
            setTargetRPM(feederIdx, FEEDER_NORMAL_RPMS[feederIdx] * mul);
        }
    }

    void deactivateFeederMotor(uint8_t feederIdx = 0) { setTargetRPM(feederIdx, 0.0f); }

    void unjamFeederMotor(uint8_t feederIdx = 0) { setTargetRPM(feederIdx, -abs(FEEDER_UNJAM_RPMS[feederIdx])); }

private:
    src::Drivers* drivers;

    std::array<DJIMotor*, FEEDER_MOTOR_COUNT> feederMotors;

    std::array<float, FEEDER_MOTOR_COUNT> desiredFeederMotorOutputs;

    void setDesiredOutputToFeederMotor(uint8_t FeederIdx);

    // float targetRPM;
    // float desiredOutput;
    std::array<float, FEEDER_MOTOR_COUNT> feederTargetRPMs;

    std::array<SmoothPID*, FEEDER_MOTOR_COUNT> feederVelocityPIDs;

    std::array<bool, FEEDER_MOTOR_COUNT> feederCustomSpeedActive;
    // DJIMotor feederMotor;

    src::Informants::LimitSwitch limitSwitch;
    //#endif

    // commands
};

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE