#pragma once
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "informants/sensors/limit_switch.hpp"
#include "subsystems/wrist/wrist_constants.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {
class WristSubsystem : public tap::control::Subsystem {
public:
    WristSubsystem(src::Drivers* drivers);
    // tagging on building the target RPM array here
    void BuildWristMotors() {
        for (auto i = 0; i < WRIST_MOTOR_COUNT; i++) {
            wristMotors[i] =
                new DJIMotor(drivers, WRIST_MOTOR_IDS[i], WRIST_BUS, WRIST_DIRECTION[i], WRIST_MOTOR_NAMES[i]);
            wristTargetRPMs[i] = 0.0f;
        }
    }

    template <class... Args>
    void ForAllWristMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& wristMotor : wristMotors) {
            (wristMotor->*func)(args...);
        }
    }

    template <class... Args>
    void ForAllWristMotors(void (WristSubsystem::*func)(uint8_t WristIdx, Args...), Args... args) {
        for (uint8_t i = 0; i < WRIST_MOTOR_COUNT; i++) {
            (this->*func)(i, args...);
        }
    }

    template <class... Args>
    void ForWristMotorGroup(WristGroup groupID, void (WristSubsystem::*func)(uint8_t WristIdx, Args...), Args... args) {
        for (uint8_t i = 0; i < WRIST_MOTOR_COUNT; i++) {
            if (groupID == ALL || WRIST_MOTOR_GROUPS[i] == groupID) {
                (this->*func)(i, args...);
            }
        }
    }

    void BuildPIDControllers() {
        for (auto i = 0; i < WRIST_MOTOR_COUNT; i++) {
            wristVelocityPIDs[i] = new SmoothPID(WRIST_VELOCITY_PID_CONFIG);
        }
    }

    mockable void initialize() override;
    mockable void refresh() override;

    //  float getMotorSpeed(FeederIdx feederIdx) const;

    inline bool isOnline() const {
        bool wristOnline = false;

        for (auto& wristMotor : wristMotors) {
            if (wristMotor->isMotorOnline()) {
                wristOnline = true;
            }
        }

        return wristOnline;
    }

    inline bool isWristMotorOnline(uint8_t WristIdx) const {
        if (WristIdx >= WRIST_MOTOR_COUNT) {
            return false;
        }
        return wristMotors[WristIdx]->isMotorOnline();
    }

    void updateMotorVelocityPID(uint8_t WristIdxIdx);

    void setDesiredWristMotorOutput(uint8_t WristIdx, float output) { desiredWristMotorOutputs[WristIdx] = output; }

    void setAllDesiredWristMotorOutputs(uint16_t output) { desiredWristMotorOutputs.fill(output); }

    void setTargetRPM(uint8_t WristIdx, float rpm);

    float getCurrentRPM(uint8_t WristIdx = 0) const {
        return (wristMotors[wristIdx]->isMotorOnline()) ? -wristMotors[wristIdx]->getShaftRPM() : 0;
    }

    inline int16_t getWristMotorTorque(uint8_t wristIdx = 0) const {
        return wristMotors[wristIdx]->isMotorOnline() ? wristMotors[wristIdx]->getTorque() : 0;
    }

    bool getPressed();

    // float getCurrentRPM() const { return feederMotor.getShaftRPM(); }

    int64_t getEncoderUnwrapped(uint8_t wristIdx = 0) const {
        //TODO Need to use rev encoders
        return 0;
    }

    int getTotalLimitCount() const;

    // Group commands: These are to be used with ForFeederMotorGroup
    void activateWristMotor(uint8_t WristIdx = 0) {
        if (!wristCustomSpeedActive[wristIdx]) {  // Ignore the default RPM for a custom speed setting
            setTargetRPM(wristIdx, WRIST_NORMAL_RPMS[wristIdx]);
        }
    }

    inline void setWristCustomStatus(uint8_t WristIdx, bool customStatus) {
        wristCustomSpeedActive[wristIdx] = customStatus;
    }

    void setWristCustomMulti(uint8_t WristIdx, float mul) {
        if (mul == 1.0) {
            setWristCustomStatus(WristIdx, false);
        } else {
            setWristCustomStatus(WristIdx, true);
            setTargetRPM(WristIdx, WRIST_NORMAL_RPMS[WristIdx] * mul);
        }
    }

    void deactivateWristMotor(uint8_t WristIdx = 0) { setTargetRPM(WristIdx, 0.0f); }

    void unjamWristMotor(uint8_t WristIdx = 0) { setTargetRPM(WristIdx, -abs(WRIST_UNJAM_RPMS[WristIdx])); }

    vector<double> diffE(double desiredPitchAngle, double desiredYawAngle, double currAngleL, double currAngleR) {
        double directionL = 1;
        double directionR = -1;

        double yaw = (directionL * currAngleL + directionR * currAngleR) / 2.0;
        double pitch = (directionL * currAngleL - directionR * currAngleR) / 2.0;

        double e_yaw   = desiredYawAngle - yaw;
        double e_pitch = desiredPitchAngle - pitch;

        double thetaL_target = directionL * (e_yaw + e_pitch);
        double thetaR_target = directionR * (e_yaw - e_pitch);

        return vector<double> {thetaL_target, thetaR_target};
    }

private:
    src::Drivers* drivers;

    std::array<DJIMotor*, WRIST_MOTOR_COUNT> wrisstMotors;

    std::array<float, WRIST_MOTOR_COUNT> desiredWristMotorOutputs;

    void setDesiredOutputToWristMotor(uint8_t WristIdx);

    // float targetRPM;
    // float desiredOutput;
    std::array<float, WRIST_MOTOR_COUNT> wristTargetRPMs;

    std::array<SmoothPID*, WRIST_MOTOR_COUNT> wristVelocityPIDs;

    std::array<bool, WRIST_MOTOR_COUNT> wrCustomSpeedActive;
    // DJIMotor feederMotor;

    src::Informants::LimitSwitch limitSwitch;
    //#endif

    // commands
};

}  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE