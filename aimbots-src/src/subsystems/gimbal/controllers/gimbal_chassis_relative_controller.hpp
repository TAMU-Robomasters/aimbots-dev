#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/controllers/gimbal_controller_interface.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <utils/common_types.hpp>
#include <utils/pid/smooth_pid_wrap.hpp>
#ifdef GIMBAL_COMPATIBLE

namespace src::Gimbal {

class GimbalChassisRelativeController : public GimbalControllerInterface {
public:
    GimbalChassisRelativeController(GimbalSubsystem*);

    void initialize() override;

    void BuildPIDControllers() {
        for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
            yawPositionPIDs[i] = new SmoothPID(YAW_POSITION_PID_CONFIG);
        }
        for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
            pitchPositionPIDs[i] = new SmoothPID(PITCH_POSITION_PID_CONFIG);
        }
    }

    void runYawController(std::optional<float> velocityLimit = std::nullopt) override;
    void runPitchController(std::optional<float> velocityLimit = std::nullopt) override;

    bool allOnlineYawControllersSettled(float errTolerance, uint32_t errTimeout) {
        bool controllersSettled = false;
        for (int i = 0; i < YAW_MOTOR_COUNT; i++) {
            if (gimbal->isYawMotorOnline(i)) {
                controllersSettled = yawPositionPIDs[i]->isSettled(errTolerance, errTimeout);
            }
        }
        return controllersSettled;
    }

    bool allOnlinePitchControllersSettled(float errTolerance, uint32_t errTimeout) {
        bool controllersSettled = false;
        for (int i = 0; i < PITCH_MOTOR_COUNT; i++) {
            if (gimbal->isPitchMotorOnline(i)) {
                controllersSettled = pitchPositionPIDs[i]->isSettled(errTolerance, errTimeout);
            }
        }
        return controllersSettled;
    }

    inline SmoothPID* getYawPositionPID() { return yawPositionPIDs[0]; }
    inline SmoothPID* getPitchPositionPID() { return pitchPositionPIDs[0]; }

    bool isOnline() const;

    void setTargetYaw(AngleUnit unit, float targetYaw) override { gimbal->setTargetYawAxisAngle(unit, targetYaw); }
    void setTargetPitch(AngleUnit unit, float targetPitch) override { gimbal->setTargetPitchAxisAngle(unit, targetPitch); }

    float getTargetYaw(AngleUnit unit) const override { return gimbal->getTargetYawAxisAngle(unit); }
    float getTargetPitch(AngleUnit unit) const override { return gimbal->getTargetPitchAxisAngle(unit); }

private:
    GimbalSubsystem* gimbal;

    std::array<SmoothPID*, YAW_MOTOR_COUNT> yawPositionPIDs;
    std::array<SmoothPID*, PITCH_MOTOR_COUNT> pitchPositionPIDs;
};

}  // namespace src::Gimbal
#endif