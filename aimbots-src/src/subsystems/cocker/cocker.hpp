#pragma once
#include "tap/control/subsystem.hpp"

#include "utils/robot_specific_inc.hpp"
#include "utils/common_types.hpp"
#include "drivers.hpp"

#ifdef COCKER_COMPATIBLE

namespace src::Cocker {

//wow
enum MotorIndex {
    ONE = 0,
    TWO = 1
};

//single axis position-controlled cocker with multiple synchronized motors

// this contains the subsystem AND commands
class CockerSubsystem : public tap::control::Subsystem {
public:
    CockerSubsystem(src::Drivers*);
    ~CockerSubsystem() = default;

    mockable void initialize() override;
    mockable void refresh() override;

    bool isOnline() const {
        for(int i = 0; i < COCKER_MOTOR_COUNT; i++)
        {
            if(!cockerMotors[i].isMotorOnline()) return false;
        }
        return true;
    }

    /**
    * Call a DJIMotor function on all cocker motors
    */
    template<class... Args>
    void ForAllCockerMotors(DJIMotorFunc<Args...> func, Args... args)
    {
        for (auto i = 0; i < COCKER_MOTOR_COUNT; i++)
            (cockerMotors[i].*func)(args...);
    }

    /**
     * Calls a CockerSubsystem function on all cocker motors
    */
    template<class... Args>
    void ForAllCockerMotors(void (CockerSubsystem::*func)(MotorIndex, Args...), Args... args) {
        for (auto i = 0; i < COCKER_MOTOR_COUNT; i++) {
            auto mi = static_cast<MotorIndex>(i);
            (this->*func)(mi, args...);
        }
    }
    
    void setTargetPositionMeters(MotorIndex motorIndex, float target) {targetPositionsMeters[motorIndex] = target;}
    float getTargetPositionMeters(MotorIndex motorIndex) const {return targetPositionsMeters[motorIndex];}
    
    void updateAllPIDs();
    void updateMotorPID(MotorIndex motorIndex);
    void setDesiredOutputToMotor(MotorIndex motorIndex) {if(cockerMotors[motorIndex].isMotorOnline()) cockerMotors[motorIndex].setDesiredOutput(desiredOutputs[motorIndex]);}

    float getCurrentPositionMeters(MotorIndex motorIndex) const;
    int16_t getMotorRPM(MotorIndex motorIndex) const {return cockerMotors[motorIndex].isMotorOnline() ? cockerMotors[motorIndex].getShaftRPM() : 0;}
private:
    src::Drivers* drivers;

    std::array<DJIMotor, COCKER_MOTOR_COUNT> cockerMotors;
    std::array<SmoothPID, COCKER_MOTOR_COUNT> motorPIDs;
    std::array<float, COCKER_MOTOR_COUNT> targetPositionsMeters;
    std::array<int32_t, COCKER_MOTOR_COUNT> desiredOutputs {};

    DJIMotor buildMotor(MotorIndex idx) {
        return DJIMotor(drivers, COCKER_MOTOR_IDS[idx], COCKER_BUS, COCKER_MOTOR_DIRECTIONS[idx], COCKER_MOTOR_NAMES[idx]);
    }
};

}  // namespace src::Indexer

#endif //#ifdef COCKER_COMPATIBLE
