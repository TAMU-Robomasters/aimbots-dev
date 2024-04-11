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

    void setTargetPositionMeters(float x);
    float getTargetPositionMeters() const;

    template<typename... Args>
    using CockerSubsystemFunc = void (CockerSubsystem::*)(Args...);

    void BuildCockerMotors() {
        for (auto i = 0; i < COCKER_MOTOR_COUNT; i++) {
            cockerMotors[i] =
                new DJIMotor(drivers, COCKER_MOTOR_IDS[i], COCKER_BUS, COCKER_MOTOR_DIRECTIONS[i], COCKER_MOTOR_NAMES[i]);
        }
    }
    
    template<class... Args>
    void ForAllCockerMotors(DJIMotorFunc<Args...> func, Args... args)
    {
        for (auto i = 0; i < COCKER_MOTOR_COUNT; i++)
            (cockerMotors[i].*func)(args...);
    }

    template<class... Args>
    void ForAllCockerMotors(CockerSubsystemFunc<MotorIndex, Args...> func, Args... args) {
        for (auto i = 0; i < COCKER_MOTOR_COUNT; i++) {
            auto mi = static_cast<MotorIndex>(i);
            (this->*func)(mi, args...);
        }
    }
    
    void updateMotorPositionPID();
    void refreshDesiredOutput();



private:
    src::Drivers* drivers;

    std::array<DJIMotor*, COCKER_MOTOR_COUNT> cockerMotors;
    std::array<SmoothPID, COCKER_MOTOR_COUNT> motorPIDs;
    std::array<int32_t, COCKER_MOTOR_COUNT> desiredOutputs {};
};

}  // namespace src::Indexer

#endif //#ifdef COCKER_COMPATIBLE
