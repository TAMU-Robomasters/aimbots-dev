#pragma once

#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"

#include "chassis.hpp"
#include "chassis_helper.hpp"
#include "drivers.hpp"

#include "informants/pathfinding/graph.hpp"


#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

class ChassisAutoNavCommand : public TapCommand {
public:
    ChassisAutoNavCommand(
        src::Drivers* drivers,
        ChassisSubsystem* chassis,
        SmoothPIDConfig linearPIDConfig,
        SmoothPIDConfig rotationPIDConfig,
        const SnapSymmetryConfig& snapSymmetryConfig = SnapSymmetryConfig(),
        float linearSettledThreshold = 0.05f,
        float angularSettledThreshold = modm::toRadian(0.5f),
        );
    ~ChassisAutoNavCommand() = default;

    void initialize() override;
    void execute() override;
    void setTargetLocation(const modm::Location2D<float>& targetLocation) {
        autoNavigator.setTargetLocation(targetLocation);
    }

    bool isSettled() {
        return xController.isSettled(linearSettledThreshold, 0) && yController.isSettled(linearSettledThreshold, 0) &&
               rotationController.isSettled(angularSettledThreshold, 0);
    }

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    //load series of points into navigation command
    void load_path(vector<Vector2f> path){
        chassis->setTargetRPMs(0.0f, 0.0f, 0.0f); //halt motion
        this->path = path;
        pop_path(); //put new point into auto navigator
    }

    //loads a new point into auto navigator from path
    void pop_path(){

        //could be wierd pointer stuff at play i dont know what im doing i fear
        Vector2f pt = path.front();
        autoNavigator.setTargetLocation(modm::Location2D<float>(pt));
        path.erase(path.front());
    }

    const char* getName() const override { return "Chassis Auto Nav"; }

private:
    vector<Vector2f> path;
    src::Drivers* drivers;
    ChassisSubsystem* chassis;

    const SnapSymmetryConfig& snapSymmetryConfig;

    AutoNav::AutoNavigatorHolonomic autoNavigator;

    SmoothPID xController;
    SmoothPID yController;
    SmoothPID rotationController;

    tap::algorithms::Ramp xRamp;
    tap::algorithms::Ramp yRamp;
    tap::algorithms::Ramp rotationRamp;

    float linearVelocityRampValue = 1.0f;
    float rotationVelocityRampValue = modm::toRadian(1.0f / 500);

    float linearSettledThreshold;
    float angularSettledThreshold;
};

static constexpr SmoothPIDConfig defaultLinearConfig = {
    .kp = 2.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.1f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig defaultRotationConfig = {
    .kp = 1.25f,
    .ki = 0.0f,
    .kd = 0.25f,
    .maxICumulative = 0.1f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE