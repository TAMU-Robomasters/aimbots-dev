#pragma once

#include "informants/pathfinding/Viz_Graph.h"
#include "informants/pathfinding/graph.hpp"
#include "utils/motion/auto_nav/auto_navigator_holonomic.hpp"

#include "chassis.hpp"
#include "chassis_helper.hpp"
#include "drivers.hpp"

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
        float angularSettledThreshold = modm::toRadian(0.5f));
    ~ChassisAutoNavCommand() = default;

    void initialize() override;
    void execute() override;
    void setTargetLocation(double x, double y);

    bool isSettled() {
        return xController.isSettled(linearSettledThreshold, 0) && yController.isSettled(linearSettledThreshold, 0);  // &&
        //         rotationController.isSettled(angularSettledThreshold, 0);
    }

    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

    // load series of points into navigation command
    void load_path(vector<Point> path);

    // loads a new point into auto navigator from path
    void pop_path();

    const char* getName() const override { return "Chassis Auto Nav"; }

private:
    vector<Point> path;
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

    float radius = 0.381;
    vector<Point> redWallHor = {
        Point(0, 3.05 - radius),
        Point(1.625 + radius, 3.05 - radius),
        Point(1.625 + radius, 3.074 + radius),
        Point(0, 3.074 + radius)};

    vector<Point> redWallVert = {
        Point(3.079 - radius, 0),
        Point(3.079 - radius, 1.625 + radius),
        Point(3.079 + radius, 1.625 + radius),
        Point(3.079 + radius, 0)};

    vector<Point> redDoohickey = {
        Point(1 - radius, 1 - radius),
        Point(1 - radius, 2 + radius),
        Point(2 + radius, 2 + radius),
        Point(2 + radius, 1 - radius)};

    vector<Point> centerLeftWall = {
        Point(5 - radius, 2.8 - radius),
        Point(5 - radius, 6 + radius),
        Point(5 + radius, 6 + radius),
        Point(5 + radius, 2.8 - radius)};

    vector<Point> blueWallHor = {
        Point(12 - 0, 8 - (3.05 - radius)),
        Point(12 - (1.625 + radius), 8 - (3.05 - radius)),
        Point(12 - (1.625 + radius), 8 - (3.074 + radius)),
        Point(12 - 0, 8 - (3.074 + radius))};

    vector<Point> blueWallVert = {
        Point(12 - (3.079 - radius), 8 - 0),
        Point(12 - (3.079 - radius), 8 - (1.625 + radius)),
        Point(12 - (3.079 + radius), 8 - (1.625 + radius)),
        Point(12 - (3.079 + radius), 8 - 0)};

    vector<Point> blueDoohickey = {
        Point(12 - (1 - radius), 8 - (1 - radius)),
        Point(12 - (1 - radius), 8 - (2 + radius)),
        Point(12 - (2 + radius), 8 - (2 + radius)),
        Point(12 - (2 + radius), 8 - (1 - radius))};

    vector<Point> centerRightWall = {
        Point(12 - (4.5 - radius), 8 - (2.8 - radius)),
        Point(12 - (4.5 - radius), 8 - (6 + radius)),
        Point(12 - (4.5 + radius), 8 - (6 + radius)),
        Point(12 - (4.5 + radius), 8 - (2.8 - radius))};

    vector<vector<Point>> polygons =
        {redWallHor, redWallVert, redDoohickey, centerLeftWall, blueWallHor, blueWallVert, blueDoohickey, centerRightWall};

    float linearSettledThreshold;
    float angularSettledThreshold;
    VizGraph pathfinder = constructVizGraph(polygons);
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

#endif  //#ifdef CHASSIS_COMPATIBLE