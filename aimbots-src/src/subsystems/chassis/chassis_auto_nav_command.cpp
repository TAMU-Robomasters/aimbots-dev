#include "chassis_auto_nav_command.hpp"
    
#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

ChassisAutoNavCommand::ChassisAutoNavCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    SmoothPIDConfig linearPIDConfig,
    SmoothPIDConfig rotationPIDConfig,
    const SnapSymmetryConfig& snapSymmetryConfig,
    float linearSettledThreshold,
    float angularSettledThreshold)
    : drivers(drivers),
      chassis(chassis),
      snapSymmetryConfig(snapSymmetryConfig),
      autoNavigator(),
      xController(linearPIDConfig),
      yController(linearPIDConfig),
      rotationController(rotationPIDConfig),
      linearSettledThreshold(linearSettledThreshold),
      angularSettledThreshold(angularSettledThreshold)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

float path_x = 0.0f;
float path_y = 0.0f;
float myX = 0.0f;
float myY = 0.0f;
bool settled = false;

void ChassisAutoNavCommand::initialize() {
    // modm::Location2D<float> targetLocation({0.5f, 0.5f}, modm::toRadian(90.0f));  // test
    // autoNavigator.setTargetLocation(targetLocation);

    // 15 inches assumed radius = 0.381 meters
    double radius = 0.381;
    vector<Point> redWallHor = {
        Point(0, 3.05 - radius),
        Point(1.625 + radius, 3.05 - radius),
        Point(1.625 + radius, 3.074 + radius),
        Point(0, 3.074 + radius)
    };

    vector<Point> redWallVert = {
        Point(3.079 - radius, 0),
        Point(3.079 - radius, 1.625 + radius),
        Point(3.079 + radius, 1.625 + radius),
        Point(3.079 + radius, 0)
    };

    vector<Point> redDoohickey = {
        Point(1 - radius, 1 - radius),
        Point(1 - radius, 2 + radius),
        Point(2 + radius, 2 + radius),
        Point(2 + radius, 1 - radius)
    };

    vector<Point> centerLeftWall = {
        Point(4.5 - radius, 2.8 - radius),
        Point(4.5 - radius, 6 + radius),
        Point(4.5 + radius, 6 + radius),
        Point(4.5 + radius, 2.8 - radius)
    };

    vector<Point> blueWallHor = {
        Point(12 - 0, 8 - (3.05 - radius)),
        Point(12 - (1.625 + radius), 8 - (3.05 - radius)),
        Point(12 - (1.625 + radius), 8 - (3.074 + radius)),
        Point(12 - 0, 8 - (3.074 + radius))
    };

    vector<Point> blueWallVert = {
        Point(12 - (3.079 - radius), 8 - 0),
        Point(12 - (3.079 - radius), 8 - (1.625 + radius)),
        Point(12 - (3.079 + radius), 8 - (1.625 + radius)),
        Point(12 - (3.079 + radius), 8 - 0)
    };

    vector<Point> blueDoohickey = {
        Point(12 - (1 - radius), 8 - (1 - radius)),
        Point(12 -(1 - radius), 8 - (2 + radius)),
        Point(12 -(2 + radius), 8 - (2 + radius)),
        Point(12 - (2 + radius), 8 - (1 - radius))
    };

    vector<Point> centerRightWall = {
        Point(12 - (4.5 - radius), 8 - (2.8 - radius)),
        Point(12 - (4.5 - radius), 8 - (6 + radius)),
        Point(12 - (4.5 + radius), 8 - (6 + radius)),
        Point(12 - (4.5 + radius), 8 - (2.8 - radius))
    };
    
    vector<vector<Point>> polygons = {
        redWallHor,
        redWallVert,
        redDoohickey,
        centerLeftWall,
        blueWallHor,
        blueWallVert,
        blueDoohickey,
        centerRightWall
    };


    
    VizGraph graph = constructVizGraph(polygons);
    

    vector<Point> path = graph.search(0.5, 0.5, 3, 3);
    this->load_path(path);
}

void ChassisAutoNavCommand::pop_path(){
        if (path.empty()) { return; }
        Point pt = path.front();
        path_x = pt.x;
        path_y = pt.x;
        autoNavigator.setTargetLocation(modm::Location2D<float>(pt.x, pt.y, 0));
        path.erase(path.begin());
    }

void ChassisAutoNavCommand::load_path(vector<Point> path){
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f); //halt motion
    this->path = path;
    pop_path(); //put new point into auto navigator
}

float rotationErrorDisplay = 0.0f;


void ChassisAutoNavCommand::execute() {
    float xError = 0.0f;
    float yError = 0.0f;
    float rotationError = 0.0f;
    settled = isSettled();

    //no points to load and controllers at target
    if (this->path.empty() && this->isSettled()){
        chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
        return; 
    }

    if (this->isSettled()){ //if controllers at target load new point
        this->pop_path();
    }
    
    modm::Location2D<float> currentWorldLocation = drivers->kinematicInformant.getRobotLocation2D();
    myX = currentWorldLocation.getX();
    myY = currentWorldLocation.getY();
    modm::Vector2f currentWorldVelocity = drivers->kinematicInformant.getRobotVelocity2D();

    autoNavigator.update(currentWorldLocation);
    autoNavigator.getDesiredInput(&xError, &yError, &rotationError);

    rotationErrorDisplay = rotationError;

    // findNearestChassisErrorTo expects a chassis-relative target angle, not an error, so we negate the error to get target
    rotationError = Helper::findNearestChassisErrorTo(-rotationError, snapSymmetryConfig);

    float desiredX = xController.runController(xError, currentWorldVelocity.getX());
    float desiredY = yController.runController(yError, currentWorldVelocity.getY());
    float desiredR = 0; 
    // float desiredR = rotationController.runController(
    //     rotationError,
    //     -RADPS_TO_RPM(drivers->kinematicInformant.getChassisIMUAngularVelocity(
    //         src::Informants::AngularAxis::YAW_AXIS,
    //         AngleUnit::Radians)));

    // Rotate world-relative desired input to chassis-relative desired input
    tap::algorithms::rotateVector(&desiredX, &desiredY, -currentWorldLocation.getOrientation());

    Helper::rescaleDesiredInputToPowerLimitedSpeeds(drivers, chassis, &desiredX, &desiredY, &desiredR);

    chassis->setTargetRPMs(desiredX, desiredY, desiredR);
}

bool ChassisAutoNavCommand::isReady() { return true; }

bool ChassisAutoNavCommand::isFinished() const { return false; }

void ChassisAutoNavCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE