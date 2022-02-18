#include "chassis_rel_drive.hpp"

namespace src::Chassis::Movement {

void calculateChassisRelativeTargets(src::Drivers* drivers,
                                     ChassisSubsystem* chassis,
                                     float moveX,
                                     float moveY,
                                     float rotate) {
    Matrix<float, 3, 1> c;
    c[0][1] = moveX;
    c[1][1] = moveY;
    c[2][1] = rotate;
}

void onExecute(src::Drivers* drivers, ChassisSubsystem* chassis) {
    // put code here too ig... (yeah)
    // take input from user

    // calculate chassis relative targets

    // set chassis targets using setDesiredOutputs
    chassis->setDesiredOutputs(0, 0, 0);
}
}  // namespace src::Chassis::Movement