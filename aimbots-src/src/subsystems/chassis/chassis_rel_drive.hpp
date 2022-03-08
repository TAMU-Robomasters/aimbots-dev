#pragma once

#include <drivers.hpp>
#include <subsystems/chassis/chassis.hpp>

namespace src::Chassis::Movement {

    void calculateChassisRelativeTargets(
        src::Drivers * drivers,
        ChassisSubsystem * chassis,
        float moveX,
        float moveY,
        float rotate);

    void onExecute(src::Drivers * drivers, ChassisSubsystem * chassis);
}  //namespace src::Chassis::Movement