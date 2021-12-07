#pragma once
#include "utils/common_types.hpp"

namespace MecanumDrive {
            Matrix<float,4,1> mecanumKinematics(float dx, float dy, float dtheta, Matrix<float,4,3> jacobian, Matrix<float,3,3> rotatiion);
            Matrix<float,4,1> mecanumKinematics(float dx, float dy, float dtheta, Matrix<float,4,3> jacobian);
            Matrix<float,4,3> createJacobian(float length, float width);
            Matrix<float,3,3> createRotationMatrix(float theta);
            
};
