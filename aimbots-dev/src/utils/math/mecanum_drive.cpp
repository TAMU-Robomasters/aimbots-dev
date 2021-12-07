#include "utils/common_types.hpp"

namespace MecanumDrive{
    Matrix<float,4,3> createJacobian(float length, float width){
        const float j[12] = {
            -1,1,length+width,
            1, 1,-(length+width),
            -1,1,length+width,
            1, 1,-(length+width)
        };

        Matrix<float,4,3> jacobian(j);

        // {
        //     {-1,1, length+width     },
        //     {1 ,1, -(length+width)  },
        //     {-1,1, length+width     },
        //     {1, 1, -(length+width)  }
        // };
        return jacobian;
    };

    Matrix<float,3,3> createRotationMatrix(float theta){
        const float r[9] = {
            cos(theta), -sin(theta), 0,
            sin(theta), cos(theta), 0,
            0,          0,          1
        };
        
        Matrix<float,3,3> rotationMatrix(r);
        // {
        //     {cos(theta),-sin(theta),0},
        //     {sin(theta),cos(theta),0},
        //     {0,0,1}
        // };

        return rotationMatrix;
    };

    Matrix<float,4,1> mecanumKinematics(float dx, float dy, float dtheta, Matrix<float,4,3> jacobian, Matrix<float,3,3> rotatiion){
        const float c[3] = {
            dx,
            dy,
            dtheta
        };
        Matrix <float,3,1> cx(c);
        // cx = {
        //     {dx},
        //     {dy},
        //     {dtheta}
        // };
        
        Matrix<float,4,1> wheelVeloicty;

        wheelVeloicty = jacobian * rotatiion * cx;

        return wheelVeloicty;
    }

Matrix<float,4,1> mecanumKinematics(float dx, float dy, float dtheta, Matrix<float,4,3> jacobian){
        const float c[3] = {
            dx,
            dy,
            dtheta
        };
        Matrix <float,3,1> cx(c);
        // cx = {
        //     {dx},
        //     {dy},
        //     {dtheta}
        // };
        
        Matrix<float,4,1> wheelVeloicty;

        wheelVeloicty = jacobian * cx;

        return wheelVeloicty;
    }


}; // namespace MecanumDrive