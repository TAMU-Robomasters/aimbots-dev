#include "utils/common_types.hpp"

#include "utils/math/swerve_drive.hpp"
#include "utils/math/math.cpp"

namespace SwerveDrive
{
    Matrix<float,4,2> caluclateSwerve(float dx, float dy, float dtheta){
        // float Vx_1, Vy_1, Vx_2, Vy_2, Vx_3, Vy_3, Vx_4, Vy_4;
        // float Vw_1, Vw_2, Vw_3, Vw_4;

        // Vx_1 = Vx_2 = Vx_3 = Vx_4 = 1.0f;
        // Vy_1 = Vy_2 = Vy_3 = Vy_4 = 1.0f;

        // Vw_1 = Vw_2 = Vw_3 = Vw_4 = 1 / theta;

        Matrix<float,4,2> power;

        const float c[3] = {
            dx,
            dy,
            dtheta
        };
        Matrix <float,3,1> cx(c);

        float l =1.0f, w = 1.0f;

        const float test[6] = {
            1,0,l,
            0,1,w
        };

        //https://www.chiefdelphi.com/t/swerve-drive-direct-and-reverse-kinematics/395803/3
        //what the math is based upon ^^^

        Matrix <float,2,3> g(test);

        Matrix<float,2,1> v1Vel = g * cx;

        // Matrix<float,1,2> v1vel_t = math::transposeAll(v1Vel);

        Matrix<float,4,2> power_heading;

        // float velocity = modm::Matrix::norm(v1Vel);
        // power_heading.set(0,0,);




        // power[0][0] = sqrt((Vy_1-Vw_1*cos(theta))*(Vy_1-Vw_1*cos(theta)) + (Vx_1+Vw_1*sin(theta))*(Vx_1+Vw_1*sin(theta)));
    }

} // namespace SwerveDrive
