#include "utils/common_types.hpp"

namespace SwerveDrive
{
    Matrix<float,4,2> caluclateSwerve(float x, float y, float theta){
        float Vx_1, Vy_1, Vx_2, Vy_2, Vx_3, Vy_3, Vx_4, Vy_4;
        float Vw_1, Vw_2, Vw_3, Vw_4;

        Vx_1 = Vx_2 = Vx_3 = Vx_4 = 1.0f;
        Vy_1 = Vy_2 = Vy_3 = Vy_4 = 1.0f;

        Vw_1 = Vw_2 = Vw_3 = Vw_4 = 1 / theta;

        Matrix<float,4,2> power;

        power[0][0] = sqrt((Vy_1-Vw_1*cos(theta))*(Vy_1-Vw_1*cos(theta)) + (Vx_1+Vw_1*sin(theta))*(Vx_1+Vw_1*sin(theta)));
    }

} // namespace SwerveDrive
