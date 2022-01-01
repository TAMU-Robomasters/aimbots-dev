#include "utils/common_types.hpp"

#include "utils/math/swerve_drive.hpp"
#include "utils/math/math.cpp"
#include <math.h> 

namespace SwerveDrive
{
    TapMatrix<4,2> caluclateSwerve(float dx, float dy, float dtheta){
        TapMatrix<4,2> power;

            const float c[3 * 1] = {
        dx,
        dy,
        dtheta
    
        };
        TapMatrix <3,1> cx;
        cx.copyData(c);

        float l =30.0f, w = 30.0f;

        const float test[8*3] = {
            1,0,-l,
            0,1,w,
            1,0,-l,
            0,1,w,
            1,0,-l,
            0,1,w,
            1,0,-l,
            0,1,w,
        };

        //https://www.chiefdelphi.com/t/swerve-drive-direct-and-reverse-kinematics/395803/3
        //what the math is based upon ^^^

        TapMatrix <8,3> g;
        // float theta = 0
        g.copyData(test);

        //     const float r[3*3] = {
        //     cos(theta), -sin(theta), 0,
        //     sin(theta), cos(theta), 0,
        //     0,          0,          1
        // };

        // TapMatrix <3,3> R(r);

        TapMatrix<8,1> velocity = g * cx;

        TapMatrix<4,2> velocityOrganized;
        for (size_t i = 0; i < 8; i+=2)
        {
            velocityOrganized.data[i] = velocity.data[i];
            velocityOrganized.data[i+1] = velocity.data[i+1];

        }
        
        TapMatrix<4,2> power_heading;
        for (size_t i = 0; i < 8; i+=2)
        {
            power_heading.data[i] = sqrt(velocity.data[i] * velocity.data[i] + velocity.data[i+1] * velocity.data[i+1]);
            power_heading.data[i+1] = atan2 (velocity.data[i+1], velocity.data[i]);
        }

        // for loop that get the largerst value of the matrix
        float max = 0;
        for (size_t i = 0; i < 4; i++)
        {
            if (power_heading.data[i*2] > max)
            {
                max = power_heading.data[i*2];
            }
        }

        for (size_t i = 0; i < 4; i++)
        {
            power_heading.data[i*2] = power_heading.data[i*2] / max;
        }

        return power_heading;
    }

} // namespace SwerveDrive
