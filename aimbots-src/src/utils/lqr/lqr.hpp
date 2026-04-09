#pragma once

#include <vector>

struct LQRConfig
{
    vector<float> Q;
    vector<float> R;
    int maxOutput = 0;
};

namespace stc::utils{

class lqr{
    public:



    private:
    LQRConfig config;
    
    float output = 0.0f;
    float prevError = 0.0f;
    float currError = 0.0f;

};
}