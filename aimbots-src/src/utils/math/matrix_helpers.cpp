#include "utils/common_types.hpp"

Matrix<float, 3, 3> xy_rotation_matrix(float angle) {
    float c = cosf(angle);
    float s = sinf(angle);
    static float xy_rotation_array[9] = {
        c, -s, 0.0f,
        s, c, 0.0f,
        0.0f, 0.0f, 1.0f};
    return Matrix<float, 3, 3>(xy_rotation_array);
}