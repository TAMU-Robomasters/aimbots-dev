#pragma once
#include "modm/math/geometry/vector.hpp"
#include "utils/common_types.hpp"
#include "utils/math/transform_setup.hpp"

namespace src::Informants {

class CoordinateFrame {
public:
    CoordinateFrame(Vector3f origin, Matrix<float, 3, 3> orientation);
    ~CoordinateFrame() = default;
    void updateTransform();
    void setOrientation(Matrix<float, 3, 3> R);
    void rotateFrame(Matrix<float, 3, 3> R);
    void moveOrigin(Vector3f r);
    void setOrigin(Vector3f r);

    Vector3f getOrigin();
    Matrix<float, 3, 3> getOrientation();
    Matrix<float, 4, 4> getTransformIn();
    Matrix<float, 4, 4> getTransformOut();

    void addPoint(Vector3f p);
    Vector3f getPoint(int n);
    Vector3f getPointInFrame(CoordinateFrame* f, int n);

private:
    // Origin of the frame with respect to center of mass of the chassis
    Vector3f origin;

    // Orientation with respect to chassis frame
    // !!! Y is forward / out X is to the right, Z is up !!!
    Matrix<float, 3, 3> orientation;

    vector<Vector3f> points;

    // Transforms in and out of Ground Frame (CHASSIS FRAME IS GROUND FRAME!!!)
    //  !!! I REPEAT: CHASSIS IS GROUND FRAME DUE TO BEING THE MOST RELIABLE FRAME !!!
    Matrix<float, 4, 4> transformIn;   // Takes inputs in ground frame
    Matrix<float, 4, 4> transformOut;  // Returns inputs in ground frame
};

}  // namespace src::Informants