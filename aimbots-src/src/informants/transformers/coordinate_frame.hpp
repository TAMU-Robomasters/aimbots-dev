#pragma once
#include <vector>

#include "utils/common_types.hpp"
#include "utils/math/transform_setup.hpp"

namespace src::Informants::Transformers {

class CoordinateFrame {
public:
    CoordinateFrame();
    CoordinateFrame(Matrix3f orientation, Vector3f origin);
    ~CoordinateFrame() = default;

    void updateTransform();

    void setOrigin(Vector3f r);
    void displaceOrigin(Vector3f r);

    void setOrientation(Matrix3f R);
    void rotateFrame(Matrix3f R);

    Vector3f getOrigin();
    Matrix3f getOrientation();

    Matrix4f getTransformIn();
    Matrix4f getTransformOut();

    Matrix4f getTransformToFrame(CoordinateFrame& f);

    void addPoint(Vector3f p);
    Vector3f getPoint(int n);
    Vector3f getPointInFrame(CoordinateFrame& f, int n);

private:
    // Origin of the frame with respect to center of mass of the chassis
    Matrix3f orientation;
    Vector3f origin;

    // Orientation with respect to chassis frame
    // !!! Y is forward / out X is to the right, Z is up !!!

    std::vector<Vector3f> points;

    // Transforms in and out of Ground Frame (CHASSIS FRAME IS GROUND FRAME!!!)
    //  !!! I REPEAT: CHASSIS IS GROUND FRAME DUE TO BEING THE MOST RELIABLE FRAME !!!
    Matrix4f transformIn;   // Takes inputs in ground frame
    Matrix4f transformOut;  // Returns inputs in ground frame
};

}  // namespace src::Informants::Transformers