#include "informants/transformers/cartesian_frames.hpp"
#include "utils/math/transformations.hpp"
#include "utils/robot_specific_inc.hpp"

using namespace src::Utils::MatrixOperations;

namespace src::Informants::Transformers {

CartesianFrame::CartesianFrame() {
    this->orientation = Matrix3f::identityMatrix();
    this->origin = Vector3f{0, 0, 0};
    updateTransform();
}

CartesianFrame::CartesianFrame(Matrix3f orientation, Vector3f origin) : orientation(orientation), origin(origin) {
    updateTransform();
}

// Update Transforms in/out
void CartesianFrame::updateTransform() {
    this->transformOut = initialization(this->orientation, this->origin);
    this->transformIn = invertTransform(this->transformOut);
}

// Set absolute orientation/rotation from ground frame
void CartesianFrame::setOrientation(Matrix3f R) {
    this->orientation = R;
    updateTransform();
}

// Set relative orientation/rotation from current orientation
void CartesianFrame::rotateFrame(Matrix3f R) {
    this->orientation = this->orientation * R;
    updateTransform();
}

// Set relative location/position from current position
void CartesianFrame::displaceOrigin(Vector3f r) {
    //this->origin = this->origin + r;
    this->origin = translate(origin, r);
    updateTransform();
}

// Set absolute location/position from ground frame
void CartesianFrame::setOrigin(Vector3f r) {
    this->origin = r;
    updateTransform();
}

// Getters
Vector3f& CartesianFrame::getOrigin() { return this->origin; }
Matrix3f& CartesianFrame::getOrientation() { return this->orientation; }
const Matrix4f& CartesianFrame::getTransformIn() { return this->transformIn; }
const Matrix4f& CartesianFrame::getTransformOut() { return this->transformOut; }

Matrix4f CartesianFrame::getTransformToFrame(CartesianFrame& f) { return f.getTransformIn() * this->transformOut; } // change of basis formula

Matrix4f transformedFrame(CartesianFrame& f) { // I made this bc I CANT FKN FIND THE ORIGINAL TRANSFORMATION
    //rotate first
    Matrix3f orient = f.getOrientation();
    Vector3f pos = f.getOrigin();

    // this->orientation * this->origin;
    //idk what im doing help

}

// Returns a point in the given frame that is stored in the current frame - THIS IS JUST CHANGE OF BASIS
Vector3f CartesianFrame::getPointInFrame(CartesianFrame& f, Vector3f& v) {
    return homogenousCoordinateCrop(getTransformToFrame(f) * homogenousCoordinateExtend(v).asMatrix());
} 
// ^ WHAT IS HAPPENING


}