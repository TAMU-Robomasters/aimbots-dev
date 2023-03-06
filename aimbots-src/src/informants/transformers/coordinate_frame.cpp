#include "informants/transformers/coordinate_frame.hpp"

#include "src/utils/robot_specific_inc.hpp"
#include "utils/math/transform_setup.hpp"

namespace src::Informants {

// Constructor
CoordinateFrame::CoordinateFrame(Vector3f origin, Matrix<float, 3, 3> orientation)
    : origin(origin),
      orientation(orientation) {
    updateTransform();
}

// Deconstructor I Guess
CoordinateFrame::~CoordinateFrame() {}

// Update Transforms in/out
void CoordinateFrame::updateTransform() {
    this->transformOut = src::Utils::MatrixHelper::initTransform(this->orientation, this->origin);
    this->transformIn = src::Utils::MatrixHelper::invertTransform(this->transformOut);
}
void CoordinateFrame::setOrientation(Matrix<float, 3, 3> R) {
    this->orientation = R;
    updateTransform();
}
void CoordinateFrame::rotateFrame(Matrix<float, 3, 3> R) {
    this->orientation = this->orientation * R;
    updateTransform();
}
void CoordinateFrame::moveOrigin(Vector3f r) {
    this->origin = this->origin + r;
    updateTransform();
}
void CoordinateFrame::setOrigin(Vector3f r) {
    this->origin = r;
    updateTransform();
}

// Getters
Vector3f CoordinateFrame::getOrigin() { return this->origin; }
Matrix<float, 3, 3> CoordinateFrame::getOrientation() { return this->orientation; }
Matrix<float, 4, 4> CoordinateFrame::getTransformIn() { return this->transformIn; }
Matrix<float, 4, 4> CoordinateFrame::getTransformOut() { return this->transformOut; }

// Returns a point in this frame
Vector3f CoordinateFrame::getPoint(int n) { this->points.at(n); }

// Add a point to a frame (to track) -> Point should be rigid w.r.t this frame
void CoordinateFrame::addPoint(Vector3f p) { this->points.push_back(p); }

// Returns a point in the given frame that is stored in the current frame
Vector3f CoordinateFrame::getPointInFrame(CoordinateFrame* f, int n) {
    src::Utils::MatrixHelper::cropCoords(
        f->transformIn * this->transformOut * src::Utils::MatrixHelper::extendCoords(getPoint(n)));
}

}  // namespace src::Informants