#include "informants/transformers/coordinate_frame.hpp"

#include "utils/math/transform_setup.hpp"
#include "utils/robot_specific_inc.hpp"

using namespace src::Utils::MatrixHelper;

namespace src::Informants::Transformers {

CoordinateFrame::CoordinateFrame() {
    this->orientation = Matrix3f::identityMatrix();
    this->origin = Vector3f{0, 0, 0};
    updateTransform();
}

// Constructor
CoordinateFrame::CoordinateFrame(Matrix3f orientation, Vector3f origin) : orientation(orientation), origin(origin) {
    updateTransform();
}

// Update Transforms in/out
void CoordinateFrame::updateTransform() {
    this->transformOut = initTransform(this->orientation, this->origin);
    this->transformIn = invertTransform(this->transformOut);
}

// Set absolute orientation/rotation from ground frame
void CoordinateFrame::setOrientation(Matrix3f R) {
    this->orientation = R;
    updateTransform();
}

// Set relative orientation/rotation from current orientation
void CoordinateFrame::rotateFrame(Matrix3f R) {
    this->orientation = this->orientation * R;
    updateTransform();
}

// Set relative location/position from current position
void CoordinateFrame::displaceOrigin(Vector3f r) {
    this->origin = this->origin + r;
    updateTransform();
}

// Set absolute location/position from ground frame
void CoordinateFrame::setOrigin(Vector3f r) {
    this->origin = r;
    updateTransform();
}

// Getters
Vector3f CoordinateFrame::getOrigin() { return this->origin; }
Matrix3f CoordinateFrame::getOrientation() { return this->orientation; }
Matrix4f CoordinateFrame::getTransformIn() { return this->transformIn; }
Matrix4f CoordinateFrame::getTransformOut() { return this->transformOut; }
Matrix4f CoordinateFrame::getTransformToFrame(CoordinateFrame& f) { return f.getTransformIn() * this->transformOut; }

// Returns a point in the given frame that is stored in the current frame
Vector3f CoordinateFrame::getPointInFrame(CoordinateFrame& f, Vector3f v) {
    return homogenousCoordinateCrop(getTransformToFrame(f) * homogenousCoordinateExtend(v).asMatrix());
}

}  // namespace src::Informants::Transformers