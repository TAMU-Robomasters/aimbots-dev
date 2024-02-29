#pragma once
#include <vector>

#include "utils/common_types.hpp"
#include "utils/math/transform_setup.hpp"

namespace src::Informants::Transformers {

class CartesianFrame {
    public:
        //HEHEHE copy pasted from the old code (coordinate frame hpp) - EDIT AS NEEDED
        
        CartesanFrame();
        CartesanFrame(Matrix3f orientation, Vector3f origin);
        ~CartesanFrame() = default;

        void updateTransform();

        void setOrigin(Vector3f r);
        void displaceOrigin(Vector3f r);

        void setOrientation(Matrix3f R);
        void rotateFrame(Matrix3f R);

        Vector3f& getOrigin();
        Matrix3f& getOrientation();

        const Matrix4f& getTransformIn();
        const Matrix4f& getTransformOut();

        Matrix4f getTransformToFrame(CartesanFrame& f);
        Vector3f getPointInFrame(CartesanFrame& f, Vector3f& v);

    private:
        /**
         * rotation matrix of the origin
        */
        Matrix3f orientation;

        /**
        * the center of the respective frame (CHASSIS IS GROUND) -- ALL FRAMES ARE IN RESPECT TO GROUND
        **/
        Vector3f origin;

        Matrix4f transformIn;   // Takes inputs in ground frame
        Matrix4f transformOut;  // Returns inputs in ground frame
}




    
}