#include "quaternion_wrapper.h"
#include <math.h>

using namespace std;

namespace MatrixWrapper {
    bool Quaternion_Wrapper::getEulerAngles(std::string axes, MatrixWrapper::ColumnVector& output) {
        bool ret = false;
        //  Normalize quaternion
        MyQuaternion& quat = (*(MyQuaternion*)this);
        quat.normalize();
        if (!(output.rows() == 3))
            ret = false;
        else {
            if (!axes.compare("xyz")) {
                double r = quat(1);
                double x = quat(2);
                double y = quat(3);
                double z = quat(4);
                output(1) = atan2(2*y*z + 2*x*r, 1 - 2*pow(x,2) - 2*pow(y,2));
                output(2) = asin(-2*x*z + 2*y*r);
                output(3) = atan2(2*x*y + 2*z*r, 1 - 2*pow(y,2) - 2*pow(z,2));
                // NOTE Reference http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
//                 output(1) = atan2(-2*x*y - 2*z*r, 1 - 2*pow(y,2) - 2*pow(z,2));
//                 output(2) = asin(-2*x*z + 2*y*r);
//                 output(3) = atan2(-2*z*y + 2*x*r, 1 - 2*pow(x,2) - 2*pow(y,2));
                ret = true;
            } else
                std::cout << "[quaternion_wrapper.cpp] Computation for axis order " << axes << " has not been implemented yet" <<  std::endl;
        }
        return ret;
    }
    
    bool Quaternion_Wrapper::getRotation ( MatrixWrapper::Matrix& Q ) {
        
        bool ret = true;
        if (Q.rows() != 3 && Q.columns() != 3) {
            std::cout << " ERROR [quaternion_wrapper.cpp] Rotation matrix passed to this method must be 3x3" << std::endl;
            ret = false;
        }
        double q0 = (*this)(1);
        double q1 = (*this)(2);
        double q2 = (*this)(3);
        double q3 = (*this)(4);
        
        Q(1,1) = 2*(q0*q0 + q1*q1) - 1; Q(1,2) = 2*(q1*q2 - q0*q3)     ; Q(1,3) = 2*(q1*q3 + q0*q2)    ;
        Q(2,1) = 2*(q1*q2 + q0*q3)    ; Q(2,2) = 2*(q0*q0 + q2*q2) - 1 ; Q(2,3) = 2*(q2*q3 - q0*q1)    ;
        Q(3,1) = 2*(q1*q3 - q0*q2)    ; Q(3,2) = 2*(q2*q3 + q0*q1)     ; Q(3,3) = 2*(q0*q0 + q3*q3) - 1;
        
        return ret;
    }
    
} // end of namespace
