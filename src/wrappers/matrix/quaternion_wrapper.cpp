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
        
        // Using Eigen native methods
        Eigen::Matrix3d m = (*(MyQuaternion*)this).toRotationMatrix();
//        std::cerr << "[Quaternion_wrapper::getRotation] Rotation matrix from quaternion is: " << m << std::endl;
        MatrixWrapper::Matrix tmp((EigenMatrix) m);
        Q = tmp;        
//        std::cerr << "[Quaternion_wrapper::getRotation] Rotation matrix old style is: " << Q << std::endl;
        
        return ret;
    }
    
} // end of namespace
