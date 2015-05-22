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
                ret = true;
            } else
                std::cout << "[quaternion_wrapper.cpp] Computation for axis order " << axes << " has not been implemented yet" <<  std::endl;
        }
        return ret;
    }
    
} // end of namespace