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
                // New tests
//                std::cerr << "[NEW_TEST] Current quaternion: " << quat << endl;
                Eigen::Matrix3d m = quat.toRotationMatrix();
                Eigen::Vector3d euler = m.eulerAngles(2, 1, 0);
//                std::cerr << "[NEW_TEST] EIGEN (2,1,0) (Deg): roll(x): " << (180/M_PI)*euler(2) << " pitch(y): " << (180/M_PI)*euler(1) << " yaw(z): " << (180/M_PI)*euler(0) << endl;
                MatrixWrapper::ColumnVector resQuat(euler.data(),3);
//                output = resMW;
                // Inverting roll and yaw, because the transformation was 2, 1, 0, thus euler(0) is yaw, euler(1) pitch and euler(2) but my output is always roll, pitch, yaw
                double tmpYaw = resQuat(1);
                resQuat(1) = resQuat(3);
                resQuat(3) = tmpYaw;
                
                // This is necessary since when the quaternion is close to the identity "jumps" of (+/-)180 can be experienced in the converted Euler angles.
                double tolDiff = (M_PI/180)*(20);
                if ( abs(resQuat(1) - output(1)) > tolDiff )
                {
                    resQuat(1) = output(1);
                }
                if ( abs(resQuat(2) - output(2)) > tolDiff )
                {
                    resQuat(2) = output(2);
                }

                output = resQuat;
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
        
        
        //DEBUGGING...
//        Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);
//        Eigen::Matrix3f rec;
//        rec = Eigen::AngleAxisf(euler[0], Eigen::Vector3f::UnitX())
//        * Eigen::AngleAxisf(euler[1], Eigen::Vector3f::UnitY())
//        * Eigen::AngleAxisf(euler[2], Eigen::Vector3f::UnitZ());
//        std::cerr << "[Quaternion_Wrapper::getEulerAngles] Reconstructed rotation matrix: " << rec << std::endl << std::endl;

        return ret;
    }
    
    void Quaternion_Wrapper::quaternion2euler (double res[], std::string axes)
    {
        //  Normalize quaternion
        MyQuaternion& quat = (*(MyQuaternion*)this);
        quat.normalize();
        // Mapping elements of the quaternion
        double w = quat(1);
        double x = quat(2);
        double y = quat(3);
        double z = quat(4);
        
        
        if (!axes.compare("xyz"))
        {
            threeaxisrot( -2*(y*z - w*x),
                         w*w - x*x - y*y + z*z,
                         2*(x*z + w*y),
                         -2*(x*y - w*z),
                         w*w + x*x - y*y - z*z,
                         res);
            
        }
    }
    
    void Quaternion_Wrapper::threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[])
    {
        res[0] = atan2( r31, r32 );
        res[1] = asin ( r21 );
        res[2] = atan2( r11, r12 );
    }
    
    
    
    
} // end of namespace
