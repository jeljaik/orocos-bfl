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
                
                cout << "Real part: " << r << " From Eigen: " << quat.w() << endl;
                
                //NOTE Reference 2 From quaternion to Euler Angles, when moving from the inertial frame to the body frame rotating first yaw, then pitch and finally roll http://www.chrobotics.com/library/understanding-quaternions
                output(1) = atan2( 2*y*z + 2*x*r , (1 - 2*pow(x,2) - 2*pow(y,2)) );
                output(2) = asin( -2*x*z + 2*y*r);
                output(3) = atan2( 2*x*y + 2*z*r , (1 - 2*pow(y,2) - 2*pow(z,2)) );
                
                // Using REF: http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
                // This should coincide with Eigen as per my tests
//                output(1) = atan2(-2*x*y + 2*z*r, 1 - 2*pow(y,2) - 2*pow(z,2));
//                output(2) = asin( 2*x*z + 2*y*r);
//                output(3) = atan2(-2*z*y + 2*x*r, 1 - 2*pow(x,2) - 2*pow(y,2));
                
                std::cerr << "Current quaternion: " << std::endl << quat << std::endl;

                
                Eigen::Matrix3f matRec;
                matRec = Eigen::AngleAxisf(output(1), Eigen::Vector3f::UnitX())
                       * Eigen::AngleAxisf(output(2), Eigen::Vector3f::UnitY())
                       * Eigen::AngleAxisf(output(3), Eigen::Vector3f::UnitZ());
                
                Eigen::Matrix3d m = quat.toRotationMatrix();
                std::cerr << "[Quaternion_Wrapper::getEulerAngles] Rotation Matrix reconstructed from my euler angles: " << std::endl << matRec << std::endl;
                std::cerr << "[Quaternion_Wrapper::getEulerAngles] Rotation Matrix using Eigen" << std::endl << m <<  std::endl;
                Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);
                
//                std::cerr << "[Quaternion_Wrapper::getEulerAngles] Computation through Eigen: " << euler << std::endl;
                
                // Reconstructing rotation matrix
                Eigen::Matrix3f rec;
                rec = Eigen::AngleAxisf(euler[0], Eigen::Vector3f::UnitX())
                    * Eigen::AngleAxisf(euler[1], Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(euler[2], Eigen::Vector3f::UnitZ());
//                std::cerr << "[Quaternion_Wrapper::getEulerAngles] Reconstructed rotation matrix: " << std::endl << rec << std::endl;
//                std::cerr << "[Quaternion_Wrapper::getEulerAngles] Error in rotation matrices : " << std::endl << rec.cast<double>() - m << std::endl;
                MatrixWrapper::ColumnVector tmp(euler.data(), 3);
//                std::cerr << "[Quaternion_Wrapper::getEulerAngles] Eigen Vector  : " << std::endl << euler << std::endl;
//                output  = tmp;
                
                
                
                
                // New tests
                std::cerr << "[NEW_TEST] Current quaternion: " << quat << endl;
                m = quat.toRotationMatrix();
                euler = m.eulerAngles(0, 1, 2);
                std::cerr << "[NEW_TEST] EIGEN (Deg): roll(x): " << (180/M_PI)*euler(0) << " pitch(y): " << (180/M_PI)*euler(1) << " yaw(z): " << (180/M_PI)*euler(2) << endl;
//                MatrixWrapper::ColumnVector resMW(*res);
                MatrixWrapper::ColumnVector resQuat(euler.data(),3);
//                output = resMW;
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
