// $Id: quaternion_wrapper.h 2015-05-20 jeljaik$
// Jorhabib Eljaik <first dot last at iit dot it>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#ifndef __QUATERNION_WRAPPER_H__
#define __QUATERNION_WRAPPER_H__

#include <string>

#define use_namespace
#define MyColumnVector MatrixWrapper::ColumnVector
#define MyRowVector MatrixWrapper::RowVector
#define MyMatrix MatrixWrapper::Matrix
#define MyQuaternion MatrixWrapper::Quaternion

namespace MatrixWrapper{
    class Matrix;
    class ColumnVector;
    class Quaternion;
    
    class Quaternion_Wrapper
    {
    public:
        /// Constructor
        Quaternion_Wrapper() {};
        /// Destructor
        virtual ~Quaternion_Wrapper() {};
        /// Operators
        virtual double& operator()(unsigned int i) = 0;
        virtual double operator()(unsigned int i) const = 0;
        // Others
        virtual MyQuaternion normalize() = 0;
        
        /// Operators
        virtual MyQuaternion& operator =(const MyQuaternion &q) = 0;
        virtual MyQuaternion operator+ (const MyQuaternion &q) const = 0;
        virtual MyQuaternion operator- (const MyQuaternion &q) const = 0;
      /*
       * \brief Converts quaternion to Euler Angles.
       * @param[in]  axes is a string indicating the order of rotation as: 'xyz', 'xyx', 'xzx', 'xzy', 'yxy', 'yxz', 'yzx', 'yzy', 'zxy', 'zxz', 'zxy', 'zyz'.
       * @param[out] output is a 3-dim ColumnVector with the corresponding Euler Angle vector in radians.
       */
        virtual bool getEulerAngles(std::string axes, MatrixWrapper::ColumnVector& output);
        
        virtual bool getRotation(MatrixWrapper::Matrix& output);
        
        virtual bool conjugate(MyQuaternion& output) = 0;
        
        virtual void quaternion2euler (double res[], std::string axes);
        
        virtual void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]);
    }; // enf of Quaternion_Wrapper class
} // end MatrixWrapper namespace

// Include implementations
#include "vector_BOOST.h"
#include "vector_EIGEN.h"

#endif // _QUATERNION_WRAPPER_
