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
        virtual double operator()(unsigned int i) = 0;
        virtual double operator()(unsigned int i) const = 0;
        // Others
        virtual MyMatrix toRotation() = 0;
        virtual MyQuaternion normalize() = 0;
        
    }; // enf of Quaternion_Wrapper class
} // end MatrixWrapper namespace

// Include implementations
#include "vector_BOOST.h"

#endif // _QUATERNION_WRAPPER_