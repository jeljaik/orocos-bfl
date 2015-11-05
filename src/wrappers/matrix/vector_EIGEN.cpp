#include "../config.h"
#ifdef __MATRIXWRAPPER_EIGEN__

#include "vector_EIGEN.h"
#include <iostream>


// Constructors
MyColumnVector::ColumnVector() : EigenColumnVector() {}
MyColumnVector::ColumnVector(int num_rows) : EigenColumnVector(num_rows){}
MyColumnVector::ColumnVector(int num_rows,double value) : EigenColumnVector(num_rows){
  ((EigenColumnVector*)this)->setConstant(value);
}
MyColumnVector::ColumnVector(const MyColumnVector& a, const MyColumnVector& b) : EigenColumnVector(a.rows() + b.rows())
{
  EigenColumnVector& opl = (*this);
  opl.head(a.rows()) = (const EigenColumnVector &)(a);
  opl.tail(b.rows()) = (const EigenColumnVector &)(b);
}

// Constructor from quaternion
MyColumnVector::ColumnVector(MyQuaternion& quat) : EigenColumnVector(4)
{
    EigenColumnVector& opl = (*this);
    // Copy elements from quaternion to column vector
    unsigned int i;
    for (i=0; i<4; i++)
        opl(i) = quat(i+1);
}

// Constructor from pointer to double
MyColumnVector::ColumnVector(double* yvec, unsigned int size) : EigenColumnVector(size)
{
    EigenColumnVector& opl = (*this);
    Eigen::Map<EigenColumnVector> map(yvec, size);
    opl = (EigenColumnVector)map;
//    for (unsigned int i=0; i<size; i++)
//        opl(i) = yvec[i];
}

// Destructor
MyColumnVector::~ColumnVector(){}

// Copy constructor
MyColumnVector::ColumnVector(const MyColumnVector& a) :
  EigenColumnVector(a){}
MyColumnVector::ColumnVector(const EigenColumnVector & a) :
  EigenColumnVector(a){}

// Resizing
void MyColumnVector::resize(int num_rows)
{
  EigenColumnVector & op1 = (*this);
    op1.conservativeResize(num_rows);
  //op1.resize(num_rows);
}

// Assign
void MyColumnVector::assign(int num_rows, double value)
{
  EigenColumnVector & op1 = (*this);
  op1.resize(num_rows);
  op1.setConstant(value);
}

// Number of Rows / Cols
unsigned int MyColumnVector::rows() const { return ((const EigenColumnVector *)this)->rows();}
unsigned int MyColumnVector::columns() const { return ((const EigenColumnVector *)this)->cols();}
unsigned int MyColumnVector::capacity() const { return ((const EigenColumnVector *)this)->size();}

MyColumnVector
MyColumnVector::vectorAdd(const MyColumnVector& v2) const
{
  const MyColumnVector& v1 = *this;
  MyColumnVector res(v1.rows() + v2.rows());
  EigenColumnVector& opl = res;
  opl.head(v1.rows()) = (const EigenColumnVector &)(v1);
  opl.tail(v2.rows()) = (const EigenColumnVector &)(v2);

  return res;
}

double& MyColumnVector::operator()(unsigned int i)
{
  //std::cout << "(BOOSTVECTOR) operator() called" << std::endl;
  EigenColumnVector& op1 = *(this);
  return op1(i-1);
}

double MyColumnVector::operator()(unsigned int i) const
{
  //std::cout << "(BOOSTVECTOR) operator() called" << std::endl;
  const EigenColumnVector op1 = (*this);
  return op1(i-1);
}


bool MyColumnVector::operator==(const MyColumnVector& a) const
{
  if (this->rows() != a.rows()) return false;
  return(((EigenColumnVector)(*this)-(EigenColumnVector)a).isApproxToConstant(0.0));
}

// Operators
MyColumnVector & MyColumnVector::operator+= (const MyColumnVector& a)
{
  EigenColumnVector & op1 = (*this);
  const EigenColumnVector & op2 = a;
  op1 += op2;
  return (MyColumnVector &) op1;
}

MyColumnVector & MyColumnVector::operator-= (const MyColumnVector& a)
{
  EigenColumnVector & op1 = (*this);
  const EigenColumnVector & op2 = a;
  op1 -= op2;
  return (MyColumnVector &) op1;
}

MyColumnVector MyColumnVector::operator+ (const MyColumnVector &a) const
{
  return (MyColumnVector) ((EigenColumnVector)(*this) + (EigenColumnVector)a);
}

MyColumnVector MyColumnVector::operator- (const MyColumnVector &a) const
{
  return (MyColumnVector) ((EigenColumnVector)(*this) - (EigenColumnVector)a);
}



MyColumnVector& MyColumnVector::operator+= (double a)
{
  EigenColumnVector & op1 = *this;
  op1 += EigenColumnVector::Constant(rows(), a);
  return (MyColumnVector&)op1;
}

MyColumnVector& MyColumnVector::operator-= (double a)
{
  EigenColumnVector & op1 = *this;
  op1 -= EigenColumnVector::Constant(rows(), a);
  return (MyColumnVector&)op1;
}

MyColumnVector& MyColumnVector::operator*= (double a)
{
  EigenColumnVector& op1 = *this;
  op1 *= a;
  return (MyColumnVector&) op1;
}

MyColumnVector& MyColumnVector::operator/= (double a)
{
  EigenColumnVector& op1 = *this;
  op1 /= a;
  return (MyColumnVector&) op1;
}


MyColumnVector MyColumnVector::operator+ (double a) const
{
  return (MyColumnVector)(((EigenColumnVector)(*this)) + EigenColumnVector::Constant(rows(), a));
}

MyColumnVector MyColumnVector::operator- (double a) const
{
  return (MyColumnVector)(((EigenColumnVector)(*this)) - EigenColumnVector::Constant(rows(), a));
}

MyColumnVector MyColumnVector::operator* (double a) const
{
  const EigenColumnVector & op1 = (*this);
  return (MyColumnVector) (op1 * a);
}

MyColumnVector MyColumnVector::operator/ (double a) const
{
  const EigenColumnVector & op1 = (*this);
  return (MyColumnVector) (op1 / a);
}



MyRowVector MyColumnVector::transpose() const
{
  const EigenColumnVector & op1 = (*this);
  return MyRowVector(op1.transpose());
}

MyMatrix MyColumnVector::operator* (const MyRowVector &a) const
{
  const EigenColumnVector & op1 = (*this);
  const EigenRowVector & op2 = a;

  return MyMatrix(op1 * op2);
}

MyColumnVector&
MyColumnVector::operator=(const MyColumnVector &a)
{
  EigenColumnVector& op1 = *this;
  op1 = (EigenColumnVector)a;
  return *this;
}

MyColumnVector&
MyColumnVector::operator=(double a)
{
  EigenColumnVector& op1 = *this;
  op1.setConstant(a);
  return *this;
}

MyColumnVector MyColumnVector::sub(int j_start , int j_end) const
{
  const EigenColumnVector& op1 = *this;
  return MyColumnVector(op1.segment(j_start-1,j_end-j_start+1));
}



//////////////////////////////////////////////////////////////////////
////////////////////////////// ROWVECTOR /////////////////////////////
//////////////////////////////////////////////////////////////////////

// Constructors
MyRowVector::RowVector() : EigenRowVector() {}
MyRowVector::RowVector(int num_cols) : EigenRowVector(num_cols){}
MyRowVector::RowVector(int num_cols,double value) : EigenRowVector(num_cols){
  ((EigenRowVector*)this)->setConstant(value);
}

// Destructor
MyRowVector::~RowVector(){}

// Copy constructor
MyRowVector::RowVector(const MyRowVector& a) :
  EigenRowVector(a){}
MyRowVector::RowVector(const EigenRowVector & a) :
  EigenRowVector(a){}

// Resizing
void MyRowVector::resize(int num_columns)
{
  EigenRowVector & op1 = (*this);
  op1.conservativeResize(num_columns);
}

// Assign
void MyRowVector::assign(int num_columns, double value)
{
  EigenRowVector & op1 = (*this);
  op1.resize(num_columns);
  op1.setConstant(value);
}

// Number of Rows / Cols
unsigned int MyRowVector::rows() const { return ((const EigenRowVector *)this)->rows();}
unsigned int MyRowVector::columns() const { return ((const EigenRowVector *)this)->cols();}
unsigned int MyRowVector::capacity() const { return ((const EigenRowVector *)this)->size();}

MyRowVector
MyRowVector::vectorAdd(const MyRowVector& v2) const
{
  const MyRowVector& v1 = *this;
  MyRowVector res(v1.rows() + v2.rows());
  EigenRowVector& opl = res;
  opl.head(v1.rows()) = (const EigenRowVector &)(v1);
  opl.tail(v2.rows()) = (const EigenRowVector &)(v2);
  return res;
}

double& MyRowVector::operator()(unsigned int i)
{
  EigenRowVector& op1 = *(this);
  return op1(i-1);
}

double MyRowVector::operator()(unsigned int i) const
{
  const EigenRowVector& op1 = (*this);
  return op1(i-1);
}

bool MyRowVector::operator==(const MyRowVector& a) const
{
  if (this->columns() != a.columns()) return false;
  return(((EigenRowVector)(*this)-(EigenRowVector)a).isApproxToConstant(0.0));
}

// Operators
MyRowVector & MyRowVector::operator+= (const MyRowVector& a)
{
  EigenRowVector & op1 = (*this);
  const EigenRowVector & op2 = a;
  op1 += op2;
  return (MyRowVector &) op1;
}

MyRowVector & MyRowVector::operator-= (const MyRowVector& a)
{
  EigenRowVector & op1 = (*this);
  const EigenRowVector & op2 = a;
  op1 -= op2;
  return (MyRowVector &) op1;
}

MyRowVector MyRowVector::operator+ (const MyRowVector &a) const
{
  return (MyRowVector) ((EigenRowVector)(*this) + (EigenRowVector)a);
}

MyRowVector MyRowVector::operator- (const MyRowVector &a) const
{
  return (MyRowVector) ((EigenRowVector)(*this) - (EigenRowVector)a);
}



MyRowVector& MyRowVector::operator+= (double a)
{
  EigenRowVector & op1 = *this;
  op1 += EigenRowVector::Constant(columns(),a);
  return (MyRowVector&)op1;
}

MyRowVector& MyRowVector::operator-= (double a)
{
  EigenRowVector & op1 = *this;
  op1 -= EigenRowVector::Constant(columns(),a);
  return (MyRowVector&)op1;
}

MyRowVector& MyRowVector::operator*= (double a)
{
  EigenRowVector& op1 = *this;
  op1 *= a;
  return (MyRowVector&) op1;
}

MyRowVector& MyRowVector::operator/= (double a)
{
  EigenRowVector& op1 = *this;
  op1 /= a;
  return (MyRowVector&) op1;
}


MyRowVector MyRowVector::operator+ (double a) const
{
  return (MyRowVector)(((EigenRowVector)(*this)) + EigenRowVector::Constant(columns(),a));
}

MyRowVector MyRowVector::operator- (double a) const
{
  return (MyRowVector)(((EigenRowVector)(*this)) - EigenRowVector::Constant(columns(),a));
}

MyRowVector MyRowVector::operator* (double a) const
{
  const EigenRowVector & op1 = (*this);
  return (MyRowVector) (op1 * a);
}

MyRowVector MyRowVector::operator/ (double a) const
{
  const EigenRowVector & op1 = (*this);
  return (MyRowVector) (op1 / a);
}



MyColumnVector MyRowVector::transpose() const
{
  const EigenRowVector & op1 = (*this);
  return MyColumnVector(op1.transpose());
}

double MyRowVector::operator* (const MyColumnVector &a) const
{
  const EigenRowVector & op1 = (*this);
  const EigenColumnVector & op2 = a;
  return (op1 * op2)(0,0);
}

MyRowVector& MyRowVector::operator=(const MyRowVector &a)
{
  EigenRowVector& op1 = *this;
  op1 = (EigenRowVector)a;
  return *this;
}

MyRowVector& MyRowVector::operator=(double a)
{
  EigenRowVector& op1 = *this;
  op1.setConstant(a);
  return *this;
}

MyRowVector MyRowVector::sub(int j_start , int j_end) const
{
  const EigenRowVector& op1 = *this;
  return MyRowVector(op1.segment(j_start-1,j_end-j_start+1));
}

///////////////////////////////////////////////////////////////////////
////////////////////////////// QUATERNION /////////////////////////////
///////////////////////////////////////////////////////////////////////

// Constructors
MyQuaternion::Quaternion() :  EigenQuaternion() { }

MyQuaternion::Quaternion(double real, double i, double j, double z) : EigenQuaternion( (const double) real, (const double) i, (const double) j, (const double) z) { }

MyQuaternion::Quaternion(MyColumnVector col) : EigenQuaternion( col(1), col(2), col(3), col(4) ) { }

// Destructor
MyQuaternion::~Quaternion() { }

// Copy constructor
MyQuaternion::Quaternion( const MyQuaternion & q ) : EigenQuaternion( q ) { }
MyQuaternion::Quaternion( const EigenQuaternion & q ) : EigenQuaternion( q ) { }

// Operators
double& MyQuaternion::operator()( unsigned int i )
{
    EigenQuaternion & op1 = *(this);
    switch (i) {
        case 1:
            return op1.w();
            break;
        case 2:
            return op1.x();
            break;
        case 3:
            return op1.y();
            break;
        case 4:
            return op1.z();
            break;
        default:
            std::cout << "[BFLERR] Index out of boundaries." << std::endl;
            break;
    }
}

double MyQuaternion::operator()( unsigned int i ) const
{
    const EigenQuaternion & op1 = *(this);
    switch (i) {
        case 1:
            return op1.w();
            break;
        case 2:
            return op1.x();
            break;
        case 3:
            return op1.y();
            break;
        case 4:
            return op1.z();
            break;
        default:
            std::cout << "[BFLERR] Index out of boundaries." << std::endl;
            break;
    }
}

MyQuaternion& MyQuaternion::operator =(const MyQuaternion &q)
{
    EigenQuaternion & op1 = *this;
    op1 = (EigenQuaternion)q;
    return *this;
}


MyQuaternion MyQuaternion::operator+ (const MyQuaternion &q) const
{
    const EigenQuaternion & op1 = *(this);
    const EigenQuaternion & op2 = q;
    Eigen::Vector4d tmp(4); tmp.setZero();
    tmp[1] = op1.w() + op2.w();                // real part sum
    tmp.bottomRows(3) = op1.vec() + op2.vec(); // im part
    EigenQuaternion result = EigenQuaternion(tmp.data());
    return (MyQuaternion) result;
}

MyQuaternion MyQuaternion::operator- (const MyQuaternion &q) const
{
    const EigenQuaternion & op1 = *(this);
    const EigenQuaternion & op2 = q;
    Eigen::Vector4d tmp(4); tmp.setZero();
    tmp[1] = op1.w() - op2.w();                // real part sum
    tmp.bottomRows(3) = op1.vec() - op2.vec(); // im part
    EigenQuaternion result = EigenQuaternion(tmp.data());
    return (MyQuaternion) result;
}

// This binary operator<< can be declared as a friend method among the public methods
// of class MyQuaternion (e.g. MatrixWrapper::Quaternion), but when implementing, remember
// that it is not a member of the Quaternion and thus, the namespace MatrixWrapper must
// be prepended.
std::ostream& MatrixWrapper::operator<< (std::ostream& os, const MyQuaternion& quat)
{
    return os << quat.toString();
}

std::string MyQuaternion::toString() const
{
    std::ostringstream os;
    for (int i = 0; i < 4; i++)
    {   
        os << (*this)(i+1) << " ";
    }
    return os.str();
}

MyQuaternion MyQuaternion::normalize()
{
    EigenQuaternion& op1 = *(this);
    op1.normalize();
}

bool MyQuaternion::conjugate(MyQuaternion& output)
{
    bool ret = false;
    const EigenQuaternion &q = *(this);
    q.conjugate();
    ret = true;
    return ret;
}




#endif