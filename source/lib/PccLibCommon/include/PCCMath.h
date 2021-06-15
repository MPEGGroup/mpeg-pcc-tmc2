/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCCMath_h
#define PCCMath_h

#include "PCCCommon.h"

namespace pcc {

typedef int16_t PCCType;

/// Vector dim 3
template <typename T>
class PCCVector3 {
 public:
  T& operator[]( size_t i ) {
    assert( i < 3 );
    return data_[i];
  }
  const T& operator[]( size_t i ) const {
    assert( i < 3 );
    return data_[i];
  }
  size_t   getElementCount() const { return 3; }
  T&       r() { return data_[0]; }
  T&       g() { return data_[1]; }
  T&       b() { return data_[2]; }
  const T& r() const { return data_[0]; }
  const T& g() const { return data_[1]; }
  const T& b() const { return data_[2]; }
  T&       x() { return data_[0]; }
  T&       y() { return data_[1]; }
  T&       z() { return data_[2]; }
  const T& x() const { return data_[0]; }
  const T& y() const { return data_[1]; }
  const T& z() const { return data_[2]; }
  void     normalize() {
    const T norm2 = data_[0] * data_[0] + data_[1] * data_[1] + data_[2] * data_[2];
    if ( norm2 != 0.0 ) {
      T invNorm = static_cast<T>( 1.0 / sqrt( norm2 ) );
      ( *this ) *= invNorm;
    }
  }
  T getNorm() const { return static_cast<T>( sqrt( getNorm2() ) ); }
  T getNorm2() const { return data_[0] * data_[0] + data_[1] * data_[1] + data_[2] * data_[2]; }

  template <typename OutT>
  PCCVector3& operator=( const PCCVector3<OutT>& rhs ) {
    data_[0] = (T)rhs.r();
    data_[1] = (T)rhs.g();
    data_[2] = (T)rhs.b();
    return *this;
  }
  PCCVector3& operator+=( const PCCVector3& rhs ) {
    data_[0] += rhs.data_[0];
    data_[1] += rhs.data_[1];
    data_[2] += rhs.data_[2];
    return *this;
  }
  PCCVector3& operator-=( const PCCVector3& rhs ) {
    data_[0] -= rhs.data_[0];
    data_[1] -= rhs.data_[1];
    data_[2] -= rhs.data_[2];
    return *this;
  }
  PCCVector3& operator-=( const T a ) {
    data_[0] -= a;
    data_[1] -= a;
    data_[2] -= a;
    return *this;
  }
  PCCVector3& operator+=( const T a ) {
    data_[0] += a;
    data_[1] += a;
    data_[2] += a;
    return *this;
  }
  PCCVector3& operator/=( const T a ) {
    assert( a != 0 );
    data_[0] /= a;
    data_[1] /= a;
    data_[2] /= a;
    return *this;
  }
  PCCVector3& operator*=( const T a ) {
    data_[0] *= a;
    data_[1] *= a;
    data_[2] *= a;
    return *this;
  }
  PCCVector3& operator=( const T a ) {
    data_[0] = a;
    data_[1] = a;
    data_[2] = a;
    return *this;
  }
  PCCVector3& operator=( const T* const rhs ) {
    data_[0] = rhs[0];
    data_[1] = rhs[1];
    data_[2] = rhs[2];
    return *this;
  }

  // T operator*(const PCCVector3 &rhs) const {
  //   return (data_[0] * rhs.data_[0] + data_[1] * rhs.data_[1] + data_[2] *
  // rhs.data_[2]);
  // }
  PCCVector3 operator^( const PCCVector3& rhs ) const {
    return PCCVector3<T>( data_[1] * rhs.data_[2] - data_[2] * rhs.data_[1],
                          data_[2] * rhs.data_[0] - data_[0] * rhs.data_[2],
                          data_[0] * rhs.data_[1] - data_[1] * rhs.data_[0] );
  }
  PCCVector3        operator-() const { return PCCVector3<T>( -data_[0], -data_[1], -data_[2] ); }
  friend PCCVector3 operator+( const PCCVector3& lhs, const PCCVector3& rhs ) {
    return PCCVector3<T>( lhs.data_[0] + rhs.data_[0], lhs.data_[1] + rhs.data_[1], lhs.data_[2] + rhs.data_[2] );
  }
  friend PCCVector3 operator+( const T lhs, const PCCVector3& rhs ) {
    return PCCVector3<T>( lhs + rhs.data_[0], lhs + rhs.data_[1], lhs + rhs.data_[2] );
  }
  friend PCCVector3 operator+( const PCCVector3& lhs, const T rhs ) {
    return PCCVector3<T>( lhs.data_[0] + rhs, lhs.data_[1] + rhs, lhs.data_[2] + rhs );
  }
  friend PCCVector3 operator-( const PCCVector3& lhs, const PCCVector3& rhs ) {
    return PCCVector3<T>( lhs.data_[0] - rhs.data_[0], lhs.data_[1] - rhs.data_[1], lhs.data_[2] - rhs.data_[2] );
  }
  friend PCCVector3 operator-( const T lhs, const PCCVector3& rhs ) {
    return PCCVector3<T>( lhs - rhs.data_[0], lhs - rhs.data_[1], lhs - rhs.data_[2] );
  }
  friend PCCVector3 operator-( const PCCVector3& lhs, const T rhs ) {
    return PCCVector3<T>( lhs.data_[0] - rhs, lhs.data_[1] - rhs, lhs.data_[2] - rhs );
  }
  friend PCCVector3 operator*( const T lhs, const PCCVector3& rhs ) {
    return PCCVector3<T>( lhs * rhs.data_[0], lhs * rhs.data_[1], lhs * rhs.data_[2] );
  }
  friend PCCVector3 operator*( const PCCVector3& lhs, const T rhs ) {
    return PCCVector3<T>( lhs.data_[0] * rhs, lhs.data_[1] * rhs, lhs.data_[2] * rhs );
  }
  friend PCCVector3 operator/( const PCCVector3& lhs, const T rhs ) {
    assert( rhs != 0 );
    return PCCVector3<T>( lhs.data_[0] / rhs, lhs.data_[1] / rhs, lhs.data_[2] / rhs );
  }
  bool operator<( const PCCVector3& rhs ) const {
    if ( data_[0] == rhs.data_[0] ) {
      if ( data_[1] == rhs.data_[1] ) { return ( data_[2] < rhs.data_[2] ); }
      return ( data_[1] < rhs.data_[1] );
    }
    return ( data_[0] < rhs.data_[0] );
  }
  bool operator>( const PCCVector3& rhs ) const {
    if ( data_[0] == rhs.data_[0] ) {
      if ( data_[1] == rhs.data_[1] ) { return ( data_[2] > rhs.data_[2] ); }
      return ( data_[1] > rhs.data_[1] );
    }
    return ( data_[0] > rhs.data_[0] );
  }
  bool operator==( const PCCVector3& rhs ) const {
    return ( data_[0] == rhs.data_[0] && data_[1] == rhs.data_[1] && data_[2] == rhs.data_[2] );
  }
  bool operator!=( const PCCVector3& rhs ) const {
    return ( data_[0] != rhs.data_[0] || data_[1] != rhs.data_[1] || data_[2] != rhs.data_[2] );
  }
  friend std::ostream& operator<<( std::ostream& os, const PCCVector3& vec ) {
    os << vec[0] << " " << vec[1] << " " << vec[2] << std::endl;
    return os;
  }
  friend std::istream& operator>>( std::istream& is, PCCVector3& vec ) {
    is >> vec[0] >> vec[1] >> vec[2];
    return is;
  }
  PCCVector3( const T a ) { data_[0] = data_[1] = data_[2] = a; }
  PCCVector3( const T x, const T y, const T z ) {
    data_[0] = x;
    data_[1] = y;
    data_[2] = z;
  }
  PCCVector3( const PCCVector3& vec ) {
    data_[0] = vec.data_[0];
    data_[1] = vec.data_[1];
    data_[2] = vec.data_[2];
  }
  PCCVector3( const T* vec ) { memcpy( data_, vec, sizeof( data_ ) ); }
  template <typename OutT>
  PCCVector3( const PCCVector3<OutT>& vec ) {
    data_[0] = (T)vec[0];
    data_[1] = (T)vec[1];
    data_[2] = (T)vec[2];
  }
  PCCVector3()        = default;
  ~PCCVector3( void ) = default;

 private:
  T data_[3];
};

template <typename T>
struct PCCBox3 {
  PCCVector3<T> min_;
  PCCVector3<T> max_;

  void init() {
    min_.x() = ( std::numeric_limits<T>::max )();
    min_.y() = ( std::numeric_limits<T>::max )();
    min_.z() = ( std::numeric_limits<T>::max )();
    max_.x() = ( std::numeric_limits<T>::min )();
    max_.y() = ( std::numeric_limits<T>::min )();
    max_.z() = ( std::numeric_limits<T>::min )();
  }
  inline uint16_t size( uint8_t index ) const { return max_[index] - min_[index]; }
  inline uint16_t sizeX() const { return max_.x() - min_.x(); }
  inline uint16_t sizeY() const { return max_.y() - min_.y(); }
  inline uint16_t sizeZ() const { return max_.z() - min_.z(); }

  bool contains( const PCCVector3<T> point ) const {
    return !( point.x() < min_.x() || point.x() > max_.x() || point.y() < min_.y() || point.y() > max_.y() ||
              point.z() < min_.z() || point.z() > max_.z() );
  }

  PCCBox3 merge( const PCCBox3& box ) {
    min_.x() = std::min( min_.x(), box.min_.x() );
    min_.y() = std::min( min_.y(), box.min_.y() );
    min_.z() = std::min( min_.z(), box.min_.z() );
    max_.x() = std::max( max_.x(), box.max_.x() );
    max_.y() = std::max( max_.y(), box.max_.y() );
    max_.z() = std::max( max_.z(), box.max_.z() );
    return box;
  }
  inline void add( const PCCVector3<T>& point ) {
    if ( min_[0] > point[0] ) { min_[0] = point[0]; }
    if ( min_[1] > point[1] ) { min_[1] = point[1]; }
    if ( min_[2] > point[2] ) { min_[2] = point[2]; }
    if ( max_[0] < point[0] ) { max_[0] = point[0]; }
    if ( max_[1] < point[1] ) { max_[1] = point[1]; }
    if ( max_[2] < point[2] ) { max_[2] = point[2]; }
  }
  bool intersects( const PCCBox3& box ) {
    return max_.x() >= box.min_.x() && min_.x() <= box.max_.x() && max_.y() >= box.min_.y() &&
           min_.y() <= box.max_.y() && max_.z() >= box.min_.z() && min_.z() <= box.max_.z();
  }

  bool fullyContains( const PCCBox3& box ) {
    return max_.x() >= box.max_.x() && min_.x() <= box.min_.x() && max_.y() >= box.max_.y() &&
           min_.y() <= box.min_.y() && max_.z() >= box.max_.z() && min_.z() <= box.min_.z();
  }

  bool fullyContains( const PCCVector3<int16_t> point ) const {
    return max_.x() >= point.x() && min_.x() <= point.x() && max_.y() >= point.y() && min_.y() <= point.y() &&
           max_.z() >= point.z() && min_.z() <= point.z();
  }

  friend std::ostream& operator<<( std::ostream& os, const PCCBox3& box ) {
    os << box.min_[0] << " " << box.min_[1] << " " << box.min_[2] << " " << box.max_[0] << " " << box.max_[1] << " "
       << box.max_[2] << std::endl;
    return os;
  }
  friend std::istream& operator>>( std::istream& is, PCCBox3& box ) {
    is >> box.min_[0] >> box.min_[1] >> box.min_[2] >> box.max_[0] >> box.max_[1] >> box.max_[2];
    return is;
  }
};

//!    3x3 Matrix
template <typename T>
class PCCMatrix3 {
 public:
  T* operator[]( const size_t rowIndex ) {
    assert( rowIndex < 3 );
    return data_[rowIndex];
  }
  const T* operator[]( const size_t rowIndex ) const {
    assert( rowIndex < 3 );
    return data_[rowIndex];
  }
  size_t      getColumnCount() const { return 3; }
  size_t      getRowCount() const { return 3; }
  PCCMatrix3& operator=( const PCCMatrix3& rhs ) {
    memcpy( data_, rhs.data_, sizeof( data_ ) );
    return *this;
  }
  void operator+=( const PCCMatrix3& rhs ) {
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { this->data_[i][j] += rhs.data_[i][j]; }
    }
  }
  void operator-=( const PCCMatrix3& rhs ) {
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { this->data_[i][j] -= rhs.data_[i][j]; }
    }
  }
  void operator-=( const T a ) {
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { this->data_[i][j] -= a; }
    }
  }
  void operator+=( const T a ) {
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { this->data_[i][j] += a; }
    }
  }
  void operator/=( const T a ) {
    assert( a != 0 );
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { this->data_[i][j] /= a; }
    }
  }
  void operator*=( const T a ) {
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { this->data_[i][j] *= a; }
    }
  }
  PCCVector3<T> operator*( const PCCVector3<T>& rhs ) const {
    PCCVector3<T> res;
    for ( int i = 0; i < 3; ++i ) {
      res[i] = 0;
      for ( int j = 0; j < 3; ++j ) { res[i] += this->data_[i][j] * rhs[j]; }
    }
    return res;
  }
  PCCMatrix3 operator*( const PCCMatrix3& rhs ) const {
    PCCMatrix3<T> res;
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) {
        res.data_[i][j] = 0;
        for ( int k = 0; k < 3; ++k ) { res.data_[i][j] += this->data_[i][k] * rhs.data_[k][j]; }
      }
    }
    return res;
  }
  PCCMatrix3 operator+( const PCCMatrix3& rhs ) const {
    PCCMatrix3<T> res;
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { res.data_[i][j] = this->data_[i][j] + rhs.data_[i][j]; }
    }
    return res;
  }
  PCCMatrix3 operator-( const PCCMatrix3& rhs ) const {
    PCCMatrix3<T> res;
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { res.data_[i][j] = this->data_[i][j] - rhs.data_[i][j]; }
    }
    return res;
  }
  PCCMatrix3 operator-() const {
    PCCMatrix3<T> res;
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { res.data_[i][j] = -this->data_[i][j]; }
    }
    return res;
  }
  PCCMatrix3 operator*( T rhs ) const {
    PCCMatrix3<T> res;
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { res.data_[i][j] = this->data_[i][j] * rhs; }
    }
    return res;
  }
  PCCMatrix3 operator/( T rhs ) const {
    assert( rhs != 0 );
    PCCMatrix3<T> res;
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { res.data_[i][j] = this->data_[i][j] / rhs; }
    }
    return res;
  }
  PCCMatrix3 transpose() const {
    PCCMatrix3<T> res;
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { res.data_[i][j] = this->data_[j][i]; }
    }
    return res;
  }
  PCCMatrix3() = default;
  PCCMatrix3( const T a ) {
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { data_[i][j] = a; }
    }
  }
  PCCMatrix3( const PCCMatrix3& rhs ) { memcpy( data_, rhs.data_, sizeof( data_ ) ); }
  ~PCCMatrix3( void ) = default;

  friend inline PCCMatrix3<T> operator*( T lhs, const PCCMatrix3<T>& rhs ) {
    PCCMatrix3<T> res;
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) { res.data_[i][j] = lhs * rhs.data_[i][j]; }
    }
    return res;
  }
  static void makeIdentity( PCCMatrix3<T>& mat ) {
    memset( mat.data_, 0, sizeof( mat.data_ ) );
    for ( int i = 0; i < 3; ++i ) { mat[i][i] = 1; }
  }

  static void makeScale( const T sx, const T sy, const T sz, PCCMatrix3<T>& mat ) {
    makeIdentity( mat );
    mat[0][0] = sx;
    mat[1][1] = sy;
    mat[2][2] = sz;
  }
  static void makeUniformScale( const T s, PCCMatrix3<T>& mat ) { makeScale( s, s, s, mat ); }
  static void makeRotation( const T angle, const T ax, const T ay, const T az, PCCMatrix3<T>& mat ) {
    T c       = cos( angle );
    T l_c     = 1 - c;
    T s       = sin( angle );
    mat[0][0] = ax * ax + ( 1 - ax * ax ) * c;
    mat[0][1] = ax * ay * l_c - az * s;
    mat[0][2] = ax * az * l_c + ay * s;
    mat[1][0] = ax * ay * l_c + az * s;
    mat[1][1] = ay * ay + ( 1 - ay * ay ) * c;
    mat[1][2] = ay * az * l_c - ax * s;
    mat[2][0] = ax * az * l_c - ay * s;
    mat[2][1] = ay * az * l_c + ax * s;
    mat[2][2] = az * az + ( 1 - az * az ) * c;
  }

 private:
  T data_[3][3];
};

typedef PCCVector3<double>   PCCVector3D;
typedef PCCVector3<int16_t>  PCCPoint3D;
typedef PCCBox3<double>      PCCBox3D;
typedef PCCBox3<int16_t>     PCCInt16Box3D;
typedef PCCVector3<uint8_t>  PCCColor3B;
typedef PCCVector3<uint16_t> PCCColor16bit;
typedef PCCVector3<double>   PCCNormal3D;
typedef PCCMatrix3<double>   PCCMatrix3D;

static inline PCCVector3D operator+( const PCCVector3D& a, const PCCPoint3D& b ) {
  return PCCVector3D( a[0] + b[0], a[1] + b[1], a[2] + b[2] );
}
static inline PCCVector3D operator+( const PCCPoint3D& a, const PCCVector3D& b ) {
  return PCCVector3D( a[0] + b[0], a[1] + b[1], a[2] + b[2] );
}
static inline PCCVector3D operator-( const PCCVector3D& a, const PCCPoint3D& b ) {
  return PCCVector3D( a[0] - b[0], a[1] - b[1], a[2] - b[2] );
}
static inline PCCVector3D operator-( const PCCPoint3D& a, const PCCVector3D& b ) {
  return PCCVector3D( a[0] - b[0], a[1] - b[1], a[2] - b[2] );
}
static inline double operator*( const PCCVector3D& a, const PCCVector3D& b ) {
  return ( a[0] * b[0] + a[1] * b[1] + a[2] * b[2] );
}
static inline double operator*( const PCCVector3D& a, const PCCPoint3D& b ) {
  return ( a[0] * b[0] + a[1] * b[1] + a[2] * b[2] );
}
static inline double operator*( const PCCPoint3D& a, const PCCVector3D& b ) {
  return ( a[0] * b[0] + a[1] * b[1] + a[2] * b[2] );
}
static inline PCCVector3D& operator+=( PCCVector3D& a, const PCCPoint3D& b ) {
  a = a + b;
  return a;
}
static inline PCCVector3D& operator-=( PCCVector3D& a, const PCCPoint3D& b ) {
  a = a - b;
  return a;
}

struct DistColor {
  double        dist;
  PCCColor16bit color;
  PCCPoint3D    point;
  size_t        index;
  size_t        indexPartial;
};

struct DistColor8Bit {
  double     dist;
  PCCColor3B color;
};

// Slightly modified version of http://www.melax.com/diag.html?attredirects=0
// A must be a symmetric matrix.
// returns Q and D such that
// Diagonal matrix D = QT * A * Q;  and  A = Q*D*QT
static inline void PCCDiagonalize( const PCCMatrix3<double>& A, PCCMatrix3<double>& Q, PCCMatrix3<double>& D ) {
  const int          maxsteps = 24;  // certainly wont need that many.
  int                k0, k1, k2;
  double             o[3], m[3];
  double             q[4] = {0.0, 0.0, 0.0, 1.0};
  double             jr[4];
  double             sqw, sqx, sqy, sqz;
  double             tmp1, tmp2, mq;
  PCCMatrix3<double> AQ;
  double             thet, sgn, t, c;
  for ( int i = 0; i < maxsteps; ++i ) {
    // quat to matrix
    sqx     = q[0] * q[0];
    sqy     = q[1] * q[1];
    sqz     = q[2] * q[2];
    sqw     = q[3] * q[3];
    Q[0][0] = ( sqx - sqy - sqz + sqw );
    Q[1][1] = ( -sqx + sqy - sqz + sqw );
    Q[2][2] = ( -sqx - sqy + sqz + sqw );
    tmp1    = q[0] * q[1];
    tmp2    = q[2] * q[3];
    Q[1][0] = 2.0 * ( tmp1 + tmp2 );
    Q[0][1] = 2.0 * ( tmp1 - tmp2 );
    tmp1    = q[0] * q[2];
    tmp2    = q[1] * q[3];
    Q[2][0] = 2.0 * ( tmp1 - tmp2 );
    Q[0][2] = 2.0 * ( tmp1 + tmp2 );
    tmp1    = q[1] * q[2];
    tmp2    = q[0] * q[3];
    Q[2][1] = 2.0 * ( tmp1 + tmp2 );
    Q[1][2] = 2.0 * ( tmp1 - tmp2 );

    // AQ = A * Q;
    AQ[0][0] = Q[0][0] * A[0][0] + Q[1][0] * A[0][1] + Q[2][0] * A[0][2];
    AQ[0][1] = Q[0][1] * A[0][0] + Q[1][1] * A[0][1] + Q[2][1] * A[0][2];
    AQ[0][2] = Q[0][2] * A[0][0] + Q[1][2] * A[0][1] + Q[2][2] * A[0][2];
    AQ[1][0] = Q[0][0] * A[0][1] + Q[1][0] * A[1][1] + Q[2][0] * A[1][2];
    AQ[1][1] = Q[0][1] * A[0][1] + Q[1][1] * A[1][1] + Q[2][1] * A[1][2];
    AQ[1][2] = Q[0][2] * A[0][1] + Q[1][2] * A[1][1] + Q[2][2] * A[1][2];
    AQ[2][0] = Q[0][0] * A[0][2] + Q[1][0] * A[1][2] + Q[2][0] * A[2][2];
    AQ[2][1] = Q[0][1] * A[0][2] + Q[1][1] * A[1][2] + Q[2][1] * A[2][2];
    AQ[2][2] = Q[0][2] * A[0][2] + Q[1][2] * A[1][2] + Q[2][2] * A[2][2];

    // D  = Q.transpose() * AQ;
    D[0][0] = AQ[0][0] * Q[0][0] + AQ[1][0] * Q[1][0] + AQ[2][0] * Q[2][0];
    D[0][1] = AQ[0][0] * Q[0][1] + AQ[1][0] * Q[1][1] + AQ[2][0] * Q[2][1];
    D[0][2] = AQ[0][0] * Q[0][2] + AQ[1][0] * Q[1][2] + AQ[2][0] * Q[2][2];
    D[1][0] = AQ[0][1] * Q[0][0] + AQ[1][1] * Q[1][0] + AQ[2][1] * Q[2][0];
    D[1][1] = AQ[0][1] * Q[0][1] + AQ[1][1] * Q[1][1] + AQ[2][1] * Q[2][1];
    D[1][2] = AQ[0][1] * Q[0][2] + AQ[1][1] * Q[1][2] + AQ[2][1] * Q[2][2];
    D[2][0] = AQ[0][2] * Q[0][0] + AQ[1][2] * Q[1][0] + AQ[2][2] * Q[2][0];
    D[2][1] = AQ[0][2] * Q[0][1] + AQ[1][2] * Q[1][1] + AQ[2][2] * Q[2][1];
    D[2][2] = AQ[0][2] * Q[0][2] + AQ[1][2] * Q[1][2] + AQ[2][2] * Q[2][2];

    o[0] = D[1][2];
    o[1] = D[0][2];
    o[2] = D[0][1];
    m[0] = fabs( o[0] );
    m[1] = fabs( o[1] );
    m[2] = fabs( o[2] );

    k0 = ( m[0] > m[1] && m[0] > m[2] ) ? 0 : ( m[1] > m[2] ) ? 1 : 2;  // index of largest element of offdiag
    k1 = ( k0 + 1 ) % 3;
    k2 = ( k0 + 2 ) % 3;
    if ( o[k0] == 0.0 ) {
      break;  // diagonal already
    }
    thet = ( D[k2][k2] - D[k1][k1] ) / ( 2.0 * o[k0] );
    sgn  = ( thet > 0.0 ) ? 1.0 : -1.0;
    thet *= sgn;                                                                  // make it positive
    t = sgn / ( thet + ( ( thet < 1.E6 ) ? sqrt( thet * thet + 1.0 ) : thet ) );  // sign(T)/(|T|+sqrt(T^2+1))
    c = 1.0 / sqrt( t * t + 1.0 );                                                //  c= 1/(t^2+1) , t=s/c
    if ( c == 1.0 ) {
      break;  // no room for improvement - reached machine precision.
    }
    jr[0] = jr[1] = jr[2] = jr[3] = 0.0;
    jr[k0]                        = sgn * sqrt( ( 1.0 - c ) / 2.0 );  // using 1/2 angle identity sin(a/2)
                                                                      // = sqrt((1-cos(a))/2)
    jr[k0] *= -1.0;  // since our quat-to-matrix convention was for v*M instead of M*v
    jr[3] = sqrt( 1.0 - jr[k0] * jr[k0] );
    if ( jr[3] == 1.0 ) {
      break;  // reached limits of floating point precision
    }
    q[0] = ( q[3] * jr[0] + q[0] * jr[3] + q[1] * jr[2] - q[2] * jr[1] );
    q[1] = ( q[3] * jr[1] - q[0] * jr[2] + q[1] * jr[3] + q[2] * jr[0] );
    q[2] = ( q[3] * jr[2] + q[0] * jr[1] - q[1] * jr[0] + q[2] * jr[3] );
    q[3] = ( q[3] * jr[3] - q[0] * jr[0] - q[1] * jr[1] - q[2] * jr[2] );
    mq   = sqrt( q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] );
    q[0] /= mq;
    q[1] /= mq;
    q[2] /= mq;
    q[3] /= mq;
  }
}

template <typename T>
T PCCClip( const T& n, const T& lower, const T& upper ) {
  return ( std::max )( lower, ( std::min )( n, upper ) );
}
template <typename T>
bool PCCApproximatelyEqual( T a, T b, T epsilon = std::numeric_limits<PCCType>::epsilon() ) {
  return fabs( a - b ) <= ( ( fabs( a ) < fabs( b ) ? fabs( b ) : fabs( a ) ) * epsilon );
}
}  // namespace pcc

#endif /* PCCMath_h */
