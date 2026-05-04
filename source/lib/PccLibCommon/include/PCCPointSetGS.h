/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2026, ISO/IEC
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

#ifndef PCCPointSetGS_h
#define PCCPointSetGS_h

#include "PCCCommon.h"
#include "PCCMath.h"

namespace pcc {

/// 3D point with float coordinates for Gaussian Splatting
struct PCCPointGS {
  float x, y, z;

  PCCPointGS() : x( 0.0f ), y( 0.0f ), z( 0.0f ) {}
  PCCPointGS( float x_, float y_, float z_ ) : x( x_ ), y( y_ ), z( z_ ) {}

  float &operator[]( size_t i ) {
    assert( i < 3 );
    return ( &x )[i];
  }
  const float &operator[]( size_t i ) const {
    assert( i < 3 );
    return ( &x )[i];
  }

  PCCPointGS operator+( const PCCPointGS &rhs ) const { return PCCPointGS( x + rhs.x, y + rhs.y, z + rhs.z ); }
  PCCPointGS operator-( const PCCPointGS &rhs ) const { return PCCPointGS( x - rhs.x, y - rhs.y, z - rhs.z ); }
  PCCPointGS operator*( float s ) const { return PCCPointGS( x * s, y * s, z * s ); }
  PCCPointGS operator/( float s ) const { return PCCPointGS( x / s, y / s, z / s ); }

  float getNorm2() const { return x * x + y * y + z * z; }
  float getNorm() const { return std::sqrt( getNorm2() ); }
};

/// 3D point with double coordinates for high precision
struct PCCPointGSD {
  double x, y, z;

  PCCPointGSD() : x( 0.0 ), y( 0.0 ), z( 0.0 ) {}
  PCCPointGSD( double x_, double y_, double z_ ) : x( x_ ), y( y_ ), z( z_ ) {}
  PCCPointGSD( const PCCPointGS &p ) : x( p.x ), y( p.y ), z( p.z ) {}

  double &operator[]( size_t i ) {
    assert( i < 3 );
    return ( &x )[i];
  }
  const double &operator[]( size_t i ) const {
    assert( i < 3 );
    return ( &x )[i];
  }

  PCCPointGSD operator+( const PCCPointGSD &rhs ) const { return PCCPointGSD( x + rhs.x, y + rhs.y, z + rhs.z ); }
  PCCPointGSD operator-( const PCCPointGSD &rhs ) const { return PCCPointGSD( x - rhs.x, y - rhs.y, z - rhs.z ); }
  PCCPointGSD operator*( double s ) const { return PCCPointGSD( x * s, y * s, z * s ); }
  PCCPointGSD operator/( double s ) const { return PCCPointGSD( x / s, y / s, z / s ); }

  double getNorm2() const { return x * x + y * y + z * z; }
  double getNorm() const { return std::sqrt( getNorm2() ); }

  PCCPointGS toFloat() const {
    return PCCPointGS( static_cast<float>( x ), static_cast<float>( y ), static_cast<float>( z ) );
  }
};

/// Gaussian Splatting rotation (quaternion)
struct GSRotation {
  float w, x, y, z;

  GSRotation() : w( 1.0f ), x( 0.0f ), y( 0.0f ), z( 0.0f ) {}
  GSRotation( float w_, float x_, float y_, float z_ ) : w( w_ ), x( x_ ), y( y_ ), z( z_ ) {}

  float &operator[]( size_t i ) {
    assert( i < 4 );
    return ( &w )[i];
  }
  const float &operator[]( size_t i ) const {
    assert( i < 4 );
    return ( &w )[i];
  }

  void normalize() {
    float norm = std::sqrt( w * w + x * x + y * y + z * z );
    if ( norm > 0.0f ) {
      w /= norm;
      x /= norm;
      y /= norm;
      z /= norm;
    }
  }
};

/// Gaussian Splatting scale
struct GSScale {
  float x, y, z;

  GSScale() : x( 1.0f ), y( 1.0f ), z( 1.0f ) {}
  GSScale( float x_, float y_, float z_ ) : x( x_ ), y( y_ ), z( z_ ) {}

  float &operator[]( size_t i ) {
    assert( i < 3 );
    return ( &x )[i];
  }
  const float &operator[]( size_t i ) const {
    assert( i < 3 );
    return ( &x )[i];
  }
};

/// Spherical Harmonics coefficients (up to 16 sets of RGB)
struct GSSphericalHarmonics {
  static constexpr size_t MAX_SH_COEFFS = 16;
  float                   coeffs[MAX_SH_COEFFS][3];  // 16 x RGB

  GSSphericalHarmonics() { std::memset( coeffs, 0, sizeof( coeffs ) ); }

  float *operator[]( size_t i ) {
    assert( i < MAX_SH_COEFFS );
    return coeffs[i];
  }
  const float *operator[]( size_t i ) const {
    assert( i < MAX_SH_COEFFS );
    return coeffs[i];
  }
};

/// Bounding box for float points
struct PCCBoxGS {
  PCCPointGS min_;
  PCCPointGS max_;

  PCCBoxGS() {
    min_ = PCCPointGS( std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max() );
    max_ = PCCPointGS( std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest() );
  }

  void update( const PCCPointGS &point ) {
    for ( size_t i = 0; i < 3; ++i ) {
      if ( point[i] < min_[i] ) min_[i] = point[i];
      if ( point[i] > max_[i] ) max_[i] = point[i];
    }
  }

  PCCPointGS getCenter() const {
    return PCCPointGS( ( min_.x + max_.x ) * 0.5f, ( min_.y + max_.y ) * 0.5f, ( min_.z + max_.z ) * 0.5f );
  }

  PCCPointGS getSize() const { return max_ - min_; }

  bool isEmpty() const { return min_.x > max_.x || min_.y > max_.y || min_.z > max_.z; }
};

/**
 * @class PCCPointSetGS
 * @brief Point cloud class for Gaussian Splatting with native float/double support.
 *
 * This class stores point cloud data with floating-point precision,
 * specifically designed for Gaussian Splatting applications where
 * fractional coordinates and attributes must be preserved.
 */
class PCCPointSetGS {
 public:
  PCCPointSetGS()                                      = default;
  PCCPointSetGS( const PCCPointSetGS & )               = default;
  PCCPointSetGS &operator=( const PCCPointSetGS &rhs ) = default;
  ~PCCPointSetGS()                                     = default;

  // ==================== Element Access ====================

  /// Access position by index (const)
  const PCCPointGS &operator[]( size_t index ) const {
    assert( index < positions_.size() );
    return positions_[index];
  }

  /// Access position by index (mutable)
  PCCPointGS &operator[]( size_t index ) {
    assert( index < positions_.size() );
    return positions_[index];
  }

  /// Get position as double precision
  PCCPointGSD getPositionDouble( size_t index ) const {
    assert( index < positions_.size() );
    return PCCPointGSD( positions_[index] );
  }

  // ==================== Container Access ====================

  std::vector<PCCPointGS>       &getPositions() { return positions_; }
  const std::vector<PCCPointGS> &getPositions() const { return positions_; }

  std::vector<PCCColor3B>       &getColors() { return colors_; }
  const std::vector<PCCColor3B> &getColors() const { return colors_; }

  std::vector<GSRotation>       &getRotations() { return rotations_; }
  const std::vector<GSRotation> &getRotations() const { return rotations_; }

  std::vector<GSScale>       &getScales() { return scales_; }
  const std::vector<GSScale> &getScales() const { return scales_; }

  std::vector<float>       &getOpacities() { return opacities_; }
  const std::vector<float> &getOpacities() const { return opacities_; }

  std::vector<GSSphericalHarmonics>       &getSphericalHarmonics() { return sphericalHarmonics_; }
  const std::vector<GSSphericalHarmonics> &getSphericalHarmonics() const { return sphericalHarmonics_; }

  // ==================== Size & Capacity ====================

  size_t getPointCount() const { return positions_.size(); }
  bool   isEmpty() const { return positions_.empty(); }

  void resize( size_t count ) {
    positions_.resize( count );
    if ( withColors_ ) colors_.resize( count );
    if ( withRotations_ ) rotations_.resize( count );
    if ( withScales_ ) scales_.resize( count );
    if ( withOpacities_ ) opacities_.resize( count );
    if ( withSphericalHarmonics_ ) sphericalHarmonics_.resize( count );
  }

  void reserve( size_t count ) {
    positions_.reserve( count );
    if ( withColors_ ) colors_.reserve( count );
    if ( withRotations_ ) rotations_.reserve( count );
    if ( withScales_ ) scales_.reserve( count );
    if ( withOpacities_ ) opacities_.reserve( count );
    if ( withSphericalHarmonics_ ) sphericalHarmonics_.reserve( count );
  }

  void clear() {
    positions_.clear();
    colors_.clear();
    rotations_.clear();
    scales_.clear();
    opacities_.clear();
    sphericalHarmonics_.clear();
    withColors_             = false;
    withRotations_          = false;
    withScales_             = false;
    withOpacities_          = false;
    withSphericalHarmonics_ = false;
  }

  // ==================== Attribute Flags ====================

  bool hasColors() const { return withColors_; }
  bool hasRotations() const { return withRotations_; }
  bool hasScales() const { return withScales_; }
  bool hasOpacities() const { return withOpacities_; }
  bool hasSphericalHarmonics() const { return withSphericalHarmonics_; }

  void addColors() {
    withColors_ = true;
    colors_.resize( positions_.size() );
  }

  void addRotations() {
    withRotations_ = true;
    rotations_.resize( positions_.size() );
  }

  void addScales() {
    withScales_ = true;
    scales_.resize( positions_.size() );
  }

  void addOpacities() {
    withOpacities_ = true;
    opacities_.resize( positions_.size() );
  }

  void addSphericalHarmonics() {
    withSphericalHarmonics_ = true;
    sphericalHarmonics_.resize( positions_.size() );
  }

  // ==================== Point Operations ====================

  size_t addPoint( const PCCPointGS &position ) {
    positions_.push_back( position );
    if ( withColors_ ) colors_.push_back( PCCColor3B() );
    if ( withRotations_ ) rotations_.push_back( GSRotation() );
    if ( withScales_ ) scales_.push_back( GSScale() );
    if ( withOpacities_ ) opacities_.push_back( 1.0f );
    if ( withSphericalHarmonics_ ) sphericalHarmonics_.push_back( GSSphericalHarmonics() );
    return positions_.size() - 1;
  }

  void setPosition( size_t index, const PCCPointGS &position ) {
    assert( index < positions_.size() );
    positions_[index] = position;
  }

  void setPosition( size_t index, float x, float y, float z ) {
    assert( index < positions_.size() );
    positions_[index] = PCCPointGS( x, y, z );
  }

  // ==================== Color Operations ====================

  PCCColor3B getColor( size_t index ) const {
    assert( index < colors_.size() && withColors_ );
    return colors_[index];
  }

  PCCColor3B &getColor( size_t index ) {
    assert( index < colors_.size() && withColors_ );
    return colors_[index];
  }

  void setColor( size_t index, const PCCColor3B &color ) {
    assert( index < colors_.size() && withColors_ );
    colors_[index] = color;
  }

  // ==================== Rotation Operations ====================

  GSRotation getRotation( size_t index ) const {
    assert( index < rotations_.size() && withRotations_ );
    return rotations_[index];
  }

  GSRotation &getRotation( size_t index ) {
    assert( index < rotations_.size() && withRotations_ );
    return rotations_[index];
  }

  void setRotation( size_t index, const GSRotation &rotation ) {
    assert( index < rotations_.size() && withRotations_ );
    rotations_[index] = rotation;
  }

  // ==================== Scale Operations ====================

  GSScale getScale( size_t index ) const {
    assert( index < scales_.size() && withScales_ );
    return scales_[index];
  }

  GSScale &getScale( size_t index ) {
    assert( index < scales_.size() && withScales_ );
    return scales_[index];
  }

  void setScale( size_t index, const GSScale &scale ) {
    assert( index < scales_.size() && withScales_ );
    scales_[index] = scale;
  }

  // ==================== Opacity Operations ====================

  float getOpacity( size_t index ) const {
    assert( index < opacities_.size() && withOpacities_ );
    return opacities_[index];
  }

  float &getOpacity( size_t index ) {
    assert( index < opacities_.size() && withOpacities_ );
    return opacities_[index];
  }

  void setOpacity( size_t index, float opacity ) {
    assert( index < opacities_.size() && withOpacities_ );
    opacities_[index] = opacity;
  }

  // ==================== Spherical Harmonics Operations ====================

  GSSphericalHarmonics &getSphericalHarmonic( size_t index ) {
    assert( index < sphericalHarmonics_.size() && withSphericalHarmonics_ );
    return sphericalHarmonics_[index];
  }

  const GSSphericalHarmonics &getSphericalHarmonic( size_t index ) const {
    assert( index < sphericalHarmonics_.size() && withSphericalHarmonics_ );
    return sphericalHarmonics_[index];
  }

  void setSphericalHarmonic( size_t index, const GSSphericalHarmonics &sh ) {
    assert( index < sphericalHarmonics_.size() && withSphericalHarmonics_ );
    sphericalHarmonics_[index] = sh;
  }

  /// Get number of SH coefficient sets present
  size_t getSphericalHarmonicsCount() const { return shCoefficientCount_; }

  /// Set number of SH coefficient sets
  void setSphericalHarmonicsCount( size_t count ) { shCoefficientCount_ = count; }

  // ==================== Geometry Operations ====================

  PCCPointGS computeCentroid() const;
  PCCBoxGS   computeBoundingBox() const;

  // ==================== I/O Operations ====================

  /// Read point cloud from PLY file (preserves float/double values)
  bool read( const std::string &fileName );

  /// Write point cloud to PLY file
  bool write( const std::string &fileName, bool asAscii = false ) const;

 private:
  // Position data (native float)
  std::vector<PCCPointGS> positions_;

  // Color data (8-bit RGB)
  std::vector<PCCColor3B> colors_;

  // Gaussian Splatting specific attributes (native float)
  std::vector<GSRotation>           rotations_;
  std::vector<GSScale>              scales_;
  std::vector<float>                opacities_;
  std::vector<GSSphericalHarmonics> sphericalHarmonics_;

  // Attribute presence flags
  bool withColors_             = false;
  bool withRotations_          = false;
  bool withScales_             = false;
  bool withOpacities_          = false;
  bool withSphericalHarmonics_ = false;

  // Number of SH coefficient sets (1-16)
  size_t shCoefficientCount_ = 0;
};

}  // namespace pcc

#endif /* PCCPointSetGS_h */
