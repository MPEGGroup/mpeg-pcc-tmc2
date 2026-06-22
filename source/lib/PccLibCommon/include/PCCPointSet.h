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

#ifndef PCCPointSet_h
#define PCCPointSet_h

#include "PCCCommon.h"
#include "PCCMath.h"

#include <unordered_set>

namespace pcc {

enum TransferAttributeType {
  TRAN_COLOR,
  TRAN_ROTATION0,
  TRAN_ROTATION1,
  TRAN_SCALE,
  TRAN_OPACITY,
  TRAN_SH0,
  TRAN_SH1,
  TRAN_SH2,
  TRAN_SH3,
  TRAN_SH4,
  TRAN_SH5,
  TRAN_SH6,
  TRAN_SH7,
  TRAN_SH8,
  TRAN_SH9,
  TRAN_SH10,
  TRAN_SH11,
  TRAN_SH12,
  TRAN_SH13,
  TRAN_SH14,
  TRAN_SH15
};

struct CameraInfo {
  int         imageId  = 0;
  int         cameraId = 0;
  double      posX = 0, posY = 0, posZ = 0;
  double      quatW = 0, quatX = 0, quatY = 0, quatZ = 0;
  double      focalX = 0, focalY = 0;
  std::string name;
};

// parameters for AAPS
struct GeometryTransformationInfo {
  bool   enabled = false;
  int    scale   = 1;
  double x = 0, y = 0, z = 0;
};

struct AttributeTransformationInfo {
  std::string name;

  bool offset_enabled = false;
  int  offset         = 0;

  bool     scale_enabled = false;
  uint32_t scale         = 0;
};

struct GeometryDequantizationParameters {
  bool   enabled = false;
  double posMin[3];
  double posMax[3];
};

struct AttributeRotationDequantizationParameters {
  bool   enabled = false;
  double rotMin[4];
  double rotMax[4];
};

struct AttributeScaleDequantizationParameters {
  bool   enabled = false;
  double scaleMin[3];
  double scaleMax[4];
};

struct AttributeOpacityDequantizationParameters {
  bool   enabled = false;
  double opMin   = 0;
  double opMax   = 0;
};

struct AttributeShDequantizationParameters {
  bool                             enabled  = false;
  int                              numLevel = 0;
  std::vector<int>                 numChannels;
  std::vector<std::vector<double>> shMin;
  std::vector<std::vector<double>> shMax;
};

class PCCPointSet3 {
 public:
  PCCPointSet3() :
      withNormals_( false ),
      withColors_( false ),
      withReflectances_( false ),
      withSphericalHarmonics_( false ),
      withRotations_( false ),
      withScales_( false ),
      withOpacities_( false ) {}
  PCCPointSet3( const PCCPointSet3 & )               = default;
  PCCPointSet3 &operator=( const PCCPointSet3 &rhs ) = default;
  ~PCCPointSet3()                                    = default;

  PCCPoint3D operator[]( const size_t index ) const {
    assert( index < positions_.size() );
    return positions_[index];
  }
  PCCPoint3D &operator[]( const size_t index ) {
    assert( index < positions_.size() );
    return positions_[index];
  }
  size_t appendPointSet( PCCPointSet3 &pointSet ) {
    std::vector<PCCPoint3D>::iterator                     itPositions;
    std::vector<PCCColor3B>::iterator                     itColors;
    std::vector<PCCColor16bit>::iterator                  itColors16bit;
    std::vector<uint16_t>::iterator                       itReflectances;
    std::vector<uint16_t>::iterator                       itBoundaryPointTypes;
    std::vector<std::pair<size_t, size_t>>::iterator      itPointPatchIndexes;
    std::vector<uint8_t>::iterator                        itTypes;
    std::vector<PCCNormal3D>::iterator                    itNormals;
    std::vector<std::vector<PCCAttribute16bit>>::iterator itRotations;
    std::vector<PCCAttribute16bit>::iterator              itScales;
    std::vector<std::vector<PCCAttribute16bit>>::iterator itSphericalHarmonics;
    std::vector<PCCAttribute16bit>::iterator              itOpacities;
    std::vector<uint32_t>::iterator                       itToSrcIdx;
    std::vector<uint16_t>::iterator                       itQuantizationGroupIds;

    itPositions            = positions_.end();
    itColors               = colors_.end();
    itColors16bit          = colors16bit_.end();
    itReflectances         = reflectances_.end();
    itBoundaryPointTypes   = boundaryPointTypes_.end();
    itPointPatchIndexes    = pointPatchIndexes_.end();
    itTypes                = types_.end();
    itNormals              = normals_.end();
    itRotations            = rotations_.end();
    itScales               = scales_.end();
    itSphericalHarmonics   = sphericalHarmonics_.end();
    itOpacities            = opacities_.end();
    itToSrcIdx             = toSrcIdx_.end();
    itQuantizationGroupIds = quantizationGroupIds_.end();

    positions_.insert( itPositions, pointSet.getPositions().begin(), pointSet.getPositions().end() );
    colors_.insert( itColors, pointSet.getColor().begin(), pointSet.getColor().end() );
    colors16bit_.insert( itColors16bit, pointSet.getColor16bit().begin(), pointSet.getColor16bit().end() );
    reflectances_.insert( itReflectances, pointSet.getReflectances().begin(), pointSet.getReflectances().end() );
    boundaryPointTypes_.insert( itBoundaryPointTypes, pointSet.getBoundaryPointTypes().begin(),
                                pointSet.getBoundaryPointTypes().end() );
    pointPatchIndexes_.insert( itPointPatchIndexes, pointSet.getPointPatchIndexes().begin(),
                               pointSet.getPointPatchIndexes().end() );
    types_.insert( itTypes, pointSet.getTypes().begin(), pointSet.getTypes().end() );
    normals_.insert( itNormals, pointSet.getNormals().begin(), pointSet.getNormals().end() );
    rotations_.insert( itRotations, pointSet.getRotations().begin(), pointSet.getRotations().end() );
    scales_.insert( itScales, pointSet.getScales().begin(), pointSet.getScales().end() );
    sphericalHarmonics_.insert( itSphericalHarmonics, pointSet.getSphericalHarmonics().begin(),
                                pointSet.getSphericalHarmonics().end() );
    opacities_.insert( itOpacities, pointSet.getOpacities().begin(), pointSet.getOpacities().end() );

    toSrcIdx_.insert( itToSrcIdx, pointSet.getToSrcIdx().begin(), pointSet.getToSrcIdx().end() );
    quantizationGroupIds_.insert( itQuantizationGroupIds, pointSet.getQuantizationGroupIds().begin(),
                                  pointSet.getQuantizationGroupIds().end() );

    resize( getPointCount() );
    return positions_.size();
  }
  std::vector<uint16_t> &getBoundaryPointTypes() { return boundaryPointTypes_; }
  void                   setPosition( const size_t index, const PCCPoint3D position ) {
    assert( index < positions_.size() );
    positions_[index] = position;
  }
  std::vector<PCCColor3B> &getColor() { return colors_; }
  PCCColor3B               getColor( const size_t index ) const {
    assert( index < colors_.size() && withColors_ );
    return colors_[index];
  }
  PCCColor3B &getColor( const size_t index ) {
    assert( index < colors_.size() && withColors_ );
    return colors_[index];
  }
  void setColor( const size_t index, const PCCColor3B color ) {
    assert( index < colors_.size() && withColors_ );
    colors_[index] = color;
  }
  std::vector<PCCColor16bit> &getColor16bit() { return colors16bit_; }
  PCCColor16bit               getColor16bit( const size_t index ) const {
    assert( index < colors16bit_.size() && withColors_ );
    return colors16bit_[index];
  }
  PCCColor16bit &getColor16bit( const size_t index ) {
    assert( index < colors16bit_.size() && withColors_ );
    return colors16bit_[index];
  }
  void setColor16bit( const size_t index, const PCCColor16bit color16bit ) {
    assert( index < colors16bit_.size() && withColors_ );
    colors16bit_[index] = color16bit;
  }
  void copyRGB16ToRGB8() {
    for ( size_t k = 0; k < getPointCount(); k++ ) {
      colors_[k][0] = uint8_t( colors16bit_[k][0] );
      colors_[k][1] = uint8_t( colors16bit_[k][1] );
      colors_[k][2] = uint8_t( colors16bit_[k][2] );
    }
  }

  void fillColor( PCCColor3B color = { 0, 0, 0 } ) {
    for ( size_t k = 0; k < getPointCount(); k++ ) { colors_[k] = color; }
  }

  /// convert yuv444 (16bit) to normalized yuv444 (format double)
  void convertYUV16ToRGB8() {
    for ( size_t k = 0; k < getPointCount(); k++ ) {
      double y1     = colors16bit_[k][0];
      double u1     = colors16bit_[k][1];
      double v1     = colors16bit_[k][2];
      double offset = 32768.0;
      double scale  = 65535.0;
      double weight = 1.0 / scale;

      y1 = weight * y1;
      u1 = weight * ( u1 - offset );
      v1 = weight * ( v1 - offset );
      y1 = (std::max)( y1, 0.0 );
      y1 = (std::min)( y1, 1.0 );
      u1 = (std::max)( u1, -0.5 );
      u1 = (std::min)( u1, 0.5 );
      v1 = (std::max)( v1, -0.5 );
      v1 = (std::min)( v1, 0.5 );

      //// convert normalized yuv444 to normalized rgb (fromat double)
      double r = y1 /*- 0.00000 * u1*/ + 1.57480 * v1;
      double g = y1 - 0.18733 * u1 - 0.46813 * v1;
      double b = y1 + 1.85563 * u1 /*+ 0.00000 * v1*/;

      //// convert normalized rgb to 8-bit rgb
      r = PCCClip( round( r * 255 ), 0.0, 255.0 );
      g = PCCClip( round( g * 255 ), 0.0, 255.0 );
      b = PCCClip( round( b * 255 ), 0.0, 255.0 );

      colors_[k][0] = static_cast<uint8_t>( r );
      colors_[k][1] = static_cast<uint8_t>( g );
      colors_[k][2] = static_cast<uint8_t>( b );
    }
  }

  uint16_t getBoundaryPointType( const size_t index ) const {
    assert( index < boundaryPointTypes_.size() );
    return boundaryPointTypes_[index];
  }
  uint16_t &getBoundaryPointType( const size_t index ) {
    assert( index < boundaryPointTypes_.size() );
    return boundaryPointTypes_[index];
  }
  void setBoundaryPointType( const size_t index, const uint16_t BoundaryPointType ) {
    assert( index < boundaryPointTypes_.size() );
    boundaryPointTypes_[index] = BoundaryPointType;
  }
  std::vector<std::pair<size_t, size_t>> &getPointPatchIndexes() { return pointPatchIndexes_; }
  std::pair<size_t, size_t>               getPointPatchIndex( const size_t index ) const {
    assert( index < pointPatchIndexes_.size() );
    return pointPatchIndexes_[index];
  }
  std::pair<size_t, size_t> &getPointPatchIndex( const size_t index ) {
    assert( index < pointPatchIndexes_.size() );
    return pointPatchIndexes_[index];
  }
  void setPointPatchIndex( const size_t index, const uint32_t tileIndex, const uint32_t patchIndex ) {
    assert( index < pointPatchIndexes_.size() );
    pointPatchIndexes_[index].first  = tileIndex;
    pointPatchIndexes_[index].second = patchIndex;
  }
  std::vector<uint64_t> &getParentPointIndex() { return parentPointIndex_; }
  uint64_t              &getParentPointIndex( const size_t index ) { return parentPointIndex_[index]; }
  void                   setParentPointIndex( const size_t index, const uint64_t parentIndex ) {
    assert( index < parentPointIndex_.size() );
    parentPointIndex_[index] = parentIndex;
  }
  uint16_t getReflectance( const size_t index ) const {
    assert( index < reflectances_.size() && withReflectances_ );
    return reflectances_[index];
  }
  uint16_t &getReflectance( const size_t index ) {
    assert( index < reflectances_.size() && withReflectances_ );
    return reflectances_[index];
  }
  void setReflectance( const size_t index, const uint16_t reflectance ) {
    assert( index < reflectances_.size() && withReflectances_ );
    reflectances_[index] = reflectance;
  }
  std::vector<PCCPoint3D>       &getPositions() { return positions_; }
  const std::vector<PCCPoint3D> &getPositions() const { return positions_; }
  std::vector<PCCColor3B>       &getColors() { return colors_; }
  std::vector<PCCColor16bit>    &getColors16bit() { return colors16bit_; }
  std::vector<uint16_t>         &getReflectances() { return reflectances_; }
  std::vector<uint8_t>          &getTypes() { return types_; }

  bool hasReflectances() const { return withReflectances_; }
  void addReflectances() {
    withReflectances_ = true;
    resize( getPointCount() );
  }
  void removeReflectances() {
    withReflectances_ = false;
    reflectances_.resize( 0 );
  }
  uint8_t getType( const size_t index ) const {
    assert( index < types_.size() );
    return types_[index];
  }
  uint8_t &getType( const size_t index ) {
    assert( index < types_.size() );
    return types_[index];
  }
  void setType( const size_t index, const uint8_t type ) {
    assert( index < types_.size() );
    types_[index] = type;
  }

  bool hasColors() const { return withColors_; }
  void addColors() {
    withColors_ = true;
    resize( getPointCount() );
  }
  void removeColors() {
    withColors_ = false;
    colors_.resize( 0 );
  }
  void addColors16bit() {
    withColors_ = true;
    resize( getPointCount() );
  }
  void removeColors16bit() {
    withColors_ = false;
    colors16bit_.resize( 0 );
  }
  const std::vector<PCCNormal3D> &getNormals() const { return normals_; }
  bool                            hasNormals() const { return withNormals_; }
  void                            addNormals() {
    withNormals_ = true;
    resize( getPointCount() );
  }
  void removeNormals() {
    withNormals_ = false;
    normals_.resize( 0 );
  }
  void setNormal( size_t idx, PCCNormal3D value ) {
    if ( idx > normals_.size() ) exit( -1 );
    normals_[idx] = value;
  }

  bool hasSphericalHarmonics() const { return withSphericalHarmonics_; }
  void addSphericalHarmonics() {
    withSphericalHarmonics_ = true;
    sphericalHarmonics_.resize( getPointCount() );
    for ( int i = 0; i < getPointCount(); i++ ) { sphericalHarmonics_[i].resize( 16 ); }
  }
  void removeSphericalHarmonics() {
    withSphericalHarmonics_ = false;
    sphericalHarmonics_.resize( 0 );
  }
  std::vector<std::vector<PCCAttribute16bit>>       &getSphericalHarmonics() { return sphericalHarmonics_; }
  const std::vector<std::vector<PCCAttribute16bit>> &getSphericalHarmonics() const { return sphericalHarmonics_; }
  std::vector<PCCAttribute16bit>                    &getSphericalHarmonic( const size_t index ) {
    assert( index < getPointCount() && withSphericalHarmonics_ );
    return sphericalHarmonics_[index];
  }
  std::vector<PCCAttribute16bit> getSphericalHarmonic( const size_t index ) const {
    assert( index < getPointCount() && withSphericalHarmonics_ );
    return sphericalHarmonics_[index];
  }

  std::vector<PCCAttribute16bit> getSphericalHarmonicPartition( const size_t index ) const {
    assert( index < 16 && withSphericalHarmonics_ );
    // now each point has 16 spherical harmonics parts
    std::vector<PCCAttribute16bit> sphericalHarmonicPart;
    sphericalHarmonicPart.resize( getPointCount() );
    for ( size_t k = 0; k < getPointCount(); k++ ) {
      sphericalHarmonicPart[k][0] = sphericalHarmonics_[k][index][0];
      sphericalHarmonicPart[k][1] = sphericalHarmonics_[k][index][1];
      sphericalHarmonicPart[k][2] = sphericalHarmonics_[k][index][2];
    }
    return sphericalHarmonicPart;
  }
  void setSphericalHarmonic( const size_t index, const std::vector<PCCAttribute16bit> sphericalHarmonic ) {
    assert( index < getPointCount() && withSphericalHarmonics_ );
    sphericalHarmonics_[index] = sphericalHarmonic;
  }
  void setSphericalHarmonic( const size_t index, const size_t partIndex, const PCCAttribute16bit sphericalHarmonic ) {
    assert( index < getPointCount() && withSphericalHarmonics_ && partIndex < 16 );
    sphericalHarmonics_[index][partIndex] = sphericalHarmonic;
  }
  void fillSphericalHarmonic( PCCAttribute16bit attribute = { 0, 0, 0 } ) {
    std::vector<PCCAttribute16bit> attributes( 16, attribute );
    for ( size_t k = 0; k < getPointCount(); k++ ) { sphericalHarmonics_[k] = attributes; }
  }
  void fillSphericalHarmonic( int index, PCCAttribute16bit attribute = { 0, 0, 0 } ) {
    assert( index < 16 );
    for ( size_t k = 0; k < getPointCount(); k++ ) { sphericalHarmonics_[k][index] = attribute; }
  }

  bool hasOpacities() const { return withOpacities_; }
  void addOpacities() {
    withOpacities_ = true;
    opacities_.resize( getPointCount() );
  }
  void removeOpacities() {
    withOpacities_ = false;
    opacities_.resize( 0 );
  }
  std::vector<PCCAttribute16bit>       &getOpacities() { return opacities_; }
  const std::vector<PCCAttribute16bit> &getOpacities() const { return opacities_; }
  PCCAttribute16bit                    &getOpacity( const size_t index ) {
    assert( index < opacities_.size() && withOpacities_ );
    return opacities_[index];
  }
  PCCAttribute16bit getOpacity( const size_t index ) const {
    assert( index < opacities_.size() && withOpacities_ );
    return opacities_[index];
  }
  void setOpacity( const size_t index, const PCCAttribute16bit opacity ) {
    assert( index < opacities_.size() && withOpacities_ );
    opacities_[index] = opacity;
  }
  void fillOpacity( PCCAttribute16bit attribute = { 0, 0, 0 } ) {
    for ( size_t k = 0; k < getPointCount(); k++ ) { opacities_[k] = attribute; }
  }

  bool hasScales() const { return withScales_; }
  void addScales() {
    withScales_ = true;
    scales_.resize( getPointCount() );
  }
  void removeScales() {
    withScales_ = false;
    scales_.resize( 0 );
  }
  std::vector<PCCAttribute16bit>       &getScales() { return scales_; }
  const std::vector<PCCAttribute16bit> &getScales() const { return scales_; }
  PCCAttribute16bit                    &getScale( const size_t index ) {
    assert( index < getPointCount() && withScales_ );
    return scales_[index];
  }
  PCCAttribute16bit getScale( const size_t index ) const {
    assert( index < getPointCount() && withScales_ );
    return scales_[index];
  }
  void setScale( const size_t index, const PCCAttribute16bit scale ) {
    assert( index < getPointCount() && withScales_ );
    scales_[index] = scale;
  }
  void fillScale( PCCAttribute16bit attribute = { 0, 0, 0 } ) {
    for ( size_t k = 0; k < getPointCount(); k++ ) { scales_[k] = attribute; }
  }

  bool hasRotations() const { return withRotations_; }
  void addRotations() {
    withRotations_ = true;
    rotations_.resize( getPointCount() );
    for ( int i = 0; i < getPointCount(); i++ ) { rotations_[i].resize( 2 ); }
  }
  void removeRotations() {
    withRotations_ = false;
    rotations_.resize( 0 );
  }
  std::vector<std::vector<PCCAttribute16bit>>       &getRotations() { return rotations_; }
  const std::vector<std::vector<PCCAttribute16bit>> &getRotations() const { return rotations_; }
  std::vector<PCCAttribute16bit>                    &getRotation( const size_t index ) {
    assert( index < getPointCount() && withRotations_ );
    return rotations_[index];
  }
  std::vector<PCCAttribute16bit> getRotation( const size_t index ) const {
    assert( index < getPointCount() && withRotations_ );
    return rotations_[index];
  }

  std::vector<PCCAttribute16bit> getRotationPartition( const size_t index ) const {
    assert( index < 2 && withRotations_ );
    // now each point has 2 parts
    std::vector<PCCAttribute16bit> rotationPart;
    rotationPart.resize( getPointCount() );
    for ( size_t k = 0; k < getPointCount(); k++ ) {
      rotationPart[k][0] = rotations_[k][index][0];
      rotationPart[k][1] = rotations_[k][index][1];
      rotationPart[k][2] = rotations_[k][index][2];
    }
    return rotationPart;
  }

  void setRotation( const size_t index, const std::vector<PCCAttribute16bit> rotation ) {
    assert( index < getPointCount() && withRotations_ );
    rotations_[index] = rotation;
  }
  void setRotation( const size_t index, const size_t partIndex, const PCCAttribute16bit rotation ) {
    assert( index < getPointCount() && withRotations_ && partIndex < 2 );
    rotations_[index][partIndex] = rotation;
  }
  void fillRotation( PCCAttribute16bit attribute = { 0, 0, 0 } ) {
    std::vector<PCCAttribute16bit> attributes( 2, attribute );
    for ( size_t k = 0; k < getPointCount(); k++ ) { rotations_[k] = attributes; }
  }
  void fillRotation( int index, PCCAttribute16bit attribute = { 0, 0, 0 } ) {
    assert( index < 2 );
    for ( size_t k = 0; k < getPointCount(); k++ ) { rotations_[k][index] = attribute; }
  }

  bool hasAttributeGS( const TransferAttributeType attributeType ) const {
    if ( attributeType == TRAN_ROTATION0 || attributeType == TRAN_ROTATION1 )
      return withRotations_;
    else if ( attributeType == TRAN_SCALE )
      return withScales_;
    else if ( attributeType == TRAN_OPACITY )
      return withOpacities_;
    else if ( attributeType >= TRAN_SH0 && attributeType <= TRAN_SH15 )
      return withSphericalHarmonics_;

    std::cerr << "Error: hasAttributeGS invalid attribute type " << int( attributeType ) << std::endl;
    exit( -1 );
  }
  void fillAttributeGS( const TransferAttributeType attributeType, PCCAttribute16bit attribute = { 0, 0, 0 } ) {
    if ( attributeType == TRAN_ROTATION0 || attributeType == TRAN_ROTATION1 )
      fillRotation( int( attributeType - TRAN_ROTATION0 ), attribute );
    else if ( attributeType == TRAN_SCALE )
      fillScale( attribute );
    else if ( attributeType == TRAN_OPACITY )
      fillOpacity( attribute );
    else if ( attributeType >= TRAN_SH0 && attributeType <= TRAN_SH15 )
      fillSphericalHarmonic( int( attributeType - TRAN_SH0 ), attribute );
  }

  bool hasAttribute16bitGS( const TransferAttributeType attributeType ) const {
    if ( attributeType == TRAN_ROTATION0 || attributeType == TRAN_ROTATION1 )
      return withRotations_;
    else if ( attributeType == TRAN_SCALE )
      return withScales_;
    else if ( attributeType == TRAN_OPACITY )
      return withOpacities_;
    else if ( attributeType >= TRAN_SH0 && attributeType <= TRAN_SH15 )
      return withSphericalHarmonics_;
  }
  void addAttribute( const TransferAttributeType attributeType ) {
    if ( attributeType == TRAN_ROTATION0 || attributeType == TRAN_ROTATION1 )
      addRotations();
    else if ( attributeType == TRAN_SCALE )
      addScales();
    else if ( attributeType == TRAN_OPACITY )
      addOpacities();
    else if ( attributeType >= TRAN_SH0 && attributeType <= TRAN_SH15 )
      addSphericalHarmonics();
  }
  PCCAttribute16bit getAttribute( const TransferAttributeType attributeType, const size_t index ) const {
    if ( attributeType == TRAN_ROTATION0 || attributeType == TRAN_ROTATION1 )
      return getRotation( index )[attributeType - TRAN_ROTATION0];
    else if ( attributeType == TRAN_SCALE )
      return getScale( index );
    else if ( attributeType == TRAN_OPACITY )
      return getOpacity( index );
    else if ( attributeType >= TRAN_SH0 && attributeType <= TRAN_SH15 )
      return getSphericalHarmonic( index )[attributeType - TRAN_SH0];

    std::cerr << "Error: getAttribute invalid attribute type " << int( attributeType ) << std::endl;
    exit( -1 );
  }
  PCCAttribute16bit &getAttribute( const TransferAttributeType attributeType, const size_t index ) {
    if ( attributeType == TRAN_ROTATION0 || attributeType == TRAN_ROTATION1 )
      return getRotation( index )[attributeType - TRAN_ROTATION0];
    else if ( attributeType == TRAN_SCALE )
      return getScale( index );
    else if ( attributeType == TRAN_OPACITY )
      return getOpacity( index );
    else if ( attributeType >= TRAN_SH0 && attributeType <= TRAN_SH15 )
      return getSphericalHarmonic( index )[attributeType - TRAN_SH0];

    std::cerr << "Error: getAttribute invalid attribute type " << int( attributeType ) << std::endl;
    exit( -1 );
  }
  void setAttribute( const TransferAttributeType attributeType,
                     const size_t                index,
                     const PCCAttribute16bit     attribute ) {
    if ( attributeType == TRAN_ROTATION0 || attributeType == TRAN_ROTATION1 )
      setRotation( index, size_t( attributeType - TRAN_ROTATION0 ), attribute );
    else if ( attributeType == TRAN_SCALE )
      setScale( index, attribute );
    else if ( attributeType == TRAN_OPACITY )
      setOpacity( index, attribute );
    else if ( attributeType >= TRAN_SH0 && attributeType <= TRAN_SH15 )
      setSphericalHarmonic( index, attributeType - TRAN_SH0, attribute );
  }

  void setAttribute( const int               attrTypeId,
                     const int               attrPartIndex,
                     const size_t            index,
                     const PCCAttribute16bit attribute ) {
    if ( attrTypeId == 7 )
      setRotation( index, attrPartIndex, attribute );
    else if ( attrTypeId == 6 )
      setScale( index, attribute );
    else if ( attrTypeId == 2 )
      setOpacity( index, attribute );
    else if ( attrTypeId == 5 )
      setSphericalHarmonic( index, attrPartIndex, attribute );
  }

  uint32_t getToSrcIdx( const size_t index ) {
    assert( index < getPointCount() );
    return toSrcIdx_[index];
  }

  void setToSrcIdx( const size_t index, const uint32_t idx ) {
    assert( index < getPointCount() );
    toSrcIdx_[index] = idx;
  }

  std::vector<uint32_t> &getToSrcIdx() { return toSrcIdx_; }

  std::vector<uint16_t>       &getQuantizationGroupIds() { return quantizationGroupIds_; }
  const std::vector<uint16_t> &getQuantizationGroupIds() const { return quantizationGroupIds_; }
  const uint16_t               getQuantizationGroupId( const size_t index ) const {
    assert( hasQuantizationGroupIds() && index < quantizationGroupIds_.size() );
    return quantizationGroupIds_[index];
  }
  uint16_t &getQuantizationGroupId( const size_t index ) {
    assert( hasQuantizationGroupIds() && index < quantizationGroupIds_.size() );
    return quantizationGroupIds_[index];
  }
  bool hasQuantizationGroupIds() const {
    return !quantizationGroupIds_.empty() && quantizationGroupIds_.size() == positions_.size();
  }
  void setQuantizationGroupId( const size_t index, const uint16_t quantizationGroupId ) {
    assert( hasQuantizationGroupIds() && index < quantizationGroupIds_.size() );
    quantizationGroupIds_[index] = quantizationGroupId;
  }
  void   addQuantizationGroupIds() { quantizationGroupIds_.resize( getPointCount() ); }
  void   removeQuantizationGroupIds() { quantizationGroupIds_.resize( 0 ); }
  size_t getMaxQuantizationGroupId() const {
    if ( !hasQuantizationGroupIds() ) { return 0; }

    uint32_t maxGroupId = 0;
    for ( const auto groupId : quantizationGroupIds_ ) {
      if ( groupId > maxGroupId ) { maxGroupId = groupId; }
    }
    return static_cast<size_t>( maxGroupId );
  }
  std::vector<int>       &getQuantizationGroupBitDepthDeltas() { return quantizationGroupBitDepthDeltas_; }
  const std::vector<int> &getQuantizationGroupBitDepthDeltas() const { return quantizationGroupBitDepthDeltas_; }
  const int               getQuantizationGroupBitDepthDelta( const size_t index ) const {
    assert( index < quantizationGroupBitDepthDeltas_.size() );
    return quantizationGroupBitDepthDeltas_[index];
  }

  std::vector<CameraInfo>       &getCameraInfo() { return cameraInfo_; }
  const std::vector<CameraInfo> &getCameraInfo() const { return cameraInfo_; }

  std::vector<AttributeTransformationInfo> &getAttributeTransformationInfo() { return attributeTransformationInfo_; }
  const std::vector<AttributeTransformationInfo> &getAttributeTransformationInfo() const {
    return attributeTransformationInfo_;
  }

  GeometryTransformationInfo       &getGeometryTransformationInfo() { return geometryTransformationInfo_; }
  const GeometryTransformationInfo &getGeometryTransformationInfo() const { return geometryTransformationInfo_; }

  GeometryDequantizationParameters &getGeometryDequantizationParameters() { return geometryDequantizationParameters_; }
  const GeometryDequantizationParameters &getGeometryDequantizationParameters() const {
    return geometryDequantizationParameters_;
  }
  AttributeRotationDequantizationParameters &getAttributeRotationDequantizationParameters() {
    return attributeRotationDequantizationParameters_;
  }
  const AttributeRotationDequantizationParameters &getAttributeRotationDequantizationParameters() const {
    return attributeRotationDequantizationParameters_;
  }
  AttributeScaleDequantizationParameters &getAttributeScaleDequantizationParameters() {
    return attributeScaleDequantizationParameters_;
  }
  const AttributeScaleDequantizationParameters &getAttributeScaleDequantizationParameters() const {
    return attributeScaleDequantizationParameters_;
  }
  AttributeOpacityDequantizationParameters &getAttributeOpacityDequantizationParameters() {
    return attributeOpacityDequantizationParameters_;
  }
  const AttributeOpacityDequantizationParameters &getAttributeOpacityDequantizationParameters() const {
    return attributeOpacityDequantizationParameters_;
  }
  AttributeShDequantizationParameters &getAttributeShDequantizationParameters() {
    return attributeShDequantizationParameters_;
  }
  const AttributeShDequantizationParameters &getAttributeShDequantizationParameters() const {
    return attributeShDequantizationParameters_;
  }

  void trace() {
    size_t pointCount = getPointCount();
    printf( "Point cloud of %zu points : \n", pointCount );
    for ( size_t i = 0; i < 16; i++ ) {
      printf( "%9zu: ( %4u, %4u, %4u ) [ %4u, %4u, %4u ] [ %4u, %4u, %4u ] \n", i, positions_[i][0], positions_[i][1],
              positions_[i][2], colors_[i][0], colors_[i][1], colors_[i][2], colors16bit_[i][0], colors16bit_[i][1],
              colors16bit_[i][2] );
    }
    for ( size_t i = 50000; i < pointCount; i += 50000 ) {
      printf( "%9zu: ( %4u, %4u, %4u ) [ %4u, %4u, %4u ] [ %4u, %4u, %4u ] \n", i, positions_[i][0], positions_[i][1],
              positions_[i][2], colors_[i][0], colors_[i][1], colors_[i][2], colors16bit_[i][0], colors16bit_[i][1],
              colors16bit_[i][2] );
    }
    fflush( stdout );
  }

  bool transferColors( PCCPointSet3 &target,
                       const int32_t searchRange,
                       const bool    losslessAttribute                       = false,
                       const int     numNeighborsColorTransferFwd            = 1,
                       const int     numNeighborsColorTransferBwd            = 1,
                       const bool    useDistWeightedAverageFwd               = true,
                       const bool    useDistWeightedAverageBwd               = true,
                       const bool    skipAvgIfIdenticalSourcePointPresentFwd = true,
                       const bool    skipAvgIfIdenticalSourcePointPresentBwd = true,
                       const double  distOffsetFwd                           = 0.0001,
                       const double  distOffsetBwd                           = 0.0001,
                       double        maxGeometryDist2Fwd                     = 10000.0,
                       double        maxGeometryDist2Bwd                     = 10000.0,
                       double        maxColorDist2Fwd                        = 10000.0,
                       double        maxColorDist2Bwd                        = 10000.0,
                       const bool    excludeColorOutlier                     = false,
                       const double  thresholdColorOutlierDist               = 10.0 ) const;

  void transferAttributes( PCCPointSet3 &target,
                           const int32_t searchRange,
                           bool          loslessTransferAttributes,
                           const bool    losslessAttribute                       = false,
                           const int     numNeighborsColorTransferFwd            = 1,
                           const int     numNeighborsColorTransferBwd            = 1,
                           const bool    useDistWeightedAverageFwd               = true,
                           const bool    useDistWeightedAverageBwd               = true,
                           const bool    skipAvgIfIdenticalSourcePointPresentFwd = true,
                           const bool    skipAvgIfIdenticalSourcePointPresentBwd = true,
                           const double  distOffsetFwd                           = 0.0001,
                           const double  distOffsetBwd                           = 0.0001,
                           double        maxGeometryDist2Fwd                     = 10000.0,
                           double        maxGeometryDist2Bwd                     = 10000.0,
                           double        maxColorDist2Fwd                        = 10000.0,
                           double        maxColorDist2Bwd                        = 10000.0,
                           const bool    excludeColorOutlier                     = false,
                           const double  thresholdColorOutlierDist               = 10.0 ) const;

  bool transferColors16bitBP( PCCPointSet3 &target,
                              const int     filterType,
                              const int32_t searchRange,
                              const bool    losslessAttribute                       = false,
                              const int     numNeighborsColorTransferFwd            = 1,
                              const int     numNeighborsColorTransferBwd            = 1,
                              const bool    useDistWeightedAverageFwd               = true,
                              const bool    useDistWeightedAverageBwd               = true,
                              const bool    skipAvgIfIdenticalSourcePointPresentFwd = true,
                              const bool    skipAvgIfIdenticalSourcePointPresentBwd = true,
                              const double  distOffsetFwd                           = 0.0001,
                              const double  distOffsetBwd                           = 0.0001,
                              double        maxGeometryDist2Fwd                     = 10000.0,
                              double        maxGeometryDist2Bwd                     = 10000.0,
                              double        maxColorDist2Fwd                        = 10000.0,
                              double        maxColorDist2Bwd                        = 10000.0,
                              const bool    excludeColorOutlier                     = false,
                              const double  thresholdColorOutlierDist               = 10.0 ) const;
  bool transferColorsBackward16bitBP( PCCPointSet3 &target,
                                      const int     filterType,
                                      const int32_t searchRange,
                                      const bool    losslessAttribute,
                                      const int     numNeighborsColorTransferFwd,
                                      const int     numNeighborsColorTransferBwd,
                                      const bool    useDistWeightedAverageFwd,
                                      const bool    useDistWeightedAverageBwd,
                                      const bool    skipAvgIfIdenticalSourcePointPresentFwd,
                                      const bool    skipAvgIfIdenticalSourcePointPresentBwd,
                                      const double  distOffsetFwd,
                                      const double  distOffsetBwd,
                                      double        maxGeometryDist2Fwd,
                                      double        maxGeometryDist2Bwd,
                                      double        maxColorDist2Fwd,
                                      double        maxColorDist2Bwd,
                                      const bool    excludeColorOutlier       = false,
                                      const double  thresholdColorOutlierDist = 10.0 ) const;

  bool transferColors16bit( PCCPointSet3 &target,
                            const int32_t searchRange,
                            const bool    losslessAttribute                       = false,
                            const int     numNeighborsColorTransferFwd            = 1,
                            const int     numNeighborsColorTransferBwd            = 1,
                            const bool    useDistWeightedAverageFwd               = true,
                            const bool    useDistWeightedAverageBwd               = true,
                            const bool    skipAvgIfIdenticalSourcePointPresentFwd = true,
                            const bool    skipAvgIfIdenticalSourcePointPresentBwd = true,
                            const double  distOffsetFwd                           = 0.0001,
                            const double  distOffsetBwd                           = 0.0001,
                            double        maxGeometryDist2Fwd                     = 10000.0,
                            double        maxGeometryDist2Bwd                     = 10000.0,
                            double        maxColorDist2Fwd                        = 10000.0,
                            double        maxColorDist2Bwd                        = 10000.0,
                            const bool    excludeColorOutlier                     = false,
                            const double  thresholdColorOutlierDist               = 10.0 ) const;

  bool transferColorsFilter3( PCCPointSet3 &target, const int32_t searchRange, const bool losslessAttribute ) const;

  bool transferColorSimple( PCCPointSet3 &target, const double bestColorSearchStep = 0.1 );

  bool transferColorWeight( PCCPointSet3 &target, const double bestColorSearchStep = 0.1 );

  bool transferAttributeWeight( PCCPointSet3               &target,
                                const TransferAttributeType attributeType,
                                const double                bestColorSearchStep = 0.1 );

  size_t getPointCount() const { return positions_.size(); }
  void   resize( const size_t size ) {
    positions_.resize( size );
    if ( hasColors() ) {
      colors_.resize( size );
      colors16bit_.resize( size );
    }
    if ( hasReflectances() ) { reflectances_.resize( size ); }
    if ( PCC_SAVE_POINT_TYPE ) { types_.resize( size ); }
    if ( hasNormals() ) { normals_.resize( size ); }
    boundaryPointTypes_.resize( size );
    pointPatchIndexes_.resize( size );
    parentPointIndex_.resize( size );
    toSrcIdx_.resize( size );
    if ( !quantizationGroupIds_.empty() ) { quantizationGroupIds_.resize( size ); }
    if ( hasSphericalHarmonics() ) {
      sphericalHarmonics_.resize( size );
      for ( int i = 0; i < getPointCount(); i++ ) { sphericalHarmonics_[i].resize( 16 ); }
    }
    if ( hasOpacities() ) { opacities_.resize( size ); }
    if ( hasScales() ) { scales_.resize( size ); }
    if ( hasRotations() ) {
      rotations_.resize( size );
      for ( int i = 0; i < getPointCount(); i++ ) { rotations_[i].resize( 2 ); }
    }
  }
  void reserve( const size_t size ) {
    positions_.reserve( size );
    if ( hasColors() ) {
      colors_.reserve( size );
      colors16bit_.reserve( size );
    }
    if ( hasReflectances() ) { reflectances_.reserve( size ); }
    if ( PCC_SAVE_POINT_TYPE ) { types_.reserve( size ); }
    boundaryPointTypes_.reserve( size );
    pointPatchIndexes_.reserve( size );
    parentPointIndex_.reserve( size );
    if ( !quantizationGroupIds_.empty() ) { quantizationGroupIds_.reserve( size ); }
    if ( hasSphericalHarmonics() ) {
      sphericalHarmonics_.reserve( size );
      for ( int i = 0; i < getPointCount(); i++ ) { sphericalHarmonics_[i].reserve( 16 ); }
    }
    if ( hasOpacities() ) { opacities_.reserve( size ); }
    if ( hasScales() ) { scales_.reserve( size ); }
    if ( hasRotations() ) {
      rotations_.reserve( size );
      for ( int i = 0; i < getPointCount(); i++ ) { rotations_[i].reserve( 16 ); }
    }
  }
  void clear() {
    positions_.clear();
    colors_.clear();
    colors16bit_.clear();
    reflectances_.clear();
    if ( PCC_SAVE_POINT_TYPE ) { types_.clear(); }
    boundaryPointTypes_.clear();
    pointPatchIndexes_.clear();
    parentPointIndex_.clear();
    normals_.clear();
    sphericalHarmonics_.clear();
    opacities_.clear();
    scales_.clear();
    rotations_.clear();
    quantizationGroupIds_.clear();
  }
  size_t addPoint( const PCCPoint3D &position ) {
    const size_t index = getPointCount();
    resize( index + 1 );
    positions_[index] = position;
    return index;
  }
  size_t addPoint( const PCCPoint3D &position, const PCCColor3B &color ) {
    withColors_        = true;
    const size_t index = getPointCount();
    resize( index + 1 );
    colors_.resize( index + 1 );
    positions_[index] = position;
    colors_[index]    = color;
    return index;
  }
  size_t addPoint( const PCCVector3D &position ) {
    const size_t index = getPointCount();
    resize( index + 1 );
    positions_[index][0] = (int16_t)position[0];
    positions_[index][1] = (int16_t)position[1];
    positions_[index][2] = (int16_t)position[2];
    return index;
  }
  size_t addPoint( const PCCVector3D &position, const PCCColor3B &color ) {
    withColors_        = true;
    const size_t index = getPointCount();
    resize( index + 1 );
    colors_.resize( index + 1 );
    positions_[index][0] = (int16_t)position[0];
    positions_[index][1] = (int16_t)position[1];
    positions_[index][2] = (int16_t)position[2];
    colors_[index]       = color;
    return index;
  }

  template <typename It>
  void remove( It indices_begin, It indices_end ) {
    std::unordered_set<size_t> toRemove( indices_begin, indices_end );
    auto                       removeForAttr = [&]( auto &vec ) {
      size_t idx = 0;
      vec.erase( std::remove_if( vec.begin(), vec.end(), [&]( const auto & ) { return toRemove.count( idx++ ); } ),
                 vec.end() );
    };

    removeForAttr( positions_ );
    if ( withColors_ ) removeForAttr( colors_ );
    if ( withRotations_ ) removeForAttr( rotations_ );
    if ( withScales_ ) removeForAttr( scales_ );
    if ( withOpacities_ ) removeForAttr( opacities_ );
    if ( withSphericalHarmonics_ ) removeForAttr( sphericalHarmonics_ );
    if ( !quantizationGroupIds_.empty() ) removeForAttr( quantizationGroupIds_ );
  }

  void swapPoints( const size_t index1, const size_t index2 ) {
    assert( index1 < getPointCount() );
    assert( index2 < getPointCount() );
    std::swap( ( *this )[index1], ( *this )[index2] );
    if ( hasColors() ) { std::swap( getColor( index1 ), getColor( index2 ) ); }
    if ( hasReflectances() ) { std::swap( getReflectance( index1 ), getReflectance( index2 ) ); }
    if ( hasSphericalHarmonics() ) { std::swap( getSphericalHarmonic( index1 ), getSphericalHarmonic( index2 ) ); }
    if ( hasOpacities() ) { std::swap( getOpacity( index1 ), getOpacity( index2 ) ); }
    if ( hasScales() ) { std::swap( getScale( index1 ), getScale( index2 ) ); }
    if ( hasRotations() ) { std::swap( getRotation( index1 ), getRotation( index2 ) ); }
    if ( hasQuantizationGroupIds() ) {
      std::swap( getQuantizationGroupId( index1 ), getQuantizationGroupId( index2 ) );
    }
    if ( PCC_SAVE_POINT_TYPE ) { std::swap( getType( index1 ), getType( index2 ) ); }
  }
  PCCPoint3D computeCentroid() const;
  PCCBox3D   computeBoundingBox() const;
  bool       isBboxEmpty( PCCBox3D bbox ) const;
  bool       isRawPointsBboxEmpty( std::vector<size_t> rawPoints, PCCBox3D bbox ) const;
  int fillRawPointsBbox( std::vector<size_t> rawPoints, PCCBox3D bbox, std::vector<size_t> &bboxRawPoints ) const;

  static bool compareSeparators( char aChar, const char *const sep ) {
    int i = 0;
    while ( sep[i] != '\0' ) {
      if ( aChar == sep[i] ) return false;
      i++;
    }
    return true;
  }
  static inline bool getTokens( const char *str, const char *const sep, std::vector<std::string> &tokens ) {
    if ( !tokens.empty() ) tokens.clear();
    std::string buf    = "";
    size_t      i      = 0;
    size_t      length = ::strlen( str );
    while ( i < length ) {
      if ( compareSeparators( str[i], sep ) ) {
        buf += str[i];
      } else if ( buf.length() > 0 ) {
        tokens.push_back( buf );
        buf = "";
      }
      i++;
    }
    if ( !buf.empty() ) tokens.push_back( buf );
    return !tokens.empty();
  }
  bool write( const std::string &fileName, const bool asAscii = false );
  bool read( const std::string &fileName, const bool readNormals = false );
  void convertRGBToYUV();
  void convertRGBToYUVClosedLoop();
  void convertYUVToRGB();

  void removeDuplicate();
  void distanceGeo( const PCCPointSet3 &pointcloud, float &distPAB, float &distPBA ) const;
  void distanceGeoColor( const PCCPointSet3 &pointcloud,
                         float              &distPAB,
                         float              &distPBA,
                         float              &distYAB,
                         float              &distYBA,
                         float              &distUAB,
                         float              &distUBA,
                         float              &distVAB,
                         float              &distVBA ) const;

  void                 removeDuplicate( PCCPointSet3 &newPointcloud, size_t dropDuplicates ) const;
  void                 copyNormals( const PCCPointSet3 &sourceWithNormal );
  void                 scaleNormals( const PCCPointSet3 &sourceWithNormal );
  std::vector<uint8_t> computeChecksum( bool reorderPoints = false );
  void                 sortColor( std::vector<size_t> &list );
  void                 reorder();
  void                 reorder( PCCPointSet3 &newPointcloud, bool dropDuplicates );
  void                 swap( PCCPointSet3 &newPointcloud );
  int                  getAttributesCount() const {
    int AttributesCount = 0;
    AttributesCount += withColors_;
    AttributesCount += withReflectances_;
    AttributesCount += withSphericalHarmonics_;
    AttributesCount += withOpacities_;
    AttributesCount += withScales_;
    AttributesCount += withRotations_;
    return AttributesCount;
  }

 private:
  void distance( const PCCPointSet3 &pointcloud,
                 float              &distPAB,
                 float              &distPBA,
                 float              &distYAB,
                 float              &distYBA,
                 float              &distUAB,
                 float              &distUBA,
                 float              &distVAB,
                 float              &distVBA ) const;
  void distance( const PCCPointSet3 &pointcloud, float &distPAB, float &distPBA ) const;
  void distance( const PCCPointSet3 &pointcloud, float &distP, float &distY, float &distU, float &distV ) const;
  void distance( const PCCPointSet3 &pointcloud, float &distP ) const;
  std::vector<uint8_t> computeMd5();

  std::vector<PCCPoint3D>                positions_;
  std::vector<PCCColor3B>                colors_;
  std::vector<PCCColor16bit>             colors16bit_;
  std::vector<uint16_t>                  reflectances_;
  std::vector<uint16_t>                  boundaryPointTypes_;
  std::vector<std::pair<size_t, size_t>> pointPatchIndexes_;
  std::vector<uint64_t>                  parentPointIndex_;
  std::vector<uint8_t>                   types_;
  std::vector<PCCNormal3D>               normals_;
  bool                                   withNormals_;
  bool                                   withColors_;
  bool                                   withReflectances_;

  std::vector<std::vector<PCCAttribute16bit>> rotations_;
  std::vector<PCCAttribute16bit>              scales_;
  std::vector<std::vector<PCCAttribute16bit>> sphericalHarmonics_;
  std::vector<PCCAttribute16bit>              opacities_;

  bool withSphericalHarmonics_;
  bool withRotations_;
  bool withScales_;
  bool withOpacities_;

  std::vector<uint32_t>                     toSrcIdx_;
  std::vector<uint16_t>                     quantizationGroupIds_;
  std::vector<int>                          quantizationGroupBitDepthDeltas_;
  std::vector<CameraInfo>                   cameraInfo_;
  std::vector<AttributeTransformationInfo>  attributeTransformationInfo_;
  GeometryTransformationInfo                geometryTransformationInfo_;
  GeometryDequantizationParameters          geometryDequantizationParameters_;
  AttributeRotationDequantizationParameters attributeRotationDequantizationParameters_;
  AttributeScaleDequantizationParameters    attributeScaleDequantizationParameters_;
  AttributeOpacityDequantizationParameters  attributeOpacityDequantizationParameters_;
  AttributeShDequantizationParameters       attributeShDequantizationParameters_;
};
}  // namespace pcc

#endif /* PCCPointSet_h */