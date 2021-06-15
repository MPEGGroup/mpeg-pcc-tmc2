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

namespace pcc {

class PCCPointSet3 {
 public:
  PCCPointSet3() : withNormals_( false ), withColors_( false ), withReflectances_( false ) {}
  PCCPointSet3( const PCCPointSet3& ) = default;
  PCCPointSet3& operator=( const PCCPointSet3& rhs ) = default;
  ~PCCPointSet3()                                    = default;

  PCCPoint3D operator[]( const size_t index ) const {
    assert( index < positions_.size() );
    return positions_[index];
  }
  PCCPoint3D& operator[]( const size_t index ) {
    assert( index < positions_.size() );
    return positions_[index];
  }
  size_t appendPointSet( PCCPointSet3& pointSet ) {
    std::vector<PCCPoint3D>::iterator                itPositions;
    std::vector<PCCColor3B>::iterator                itColors;
    std::vector<PCCColor16bit>::iterator             itColors16bit;
    std::vector<uint16_t>::iterator                  itReflectances;
    std::vector<uint16_t>::iterator                  itBoundaryPointTypes;
    std::vector<std::pair<size_t, size_t>>::iterator itPointPatchIndexes;
    std::vector<uint8_t>::iterator                   itTypes;
    std::vector<PCCNormal3D>::iterator               itNormals;

    itPositions          = positions_.end();
    itColors             = colors_.end();
    itColors16bit        = colors16bit_.end();
    itReflectances       = reflectances_.end();
    itBoundaryPointTypes = boundaryPointTypes_.end();
    itPointPatchIndexes  = pointPatchIndexes_.end();
    itTypes              = types_.end();
    itNormals            = normals_.end();

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

    resize( getPointCount() );
    return positions_.size();
  }
  std::vector<uint16_t>& getBoundaryPointTypes() { return boundaryPointTypes_; }
  void                   setPosition( const size_t index, const PCCPoint3D position ) {
    assert( index < positions_.size() );
    positions_[index] = position;
  }
  std::vector<PCCColor3B>& getColor() { return colors_; }
  PCCColor3B               getColor( const size_t index ) const {
    assert( index < colors_.size() && withColors_ );
    return colors_[index];
  }
  PCCColor3B& getColor( const size_t index ) {
    assert( index < colors_.size() && withColors_ );
    return colors_[index];
  }
  void setColor( const size_t index, const PCCColor3B color ) {
    assert( index < colors_.size() && withColors_ );
    colors_[index] = color;
  }
  std::vector<PCCColor16bit>& getColor16bit() { return colors16bit_; }
  PCCColor16bit               getColor16bit( const size_t index ) const {
    assert( index < colors16bit_.size() && withColors_ );
    return colors16bit_[index];
  }
  PCCColor16bit& getColor16bit( const size_t index ) {
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
  void fillColor( PCCColor3B color = {0, 0, 0} ) {
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
      y1 = ( std::max )( y1, 0.0 );
      y1 = ( std::min )( y1, 1.0 );
      u1 = ( std::max )( u1, -0.5 );
      u1 = ( std::min )( u1, 0.5 );
      v1 = ( std::max )( v1, -0.5 );
      v1 = ( std::min )( v1, 0.5 );

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
  uint16_t& getBoundaryPointType( const size_t index ) {
    assert( index < boundaryPointTypes_.size() );
    return boundaryPointTypes_[index];
  }
  void setBoundaryPointType( const size_t index, const uint16_t BoundaryPointType ) {
    assert( index < boundaryPointTypes_.size() );
    boundaryPointTypes_[index] = BoundaryPointType;
  }
  std::vector<std::pair<size_t, size_t>>& getPointPatchIndexes() { return pointPatchIndexes_; }
  std::pair<size_t, size_t>               getPointPatchIndex( const size_t index ) const {
    assert( index < pointPatchIndexes_.size() );
    return pointPatchIndexes_[index];
  }
  std::pair<size_t, size_t>& getPointPatchIndex( const size_t index ) {
    assert( index < pointPatchIndexes_.size() );
    return pointPatchIndexes_[index];
  }
  void setPointPatchIndex( const size_t index, const uint32_t tileIndex, const uint32_t patchIndex ) {
    assert( index < pointPatchIndexes_.size() );
    pointPatchIndexes_[index].first  = tileIndex;
    pointPatchIndexes_[index].second = patchIndex;
  }
  std::vector<uint64_t>& getParentPointIndex() { return parentPointIndex_; }
  uint64_t&              getParentPointIndex( const size_t index ) { return parentPointIndex_[index]; }
  void                   setParentPointIndex( const size_t index, const uint64_t parentIndex ) {
    assert( index < parentPointIndex_.size() );
    parentPointIndex_[index] = parentIndex;
  }
  uint16_t getReflectance( const size_t index ) const {
    assert( index < reflectances_.size() && withReflectances_ );
    return reflectances_[index];
  }
  uint16_t& getReflectance( const size_t index ) {
    assert( index < reflectances_.size() && withReflectances_ );
    return reflectances_[index];
  }
  void setReflectance( const size_t index, const uint16_t reflectance ) {
    assert( index < reflectances_.size() && withReflectances_ );
    reflectances_[index] = reflectance;
  }
  std::vector<PCCPoint3D>&    getPositions() { return positions_; }
  std::vector<PCCColor3B>&    getColors() { return colors_; }
  std::vector<PCCColor16bit>& getColors16bit() { return colors16bit_; }
  std::vector<uint16_t>&      getReflectances() { return reflectances_; }
  std::vector<uint8_t>&       getTypes() { return types_; }

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
  uint8_t& getType( const size_t index ) {
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
  const std::vector<PCCNormal3D>& getNormals() const { return normals_; }
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

  bool transferColors( PCCPointSet3& target,
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

  bool transferColors16bitBP( PCCPointSet3& target,
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
  bool transferColorsBackward16bitBP( PCCPointSet3& target,
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

  bool transferColors16bit( PCCPointSet3& target,
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

  bool transferColorsFilter3( PCCPointSet3& target, const int32_t searchRange, const bool losslessAttribute ) const;

  bool transferColorSimple( PCCPointSet3& target, const double bestColorSearchStep = 0.1 );

  bool transferColorWeight( PCCPointSet3& target, const double bestColorSearchStep = 0.1 );

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
  }
  size_t addPoint( const PCCPoint3D& position ) {
    const size_t index = getPointCount();
    resize( index + 1 );
    positions_[index] = position;
    return index;
  }
  size_t addPoint( const PCCPoint3D& position, const PCCColor3B& color ) {
    withColors_        = true;
    const size_t index = getPointCount();
    resize( index + 1 );
    colors_.resize( index + 1 );
    positions_[index] = position;
    colors_[index]    = color;
    return index;
  }
  size_t addPoint( const PCCVector3D& position ) {
    const size_t index = getPointCount();
    resize( index + 1 );
    positions_[index][0] = (int16_t)position[0];
    positions_[index][1] = (int16_t)position[1];
    positions_[index][2] = (int16_t)position[2];
    return index;
  }
  size_t addPoint( const PCCVector3D& position, const PCCColor3B& color ) {
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

  void swapPoints( const size_t index1, const size_t index2 ) {
    assert( index1 < getPointCount() );
    assert( index2 < getPointCount() );
    std::swap( ( *this )[index1], ( *this )[index2] );
    if ( hasColors() ) { std::swap( getColor( index1 ), getColor( index2 ) ); }
    if ( hasReflectances() ) { std::swap( getReflectance( index1 ), getReflectance( index2 ) ); }
    if ( PCC_SAVE_POINT_TYPE ) { std::swap( getType( index1 ), getType( index2 ) ); }
  }
  PCCPoint3D computeCentroid() const;
  PCCBox3D   computeBoundingBox() const;
  bool       isBboxEmpty( PCCBox3D bbox ) const;
  bool       isRawPointsBboxEmpty( std::vector<size_t> rawPoints, PCCBox3D bbox ) const;
  int fillRawPointsBbox( std::vector<size_t> rawPoints, PCCBox3D bbox, std::vector<size_t>& bboxRawPoints ) const;

  static bool compareSeparators( char aChar, const char* const sep ) {
    int i = 0;
    while ( sep[i] != '\0' ) {
      if ( aChar == sep[i] ) return false;
      i++;
    }
    return true;
  }
  static inline bool getTokens( const char* str, const char* const sep, std::vector<std::string>& tokens ) {
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
  bool write( const std::string& fileName, const bool asAscii = false );
  bool read( const std::string& fileName, const bool readNormals = false );
  void convertRGBToYUV();
  void convertRGBToYUVClosedLoop();
  void convertYUVToRGB();

  void removeDuplicate();
  void distanceGeo( const PCCPointSet3& pointcloud, float& distPAB, float& distPBA ) const;
  void distanceGeoColor( const PCCPointSet3& pointcloud,
                         float&              distPAB,
                         float&              distPBA,
                         float&              distYAB,
                         float&              distYBA,
                         float&              distUAB,
                         float&              distUBA,
                         float&              distVAB,
                         float&              distVBA ) const;

  void                 removeDuplicate( PCCPointSet3& newPointcloud, size_t dropDuplicates ) const;
  void                 copyNormals( const PCCPointSet3& sourceWithNormal );
  void                 scaleNormals( const PCCPointSet3& sourceWithNormal );
  std::vector<uint8_t> computeChecksum( bool reorderPoints = false );
  void                 sortColor( std::vector<size_t>& list );
  void                 reorder();
  void                 reorder( PCCPointSet3& newPointcloud, bool dropDuplicates );
  void                 swap( PCCPointSet3& newPointcloud );

 private:
  void distance( const PCCPointSet3& pointcloud,
                 float&              distPAB,
                 float&              distPBA,
                 float&              distYAB,
                 float&              distYBA,
                 float&              distUAB,
                 float&              distUBA,
                 float&              distVAB,
                 float&              distVBA ) const;
  void distance( const PCCPointSet3& pointcloud, float& distPAB, float& distPBA ) const;
  void distance( const PCCPointSet3& pointcloud, float& distP, float& distY, float& distU, float& distV ) const;
  void distance( const PCCPointSet3& pointcloud, float& distP ) const;
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
};
}  // namespace pcc

#endif /* PCCPointSet_h */
