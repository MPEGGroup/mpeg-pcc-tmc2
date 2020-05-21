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

#ifndef PCCPatch_h
#define PCCPatch_h

#include "PCCCommon.h"
#include "PCCPointSet.h"

namespace pcc {

struct GPAPatchData {
  bool              isMatched;
  bool              isGlobalPatch;
  int               globalPatchIndex;
  size_t            sizeU0;
  size_t            sizeV0;
  size_t            u0;
  size_t            v0;
  size_t            patchOrientation;
  std::vector<bool> occupancy;
  void              initialize() {
    isMatched        = false;
    isGlobalPatch    = false;
    globalPatchIndex = -1;
    sizeU0           = 0;
    sizeV0           = 0;
    occupancy.clear();
    u0               = -1;
    v0               = -1;
    patchOrientation = -1;
  }
  bool isPatchDimensionSwitched() {
    if ( ( patchOrientation == PATCH_ORIENTATION_DEFAULT ) || ( patchOrientation == PATCH_ORIENTATION_ROT180 ) ||
         ( patchOrientation == PATCH_ORIENTATION_MIRROR ) || ( patchOrientation == PATCH_ORIENTATION_MROT180 ) ) {
      return false;
    } else {
      return true;
    }
  }
};

static const std::vector<uint8_t> g_orientation = {
    0, 0, 6, 0, 0, 0, 0, 6, 4, 0, 0, 5, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 7, 7, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 4, 0, 5, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 5, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 0, 4, 3, 0, 0,
    5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 7, 7, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 7, 0, 0, 0, 0, 0, 0,
    0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 3, 3, 0, 3, 0, 0, 0, 4, 1, 0, 0, 0, 1, 0, 1, 0, 2, 3, 0, 0, 1, 2, 0, 0};
static const int8_t g_dilate[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};

class PCCPatch {
 public:
  PCCPatch() :
      index_( 0 ),
      u1_( 0 ),
      v1_( 0 ),
      d1_( 0 ),
      sizeD_( 0 ),
      sizeU_( 0 ),
      sizeV_( 0 ),
      u0_( 0 ),
      v0_( 0 ),
      sizeU0_( 0 ),
      sizeV0_( 0 ),
      occupancyResolution_( 0 ),
      projectionMode_( 0 ),
      levelOfDetailX_( 1 ),
      levelOfDetailY_( 1 ),
      normalAxis_( 0 ),
      tangentAxis_( 0 ),
      bitangentAxis_( 0 ),
      viewId_( 0 ),
      bestMatchIdx_( InvalidPatchIndex ),
      refAtlasFrameIdx_( 0 ),
      predType_( 0 ),
      patchOrientation_( 0 ),
      isGlobalPatch_( false ),
      d0Count_( 0 ),
      eomCount_( 0 ),
      eomandD1Count_( 0 ),
      patchType_( PATCH_MODE_I_INTRA ) {
    depth_[0].clear();
    depth_[1].clear();
    occupancy_.clear();
    depthEnhancedDeltaD_.clear();
    depth0PCidx_.clear();
    pointLocalReconstructionModeByBlock_.clear();
    occupancyMap_.clear();
    borderPoints_.clear();
    neighboringPatches_.clear();
    depthMap_.clear();
  };
  ~PCCPatch() {
    depth_[0].clear();
    depth_[1].clear();
    occupancy_.clear();
    depthEnhancedDeltaD_.clear();
    depth0PCidx_.clear();
    pointLocalReconstructionModeByBlock_.clear();
    occupancyMap_.clear();
    borderPoints_.clear();
    neighboringPatches_.clear();
    depthMap_.clear();
  };
  size_t                      getEOMCount() { return eomCount_; }
  void                        setEOMCount( size_t value ) { eomCount_ = value; }
  size_t                      getEOMandD1Count() { return eomandD1Count_; }
  void                        setEOMandD1Count( size_t value ) { eomandD1Count_ = value; }
  size_t                      getD0Count() { return d0Count_; }
  void                        setD0Count( size_t value ) { d0Count_ = value; }
  size_t&                     getIndex() { return index_; }
  size_t&                     getOriginalIndex() { return originalIndex_; }
  size_t&                     getU1() { return u1_; }
  size_t&                     getV1() { return v1_; }
  size_t&                     getD1() { return d1_; }
  size_t&                     getSizeD() { return sizeD_; }
  size_t&                     getSizeU() { return sizeU_; }
  size_t&                     getSizeV() { return sizeV_; }
  size_t&                     getU0() { return u0_; }
  size_t&                     getV0() { return v0_; }
  size_t&                     getSizeU0() { return sizeU0_; }
  size_t&                     getSizeV0() { return sizeV0_; }
  void                        setBestMatchIdx( int32_t value ) { bestMatchIdx_ = value; }
  size_t                      getRefAtlasFrameIndex() const { return refAtlasFrameIdx_; }
  void                        setRefAtlasFrameIndex( size_t value ) { refAtlasFrameIdx_ = value; }
  size_t                      getPredType() { return predType_; }
  size_t                      getPredType() const { return predType_; }
  void                        setPredType( size_t value ) { predType_ = value; }
  uint8_t                     getPatchType() const { return patchType_; }
  void                        setPatchType( uint8_t value ) { patchType_ = value; }
  size_t&                     getOccupancyResolution() { return occupancyResolution_; }
  size_t&                     getProjectionMode() { return projectionMode_; }
  size_t&                     getNormalAxis() { return normalAxis_; }
  size_t&                     getTangentAxis() { return tangentAxis_; }
  size_t&                     getBitangentAxis() { return bitangentAxis_; }
  std::vector<int16_t>&       getDepth( int i ) { return depth_[i]; }
  std::vector<bool>&          getOccupancy() { return occupancy_; }
  size_t                      getLodScaleX() { return levelOfDetailX_; }
  size_t                      getLodScaleY() { return levelOfDetailY_; }
  size_t                      getLodScaleX() const { return levelOfDetailX_; }
  size_t                      getLodScaleY() const { return levelOfDetailY_; }
  void                        setLodScaleX( size_t value ) { levelOfDetailX_ = value; }
  void                        setLodScaleY( size_t value ) { levelOfDetailY_ = value; }
  size_t&                     getAxisOfAdditionalPlane() { return axisOfAdditionalPlane_; }
  size_t                      getIndex() const { return index_; }
  size_t                      getOriginalIndex() const { return originalIndex_; }
  size_t                      getU1() const { return u1_; }
  size_t                      getV1() const { return v1_; }
  size_t                      getD1() const { return d1_; }
  size_t                      getSizeD() const { return sizeD_; }
  size_t                      getSizeU() const { return sizeU_; }
  size_t                      getSizeV() const { return sizeV_; }
  size_t                      getU0() const { return u0_; }
  size_t                      getV0() const { return v0_; }
  size_t                      getSizeU0() const { return sizeU0_; }
  size_t                      getSizeV0() const { return sizeV0_; }
  size_t                      getOccupancyResolution() const { return occupancyResolution_; }
  size_t                      getProjectionMode() const { return projectionMode_; }
  size_t                      getNormalAxis() const { return normalAxis_; }
  size_t                      getTangentAxis() const { return tangentAxis_; }
  size_t                      getBitangentAxis() const { return bitangentAxis_; }
  size_t                      getViewId() const { return viewId_; }
  int32_t                     getBestMatchIdx() const { return bestMatchIdx_; }
  const std::vector<int16_t>& getDepth( int i ) const { return depth_[i]; }
  const std::vector<bool>&    getOccupancy() const { return occupancy_; }
  std::vector<int16_t>&       getDepthEnhancedDeltaD() { return depthEnhancedDeltaD_; }
  const std::vector<int16_t>& getDepthEnhancedDeltaD() const { return depthEnhancedDeltaD_; }
  const size_t&               getAxisOfAdditionalPlane() const { return axisOfAdditionalPlane_; }
  const std::vector<int64_t>& getDepth0PccIdx() const { return depth0PCidx_; }
  std::vector<int64_t>&       getDepth0PccIdx() { return depth0PCidx_; }
  bool&                       getIsRoiPatch() { return isRoiPatch_; }
  size_t&                     getRoiIndex() { return roiIndex_; }
  size_t&                     getPatchOrientation() { return patchOrientation_; }
  size_t                      getPatchOrientation() const { return patchOrientation_; }
  bool&                       getIsGlobalPatch() { return isGlobalPatch_; }
  bool                        getIsGlobalPatch() const { return isGlobalPatch_; }
  size_t                      getPatchSize2DXInPixel() const { return size2DXInPixel_; }
  size_t                      getPatchSize2DYInPixel() const { return size2DYInPixel_; }
  size_t                      getPatchSize2DXInPixel() { return size2DXInPixel_; }
  size_t                      getPatchSize2DYInPixel() { return size2DYInPixel_; }
  void                        setPatchSize2DXInPixel( size_t value ) { size2DXInPixel_ = value; }
  void                        setPatchSize2DYInPixel( size_t value ) { size2DYInPixel_ = value; }
  inline double               generateNormalCoordinate( const uint16_t depth ) const {
    double coord = 0;
    if ( projectionMode_ == 0 ) {
      coord = ( (double)depth + (double)d1_ );
    } else {
      double tmp_depth = double( d1_ ) - double( depth );
      if ( tmp_depth > 0 ) { coord = tmp_depth; }
    }
    return coord;
  }

  void                        setViewId(size_t viewId) {
    viewId_ = viewId;
    //now set the other variables according to the viewId
    switch (viewId)
    {
    case 0:
      getAxisOfAdditionalPlane() = 0;
      getNormalAxis()            = 0;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 0;
      break;
    case 1:
      getAxisOfAdditionalPlane() = 0;
      getNormalAxis()            = 1;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 0;
      getProjectionMode()        = 0;
      break;
    case 2:
      getAxisOfAdditionalPlane() = 0;
      getNormalAxis()            = 2;
      getTangentAxis()           = 0;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 0;
      break;
    case 3:
      getAxisOfAdditionalPlane() = 0;
      getNormalAxis()            = 0;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 1;
      break;
    case 4:
      getAxisOfAdditionalPlane() = 0;
      getNormalAxis()            = 1;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 0;
      getProjectionMode()        = 1;
      break;
    case 5:
      getAxisOfAdditionalPlane() = 0;
      getNormalAxis()            = 2;
      getTangentAxis()           = 0;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 1;
      break;
    case 6:
      getAxisOfAdditionalPlane() = 1;
      getNormalAxis()            = 0;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 0;
      break;
    case 7:
      getAxisOfAdditionalPlane() = 1;
      getNormalAxis()            = 2;
      getTangentAxis()           = 0;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 0;
      break;
    case 8:
      getAxisOfAdditionalPlane() = 1;
      getNormalAxis()            = 0;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 1;
      break;
    case 9:
      getAxisOfAdditionalPlane() = 1;
      getNormalAxis()            = 2;
      getTangentAxis()           = 0;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 1;
      break;
    case 10:
      getAxisOfAdditionalPlane() = 2;
      getNormalAxis()            = 2;
      getTangentAxis()           = 0;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 0;
      break;
    case 11:
      getAxisOfAdditionalPlane() = 2;
      getNormalAxis()            = 1;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 0;
      getProjectionMode()        = 0;
      break;
    case 12:
      getAxisOfAdditionalPlane() = 2;
      getNormalAxis()            = 2;
      getTangentAxis()           = 0;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 1;
      break;
    case 13:
      getAxisOfAdditionalPlane() = 2;
      getNormalAxis()            = 1;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 0;
      getProjectionMode()        = 1;
      break;
    case 14:
      getAxisOfAdditionalPlane() = 3;
      getNormalAxis()            = 1;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 0;
      getProjectionMode()        = 0;
      break;
    case 15:
      getAxisOfAdditionalPlane() = 3;
      getNormalAxis()            = 0;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 0;
      break;
    case 16:
      getAxisOfAdditionalPlane() = 3;
      getNormalAxis()            = 1;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 0;
      getProjectionMode()        = 1;
      break;
    case 17:
      getAxisOfAdditionalPlane() = 3;
      getNormalAxis()            = 0;
      getTangentAxis()           = 2;
      getBitangentAxis()         = 1;
      getProjectionMode()        = 1;
      break;
    default:
      std::cout << "ViewId (" << viewId << ") not allowed... exiting" << std::endl;
      exit(-1);
      break;
    }
  }

  PCCPoint3D generatePoint( const size_t u, const size_t v, const uint16_t depth ) const {
    PCCPoint3D point0;
    point0[normalAxis_]    = generateNormalCoordinate( depth );
    point0[tangentAxis_]   = ( double( u ) * (double)levelOfDetailX_ + u1_ );
    point0[bitangentAxis_] = ( double( v ) * (double)levelOfDetailY_ + v1_ );
    return point0;
  }

  PCCPoint3D canvasTo3D( const size_t x, const size_t y, const uint16_t depth ) const {
    PCCPoint3D point0;
    size_t     u = 0, v = 0;
    switch ( patchOrientation_ ) {
      case PATCH_ORIENTATION_DEFAULT:
        u = x - u0_ * occupancyResolution_;
        v = y - v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_ROT90:
        v = ( sizeV0_ * occupancyResolution_ - 1 - ( x - u0_ * occupancyResolution_ ) );
        u = y - v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_ROT180:
        u = ( sizeU0_ * occupancyResolution_ - 1 - ( x - u0_ * occupancyResolution_ ) );
        v = ( sizeV0_ * occupancyResolution_ - 1 - ( y - v0_ * occupancyResolution_ ) );
        break;
      case PATCH_ORIENTATION_ROT270:
        v = x - u0_ * occupancyResolution_;
        u = ( sizeU0_ * occupancyResolution_ - 1 - ( y - v0_ * occupancyResolution_ ) );
        break;
      case PATCH_ORIENTATION_MIRROR:
        u = ( sizeU0_ * occupancyResolution_ - 1 - ( x - u0_ * occupancyResolution_ ) );
        v = y - v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_MROT90:
        v = ( sizeV0_ * occupancyResolution_ - 1 - ( x - u0_ * occupancyResolution_ ) );
        u = ( sizeU0_ * occupancyResolution_ - 1 - ( y - v0_ * occupancyResolution_ ) );
        break;
      case PATCH_ORIENTATION_MROT180:
        u = x - u0_ * occupancyResolution_;
        v = ( sizeV0_ * occupancyResolution_ - 1 - ( y - v0_ * occupancyResolution_ ) );
        break;
      case PATCH_ORIENTATION_MROT270:
        v = x - u0_ * occupancyResolution_;
        u = y - v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_SWAP:  // swapAxis
        v = x - u0_ * occupancyResolution_;
        u = y - v0_ * occupancyResolution_;
        break;
      default: assert( 0 ); break;
    }
    point0[normalAxis_]    = generateNormalCoordinate( depth );
    point0[tangentAxis_]   = ( double( u ) * levelOfDetailX_ + u1_ );
    point0[bitangentAxis_] = ( double( v ) * levelOfDetailY_ + v1_ );
    return point0;
  }

  size_t patch2Canvas( const size_t u,
                       const size_t v,
                       size_t       canvasStride,
                       size_t       canvasHeight,
                       size_t&      x,
                       size_t&      y ) {
    switch ( patchOrientation_ ) {
      case PATCH_ORIENTATION_DEFAULT:
        x = u + u0_ * occupancyResolution_;
        y = v + v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_ROT90:
        x = ( sizeV0_ * occupancyResolution_ - 1 - v ) + u0_ * occupancyResolution_;
        y = u + v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_ROT180:
        x = ( sizeU0_ * occupancyResolution_ - 1 - u ) + u0_ * occupancyResolution_;
        y = ( sizeV0_ * occupancyResolution_ - 1 - v ) + v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_ROT270:
        x = v + u0_ * occupancyResolution_;
        y = ( sizeU0_ * occupancyResolution_ - 1 - u ) + v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_MIRROR:
        x = ( sizeU0_ * occupancyResolution_ - 1 - u ) + u0_ * occupancyResolution_;
        y = v + v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_MROT90:
        x = ( sizeV0_ * occupancyResolution_ - 1 - v ) + u0_ * occupancyResolution_;
        y = ( sizeU0_ * occupancyResolution_ - 1 - u ) + v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_MROT180:
        x = u + u0_ * occupancyResolution_;
        y = ( sizeV0_ * occupancyResolution_ - 1 - v ) + v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_MROT270:
        x = v + u0_ * occupancyResolution_;
        y = u + v0_ * occupancyResolution_;
        break;
      case PATCH_ORIENTATION_SWAP:  // swapAxis
        x = v + u0_ * occupancyResolution_;
        y = u + v0_ * occupancyResolution_;
        break;
      default: assert( 0 ); break;
    }
    // checking the results are within canvas boundary (missing y check)
    assert( x >= 0 );
    assert( y >= 0 );
    assert( x < canvasStride );
    assert( y < canvasHeight );
    return ( x + canvasStride * y );
  }

  int patchBlock2CanvasBlock( const size_t uBlk,
                              const size_t vBlk,
                              size_t       canvasStrideBlk,
                              size_t       canvasHeightBlk,
                              const Tile   tile = Tile() ) const {
    size_t x, y;
    switch ( patchOrientation_ ) {
      case PATCH_ORIENTATION_DEFAULT:
        x = uBlk + u0_;
        y = vBlk + v0_;
        break;
      case PATCH_ORIENTATION_ROT90:
        x = ( sizeV0_ - 1 - vBlk ) + u0_;
        y = uBlk + v0_;
        break;
      case PATCH_ORIENTATION_ROT180:
        x = ( sizeU0_ - 1 - uBlk ) + u0_;
        y = ( sizeV0_ - 1 - vBlk ) + v0_;
        break;
      case PATCH_ORIENTATION_ROT270:
        x = vBlk + u0_;
        y = ( sizeU0_ - 1 - uBlk ) + v0_;
        break;
      case PATCH_ORIENTATION_MIRROR:
        x = ( sizeU0_ - 1 - uBlk ) + u0_;
        y = vBlk + v0_;
        break;
      case PATCH_ORIENTATION_MROT90:
        x = ( sizeV0_ - 1 - vBlk ) + u0_;
        y = ( sizeU0_ - 1 - uBlk ) + v0_;
        break;
      case PATCH_ORIENTATION_MROT180:
        x = uBlk + u0_;
        y = ( sizeV0_ - 1 - vBlk ) + v0_;
        break;
      case PATCH_ORIENTATION_MROT270:
        x = vBlk + u0_;
        y = uBlk + v0_;
        break;
      case PATCH_ORIENTATION_SWAP:  // swapAxis
        x = vBlk + u0_;
        y = uBlk + v0_;
        break;
      default: return -1; break;
    }
    // checking the results are within canvas boundary (missing y check)
    if ( x < 0 ) return -1;
    if ( y < 0 ) return -1;
    if ( x >= canvasStrideBlk ) return -1;
    if ( y >= canvasHeightBlk ) return -1;
    if ( tile.minU != -1 ) {
      if ( x < tile.minU ) return -1;
      if ( y < tile.minV ) return -1;
      if ( x > tile.maxU ) return -1;
      if ( y > tile.maxV ) return -1;
    }
    return int( x + canvasStrideBlk * y );
  }

  bool checkFitPatchCanvas( std::vector<bool> canvas,
                            size_t            canvasStrideBlk,
                            size_t            canvasHeightBlk,
                            bool              bPrecedence,
                            int               safeguard = 0,
                            const Tile        tile      = Tile() ) {
    for ( size_t v0 = 0; v0 < getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < getSizeU0(); ++u0 ) {
        for ( int deltaY = -safeguard; deltaY < safeguard + 1; deltaY++ ) {
          for ( int deltaX = -safeguard; deltaX < safeguard + 1; deltaX++ ) {
            int pos = patchBlock2CanvasBlock( u0 + deltaX, v0 + deltaY, canvasStrideBlk, canvasHeightBlk, tile );
            if ( pos < 0 ) {
              return false;
            } else {
              if ( bPrecedence ) {
                if ( canvas[pos] && occupancy_[u0 + getSizeU0() * v0] ) { return false; }
              } else {
                if ( canvas[pos] ) { return false; }
              }
            }
          }
        }
      }
    }
    return true;
  }

  bool smallerRefFirst( const PCCPatch& rhs ) {
    if ( bestMatchIdx_ == -1 && rhs.getBestMatchIdx() == -1 ) {
      return gt( rhs );
    } else if ( bestMatchIdx_ == -1 || rhs.getBestMatchIdx() == -1 ) {
      return ( bestMatchIdx_ != -1 ) ? true : false;
    } else if ( bestMatchIdx_ == rhs.getBestMatchIdx() ) {
      return refAtlasFrameIdx_ < rhs.getRefAtlasFrameIndex();
    } else {
      return bestMatchIdx_ < rhs.getBestMatchIdx();
    }
  }

  bool gt( const PCCPatch& rhs ) {
    size_t maxDim, rhsMaxDim;
    size_t minDim, rhsMinDim;
    // setting the largest dimension
    if ( sizeU0_ > sizeV0_ ) {
      maxDim = sizeU0_;
      minDim = sizeV0_;
    } else {
      maxDim = sizeV0_;
      minDim = sizeU0_;
    }
    // getting the largest dimensions for the block to be tested
    if ( rhs.getSizeU0() > rhs.getSizeV0() ) {
      rhsMaxDim = rhs.getSizeU0();
      rhsMinDim = rhs.getSizeV0();
    } else {
      rhsMaxDim = rhs.getSizeV0();
      rhsMinDim = rhs.getSizeU0();
    }
    // if the dimensions are the same, decide by the index
    return maxDim != rhsMaxDim ? ( maxDim > rhsMaxDim )
                               : ( minDim != rhsMinDim ) ? ( minDim > rhsMinDim ) : ( index_ < rhs.index_ );
  }

  void print() const {
    printf(
        "Patch[%3zu] uv0 = %4zu %4zu / %4zu %4zu uvd1 = %4zu %4zu %4zu / %4zu "
        "%4zu %4zu \n",
        index_, u0_, v0_, sizeU0_, sizeV0_, u1_, v1_, d1_, sizeU_, sizeV_, sizeD_ );
  }

  void printDecoder() const {
    printf(
        "Patch uv0 = %4zu %4zu / %4zu %4zu uvd1 = %4zu %4zu %4zu orientation = "
        "%4zu  \n",
        u0_, v0_, sizeU0_, sizeV0_, u1_, v1_, d1_, patchOrientation_ );
  }

  friend bool operator<( const PCCPatch& lhs, const PCCPatch& rhs ) {
    return lhs.sizeV_ != rhs.sizeV_ ? lhs.sizeV_ > rhs.sizeV_
                                    : ( lhs.sizeU_ != rhs.sizeU_ ? lhs.sizeU_ > rhs.sizeU_ : lhs.index_ < rhs.index_ );
  }

  void get_patch_horizons( std::vector<int>& top_horizon,
                           std::vector<int>& bottom_horizon,
                           std::vector<int>& right_horizon,
                           std::vector<int>& left_horizon ) {
    top_horizon.resize( getSizeU0(), 0 );
    bottom_horizon.resize( getSizeU0(), 0 );
    right_horizon.resize( getSizeV0(), 0 );
    left_horizon.resize( getSizeV0(), 0 );

    if ( printDetailedInfo ) std::cout << "Top Horizon :[";
    for ( int i = 0; i < getSizeU0(); i++ ) {
      while ( !occupancy_[( getSizeV0() - 1 - top_horizon[i] ) * getSizeU0() + i] &&
              ( top_horizon[i] < getSizeV0() - 1 ) )
        top_horizon[i]++;
      if ( printDetailedInfo ) std::cout << top_horizon[i] << ",";
    }
    if ( printDetailedInfo ) std::cout << "]" << std::endl;

    if ( printDetailedInfo ) std::cout << "Bottom Horizon :[";
    for ( int i = 0; i < getSizeU0(); i++ ) {
      while ( !occupancy_[bottom_horizon[i] * getSizeU0() + i] && ( bottom_horizon[i] < getSizeV0() - 1 ) )
        bottom_horizon[i]++;
      if ( printDetailedInfo ) std::cout << bottom_horizon[i] << ",";
    }
    if ( printDetailedInfo ) std::cout << "]" << std::endl;
    if ( printDetailedInfo ) std::cout << "Right Horizon :[";
    for ( int i = 0; i < getSizeV0(); i++ ) {
      while ( !occupancy_[i * getSizeU0() + getSizeU0() - 1 - right_horizon[i]] &&
              ( right_horizon[i] < getSizeU0() - 1 ) )
        right_horizon[i]++;
      if ( printDetailedInfo ) std::cout << right_horizon[i] << ",";
    }
    if ( printDetailedInfo ) std::cout << "]" << std::endl;

    if ( printDetailedInfo ) std::cout << "Left Horizon :[";
    for ( int i = 0; i < getSizeV0(); i++ ) {
      while ( !occupancy_[i * getSizeU0() + left_horizon[i]] && ( left_horizon[i] < getSizeU0() - 1 ) )
        left_horizon[i]++;
      if ( printDetailedInfo ) std::cout << left_horizon[i] << ",";
    }
    if ( printDetailedInfo ) std::cout << "]" << std::endl;
  }

  int calculate_wasted_space( std::vector<int>& horizon,
                              std::vector<int>& top_horizon,
                              std::vector<int>& bottom_horizon,
                              std::vector<int>& right_horizon,
                              std::vector<int>& left_horizon ) {
    int wasted_space          = 0;
    int wasted_space_external = 0;
    int wasted_space_internal = 0;
    int lambda                = 100;  //--> bias towards the upper part of the canvas
    if ( getPatchOrientation() == PATCH_ORIENTATION_DEFAULT ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        wasted_space_external += getV0() + bottom_horizon[idx] - horizon[getU0() + idx];
        // calculating internal wasted space --> because of new block2patch
        // restriction, this area only contains
        // locations for the local patch
        for ( int idx2 = bottom_horizon[idx] + 1; idx2 < getSizeV0() - top_horizon[idx]; idx2++ ) {
          if ( !occupancy_[idx2 * getSizeU0() + idx] ) wasted_space_internal++;
        }
      }
      wasted_space = int( lambda * getV0() + wasted_space_external + wasted_space_internal );
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_ROT90 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        wasted_space_external += getV0() + left_horizon[getSizeV0() - 1 - idx] - horizon[getU0() + idx];
        // calculating internal wasted space
        for ( int idx2 = int( getSizeU0() - 1 - right_horizon[idx] ); idx2 >= left_horizon[idx]; idx2-- ) {
          if ( !occupancy_[idx * getSizeU0() + idx2] ) wasted_space_internal++;
        }
      }
      wasted_space = int( lambda * getV0() + wasted_space_external + wasted_space_internal );
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_ROT180 ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        wasted_space_external += getV0() + top_horizon[getSizeU0() - 1 - idx] - horizon[getU0() + idx];
        // calculating internal wasted space
        for ( int idx2 = int( getSizeV0() - 1 - top_horizon[idx] ); idx2 >= bottom_horizon[idx]; idx2-- ) {
          if ( !occupancy_[idx2 * getSizeU0() + idx] ) wasted_space_internal++;
        }
      }
      wasted_space = int( lambda * getV0() + wasted_space_external + wasted_space_internal );
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_ROT270 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        wasted_space_external += getV0() + right_horizon[idx] - horizon[getU0() + idx];
        // calculating internal wasted space
        for ( int idx2 = left_horizon[idx] + 1; idx2 < getSizeU0() - right_horizon[idx]; idx2++ ) {
          if ( !occupancy_[idx * getSizeU0() + idx2] ) wasted_space_internal++;
        }
      }
      wasted_space = int( lambda * getV0() + wasted_space_external + wasted_space_internal );
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_MIRROR ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        wasted_space_external += getV0() + bottom_horizon[getSizeU0() - 1 - idx] - horizon[getU0() + idx];
        for ( int idx2 = bottom_horizon[idx] + 1; idx2 < getSizeV0() - top_horizon[idx]; idx2++ ) {
          if ( !occupancy_[idx2 * getSizeU0() + idx] ) wasted_space_internal++;
        }
      }
      wasted_space = int( lambda * getV0() + wasted_space_external + wasted_space_internal );
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_MROT90 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        wasted_space_external += getV0() + right_horizon[getSizeV0() - 1 - idx] - horizon[getU0() + idx];
        // calculating internal wasted space
        for ( int idx2 = left_horizon[idx] + 1; idx2 < getSizeU0() - right_horizon[idx]; idx2++ ) {
          if ( !occupancy_[idx * getSizeU0() + idx2] ) wasted_space_internal++;
        }
      }
      wasted_space = int( lambda * getV0() + wasted_space_external + wasted_space_internal );
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_MROT180 ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        wasted_space_external += getV0() + top_horizon[idx] - horizon[getU0() + idx];
        // calculating internal wasted space
        for ( int idx2 = int( getSizeV0() - 1 - top_horizon[idx] ); idx2 >= bottom_horizon[idx]; idx2-- ) {
          if ( !occupancy_[idx2 * getSizeU0() + idx] ) wasted_space_internal++;
        }
      }
      wasted_space = int( lambda * getV0() + wasted_space_external + wasted_space_internal );
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_MROT270 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        wasted_space_external += getV0() + left_horizon[idx] - horizon[getU0() + idx];
        for ( int idx2 = int( getSizeU0() - 1 - left_horizon[idx] ); idx2 >= right_horizon[idx]; idx2-- ) {
          if ( !occupancy_[idx * getSizeU0() + idx2] ) wasted_space_internal++;
        }
      }
      wasted_space = int( lambda * getV0() + wasted_space_external + wasted_space_internal );
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_SWAP ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        wasted_space_external += getV0() + left_horizon[idx] - horizon[getU0() + idx];
        for ( int idx2 = int( getSizeU0() - 1 - left_horizon[idx] ); idx2 >= right_horizon[idx]; idx2-- ) {
          if ( !occupancy_[idx * getSizeU0() + idx2] ) wasted_space_internal++;
        }
      }
      wasted_space = int( lambda * getV0() + wasted_space_external + wasted_space_internal );
    }
    return wasted_space;
  }

  bool isPatchLocationAboveHorizon( std::vector<int>& horizon,
                                    std::vector<int>& top_horizon,
                                    std::vector<int>& bottom_horizon,
                                    std::vector<int>& right_horizon,
                                    std::vector<int>& left_horizon ) {
    if ( getPatchOrientation() == PATCH_ORIENTATION_DEFAULT ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        if ( getV0() + bottom_horizon[idx] < horizon[getU0() + idx] ) { return false; }
      }
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_ROT90 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        if ( getV0() + left_horizon[getSizeV0() - 1 - idx] < horizon[getU0() + idx] ) { return false; }
      }
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_ROT180 ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        if ( getV0() + top_horizon[getSizeU0() - 1 - idx] < horizon[getU0() + idx] ) { return false; }
      }
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_ROT270 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        if ( getV0() + right_horizon[idx] < horizon[getU0() + idx] ) { return false; }
      }
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_MIRROR ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        if ( getV0() + bottom_horizon[getSizeU0() - 1 - idx] < horizon[getU0() + idx] ) { return false; }
      }
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_MROT90 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        if ( getV0() + right_horizon[getSizeV0() - 1 - idx] < horizon[getU0() + idx] ) { return false; }
      }
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_MROT180 ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        if ( getV0() + top_horizon[idx] < horizon[getU0() + idx] ) { return false; }
      }
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_MROT270 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        if ( getV0() + left_horizon[idx] < horizon[getU0() + idx] ) { return false; }
      }
    } else if ( getPatchOrientation() == PATCH_ORIENTATION_SWAP ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        if ( getV0() + left_horizon[idx] < horizon[getU0() + idx] ) { return false; }
      }
    }
    return true;
  }

  bool isPatchDimensionSwitched() {
    if ( ( getPatchOrientation() == PATCH_ORIENTATION_DEFAULT ) ||
         ( getPatchOrientation() == PATCH_ORIENTATION_ROT180 ) ||
         ( getPatchOrientation() == PATCH_ORIENTATION_MIRROR ) ||
         ( getPatchOrientation() == PATCH_ORIENTATION_MROT180 ) )
      return false;
    else
      return true;
  }

  void update_horizon( std::vector<int>& horizon,
                       std::vector<int>& top_horizon,
                       std::vector<int>& bottom_horizon,
                       std::vector<int>& right_horizon,
                       std::vector<int>& left_horizon ) {
    size_t best_u           = getU0();
    size_t best_v           = getV0();
    size_t best_orientation = getPatchOrientation();
    int    newVal;
    if ( best_orientation == PATCH_ORIENTATION_DEFAULT ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        newVal = int( best_v + getSizeV0() - 1 - top_horizon[idx] );
        if ( newVal > horizon[best_u + idx] ) horizon[best_u + idx] = newVal;
      }
    } else if ( best_orientation == PATCH_ORIENTATION_ROT90 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        newVal = int( best_v + getSizeU0() - 1 - right_horizon[getSizeV0() - 1 - idx] );
        if ( newVal > horizon[best_u + idx] ) horizon[best_u + idx] = newVal;
      }
    } else if ( best_orientation == PATCH_ORIENTATION_ROT180 ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        newVal = int( best_v + getSizeV0() - 1 - bottom_horizon[getSizeU0() - 1 - idx] );
        if ( newVal > horizon[best_u + idx] ) horizon[best_u + idx] = newVal;
      }
    } else if ( best_orientation == PATCH_ORIENTATION_ROT270 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        newVal = int( best_v + getSizeU0() - 1 - left_horizon[idx] );
        if ( newVal > horizon[best_u + idx] ) horizon[best_u + idx] = newVal;
      }
    } else if ( best_orientation == PATCH_ORIENTATION_MIRROR ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        newVal = int( best_v + getSizeV0() - 1 - top_horizon[getSizeU0() - 1 - idx] );
        if ( newVal > horizon[best_u + idx] ) horizon[best_u + idx] = newVal;
      }
    } else if ( best_orientation == PATCH_ORIENTATION_MROT90 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        newVal = int( best_v + getSizeU0() - 1 - left_horizon[getSizeV0() - 1 - idx] );
        if ( newVal > horizon[best_u + idx] ) horizon[best_u + idx] = newVal;
      }
    } else if ( best_orientation == PATCH_ORIENTATION_MROT180 ) {
      for ( int idx = 0; idx < getSizeU0(); idx++ ) {
        newVal = int( best_v + getSizeV0() - 1 - bottom_horizon[idx] );
        if ( newVal > horizon[best_u + idx] ) horizon[best_u + idx] = newVal;
      }
    } else if ( best_orientation == PATCH_ORIENTATION_MROT270 ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        newVal = int( best_v + getSizeU0() - 1 - right_horizon[idx] );
        if ( newVal > horizon[best_u + idx] ) horizon[best_u + idx] = newVal;
      }
    } else if ( best_orientation == PATCH_ORIENTATION_SWAP ) {
      for ( int idx = 0; idx < getSizeV0(); idx++ ) {
        newVal = int( best_v + getSizeU0() - 1 - right_horizon[idx] );
        if ( newVal > horizon[best_u + idx] ) horizon[best_u + idx] = newVal;
      }
    }
  }

  GPAPatchData& getPreGPAPatchData() { return preGPAPatchData_; }
  GPAPatchData  getPreGPAPatchData() const { return preGPAPatchData_; }

  GPAPatchData& getCurGPAPatchData() { return curGPAPatchData_; }
  GPAPatchData  getCurGPAPatchData() const { return curGPAPatchData_; }

  int patchBlock2CanvasBlockForGPA( const size_t uBlk,
                                    const size_t vBlk,
                                    size_t       canvasStrideBlk,
                                    size_t       canvasHeightBlk ) const {
    size_t x, y;
    switch ( curGPAPatchData_.patchOrientation ) {
      case PATCH_ORIENTATION_DEFAULT:
        x = uBlk + curGPAPatchData_.u0;
        y = vBlk + curGPAPatchData_.v0;
        break;
      case PATCH_ORIENTATION_ROT90:
        x = ( curGPAPatchData_.sizeV0 - 1 - vBlk ) + curGPAPatchData_.u0;
        y = uBlk + curGPAPatchData_.v0;
        break;
      case PATCH_ORIENTATION_ROT180:
        x = ( curGPAPatchData_.sizeU0 - 1 - uBlk ) + curGPAPatchData_.u0;
        y = ( curGPAPatchData_.sizeV0 - 1 - vBlk ) + curGPAPatchData_.v0;
        break;
      case PATCH_ORIENTATION_ROT270:
        x = vBlk + curGPAPatchData_.u0;
        y = ( curGPAPatchData_.sizeU0 - 1 - uBlk ) + curGPAPatchData_.v0;
        break;
      case PATCH_ORIENTATION_MIRROR:
        x = ( curGPAPatchData_.sizeU0 - 1 - uBlk ) + curGPAPatchData_.u0;
        y = vBlk + curGPAPatchData_.v0;
        break;
      case PATCH_ORIENTATION_MROT90:
        x = ( curGPAPatchData_.sizeV0 - 1 - vBlk ) + curGPAPatchData_.u0;
        y = ( curGPAPatchData_.sizeU0 - 1 - uBlk ) + curGPAPatchData_.v0;
        break;
      case PATCH_ORIENTATION_MROT180:
        x = uBlk + curGPAPatchData_.u0;
        y = ( curGPAPatchData_.sizeV0 - 1 - vBlk ) + curGPAPatchData_.v0;
        break;
      case PATCH_ORIENTATION_MROT270:
        x = vBlk + curGPAPatchData_.u0;
        y = uBlk + curGPAPatchData_.v0;
        break;
      case PATCH_ORIENTATION_SWAP:  // swapAxis
        x = vBlk + curGPAPatchData_.u0;
        y = uBlk + curGPAPatchData_.v0;
        break;
      default: return -1; break;
    }
    // checking the results are within canvas boundary (missing y check)
    if ( x < 0 ) return -1;
    if ( y < 0 ) return -1;
    if ( x >= canvasStrideBlk ) return -1;
    if ( y >= canvasHeightBlk ) return -1;
    return int( x + canvasStrideBlk * y );
  }

  bool checkFitPatchCanvasForGPA( std::vector<bool> canvas,
                                  size_t            canvasStrideBlk,
                                  size_t            canvasHeightBlk,
                                  bool              bPrecedence,
                                  int               safeguard = 0 ) {
    for ( size_t v0 = 0; v0 < curGPAPatchData_.sizeV0; ++v0 ) {
      for ( size_t u0 = 0; u0 < curGPAPatchData_.sizeU0; ++u0 ) {
        for ( int deltaY = -safeguard; deltaY < safeguard + 1; deltaY++ ) {
          for ( int deltaX = -safeguard; deltaX < safeguard + 1; deltaX++ ) {
            int pos = patchBlock2CanvasBlockForGPA( u0 + deltaX, v0 + deltaY, canvasStrideBlk, canvasHeightBlk );
            if ( pos < 0 ) {
              return false;
            } else {
              if ( bPrecedence ) {
                if ( canvas[pos] && occupancy_[u0 + getSizeU0() * v0] ) { return false; }
              } else {
                if ( canvas[pos] ) { return false; }
              }
            }
          }
        }
      }
    }

    return true;
  }

  void allocOneLayerData() {
    pointLocalReconstructionLevel_       = 0;
    pointLocalReconstructionModeByPatch_ = 0;
    // printf( "sizeU0_ =%d,  sizeV0_ = %d\n", sizeU0_ , sizeV0_ );
    pointLocalReconstructionModeByBlock_.resize( sizeU0_ * sizeV0_, 0 );
    std::fill( pointLocalReconstructionModeByBlock_.begin(), pointLocalReconstructionModeByBlock_.end(), 0 );
  }
  uint8_t& getPointLocalReconstructionLevel() { return pointLocalReconstructionLevel_; }
  uint8_t  getPointLocalReconstructionLevel() const { return pointLocalReconstructionLevel_; }
  uint8_t& getPointLocalReconstructionMode( const size_t u = 0, const size_t v = 0 ) {
    if ( pointLocalReconstructionLevel_ == 1 ) {
      return pointLocalReconstructionModeByPatch_;
    } else {
      return pointLocalReconstructionModeByBlock_[v * sizeU0_ + u];
    }
  }
  uint8_t getPointLocalReconstructionMode( const size_t u = 0, const size_t v = 0 ) const {
    if ( pointLocalReconstructionLevel_ == 1 ) {
      return pointLocalReconstructionModeByPatch_;
    } else {
      return pointLocalReconstructionModeByBlock_[v * sizeU0_ + u];
    }
  }

  static void InverseRotatePosition45DegreeOnAxis( size_t Axis, size_t lod, PCCPoint3D input, PCCVector3D& output ) {
#ifdef EXPAND_RANGE_ENCODER
    size_t s = ( 1u << (lod-1) ) - 1;
#else
    size_t s = ( 1u << lod ) - 1;
#endif
    output.x() = input.x();
    output.y() = input.y();
    output.z() = input.z();
    if ( Axis == 1 ) {  // projection plane is defined by Y Axis.
      output.x() = input.x() - input.z() + s;
      output.x() /= 2.0;
      output.z() = input.x() + input.z() - s;
      output.z() /= 2.0;
    }
    if ( Axis == 2 ) {  // projection plane is defined by X Axis.
      output.z() = input.z() - input.y() + s;
      output.z() /= 2.0;
      output.y() = input.z() + input.y() - s;
      output.y() /= 2.0;
    }
    if ( Axis == 3 ) {  // projection plane is defined by Z Axis.
      output.y() = input.y() - input.x() + s;
      output.y() /= 2.0;
      output.x() = input.y() + input.x() - s;
      output.x() /= 2.0;
    }
  }

  static void RotatePosition45DegreeOnAxis( size_t Axis, size_t lod, PCCPoint3D input, PCCPoint3D& output ) {
#ifdef EXPAND_RANGE_ENCODER
    size_t shif = ( 1u << (lod-1) ) - 1;
#else
    size_t shif = ( 1u << lod ) - 1;
#endif
    output      = input;
    if ( Axis == 1 ) {  // Additional plane are defined by Y Axis.
      output.x() = input.x() + input.z();
      output.z() = -input.x() + input.z() + shif;
    }
    if ( Axis == 2 ) {  // Additional plane are defined by X Axis.
      output.z() = input.z() + input.y();
      output.y() = -input.z() + input.y() + shif;
    }
    if ( Axis == 3 ) {
      output.y() = input.y() + input.x();
      output.x() = -input.y() + input.x() + shif;
    }
  }

#define OCC( tl, t, tr, l, v, r, bl, b, br ) \
  ( ( tl << 7 ) | ( t << 6 ) | ( tr << 5 ) | ( l << 4 ) | ( r << 3 ) | ( bl << 2 ) | ( b << 1 ) | ( br ) )
#define ORIENTATION( p, c, w )                                                                               \
  g_orientation[OCC( p[c - w - 1], p[c - w], p[c - w + 1], p[c - 1], p[c], p[c + 1], p[c + w - 1], p[c + w], \
                     p[c + w + 1] )];

  inline void   setIndexCopy( size_t index ) { indexCopy_ = index; }
  inline size_t getIndexCopy() { return indexCopy_; }

  void setDepthFromGeometryVideo( const std::vector<uint16_t>& geometryVideo,
                                  const int32_t                u2,
                                  const int32_t                v2,
                                  int32_t                      width,
                                  int32_t                      height,
                                  int32_t                      occupancyPrecision,
                                  int16_t*                     depth ) {
    const int32_t x0 = ( int32_t )( u0_ * occupancyResolution_ );
    const int32_t y0 = ( int32_t )( v0_ * occupancyResolution_ );
    depth += v2 * depthMapWidth_;
    switch ( patchOrientation_ ) {
      case PATCH_ORIENTATION_DEFAULT:
        for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
          int32_t y = ( v + y0 ) * width + x0;
          for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) { depth[u] = geometryVideo[u + y]; }
        }
        break;
      case PATCH_ORIENTATION_SWAP:  // swapAxis
        for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
          int32_t x = v + x0 + width * y0;
          for ( int32_t i = 0, u = u2, y = u2 * width; i < occupancyPrecision; i++, u += 1, y += width ) {
            depth[u] = geometryVideo[x + y];
          }
        }
        break;
      case PATCH_ORIENTATION_ROT90:
        for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
          int32_t x = int32_t( ( sizeV0_ * occupancyResolution_ - 1 - v ) + x0 );
          for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
            int32_t y = u + y0;
            depth[u]  = geometryVideo[x + width * y];
          }
        }
        break;
      case PATCH_ORIENTATION_ROT180:
        for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
          int32_t y = ( int32_t )( ( sizeV0_ * occupancyResolution_ - 1 - v ) + y0 );
          for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
            int32_t x = ( int32_t )( ( sizeU0_ * occupancyResolution_ - 1 - u ) + x0 );
            depth[u]  = geometryVideo[x + width * y];
          }
        }
        break;
      case PATCH_ORIENTATION_ROT270:
        for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
          int32_t x = v + x0;
          for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
            int32_t y = int32_t( ( sizeU0_ * occupancyResolution_ - 1 - u ) + y0 );
            depth[u]  = geometryVideo[x + width * y];
          }
        }
        break;
      case PATCH_ORIENTATION_MIRROR:
        for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
          int32_t y = v + y0;
          for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
            int32_t x = ( sizeU0_ * occupancyResolution_ - 1 - u ) + x0;
            depth[u]  = geometryVideo[x + width * y];
          }
        }
        break;
      case PATCH_ORIENTATION_MROT90:
        for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
          int32_t x = ( sizeV0_ * occupancyResolution_ - 1 - v ) + x0;
          for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
            int32_t y = ( sizeU0_ * occupancyResolution_ - 1 - u ) + y0;
            depth[u]  = geometryVideo[x + width * y];
          }
        }
        break;
      case PATCH_ORIENTATION_MROT180:
        for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
          int32_t y = int32_t( ( sizeV0_ * occupancyResolution_ - 1 - v ) + y0 );
          for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
            int32_t x = u + x0;
            depth[u]  = geometryVideo[x + width * y];
          }
        }
        break;
      case PATCH_ORIENTATION_MROT270:
        for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
          int32_t x = v + x0;
          for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
            int32_t y = u + y0;
            depth[u]  = geometryVideo[x + width * y];
          }
        }
        break;
      default:
        assert( 0 );
        printf( "patchOrientation_ = %zu not supported \n", patchOrientation_ );
        exit( -1 );
        break;
    }
  }

  void setLocalData( const std::vector<uint8_t>&  occupancyMapVideo,
                     const std::vector<uint16_t>& geometryVideo,
                     std::vector<size_t>&         blockToPatch,
                     const int32_t                width,
                     const int32_t                height,
                     const int32_t                occupancyPrecision,
                     const int32_t                threhold ) {
    border_         = occupancyPrecision >= 8 ? 16 : 8;
    depthMapWidth_  = sizeU0_ * occupancyResolution_ + 2 * border_;
    depthMapHeight_ = sizeV0_ * occupancyResolution_ + 2 * border_;
    depthMap_.resize( depthMapWidth_ * depthMapHeight_, 0 );
    occupancyMap_.resize( depthMapWidth_ * depthMapHeight_, 0 );
    const int32_t patchIndexPlusOne  = indexCopy_ + 1;
    const auto    ocmWidth           = width / occupancyPrecision;
    const int32_t blockToPatchWidth  = width / occupancyResolution_;
    const int32_t blockToPatchHeight = height / occupancyResolution_;
    const int32_t occupancyResByPres = occupancyResolution_ / occupancyPrecision;
    const int32_t shiftPrecision =
        occupancyPrecision == 1 ? 0 : occupancyPrecision == 2 ? 1 : occupancyPrecision == 4 ? 2 : 3;
    uint8_t* ocm   = occupancyMap_.data() + border_ + depthMapWidth_ * border_;
    int16_t* depth = depthMap_.data() + border_ + depthMapWidth_ * border_;

    for ( int32_t v0 = 0; v0 < sizeV0_; ++v0 ) {
      for ( int32_t u0 = 0; u0 < sizeU0_; ++u0 ) {
        const int32_t blockIndex = patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
        if ( blockToPatch[blockIndex] == patchIndexPlusOne ) {
          for ( int32_t v1 = 0; v1 < occupancyResByPres; v1++ ) {
            const int32_t v2 = v0 * occupancyResolution_ + v1 * occupancyPrecision;
            for ( int32_t u1 = 0; u1 < occupancyResByPres; u1++ ) {
              const int32_t u2 = u0 * occupancyResolution_ + u1 * occupancyPrecision;
              size_t        x, y;
              patch2Canvas( u2, v2, width, height, x, y );
              if ( occupancyMapVideo[( y >> shiftPrecision ) * ocmWidth + ( x >> shiftPrecision )] > threhold ) {
                setDepthFromGeometryVideo( geometryVideo, u2, v2, width, height, occupancyPrecision, depth );
                for ( int32_t j = 0, v = v2 * depthMapWidth_; j < occupancyPrecision; j++, v += depthMapWidth_ ) {
                  for ( int32_t i = 0, u = u2 + v; i < occupancyPrecision; i++, u++ ) { ocm[u] = 1; }
                }
              }
            }
          }
        }
      }
    }
  }

  void generateBorderPoints3D() {
    const int32_t w   = depthMapWidth_;
    boundingBox_.min_ = PCCPoint3D( ( std::numeric_limits<int16_t>::max )() );
    boundingBox_.max_ = PCCPoint3D( ( std::numeric_limits<int16_t>::min )() );
    int32_t c         = border_ + border_ * w;
    for ( int32_t v = 0; v < sizeV0_ * occupancyResolution_; v++, c += border_ << 1 ) {
      for ( int32_t u = 0; u < sizeU0_ * occupancyResolution_; u++, c++ ) {
        if ( ( occupancyMap_[c] ) &&
             ( ( !occupancyMap_[c - 1] ) || ( !occupancyMap_[c + 1] ) || ( !occupancyMap_[c - w] ) ||
               ( !occupancyMap_[c + w] ) || ( !occupancyMap_[c - 2] ) || ( !occupancyMap_[c + 2] ) ||
               ( !occupancyMap_[c - 2 * w] ) || ( !occupancyMap_[c + 2 * w] ) || ( !occupancyMap_[c + w - 1] ) ||
               ( !occupancyMap_[c + w + 1] ) || ( !occupancyMap_[c - w - 1] ) || ( !occupancyMap_[c - w + 1] ) ) ) {
          auto point = generatePoint( u, v, depthMap_[border_ + ( v + border_ ) * w + u] );
          borderPoints_.push_back( point );
          boundingBox_.add( point );
        }
      }
    }
  }

  inline int16_t               getDepthMapWidth() { return depthMapWidth_; }
  inline int16_t               getDepthMapHeight() { return depthMapHeight_; }
  inline int16_t               getBorder() { return border_; }
  inline std::vector<uint8_t>& getOccupancyMap() { return occupancyMap_; }
  inline std::vector<size_t>&  getNeighboringPatches() { return neighboringPatches_; }
  inline const int16_t         getDepthMap( size_t u, size_t v ) const {
    return depthMap_[( v + border_ ) * depthMapWidth_ + u + border_];
  }
  inline const int16_t getOccupancyMap( size_t u, size_t v ) const {
    return occupancyMap_[( v + border_ ) * depthMapWidth_ + u + border_];
  }
  inline int16_t generateDepth( PCCPoint3D point ) const {
    return projectionMode_ == 0 ? point[normalAxis_] - d1_ : d1_ - point[normalAxis_];
  }
  inline bool intersects( PCCPatch& other ) { return boundingBox_.intersects( other.boundingBox_ ); }
  inline void clearPatchBlockFilteringData() {
    borderPoints_.clear();
    neighboringPatches_.clear();
    depthMap_.clear();
  }

  void filtering( const int8_t           passesCount,
                  const int8_t           filterSize,
                  const int8_t           log2Threshold,
                  std::vector<PCCPatch>& patches ) {
    const int8_t         localWindowSizeU = filterSize;
    const int32_t        localWindowSizeV = filterSize >> 1;
    PCCInt16Box3D        boundingBox      = boundingBox_;
    const int16_t        sizeX            = sizeU0_ * occupancyResolution_;
    const int16_t        sizeY            = sizeV0_ * occupancyResolution_;
    const int16_t        threshold        = log2Threshold * log2Threshold;
    const int32_t        size             = depthMapWidth_ * depthMapHeight_;
    const int16_t        undefined        = ( std::numeric_limits<int16_t>::max )();
    std::vector<int16_t> neighborDepth;
    neighborDepth.resize( size, undefined );
    boundingBox.min_ -= PCCPoint3D( 8 );
    boundingBox.max_ += PCCPoint3D( 8 );
    const int32_t shift = ( int32_t )( ( -(int32_t)v1_ + border_ ) * depthMapWidth_ - (int32_t)u1_ + border_ );
    for ( auto& i : neighboringPatches_ ) {
      for ( auto& point : patches[i].borderPoints_ ) {
        if ( boundingBox.contains( point ) ) {
          int32_t       d = generateDepth( point );
          const int32_t c = shift + point[bitangentAxis_] * depthMapWidth_ + point[tangentAxis_];
          if ( abs( d - depthMap_[c] ) <= threshold ) {
            if ( abs( d - depthMap_[c] ) < abs( neighborDepth[c] - depthMap_[c] ) ) { neighborDepth[c] = d; }
          }
        }
      }
    }
    std::vector<uint8_t> newOccupancyMap;
    newOccupancyMap.resize( size, 0 );
    for ( size_t iter = 0; iter < passesCount; iter++ ) {  // HN : OMap precision =4, passescount = 2
      uint8_t* src = iter % 2 == 0 ? occupancyMap_.data() : newOccupancyMap.data();  // iter=0, occupancyMap_.data(),
                                                                                     // iter=1, newOccupancyMap.data()
      uint8_t* dst = iter % 2 == 1 ? occupancyMap_.data() : newOccupancyMap.data();
      for ( int32_t v = 0, c = border_ * depthMapWidth_ + border_; v < sizeY; v++, c += 2 * border_ ) {
        for ( int32_t u = 0; u < sizeX; u++, c++ ) {
          if ( src[c] == 0 ) {
            dst[c] = 0;
          } else {
            int8_t numNeigblor = src[c - 1] + src[c + 1] + src[c - depthMapWidth_] + src[c + depthMapWidth_];
            if ( numNeigblor == 0 ) {
              dst[c] = 0;
            } else if ( numNeigblor == 4 ) {
              dst[c] = 1;
            } else {
              float         sumE = 0, sumP = 0;
              int32_t       count  = 0;
              const int8_t  orX    = ORIENTATION( src, c, depthMapWidth_ );
              const int8_t  orY    = ( orX + 2 ) % 8;
              const int8_t* dX     = g_dilate[orX];
              const int8_t* dY     = g_dilate[orY];
              const int32_t shiftX = dX[0] + dX[1] * depthMapWidth_;
              const int32_t shiftY = dY[0] + dY[1] * depthMapWidth_;
              const int32_t dE     = depthMap_[c - shiftX];
              const int32_t dP     = depthMap_[c];
              int32_t       cx     = c - localWindowSizeU * shiftX - localWindowSizeV * shiftY;
              int32_t       du1    = -localWindowSizeU * dX[0] + -localWindowSizeV * dY[0];
              int32_t       dv1    = -localWindowSizeU * dX[1] + -localWindowSizeV * dY[1];
              for ( int32_t dx = -localWindowSizeU; dx <= localWindowSizeU;
                    dx++, cx += shiftX, du1 += dX[0], dv1 += dX[1] ) {
                for ( int32_t dy = -localWindowSizeV, cn = cx, du = du1, dv = dv1; dy <= localWindowSizeV;
                      dy++, cn += shiftY, du += dY[0], dv += dY[1] ) {
                  if ( neighborDepth[cn] != undefined ) {
                    sumP += sqrt( du * du + dv * dv + ( neighborDepth[cn] - dP ) * ( neighborDepth[cn] - dP ) );
                    sumE += sqrt( ( du + dX[0] ) * ( du + dX[0] ) + ( dv + dX[1] ) * ( dv + dX[1] ) +
                                  ( neighborDepth[cn] - dE ) * ( neighborDepth[cn] - dE ) );
                    count++;
                  }
                }
              }
              dst[c] = ( ( count == 0 ) || ( sumE >= sumP ) );
            }
          }
        }
      }
    }
    if ( passesCount % 2 == 1 ) { memcpy( occupancyMap_.data(), newOccupancyMap.data(), size * sizeof( uint8_t ) ); }
    //   int8_t diffMap;
    //   uint32_t countM1 = 0;
    //   uint32_t count0 = 0;
    //   uint32_t countP1 = 0;
    //   uint32_t sumOMap = 0;
    //   uint32_t sumNewOMap = 0;
    // for ( size_t k = 0; k < size; k++ ) {
    //	diffMap = occupancyMap_[k] - newOccupancyMap[k];
    //         if ( diffMap == -1 ) countM1++;
    //         if ( diffMap == 0 ) count0++;
    //         if ( diffMap == 1 ) countP1++;
    //         sumOMap += occupancyMap_[k];
    //         sumNewOMap += newOccupancyMap[k];
    //}
  }

 private:
  size_t index_;          // patch index
  size_t originalIndex_;  // patch original index
  size_t u1_;             // tangential shift
  size_t v1_;             // bitangential shift
  size_t d1_;             // depth shift
  size_t sizeD_;          // size for depth
  size_t sizeU_;          // size for depth
  size_t sizeV_;          // size for depth
  size_t u0_;             // location in packed image (n*occupancyResolution_)
  size_t v0_;             // location in packed image (n*occupancyResolution_)
  size_t sizeU0_;         // size of occupancy map (n*occupancyResolution_)
  size_t sizeV0_;         // size of occupancy map (n*occupancyResolution_)
  size_t size2DXInPixel_;
  size_t size2DYInPixel_;
  size_t occupancyResolution_;  // occupancy map resolution
  size_t projectionMode_;       // 0: related to the min depth value; 1: related to
                                // the max value
  size_t                  levelOfDetailX_;
  size_t                  levelOfDetailY_;
  size_t                  normalAxis_;     // x
  size_t                  tangentAxis_;    // y
  size_t                  bitangentAxis_;  // z
  std::vector<int16_t>    depth_[2];       // depth
  std::vector<bool>       occupancy_;      // occupancy map
  size_t                  viewId_;         // viewId in [0,1,2,3,4,5]
  int32_t                 bestMatchIdx_;   // index of matched patch from pre-frame patch.
  size_t                  refAtlasFrameIdx_;
  size_t                  predType_;
  std::vector<int16_t>    depthEnhancedDeltaD_;  // Enhanced delta depht
  std::vector<int64_t>    depth0PCidx_;          // for Surface separation
  size_t                  patchOrientation_;     // patch orientation in canvas atlas
  uint8_t                 pointLocalReconstructionLevel_;
  uint8_t                 pointLocalReconstructionModeByPatch_;
  std::vector<uint8_t>    pointLocalReconstructionModeByBlock_;
  GPAPatchData            curGPAPatchData_;
  GPAPatchData            preGPAPatchData_;
  bool                    isGlobalPatch_;
  size_t                  axisOfAdditionalPlane_;
  size_t                  d0Count_;
  size_t                  eomCount_;
  size_t                  eomandD1Count_;
  uint8_t                 patchType_;
  bool                    isRoiPatch_;
  size_t                  roiIndex_;
  size_t                  indexCopy_;           // patch index
  PCCInt16Box3D           boundingBox_;         // 3D bouding box of patch
  int16_t                 border_;              // Size of the depht Map (width + 2 border)
  int16_t                 depthMapWidth_;       // Size of the depht Map (width + 2 border)
  int16_t                 depthMapHeight_;      // Size of the depht Map (width + 2 border)
  std::vector<size_t>     neighboringPatches_;  // List of neighboring patch index
  std::vector<int16_t>    depthMap_;            // Depth map
  std::vector<uint8_t>    occupancyMap_;        // Occupancy map
  std::vector<PCCPoint3D> borderPoints_;        // 3D points created from borders of
                                                // the patch
};

class PatchBlockFiltering {
 public:
  PatchBlockFiltering() {}
  ~PatchBlockFiltering() {}

  inline void setPatches( std::vector<PCCPatch>* patches ) { patches_ = patches; }
  inline void setBlockToPatch( std::vector<size_t>* value ) { blockToPatch_ = value; }
  inline void setOccupancyMapEncoder( std::vector<uint32_t>* value ) { occupancyMapEncoder_ = value; }
  inline void setOccupancyMapVideo( const std::vector<uint8_t>* value ) { occupancyMapVideo_ = value; }
  inline void setGeometryVideo( const std::vector<uint16_t>* value ) { geometryVideo_ = value; }

  void patchBorderFiltering( size_t imageWidth,
                             size_t imageHeight,
                             size_t occupancyResolution,
                             size_t occupancyPrecision,
                             size_t thresholdLossyOM,
                             int8_t passesCount,
                             int8_t filterSize,
                             int8_t log2Threshold ) {
    // Generate border points
    for ( size_t patchIndex = 0; patchIndex < patches_->size(); patchIndex++ ) {
      auto& patch = patches_->at( patchIndex );
      patch.setIndexCopy( patchIndex );
      patch.setLocalData( *occupancyMapVideo_, *geometryVideo_, *blockToPatch_, imageWidth, imageHeight,
                          occupancyPrecision, thresholdLossyOM );
      patch.generateBorderPoints3D();
    }
    for ( auto& patch : *patches_ ) {
      for ( auto& other : *patches_ ) {
        if ( patch.getIndexCopy() != other.getIndexCopy() && patch.intersects( other ) ) {
          patch.getNeighboringPatches().push_back( other.getIndexCopy() );
        }
      }
    }
    // Filtering;
    for ( auto& patch : *patches_ ) { patch.filtering( passesCount, filterSize, log2Threshold, *patches_ ); }
    for ( auto& patch : *patches_ ) { patch.clearPatchBlockFilteringData(); }
  }

 private:
  std::vector<PCCPatch>*       patches_;
  std::vector<size_t>*         blockToPatch_;
  std::vector<uint32_t>*       occupancyMapEncoder_;
  const std::vector<uint8_t>*  occupancyMapVideo_;
  const std::vector<uint16_t>* geometryVideo_;
};

struct PCCEOMInfosPerPatch {
  size_t patchIdx_;
  size_t numOfEOMPoints_;
  size_t offset_;
};

struct PCCEomPatch {
  size_t              u0_;
  size_t              v0_;
  size_t              sizeU_;
  size_t              sizeV_;
  size_t              eomCount_;  // in this EomPatch
  std::vector<size_t> memberPatches;
  std::vector<size_t> eomCountPerPatch;
};

struct PCCRawPointsPatch {
  size_t                u1_;  // tangential shift
  size_t                v1_;  // bitangential shift
  size_t                d1_;  // depth shift
  size_t                sizeU_;
  size_t                sizeV_;
  size_t                u0_;
  size_t                v0_;
  size_t                sizeV0_;
  size_t                sizeU0_;
  size_t                occupancyResolution_;
  std::vector<bool>     occupancy_;
  std::vector<uint16_t> x_;
  std::vector<uint16_t> y_;
  std::vector<uint16_t> z_;
  std::vector<uint16_t> r_;
  std::vector<uint16_t> g_;
  std::vector<uint16_t> b_;

  size_t numberOfEOMPoints_;
  size_t numberOfRawPoints_;
  size_t numberOfRawPointsColors_;

  // GPA.
  size_t preV0_;
  size_t tempV0_;

  void resize( const size_t size ) {
    x_.resize( size );
    y_.resize( size );
    z_.resize( size );
  }
  void resize( const size_t size, const uint16_t val ) {
    x_.resize( size, val );
    y_.resize( size, val );
    z_.resize( size, val );
  }

  const size_t size() { return x_.size(); }

  const size_t sizeOfColor() { return r_.size(); }
  void         setNumberOfRawPoints( size_t numberOfRawPoints ) { numberOfRawPoints_ = numberOfRawPoints; }
  void         setNumberOfRawPointsColors( size_t numberOfRawPointsColors ) {
    numberOfRawPointsColors_ = numberOfRawPointsColors;
  }
  const size_t getNumberOfRawPoints() { return numberOfRawPoints_; }
  const size_t getNumberOfRawPointsColors() { return numberOfRawPointsColors_; }

  void resizeColor( const size_t size ) {
    r_.resize( size );
    g_.resize( size );
    b_.resize( size );
  }
  void resizeColor( const size_t size, const uint16_t val ) {
    r_.resize( size, val );
    g_.resize( size, val );
    b_.resize( size, val );
  }
  void reset() {
    sizeU_                   = 0;
    sizeV_                   = 0;
    u0_                      = 0;
    v0_                      = 0;
    sizeV0_                  = 0;
    sizeU0_                  = 0;
    occupancyResolution_     = 0;
    numberOfEOMPoints_       = 0;
    numberOfRawPoints_       = 0;
    numberOfRawPointsColors_ = 0;
    preV0_                   = 0;
    tempV0_                  = 0;
    occupancy_.resize( 0 );
    x_.resize( 0 );
    y_.resize( 0 );
    z_.resize( 0 );
    r_.resize( 0 );
    g_.resize( 0 );
    b_.resize( 0 );
  }
};
}  // namespace pcc

#endif /* PCCPatch_h */
