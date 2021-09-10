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
  bool              isMatched_;
  bool              isGlobalPatch_;
  int               globalPatchIndex_;
  size_t            sizeU0_;
  size_t            sizeV0_;
  size_t            u0_;
  size_t            v0_;
  size_t            patchOrientation_;
  std::vector<bool> occupancy_;
  void              initialize() {
    isMatched_        = false;
    isGlobalPatch_    = false;
    globalPatchIndex_ = -1;
    sizeU0_           = 0;
    sizeV0_           = 0;
    occupancy_.clear();
    u0_               = -1;
    v0_               = -1;
    patchOrientation_ = -1;
  }
  bool isPatchDimensionSwitched() {
    if ( ( patchOrientation_ == PATCH_ORIENTATION_DEFAULT ) || ( patchOrientation_ == PATCH_ORIENTATION_ROT180 ) ||
         ( patchOrientation_ == PATCH_ORIENTATION_MIRROR ) || ( patchOrientation_ == PATCH_ORIENTATION_MROT180 ) ) {
      return false;
    } else {
      return true;
    }
  }
};

class PCCPatch {
 public:
  PCCPatch();
  ~PCCPatch();
  size_t                      getEOMCount() const { return eomCount_; }
  size_t                      getEOMandD1Count() const { return eomandD1Count_; }
  size_t                      getD0Count() const { return d0Count_; }
  size_t                      getFrameIndex() const { return frameIndex_; }
  size_t                      getTileIndex() const { return tileIndex_; }
  size_t                      getIndexInFrame() { return indexInFrame_; }
  size_t                      getRefAtlasFrameIndex() const { return refAtlasFrameIdx_; }
  size_t                      getPredType() { return predType_; }
  size_t                      getPredType() const { return predType_; }
  uint8_t                     getPatchType() const { return patchType_; }
  size_t                      getLodScaleX() { return levelOfDetailX_; }
  size_t                      getLodScaleY() { return levelOfDetailY_; }
  size_t                      getLodScaleX() const { return levelOfDetailX_; }
  size_t                      getLodScaleY() const { return levelOfDetailY_; }
  size_t                      getIndex() const { return index_; }
  size_t                      getRoiIndex() const { return roiIndex_; }
  size_t                      getOriginalIndex() const { return originalIndex_; }
  size_t                      getU1() const { return u1_; }
  size_t                      getV1() const { return v1_; }
  size_t                      getD1() const { return d1_; }
  size_t                      getSizeD() const { return sizeD_; }
  size_t                      getSizeDPixel() const { return sizeDPixel_; }
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
  size_t                      getPatchOrientation() const { return patchOrientation_; }
  bool                        getIsGlobalPatch() const { return isGlobalPatch_; }
  size_t                      getPatchSize2DXInPixel() const { return size2DXInPixel_; }
  size_t                      getPatchSize2DYInPixel() const { return size2DYInPixel_; }
  size_t                      getAxisOfAdditionalPlane() const { return axisOfAdditionalPlane_; }
  const std::vector<int16_t>& getDepth( size_t i ) const { return depth_[i]; }
  const std::vector<bool>&    getOccupancy() const { return occupancy_; }
  const std::vector<int16_t>& getDepthEOM() const { return depthEOM_; }
  const std::vector<int64_t>& getDepth0PccIdx() const { return depth0PCidx_; }
  const int16_t               getDepth( size_t i, size_t j ) const { return depth_[i][j]; }
  const bool                  getOccupancy( size_t i ) const { return occupancy_[i]; }
  const int16_t               getDepthEOM( size_t i ) const { return depthEOM_[i]; }
  const int64_t               getDepth0PccIdx( size_t i ) const { return depth0PCidx_[i]; }

  void setIndex( size_t value ) { index_ = value; }
  void setOriginalIndex( size_t value ) { originalIndex_ = value; }
  void setU1( size_t value ) { u1_ = value; }
  void setV1( size_t value ) { v1_ = value; }
  void setD1( size_t value ) { d1_ = value; }
  void setSizeD( size_t value ) { sizeD_ = value; }
  void setSizeDPixel( size_t value ) { sizeDPixel_ = value; }
  void setSizeU( size_t value ) { sizeU_ = value; }
  void setSizeV( size_t value ) { sizeV_ = value; }
  void setU0( size_t value ) { u0_ = value; }
  void setV0( size_t value ) { v0_ = value; }
  void setSizeU0( size_t value ) { sizeU0_ = value; }
  void setSizeV0( size_t value ) { sizeV0_ = value; }
  void setOccupancyResolution( size_t value ) { occupancyResolution_ = value; }
  void setProjectionMode( size_t value ) { projectionMode_ = value; }
  void setNormalAxis( size_t value ) { normalAxis_ = value; }
  void setTangentAxis( size_t value ) { tangentAxis_ = value; }
  void setBitangentAxis( size_t value ) { bitangentAxis_ = value; }
  void setOccupancy( const std::vector<bool>& occupancy ) { occupancy_ = occupancy; }
  void setDepth( size_t i, const std::vector<int16_t>& depth ) { depth_[i] = depth; }
  void setDepth( size_t i, size_t j, int16_t value ) { depth_[i][j] = value; }
  void setOccupancy( size_t i, bool value ) { occupancy_[i] = value; }
  void setDepth0PccIdx( size_t i, int64_t value ) { depth0PCidx_[i] = value; }
  void setDepthEOM( size_t i, int16_t value ) { depthEOM_[i] = value; }
  void setAxisOfAdditionalPlane( size_t value ) { axisOfAdditionalPlane_ = value; }
  void setIsRoiPatch( bool value ) { isRoiPatch_ = value; }
  void setIsGlobalPatch( bool value ) { isGlobalPatch_ = value; }
  void setRoiIndex( size_t value ) { roiIndex_ = value; }
  void setPatchOrientation( size_t value ) { patchOrientation_ = value; }
  void setEOMCount( size_t value ) { eomCount_ = value; }
  void setEOMandD1Count( size_t value ) { eomandD1Count_ = value; }
  void setD0Count( size_t value ) { d0Count_ = value; }
  void setFrameIndex( size_t value ) { frameIndex_ = value; }
  void setTileIndex( size_t value ) { tileIndex_ = value; }
  void setIndexInFrame( size_t value ) { indexInFrame_ = value; }
  void setBestMatchIdx( int32_t value ) { bestMatchIdx_ = value; }
  void setRefAtlasFrameIndex( size_t value ) { refAtlasFrameIdx_ = value; }
  void setPredType( size_t value ) { predType_ = value; }
  void setPatchType( uint8_t value ) { patchType_ = value; }
  void setLodScaleX( size_t value ) { levelOfDetailX_ = value; }
  void setLodScaleYIdc( size_t value ) { levelOfDetailY_ = value; }
  void setPatchSize2DXInPixel( size_t value ) { size2DXInPixel_ = value; }
  void setPatchSize2DYInPixel( size_t value ) { size2DYInPixel_ = value; }

  void allocDepth( size_t i, size_t size, int16_t value ) { depth_[i].resize( size, value ); }
  void allocOccupancy( size_t size, bool value ) { occupancy_.resize( size, value ); }
  void allocDepth0PccIdx( size_t size, int64_t value ) { depth0PCidx_.resize( size, value ); }
  void allocDepthEOM( size_t size, int16_t value ) { depthEOM_.resize( size, value ); }
  void clearOccupancy() { occupancy_.clear(); }
  void clearDepth( size_t i ) { depth_[i].clear(); }

  inline double generateNormalCoordinate( const uint16_t depth ) const {
    double coord = 0;
    if ( projectionMode_ == 0 ) {
      coord = ( (double)depth + (double)d1_ );
    } else {
      double tmp_depth = double( d1_ ) - double( depth );
      if ( tmp_depth > 0 ) { coord = tmp_depth; }
    }
    return coord;
  }
  void setAxis( size_t axisOfAdditionalPlane,
                size_t normalAxis,
                size_t tangentAxis,
                size_t bitangentAxis,
                size_t projectionMode ) {
    axisOfAdditionalPlane_ = axisOfAdditionalPlane;
    normalAxis_            = normalAxis;
    tangentAxis_           = tangentAxis;
    bitangentAxis_         = bitangentAxis;
    projectionMode_        = projectionMode;
  }

  void setViewId( size_t viewId );

  PCCPoint3D generatePoint( const size_t u, const size_t v, const uint16_t depth ) const {
    PCCPoint3D point0;
    point0[normalAxis_]    = generateNormalCoordinate( depth );
    point0[tangentAxis_]   = ( double( u ) * (double)levelOfDetailX_ + u1_ );
    point0[bitangentAxis_] = ( double( v ) * (double)levelOfDetailY_ + v1_ );
    return point0;
  }

  PCCPoint3D canvasTo3D( const size_t x, const size_t y, const uint16_t depth ) const;

  size_t patch2Canvas( const size_t u, const size_t v, size_t canvasStride, size_t canvasHeight );
  size_t patch2Canvas( const size_t u, const size_t v, size_t canvasStride, size_t canvasHeight, size_t& x, size_t& y );
  int    patchBlock2CanvasBlock( const size_t uBlk,
                                 const size_t vBlk,
                                 size_t       canvasStrideBlk,
                                 size_t       canvasHeightBlk,
                                 const Tile   tile = Tile() ) const;

  bool checkFitPatchCanvas( std::vector<bool> canvas,
                            size_t            canvasStrideBlk,
                            size_t            canvasHeightBlk,
                            bool              bPrecedence,
                            int               safeguard = 0,
                            const Tile        tile      = Tile() );

  bool        smallerRefFirst( const PCCPatch& rhs );
  bool        gt( const PCCPatch& rhs );
  void        print() const;
  void        printDecoder() const;
  friend bool operator<( const PCCPatch& lhs, const PCCPatch& rhs ) {
    return lhs.sizeV_ != rhs.sizeV_ ? lhs.sizeV_ > rhs.sizeV_
                                    : ( lhs.sizeU_ != rhs.sizeU_ ? lhs.sizeU_ > rhs.sizeU_ : lhs.index_ < rhs.index_ );
  }

  void getPatchHorizons( std::vector<int>& topHorizon,
                         std::vector<int>& bottomHorizon,
                         std::vector<int>& rightHorizon,
                         std::vector<int>& leftHorizon );

  int calculateWastedSpace( std::vector<int>& horizon,
                            std::vector<int>& topHorizon,
                            std::vector<int>& bottomHorizon,
                            std::vector<int>& rightHorizon,
                            std::vector<int>& leftHorizon );

  bool isPatchLocationAboveHorizon( std::vector<int>& horizon,
                                    std::vector<int>& topHorizon,
                                    std::vector<int>& bottomHorizon,
                                    std::vector<int>& rightHorizon,
                                    std::vector<int>& leftHorizon );

  bool isPatchDimensionSwitched() {
    return !( ( getPatchOrientation() == PATCH_ORIENTATION_DEFAULT ) ||
              ( getPatchOrientation() == PATCH_ORIENTATION_ROT180 ) ||
              ( getPatchOrientation() == PATCH_ORIENTATION_MIRROR ) ||
              ( getPatchOrientation() == PATCH_ORIENTATION_MROT180 ) );
  }
  void updateHorizon( std::vector<int>& horizon,
                      std::vector<int>& topHorizon,
                      std::vector<int>& bottomHorizon,
                      std::vector<int>& rightHorizon,
                      std::vector<int>& leftHorizon );

  GPAPatchData& getPreGPAPatchData() { return preGPAPatchData_; }
  GPAPatchData  getPreGPAPatchData() const { return preGPAPatchData_; }
  GPAPatchData& getCurGPAPatchData() { return curGPAPatchData_; }
  GPAPatchData  getCurGPAPatchData() const { return curGPAPatchData_; }

  int patchBlock2CanvasBlockForGPA( const size_t uBlk,
                                    const size_t vBlk,
                                    size_t       canvasStrideBlk,
                                    size_t       canvasHeightBlk ) const;

  bool checkFitPatchCanvasForGPA( std::vector<bool> canvas,
                                  size_t            canvasStrideBlk,
                                  size_t            canvasHeightBlk,
                                  bool              bPrecedence,
                                  int               safeguard = 0 );

  void     allocOneLayerData();
  uint8_t& getPointLocalReconstructionLevel() { return pointLocalReconstructionLevel_; }
  uint8_t  getPointLocalReconstructionLevel() const { return pointLocalReconstructionLevel_; }
  uint8_t  getPointLocalReconstructionMode( const size_t u = 0, const size_t v = 0 ) const {
    if ( pointLocalReconstructionLevel_ == 1 ) {
      return pointLocalReconstructionModeByPatch_;
    } else {
      return pointLocalReconstructionModeByBlock_[v * sizeU0_ + u];
    }
  }

  inline void   setIndexCopy( size_t index ) { indexCopy_ = index; }
  inline size_t getIndexCopy() { return indexCopy_; }
  void          setDepthFromGeometryVideo( const std::vector<uint16_t>& geometryVideo,
                                           const int32_t                u2,
                                           const int32_t                v2,
                                           int32_t                      width,
                                           int32_t                      height,
                                           int32_t                      occupancyPrecision,
                                           int16_t*                     depth );

  void setLocalData( const std::vector<uint8_t>&  occupancyMapVideo,
                     const std::vector<uint16_t>& geometryVideo,
                     std::vector<size_t>&         blockToPatch,
                     const int32_t                width,
                     const int32_t                height,
                     const int32_t                occupancyPrecision,
                     const int32_t                threhold );
  void setPointLocalReconstructionMode( const size_t u, const size_t v, const uint8_t value ) {
    if ( pointLocalReconstructionLevel_ == 1 ) {
      pointLocalReconstructionModeByPatch_ = value;
    } else {
      pointLocalReconstructionModeByBlock_[v * sizeU0_ + u] = value;
    }
  }
  void setPointLocalReconstructionMode( const uint8_t value ) {
    if ( pointLocalReconstructionLevel_ == 1 ) {
      pointLocalReconstructionModeByPatch_ = value;
    } else {
      pointLocalReconstructionModeByBlock_[0] = value;
    }
  }

  void generateBorderPoints3D();

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
  const bool isBorder( size_t u, size_t v );
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
                  std::vector<PCCPatch>& patches );

 private:
  size_t                  index_;          // patch index
  size_t                  originalIndex_;  // patch original index
  size_t                  frameIndex_;     // Frame index
  size_t                  tileIndex_;      // Tile index
  size_t                  indexInFrame_;   // index in frame
  size_t                  u1_;             // tangential shift
  size_t                  v1_;             // bitangential shift
  size_t                  d1_;             // depth shift
  size_t                  sizeD_;          // size for depth
  size_t                  sizeDPixel_;     // Size D pixel
  size_t                  sizeU_;          // size for depth
  size_t                  sizeV_;          // size for depth
  size_t                  u0_;             // location in packed image (n*occupancyResolution_)
  size_t                  v0_;             // location in packed image (n*occupancyResolution_)
  size_t                  sizeU0_;         // size of occupancy map (n*occupancyResolution_)
  size_t                  sizeV0_;         // size of occupancy map (n*occupancyResolution_)
  size_t                  size2DXInPixel_;
  size_t                  size2DYInPixel_;
  size_t                  occupancyResolution_;  // occupancy map resolution
  size_t                  projectionMode_;       // 0: related to the min depth value; 1: related to the max value
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
  std::vector<int16_t>    depthEOM_;          // Enhanced delta depht
  std::vector<int64_t>    depth0PCidx_;       // for Surface separation
  size_t                  patchOrientation_;  // patch orientation in canvas atlas
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
  std::vector<PCCPoint3D> borderPoints_;        // 3D points created from borders of the patch
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
                             int8_t log2Threshold );

 private:
  std::vector<PCCPatch>*       patches_;
  std::vector<size_t>*         blockToPatch_;
  std::vector<uint32_t>*       occupancyMapEncoder_;
  const std::vector<uint8_t>*  occupancyMapVideo_;
  const std::vector<uint16_t>* geometryVideo_;
};

struct PCCEomPatch {
  size_t              u0_;
  size_t              v0_;
  size_t              sizeU_;
  size_t              sizeV_;
  bool                isPatchInAuxVideo_;
  size_t              tileIndex_;
  size_t              frameIndex_;
  size_t              eomCount_;
  size_t              occupancyResolution_;
  std::vector<size_t> memberPatches_;
  std::vector<size_t> eomCountPerPatch_;
};

struct PCCRawPointsPatch {
  size_t                u1_;  // tangential shift
  size_t                v1_;  // bitangential shift
  size_t                d1_;  // depth shift
  size_t                sizeU_;
  size_t                sizeV_;
  size_t                u0_;
  size_t                v0_;
  bool                  isPatchInAuxVideo_;
  size_t                tileIndex_;
  size_t                frameIndex_;
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
  size_t                numberOfRawPoints_;
  size_t                numberOfRawPointsColors_;
  size_t                preV0_;   // GPA.
  size_t                tempV0_;  // GPA.

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

  void         setNumberOfRawPoints( size_t numberOfRawPoints ) { numberOfRawPoints_ = numberOfRawPoints; }
  const size_t getNumberOfRawPoints() { return numberOfRawPoints_; }
  void         resizeColor( const size_t size ) {
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
    sizeU_               = 0;
    sizeV_               = 0;
    u0_                  = 0;
    v0_                  = 0;
    sizeV0_              = 0;
    sizeU0_              = 0;
    occupancyResolution_ = 0;
    numberOfRawPoints_   = 0;
    preV0_               = 0;
    tempV0_              = 0;
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
