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
#ifndef PCCFrameContext_h
#define PCCFrameContext_h

#include "PCCCommon.h"

#include "PCCPointSet.h"
#include "PCCPatch.h"
#include "PCCContext.h"

namespace pcc {
struct PCCGPAFrameSize {
  size_t widthGPA_;
  size_t heightGPA_;
};

struct PCCFrameOCMInfo {
  size_t            occupancySizeU_;
  size_t            occupancySizeV_;
  size_t            maxOccupancyRow_;
  std::vector<bool> occupancyMap_;
};

class PCCFrameContext {
 public:
  PCCFrameContext();
  ~PCCFrameContext();
  std::vector<PCCVector3<size_t>>& getPointToPixel() { return pointToPixel_; }
  size_t                           appendPointToPixel( std::vector<PCCVector3<size_t>>& value ) {
    for ( size_t i = 0; i < value.size(); i++ ) pointToPixel_.push_back( value[i] );
    return pointToPixel_.size();
  }
  std::vector<size_t>&            getBlockToPatch() { return blockToPatch_; }
  std::vector<uint32_t>&          getOccupancyMap() { return occupancyMap_; }
  std::vector<uint32_t>&          getFullOccupancyMap() { return fullOccupancyMap_; }
  std::vector<PCCPatch>&          getPatches() { return patches_; }
  PCCPatch&                       getPatch( size_t index ) { return patches_[index]; }
  const PCCPatch&                 getPatch( size_t index ) const { return patches_[index]; }
  std::vector<PCCRawPointsPatch>& getRawPointsPatches() { return rawPointsPatches_; }
  PCCRawPointsPatch&              getRawPointsPatch( size_t index ) { return rawPointsPatches_[index]; }
  std::vector<size_t>&            getNumberOfRawPoints() { return numberOfRawPoints_; };
  std::vector<PCCColor3B>&        getRawPointsTextures() { return rawTextures_; };
  std::vector<PCCColor3B>&        getEOMTextures() { return eomTextures_; };
  size_t&                         getWidth() { return width_; }
  size_t&                         getHeight() { return height_; }
  const size_t                    getIndex() { return index_; }
  const size_t                    getTileIndex() { return tileIndex_; }
  const size_t                    getTotalNumberOfEOMPoints() { return totalNumberOfEOMPoints_; }
  const size_t                    getTotalNumberOfRegularPoints() { return totalNumberOfRegularPoints_; }
  const size_t                    getNumberOfRawPoints( int index ) { return numberOfRawPoints_[index]; }
  const size_t                    getNumMatchedPatches() { return numMatchedPatches_; }
  const size_t                    getNumberOfRawPointsPatches() { return rawPointsPatches_.size(); }
  const size_t                    getTotalNumberOfRawPoints() { return totalNumberOfRawPoints_; }
  bool                            getUseRawPointsSeparateVideo() { return useRawPointsSeparateVideo_; }
  const bool                      getRawPatchEnabledFlag() { return rawPatchEnabledFlag_; }
  const size_t                    getMaxDepth() { return maxDepth_; }
  void                            setMaxDepth( size_t value ) { maxDepth_ = value; }
  size_t                          getGeometry2dBitdepth() { return geometry2dBitdepth_; }
  std::vector<PCCPointSet3>&      getSrcPointCloudByPatch() { return srcPointCloudByPatch_; }
  PCCPointSet3&              getSrcPointCloudByPatch( size_t patchIndex ) { return srcPointCloudByPatch_[patchIndex]; }
  std::vector<PCCPointSet3>& getSrcPointCloudByBlock() { return srcPointCloudByBlock_; }
  PCCVector3D&               getWeightNormal() { return weightNormal_; }
  uint8_t&                   getPointLocalReconstructionNumber() { return pointLocalReconstructionNumber_; }
  PCCGPAFrameSize&           getPrePCCGPAFrameSize() { return prePCCGPAFrameSize_; }
  PCCGPAFrameSize&           getCurPCCGPAFrameSize() { return curPCCGPAFrameSize_; }
  PCCFrameOCMInfo&           getPCCOCPGPAInfo() { return ocpGPAInfo_; }
  size_t&                    getGlobalPatchCount() { return globalPatchCount_; }
  size_t                     getGeometry3dCoordinatesBitdepth() { return geometry3dCoordinatesBitdepth_; }
  void                       setIndex( size_t value ) { index_ = value; }
  void                       setTileIndex( size_t value ) { tileIndex_ = value; }
  void                       setWidth( size_t value ) { width_ = value; }
  void                       setHeight( size_t value ) { height_ = value; }
  void                       setUseRawPointsSeparateVideo( bool value ) { useRawPointsSeparateVideo_ = value; }
  void                       setTotalNumberOfEOMPoints( size_t numPoints ) { totalNumberOfEOMPoints_ = numPoints; }
  void setTotalNumberOfRegularPoints( size_t numPoints ) { totalNumberOfRegularPoints_ = numPoints; }
  void setNumberOfRawPoints( int index, size_t value ) { numberOfRawPoints_[index] = value; }
  void setNumberOfRawPointsPatches( size_t numPoints ) { numberOfRawPointsPatches_ = numPoints; }
  void setTotalNumberOfRawPoints( size_t numPoints ) { totalNumberOfRawPoints_ = numPoints; }
  void setGeometry3dCoordinatesBitdepth( size_t value ) { geometry3dCoordinatesBitdepth_ = value; }
  void setRawPatchEnabledFlag( bool value ) { rawPatchEnabledFlag_ = value; }
  void setGeometry2dBitdepth( size_t value ) { geometry2dBitdepth_ = value; }
  void setNumMatchedPatches( size_t value ) { numMatchedPatches_ = value; }

  void                      setEomPatches( PCCEomPatch value, size_t idx ) { eomPatches_[idx] = value; }
  std::vector<PCCEomPatch>& getEomPatches() { return eomPatches_; }
  PCCEomPatch&              getEomPatches( size_t idx ) { return eomPatches_[idx]; }

  uint8_t getLog2PatchQuantizerSizeX() { return log2PatchQuantizerSizeX_; }
  uint8_t getLog2PatchQuantizerSizeY() { return log2PatchQuantizerSizeY_; }
  void    setLog2PatchQuantizerSizeX( uint8_t value ) { log2PatchQuantizerSizeX_ = value; }
  void    setLog2PatchQuantizerSizeY( uint8_t value ) { log2PatchQuantizerSizeY_ = value; }
#if REFERENCELIST_BUGFIX
  void resizeRefAfoc( size_t value ) { refAFOCList_.resize( value ); }
  void setRefAfoc( size_t refIndex, int32_t value ) { refAFOCList_[refIndex] = value; }
  void setBestRefListIndexInAsps( size_t value ) { bestRefListIndexInAsps_ = value; }
#else
  void setNumOfRefAFOC( size_t value ) { refAFOCList_[0].resize( value ); }
  void setRefAFOC( size_t refIndex, size_t value ) { refAFOCList_[0][refIndex] = value; }
  void setNumOfRefAtlasFrame( size_t idx, size_t value ) { refAFOCList_[idx].resize( value ); }
  void setRefAFOC( size_t idx, size_t refIndex, size_t value ) { refAFOCList_[idx][refIndex] = value; }
#endif
  void setAtlasFrmOrderCntVal( size_t value ) { atlasFrmOrderCntVal_ = value; }
  void setAtlasFrmOrderCntMsb( size_t value ) { atlasFrmOrderCntMsb_ = value; }
  void setAtlasFrmOrderCntLsb( size_t value ) { atlasFrmOrderCntLsb_ = value; }
#if REFERENCELIST_BUGFIX
  size_t getAtlasFrmOrderCntVal() { return atlasFrmOrderCntVal_; }
  size_t getAtlasFrmOrderCntMsb() { return atlasFrmOrderCntMsb_; }
  size_t getAtlasFrmOrderCntLsb() { return atlasFrmOrderCntLsb_; }

  void    constructAtghRefListStruct( PCCContext& context, AtlasTileHeader& ath );
  void    setNumRefIdxActive( size_t value ) { numRefIdxActive_ = value; }
  void    setRefAfocList( PCCContext& context, AtlasTileHeader& ath, size_t afpsIndex );
  void    setRefAfocList( PCCContext& context, size_t refListIdx );
  int32_t getRefAfoc( size_t refIndex ) { return refAFOCList_[refIndex]; }
  size_t  getRefAfocListSize() { return refAFOCList_.size(); }
  size_t  getBestRefListIndexInAsps() { return bestRefListIndexInAsps_; }
  size_t  getNumRefIdxActive() { return numRefIdxActive_; }
#else
  void setNumOfRefAtlasFrameList( size_t value ) { numOfAvailableRefAtlasFrameList_ = value; }
  void setActiveRefAtlasFrameIndex( size_t value ) { activeRefAtlasFrameIndex_ = value; }

  void setRefAtlasListIndexInSPS( size_t listIdx ) { refAtlasListIndexInSPS_ = listIdx; }
  void setRefAFOCList( std::vector<std::vector<size_t>>& list );
  void setRefAFOCList( PCCContext& context );
  void constructAtghRefListStruct( PCCContext& context, AtlasTileHeader& ath );
  void setNumRefIdxActive( size_t value ) { numRefIdxActive_ = value; }
  void setNumRefIdxActive( PCCContext& context, AtlasTileHeader& ath );
  void addRefAFOC( size_t value ) { refAFOCList_[0].push_back( value ); }
  void addRefAFOC( size_t idx, size_t value ) { refAFOCList_[idx].push_back( value ); }

  size_t getRefAFOC( size_t refIndex ) { return refAFOCList_[0][refIndex]; }
  size_t getRefAFOC( size_t listIdx, size_t refIndex ) { return refAFOCList_[listIdx][refIndex]; }
  size_t getRefAFOCListSize( size_t idx ) { return refAFOCList_[idx].size(); }
  size_t getAtlasFrmOrderCntVal() { return atlasFrmOrderCntVal_; }
  size_t getAtlasFrmOrderCntMsb() { return atlasFrmOrderCntMsb_; }
  size_t getAtlasFrmOrderCntLsb() { return atlasFrmOrderCntLsb_; }

  void    constructAtghRefListStruct( PCCContext& context, AtlasTileHeader& ath );
  void    setNumRefIdxActive( size_t value ) { numRefIdxActive_ = value; }
  void    setRefAfocList( PCCContext& context, AtlasTileHeader& ath, size_t afpsIndex );
  void    setRefAfocList( PCCContext& context, size_t refListIdx );
  int32_t getRefAfoc( size_t refIndex ) { return refAFOCList_[refIndex]; }
  size_t  getRefAfocListSize() { return refAFOCList_.size(); }
  size_t  getBestRefListIndexInAsps() { return bestRefListIndexInAsps_; }

  size_t                           getNumRefIdxActive() { return numRefIdxActive_; }
  size_t                           getActiveRefAtlasFrameIndex() { return activeRefAtlasFrameIndex_; }
  size_t                           getRefAtlasListIndexInSPS() { return refAtlasListIndexInSPS_; }
#endif
  void   setLeftTopXInFrame( size_t value ) { leftTopXInFrame_ = value; }
  void   setLeftTopYInFrame( size_t value ) { leftTopYInFrame_ = value; }
  size_t getLeftTopXInFrame() { return leftTopXInFrame_; }
  size_t getLeftTopYInFrame() { return leftTopYInFrame_; }

  void allocOneLayerData();
  void printBlockToPatch( const size_t occupancyResolution );
  void printPatch();
  void printPatchDecoder();

 private:
  size_t  index_;
  size_t  tileIndex_;
  size_t  numMatchedPatches_;
  size_t  width_;
  size_t  height_;
  size_t  leftTopXInFrame_;
  size_t  leftTopYInFrame_;
  size_t  numberOfRawPointsPatches_;
  size_t  totalNumberOfRawPoints_;
  size_t  totalNumberOfEOMPoints_;
  size_t  totalNumberOfRegularPoints_;
  size_t  globalPatchCount_;
  size_t  geometry3dCoordinatesBitdepth_;
  uint8_t pointLocalReconstructionNumber_;
  bool    useRawPointsSeparateVideo_;
  bool    rawPatchEnabledFlag_;
  size_t  geometry2dBitdepth_;
  size_t  maxDepth_;
#if REFERENCELIST_BUGFIX
  size_t               atlasFrmOrderCntVal_;
  size_t               atlasFrmOrderCntMsb_;
  size_t               atlasFrmOrderCntLsb_;
  std::vector<int32_t> refAFOCList_;  // jkei: one list for a tile
  size_t               numRefIdxActive_;
  size_t               bestRefListIndexInAsps_;  // listIndex in context
#else
  size_t                           refAtlasListIndexInSPS_;
  size_t                           numRefIdxActive_;
  size_t                           atlasFrmOrderCntVal_;
  size_t                           atlasFrmOrderCntMsb_;
  size_t                           atlasFrmOrderCntLsb_;
  size_t                           numOfAvailableRefAtlasFrameList_;
  size_t                           activeRefAtlasFrameIndex_;
  std::vector<std::vector<size_t>> refAFOCList_;
#endif
  size_t                                       log2PatchQuantizerSizeX_;
  size_t                                       log2PatchQuantizerSizeY_;
  std::vector<PCCVector3<size_t>>              pointToPixel_;
  std::vector<size_t>                          blockToPatch_;
  std::vector<uint32_t>                        occupancyMap_;
  std::vector<uint32_t>                        fullOccupancyMap_;
  std::vector<PCCPatch>                        patches_;
  std::vector<PCCRawPointsPatch>               rawPointsPatches_;
  std::vector<size_t>                          numberOfRawPoints_;
  std::vector<PCCColor3B>                      rawTextures_;
  std::vector<PCCColor3B>                      eomTextures_;
  std::vector<PCCPointSet3>                    srcPointCloudByPatch_;
  std::vector<PCCPointSet3>                    srcPointCloudByBlock_;
  std::vector<PCCPointSet3>                    recPointCloudByBlock_;
  std::vector<std::vector<PCCVector3<size_t>>> pointToPixelByBlock_;
  PCCGPAFrameSize                              prePCCGPAFrameSize_;
  PCCGPAFrameSize                              curPCCGPAFrameSize_;
  PCCFrameOCMInfo                              ocpGPAInfo_;
  PCCVector3D                                  weightNormal_;
  std::vector<PCCEomPatch>                     eomPatches_;
};

class PCCAtlasFrameContext {
 public:
  PCCAtlasFrameContext() {
    atlasFrameIndex_         = 0;
    atlasWidth_              = 0;
    atlasHeight_             = 0;
    numTilesInAtlasFrame_    = 0;
    uniformPartitionSpacing_ = 0;
    singlePartitionPerTile_  = 1;
    numPartitionCols_        = 0;
    numPartitionRows_        = 0;
    signalledTileId_         = 0;
  };
  ~PCCAtlasFrameContext() {
#if TILEINFO_DERIVATION
    partitionWidth_.clear();
    partitionHeight_.clear();
    partitionPosX_.clear();
    partitionPosY_.clear();
#else
    colWidth_.clear();
    rowHeight_.clear();
#endif
    tileGroupId_.clear();
    tileContexts_.clear();
  };

  PCCFrameContext& operator[]( int index ) { return tileContexts_[index]; }
  void             initNumTiles( size_t value ) { return tileContexts_.resize( value ); }
  void             addTile( PCCFrameContext& tile ) { tileContexts_.push_back( tile ); }

  size_t getFrameWidth() { return atlasWidth_; }
  size_t getFrameHeight() { return atlasHeight_; }
  size_t getNumTilesInAtlasFrame() { return numTilesInAtlasFrame_; }
  bool   getUniformPartitionSpacing() { return uniformPartitionSpacing_; }
  size_t getNumPartitionCols() { return numPartitionCols_; }
  size_t getNumPartitionRows() { return numPartitionRows_; }
  bool   getSignalledTileId() { return signalledTileId_; }
#if TILEINFO_DERIVATION
  std::vector<size_t>& getPartitionWidth() { return partitionWidth_; }
  std::vector<size_t>& getPartitionHeight() { return partitionHeight_; }
  std::vector<size_t>& getPartitionPosX() { return partitionPosX_; }
  std::vector<size_t>& getPartitionPosY() { return partitionPosY_; }
#else
  std::vector<size_t>& getColWidth() { return colWidth_; }
  std::vector<size_t>& getRowHeight() { return rowHeight_; }
#endif
  std::vector<size_t>&          getTileGroupId() { return tileGroupId_; }
  std::vector<PCCFrameContext>& getTiles() { return tileContexts_; }
  PCCFrameContext&              getTile( size_t index ) {
    if ( numTilesInAtlasFrame_ == 1 ) {
      assert( index == 0 );
      return atlasFrameContext_;
    } else
      return tileContexts_[index];
  }

  size_t getAtlasFrameIndex() { return atlasFrameIndex_; }
  void   setAtlasFrameIndex( size_t value ) { atlasFrameIndex_ = value; }
  void   setAtlasWidth( size_t value ) {
    atlasFrameContext_.setWidth( value );
    atlasWidth_ = value;
  }
  void setAtlasHeight( size_t value ) {
    atlasFrameContext_.setHeight( value );
    atlasHeight_ = value;
  }
  void setNumTilesInAtlasFrame( size_t value ) { numTilesInAtlasFrame_ = value; }
  void setUniformPartitionSpacing( bool value ) { uniformPartitionSpacing_ = value; }
  void setNumPartitionCols( size_t value ) { numPartitionCols_ = value; }
  void setNumPartitionRows( size_t value ) { numPartitionRows_ = value; }
  void setSignalledTileId( bool value ) { signalledTileId_ = value; }
#if TILEINFO_DERIVATION
  void setPartitionWidth( std::vector<size_t>& value ) { partitionWidth_ = value; }
  void setPartitionHeight( std::vector<size_t>& value ) { partitionHeight_ = value; }
  void setPartitionPosX( std::vector<size_t>& value ) { partitionPosX_ = value; }
  void setPartitionPosY( std::vector<size_t>& value ) { partitionPosY_ = value; }
#else
  void                 setColWidth( std::vector<size_t>& value ) { colWidth_ = value; }
  void                 setRowHeight( std::vector<size_t>& value ) { rowHeight_ = value; }
#endif
  void setTileGroupId( std::vector<size_t>& value ) { tileGroupId_ = value; }
  void setTiles( std::vector<PCCFrameContext>& value ) { tileContexts_ = value; }

  void             setAtlasFrameContext( PCCFrameContext& value ) { atlasFrameContext_ = value; }
  PCCFrameContext& getAtlasFrameContext() { return atlasFrameContext_; }

  void setSinglePartitionPerTile( bool value ) { singlePartitionPerTile_ = value; }
  bool getSinglePartitionPerTile() { return singlePartitionPerTile_; }

  void initPartitionInfoPerFrame( size_t index,
                                  size_t width,
                                  size_t height,
                                  size_t numTiles,
                                  bool   uniformPartitionSpacing,
                                  size_t partitionWidthIn64,
                                  size_t partitionHeightIn64 ) {
    atlasFrameContext_.setWidth( width );
    atlasFrameContext_.setHeight( height );
    atlasWidth_           = width;
    atlasHeight_          = height;
    numTilesInAtlasFrame_ = numTiles;
#if TILEINFO_DERIVATION
    size_t partitionWidthIn  = partitionWidthIn64 * 64;
    size_t partitionHeightIn = partitionHeightIn64 * 64;
#endif
    if ( numTiles == 1 ) {
      numPartitionCols_ = 1;
      numPartitionRows_ = 1;
#if TILEINFO_DERIVATION
      partitionWidth_.resize( numPartitionCols_ );
      partitionHeight_.resize( numPartitionRows_ );
      partitionPosX_.resize( numPartitionCols_ );
      partitionPosY_.resize( numPartitionRows_ );
      partitionPosX_[0]   = 0;
      partitionWidth_[0]  = atlasWidth_;
      partitionPosY_[0]   = 0;
      partitionHeight_[0] = atlasHeight_;
#endif
    } else {
      uniformPartitionSpacing_ = uniformPartitionSpacing;
      // jkei : how to get multiple partition size?
#if TILEINFO_DERIVATION
      numPartitionCols_ =
          (size_t)std::ceil( static_cast<double>( atlasWidth_ ) / static_cast<double>( partitionWidthIn ) );
      numPartitionRows_ =
          (size_t)std::ceil( static_cast<double>( atlasHeight_ ) / static_cast<double>( partitionHeightIn ) );
      partitionWidth_.resize( numPartitionCols_ );
      partitionHeight_.resize( numPartitionRows_ );
      partitionPosX_.resize( numPartitionCols_ );
      partitionPosY_.resize( numPartitionRows_ );

      if ( uniformPartitionSpacing ) {
        // jkei: issue: the last one can be less than 1 -> the spec update required
        partitionPosX_[0]  = 0;
        partitionWidth_[0] = partitionWidthIn;
        for ( size_t col = 1; col < numPartitionCols_ - 1; col++ ) {
          partitionPosX_[col]  = partitionPosX_[col - 1] + partitionWidth_[col - 1];
          partitionWidth_[col] = partitionWidthIn;
        }
        if ( numPartitionCols_ > 1 ) {
          partitionPosX_[numPartitionCols_ - 1] =
              partitionPosX_[numPartitionCols_ - 2] + partitionWidth_[numPartitionCols_ - 2];
          partitionWidth_[numPartitionCols_ - 1] = atlasWidth_ - partitionPosX_[numPartitionCols_ - 1];
        }

        partitionPosY_[0]   = 0;
        partitionHeight_[0] = partitionHeightIn;
        for ( size_t row = 1; row < numPartitionRows_ - 1; row++ ) {
          partitionPosY_[row]   = partitionPosY_[row - 1] + partitionHeight_[row - 1];
          partitionHeight_[row] = partitionHeightIn;
        }
        if ( numPartitionRows_ > 1 ) {
          partitionPosY_[numPartitionRows_ - 1] =
              partitionPosY_[numPartitionRows_ - 2] + partitionHeight_[numPartitionRows_ - 2];
          partitionHeight_[numPartitionRows_ - 1] = atlasHeight_ - partitionPosY_[numPartitionRows_ - 1];
        }
      } else {
        // jkei:we need to figure out how to set non-uniform partition spcaing
        printf( "non uniform tile partitioning\n" );
        exit( 128 );
        for ( size_t col = 0; col < numPartitionCols_; col++ ) {
          partitionWidth_[col] = partitionWidthIn;  // jkei: input needs to be array
          partitionPosX_[col]  = partitionPosX_[col - 1] + partitionWidth_[col];
        }
        for ( size_t row = 0; row < numPartitionRows_; row++ ) {
          partitionHeight_[row] = partitionHeightIn;
          partitionPosY_[row]   = partitionPosY_[row - 1] + partitionHeight_[row];
        }
      }
#else
      numPartitionCols_ =
          (size_t)std::ceil( static_cast<double>( atlasWidth_ ) / static_cast<double>( partitionWidthIn64 * 64 ) );
      numPartitionRows_ =
          (size_t)std::ceil( static_cast<double>( atlasHeight_ ) / static_cast<double>( partitionHeightIn64 * 64 ) );
      colWidth_.resize( numPartitionCols_ );
      rowHeight_.resize( numPartitionRows_ );
      if ( uniformPartitionSpacing ) {
        // jkei: issue: the last one can be less than 1 -> the spec update required
        for ( size_t col = 0; col < numPartitionCols_; col++ ) colWidth_[col] = partitionWidthIn64;
        for ( size_t row = 0; row < numPartitionRows_; row++ ) rowHeight_[row] = partitionHeightIn64;
      } else {
        // jkei:we need to figure out how to set non-uniform partition spcaing
        printf( "non uniform tile partitioning\n" );
        exit( 128 );
      }
#endif
    }  // numTiles!=1

    singlePartitionPerTile_ = false;
    signalledTileId_        = false;
  }
  void updatePartitionInfoPerFrame( size_t index,
                                    size_t width,
                                    size_t height,
                                    size_t numTiles,
                                    bool   uniformPartitionSpacing,
                                    size_t partitionWidthIn64,
                                    size_t partitionHeightIn64,
                                    int    numPartitionWidth      = -1,
                                    int    numPartitionHeight     = -1,
                                    bool   singlePartitionPerTile = false,
                                    bool   signalledTileId        = false ) {
    atlasFrameContext_.setWidth( width );
    atlasFrameContext_.setHeight( height );
    atlasWidth_           = width;
    atlasHeight_          = height;
    numTilesInAtlasFrame_ = numTiles;
#if TILEINFO_DERIVATION
    size_t partitionWidthIn  = partitionWidthIn64 * 64;
    size_t partitionHeightIn = partitionHeightIn64 * 64;
#endif
    if ( numTiles == 1 ) {
      numPartitionCols_ = 1;
      numPartitionRows_ = 1;
#if TILEINFO_DERIVATION
      partitionWidth_.resize( numPartitionCols_ );
      partitionHeight_.resize( numPartitionRows_ );
      partitionPosX_.resize( numPartitionCols_ );
      partitionPosY_.resize( numPartitionRows_ );
      partitionPosX_[0]   = 0;
      partitionWidth_[0]  = atlasWidth_;
      partitionPosY_[0]   = 0;
      partitionHeight_[0] = atlasHeight_;
#endif
    } else {
      uniformPartitionSpacing_ = uniformPartitionSpacing;
#if TILEINFO_DERIVATION
      numPartitionCols_ =
          (size_t)std::ceil( static_cast<double>( atlasWidth_ ) / static_cast<double>( partitionWidthIn ) );
      numPartitionRows_ =
          (size_t)std::ceil( static_cast<double>( atlasHeight_ ) / static_cast<double>( partitionHeightIn ) );
      partitionWidth_.resize( numPartitionCols_ );
      partitionHeight_.resize( numPartitionRows_ );
      partitionPosX_.resize( numPartitionCols_ );
      partitionPosY_.resize( numPartitionRows_ );
      if ( uniformPartitionSpacing ) {
        // jkei: issue: the last one can be less than 1 -> the spec update required
        partitionPosX_[0]  = 0;
        partitionWidth_[0] = partitionWidthIn;
        for ( size_t col = 1; col < numPartitionCols_ - 1; col++ ) {
          partitionPosX_[col]  = partitionPosX_[col - 1] + partitionWidth_[col - 1];
          partitionWidth_[col] = partitionWidthIn;
        }
        if ( numPartitionCols_ > 1 ) {
          partitionPosX_[numPartitionCols_ - 1] =
              partitionPosX_[numPartitionCols_ - 2] + partitionWidth_[numPartitionCols_ - 2];
          partitionWidth_[numPartitionCols_ - 1] = atlasWidth_ - partitionPosX_[numPartitionCols_ - 1];
        }

        partitionPosY_[0]   = 0;
        partitionHeight_[0] = partitionHeightIn;
        for ( size_t row = 1; row < numPartitionRows_ - 1; row++ ) {
          partitionPosY_[row]   = partitionPosY_[row - 1] + partitionHeight_[row - 1];
          partitionHeight_[row] = partitionHeightIn;
        }
        if ( numPartitionRows_ > 1 ) {
          partitionPosY_[numPartitionRows_ - 1] =
              partitionPosY_[numPartitionRows_ - 2] + partitionHeight_[numPartitionRows_ - 2];
          partitionHeight_[numPartitionRows_ - 1] = atlasHeight_ - partitionPosY_[numPartitionRows_ - 1];
        }
      } else {
        // jkei:we need to figure out how to set non-uniform partition spcaing
        printf( "non uniform tile partitioning\n" );
        exit( 128 );
        for ( size_t col = 0; col < numPartitionCols_; col++ ) {
          partitionWidth_[col] = partitionWidthIn;  // jkei: input needs to be array
          partitionPosX_[col]  = partitionPosX_[col - 1] + partitionWidth_[col];
        }
        for ( size_t row = 0; row < numPartitionRows_; row++ ) {
          partitionHeight_[row] = partitionHeightIn;
          partitionPosY_[row]   = partitionPosY_[row - 1] + partitionHeight_[row];
        }
      }
#else
      if ( uniformPartitionSpacing ) {
        numPartitionCols_ =
            (size_t)std::ceil( static_cast<double>( atlasWidth_ ) / static_cast<double>( partitionWidthIn64 * 64 ) );
        numPartitionRows_ =
            (size_t)std::ceil( static_cast<double>( atlasHeight_ ) / static_cast<double>( partitionHeightIn64 * 64 ) );
        colWidth_.resize( numPartitionCols_ );
        rowHeight_.resize( numPartitionRows_ );
        // jkei: the last one can be less than 1 -> the spec update required
        // colWidth_[numPartitionCols_-1] =
        for ( size_t col = 0; col < numPartitionCols_; col++ ) { colWidth_[col] = partitionWidthIn64; }
        for ( size_t row = 0; row < numPartitionRows_; row++ ) { rowHeight_[row] = partitionHeightIn64; }
      } else {
        // jkei:we need to figure out how to set non-uniform partition spcaing
        numPartitionCols_ = numPartitionWidth;
        numPartitionRows_ = numPartitionHeight, colWidth_.resize( numPartitionCols_ );
        rowHeight_.resize( numPartitionRows_ );
        //      for(size_t col=0; col<numPartitionCols_; col++)
        //        colWidth_[col] = partitionWidthIn64[col];
        //      for(size_t row=0; row<numPartitionRows_; row++)
        //        rowHeight_[row] = partitionHeightIn64[row];

        printf( "non uniform tile partitioning\n" );
        exit( 128 );
      }
#endif
    }  // numTiles!=1

    singlePartitionPerTile_ = singlePartitionPerTile;
    signalledTileId_        = signalledTileId;
  }

  std::vector<int>& getTileToTileGroupMap() { return tileToTileGroupMap_; }
  std::vector<int>  tileToTileGroupMap_;

 private:
  size_t atlasFrameIndex_;
  size_t atlasWidth_;
  size_t atlasHeight_;
  size_t numTilesInAtlasFrame_;
  bool   uniformPartitionSpacing_;
  bool   singlePartitionPerTile_;
  size_t numPartitionCols_;
  size_t numPartitionRows_;
  bool   signalledTileId_;
#if TILEINFO_DERIVATION
  std::vector<size_t> partitionWidth_;   // in pixel
  std::vector<size_t> partitionHeight_;  // in pixel
  std::vector<size_t> partitionPosX_;    // in pixel
  std::vector<size_t> partitionPosY_;    // in pixel
#else
  std::vector<size_t> colWidth_;
  std::vector<size_t> rowHeight_;
#endif
  std::vector<size_t>          tileGroupId_;
  std::vector<PCCFrameContext> tileContexts_;
  PCCFrameContext              atlasFrameContext_;
};

};  // namespace pcc

#endif /* PCCFrameContext_h */
