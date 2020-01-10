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
#ifndef PCC_BITSTREAM_VPCCPARAMETERSET_H
#define PCC_BITSTREAM_VPCCPARAMETERSET_H

#include "PCCBitstreamCommon.h"
#include "PCCProfileTierLevel.h"
#include "PCCGeometryInformation.h"
#include "PCCOccupancyInformation.h"
#include "PCCAttributeInformation.h"

namespace pcc {
    
// 7.3.4.1  General V-PCC Sequence parameter set syntax
class VpccParameterSet {
 public:
  VpccParameterSet() :
      vpccParameterSetId_( 0 ),
      atlasCountMinus1_( 0 ),
      extensionPresentFlag_( false ),
      extensionLength_( 0 ) {}
  ~VpccParameterSet() {
    for ( auto& data : mapAbsoluteCodingEnableFlag_ ) { data.clear(); }
    mapAbsoluteCodingEnableFlag_.clear();
    for ( auto& data : mapPredictorIndexDiff_ ) { data.clear(); }
    mapPredictorIndexDiff_.clear();
    frameWidth_.clear();
    frameHeight_.clear();
    mapCountMinus1_.clear();
    multipleMapStreamsPresentFlag_.clear();
    rawPatchEnabledFlag_.clear();
    rawSeparateVideoPresentFlag_.clear();
    extensionDataByte_.clear();
  }

  VpccParameterSet& operator=( const VpccParameterSet& ) = default;
  void              init( uint32_t vpccParameterSetId,
                          uint16_t frameWidth,
                          uint16_t frameHeight,
                          uint16_t avgFrameRate,
                          uint32_t mapCountMinus1,
                          uint16_t attributeCount,
                          bool     avgFrameRatePresentFlag,
                          bool     enhancedOccupancyMapForDepthFlag,
                          bool     multipleMapStreamsPresentFlag,
                          bool     pcmPatchEnabledFlag,
                          bool     pcmSeparateVideoPresentFlag,
                          bool     patchSequenceOrientationEnabledFlag,
                          bool     patchInterPredictionEnabledFlag,
                          bool     pixelDeinterleavingFlag,
                          bool     pointLocalReconstructionEnabledFlag,
                          bool     removeDuplicatePointEnabledFlag,
                          bool     projection45degreeEnabledFlag,
                          bool     patchPrecedenceOrderFlag ) {
    vpccParameterSetId_ = vpccParameterSetId;
    atlasCountMinus1_   = 0;
    allocateAltas();
    frameWidth_[0]                    = frameWidth;
    frameHeight_[0]                   = frameHeight;
    mapCountMinus1_[0]                = mapCountMinus1;
    multipleMapStreamsPresentFlag_[0] = multipleMapStreamsPresentFlag;
    rawPatchEnabledFlag_[0]           = pcmPatchEnabledFlag;
    rawSeparateVideoPresentFlag_[0]   = pcmSeparateVideoPresentFlag;
  }

  void allocateAltas() {
    frameWidth_.resize( atlasCountMinus1_ + 1, 1 );
    frameHeight_.resize( atlasCountMinus1_ + 1 );
    mapCountMinus1_.resize( atlasCountMinus1_ + 1 );
    multipleMapStreamsPresentFlag_.resize( atlasCountMinus1_ + 1 );
    mapAbsoluteCodingEnableFlag_.resize( atlasCountMinus1_ + 1 );
    mapPredictorIndexDiff_.resize( atlasCountMinus1_ + 1 );
    rawPatchEnabledFlag_.resize( atlasCountMinus1_ + 1 );
    rawSeparateVideoPresentFlag_.resize( atlasCountMinus1_ + 1 );
    geometryInformation_.resize( atlasCountMinus1_ + 1 );
    occupancyInformation_.resize( atlasCountMinus1_ + 1 );
    attributeInformation_.resize( atlasCountMinus1_ + 1 );
  }
  void allocateMap( size_t atlasIndex ) {
    mapAbsoluteCodingEnableFlag_[atlasIndex].resize( mapCountMinus1_[atlasIndex] + 1, 1 );
    mapPredictorIndexDiff_[atlasIndex].resize( mapCountMinus1_[atlasIndex] + 1 );
  }
  void     allocateExtensionDataByte() { extensionDataByte_.resize( extensionLength_ ); }
  uint32_t getVpccParameterSetId() { return vpccParameterSetId_; }
  uint32_t getAtlasCountMinus1() { return atlasCountMinus1_; }
  uint16_t getFrameWidth( size_t atlasIndex ) { return frameWidth_[atlasIndex]; }
  uint16_t getFrameHeight( size_t atlasIndex ) { return frameHeight_[atlasIndex]; }
  uint32_t getMapCountMinus1( size_t atlasIndex ) { return mapCountMinus1_[atlasIndex]; }
  bool     getMultipleMapStreamsPresentFlag( size_t atlasIndex ) { return multipleMapStreamsPresentFlag_[atlasIndex]; }
  bool     getRawPatchEnabledFlag( size_t atlasIndex ) { return rawPatchEnabledFlag_[atlasIndex]; }
  bool     getRawSeparateVideoPresentFlag( size_t atlasIndex ) { return rawSeparateVideoPresentFlag_[atlasIndex]; }
  size_t   getMapPredictorIndexDiff( size_t atlasIndex, size_t index ) {
    return mapPredictorIndexDiff_[atlasIndex][index];
  }
  bool getMapAbsoluteCodingEnableFlag( size_t atlasIndex, size_t index ) {
    return mapAbsoluteCodingEnableFlag_[atlasIndex][index];
  }
  ProfileTierLevel&     getProfileTierLevel() { return profileTierLevel_; }
  GeometryInformation&  getGeometryInformation( size_t atlasIndex ) { return geometryInformation_[atlasIndex]; }
  OccupancyInformation& getOccupancyInformation( size_t atlasIndex ) { return occupancyInformation_[atlasIndex]; }
  AttributeInformation& getAttributeInformation( size_t atlasIndex ) { return attributeInformation_[atlasIndex]; }
  bool                  getExtensionPresentFlag() { return extensionPresentFlag_; }
  size_t                getExtensionLength() { return extensionLength_; }
  std::vector<uint8_t>& getExtensionDataByte() { return extensionDataByte_; }
  uint8_t               getExtensionDataByte( size_t index ) { return extensionDataByte_[index]; }

  void setVpccParameterSetId( uint32_t value ) { vpccParameterSetId_ = value; }
  void setAtlasCountMinus1( uint32_t value ) { atlasCountMinus1_ = value; }
  void setFrameWidth( size_t atlasIndex, uint16_t value ) { frameWidth_[atlasIndex] = value; }
  void setFrameHeight( size_t atlasIndex, uint16_t value ) { frameHeight_[atlasIndex] = value; }
  void setMapCountMinus1( size_t atlasIndex, uint32_t value ) { mapCountMinus1_[atlasIndex] = value; }
  void setMultipleMapStreamsPresentFlag( size_t atlasIndex, bool value ) {
    multipleMapStreamsPresentFlag_[atlasIndex] = value;
  }
  void setRawPatchEnabledFlag( size_t atlasIndex, bool value ) { rawPatchEnabledFlag_[atlasIndex] = value; }
  void setRawSeparateVideoPresentFlag( size_t atlasIndex, bool value ) {
    rawSeparateVideoPresentFlag_[atlasIndex] = value;
  }
  void setGeometryInformation( size_t atlasIndex, GeometryInformation value ) {
    geometryInformation_[atlasIndex] = value;
  }
  void setOccupancyInformation( size_t atlasIndex, OccupancyInformation value ) {
    occupancyInformation_[atlasIndex] = value;
  }
  void setMapAbsoluteCodingEnableFlag( size_t atlasIndex, size_t index, bool value ) {
    if ( mapAbsoluteCodingEnableFlag_[atlasIndex].size() < index + 1 ) {
      mapAbsoluteCodingEnableFlag_[atlasIndex].resize( index + 1, 1 );
    }
    mapAbsoluteCodingEnableFlag_[atlasIndex][index] = value;
  }
  void setMapPredictorIndexDiff( size_t atlasIndex, size_t index, bool value ) {
    if ( mapPredictorIndexDiff_[atlasIndex].size() < index + 1 ) {
      mapPredictorIndexDiff_[atlasIndex].resize( index + 1 );
    }
    mapPredictorIndexDiff_[atlasIndex][index] = value;
  }
  void addMapAbsoluteCodingEnableFlag( size_t atlasIndex, bool value ) {
    mapAbsoluteCodingEnableFlag_[atlasIndex].push_back( value );
  }
  void addMapPredictorIndexDiff( size_t atlasIndex, bool value ) {
    mapPredictorIndexDiff_[atlasIndex].push_back( value );
  }
  void setExtensionDataByte( size_t index, uint8_t value ) { extensionDataByte_[index] = value; }
  void setExtensionPresentFlag( bool value ) { extensionPresentFlag_ = value; }
  void setExtensionLength( size_t value ) { extensionLength_ = value; }

 private:
  ProfileTierLevel                  profileTierLevel_;
  uint32_t                          vpccParameterSetId_;
  uint32_t                          atlasCountMinus1_;
  std::vector<uint16_t>             frameWidth_;
  std::vector<uint16_t>             frameHeight_;
  std::vector<uint8_t>              mapCountMinus1_;
  std::vector<bool>                 multipleMapStreamsPresentFlag_;
  std::vector<std::vector<bool>>    mapAbsoluteCodingEnableFlag_;
  std::vector<std::vector<size_t>>  mapPredictorIndexDiff_;
  std::vector<bool>                 rawPatchEnabledFlag_;
  std::vector<bool>                 rawSeparateVideoPresentFlag_;
  std::vector<GeometryInformation>  geometryInformation_;
  std::vector<OccupancyInformation> occupancyInformation_;
  std::vector<AttributeInformation> attributeInformation_;
  bool                              extensionPresentFlag_;
  size_t                            extensionLength_;
  std::vector<uint8_t>              extensionDataByte_;

  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
 public:
  bool    getLosslessGeo444() { return losslessGeo444_; }
  bool    getLosslessGeo() { return losslessGeo_; }
  uint8_t getMinLevel() { return minLevel_; }
  void    setLosslessGeo444( bool losslessGeo444 ) { losslessGeo444_ = losslessGeo444; }
  void    setLosslessGeo( bool losslessGeo ) { losslessGeo_ = losslessGeo; }
  void    setMinLevel( uint8_t minLevel ) { minLevel_ = minLevel; }

 private:
  bool    losslessGeo444_;
  bool    losslessGeo_;
  uint8_t minLevel_;
  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
};

};  // namespace pcc

#endif //~PCC_BITSTREAM_VPCCPARAMETERSET_H