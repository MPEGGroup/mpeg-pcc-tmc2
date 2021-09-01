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
#include "PCCVpccExtension.h"

namespace pcc {

// 8.3.4.1  General V3C Sequence parameter set syntax
class V3CParameterSet {
 public:
  V3CParameterSet() :
      v3cParameterSetId_( 0 ),
      atlasCountMinus1_( 0 ),
      extensionPresentFlag_( false ),
      extension8Bits_( 0 ),
      extensionLengthMinus1_( 0 ) {}
  ~V3CParameterSet() {
    for ( auto& data : mapAbsoluteCodingEnableFlag_ ) { data.clear(); }
    mapAbsoluteCodingEnableFlag_.clear();
    for ( auto& data : mapPredictorIndexDiff_ ) { data.clear(); }
    mapPredictorIndexDiff_.clear();
    frameWidth_.clear();
    frameHeight_.clear();
    mapCountMinus1_.clear();
    multipleMapStreamsPresentFlag_.clear();
    auxiliaryVideoPresentFlag_.clear();
    occupancyVideoPresentFlag_.clear();
    geometryVideoPresentFlag_.clear();
    attributeVideoPresentFlag_.clear();
    extensionDataByte_.clear();
  }

  V3CParameterSet& operator=( const V3CParameterSet& ) = default;
  void             init( uint32_t v3cParameterSetId,
                         uint16_t atlasId,
                         uint16_t frameWidth,
                         uint16_t frameHeight,
                         uint16_t avgFrameRate,
                         uint32_t mapCountMinus1,
                         bool     multipleMapStreamsPresentFlag,
                         bool     auxiliaryVideoPresentFlag,
                         bool     occupancyVideoPresentFlag,
                         bool     geometryVideoPresentFlag,
                         bool     attributeVideoPresentFlag ) {
    v3cParameterSetId_ = v3cParameterSetId;
    atlasCountMinus1_  = 0;
    allocateAtlas();
    atlasId_[0]                       = atlasId;
    frameWidth_[0]                    = frameWidth;
    frameHeight_[0]                   = frameHeight;
    mapCountMinus1_[0]                = mapCountMinus1;
    multipleMapStreamsPresentFlag_[0] = multipleMapStreamsPresentFlag;
    auxiliaryVideoPresentFlag_[0]     = auxiliaryVideoPresentFlag;
    occupancyVideoPresentFlag_[0]     = occupancyVideoPresentFlag;
    geometryVideoPresentFlag_[0]      = geometryVideoPresentFlag;
    attributeVideoPresentFlag_[0]     = attributeVideoPresentFlag;
  }

  void allocateAtlas() {
    atlasId_.resize( atlasCountMinus1_ + 1, 0 );
    frameWidth_.resize( atlasCountMinus1_ + 1, 1 );
    frameHeight_.resize( atlasCountMinus1_ + 1 );
    mapCountMinus1_.resize( atlasCountMinus1_ + 1 );
    multipleMapStreamsPresentFlag_.resize( atlasCountMinus1_ + 1 );
    mapAbsoluteCodingEnableFlag_.resize( atlasCountMinus1_ + 1 );
    mapPredictorIndexDiff_.resize( atlasCountMinus1_ + 1 );
    auxiliaryVideoPresentFlag_.resize( atlasCountMinus1_ + 1 );
    geometryVideoPresentFlag_.resize( atlasCountMinus1_ + 1 );
    occupancyVideoPresentFlag_.resize( atlasCountMinus1_ + 1 );
    attributeVideoPresentFlag_.resize( atlasCountMinus1_ + 1 );
    geometryInformation_.resize( atlasCountMinus1_ + 1 );
    occupancyInformation_.resize( atlasCountMinus1_ + 1 );
    attributeInformation_.resize( atlasCountMinus1_ + 1 );
  }
  void allocateMap( size_t index ) {
    mapAbsoluteCodingEnableFlag_[index].resize( mapCountMinus1_[index] + 1, 1 );
    mapPredictorIndexDiff_[index].resize( mapCountMinus1_[index] + 1 );
  }
  void              allocateExtensionDataByte() { extensionDataByte_.resize( extensionLengthMinus1_ + 1 ); }
  uint32_t          getV3CParameterSetId() { return v3cParameterSetId_; }
  uint32_t          getAtlasCountMinus1() { return atlasCountMinus1_; }
  uint16_t          getAtlasId( size_t index ) { return atlasId_[index]; }
  uint16_t          getFrameWidth( size_t index ) { return frameWidth_[index]; }
  uint16_t          getFrameHeight( size_t index ) { return frameHeight_[index]; }
  uint32_t          getMapCountMinus1( size_t index ) { return mapCountMinus1_[index]; }
  bool              getMultipleMapStreamsPresentFlag( size_t index ) { return multipleMapStreamsPresentFlag_[index]; }
  bool              getAuxiliaryVideoPresentFlag( size_t index ) { return auxiliaryVideoPresentFlag_[index]; }
  bool              getOccupancyVideoPresentFlag( size_t index ) { return occupancyVideoPresentFlag_[index]; }
  bool              getGeometryVideoPresentFlag( size_t index ) { return geometryVideoPresentFlag_[index]; }
  bool              getAttributeVideoPresentFlag( size_t index ) { return attributeVideoPresentFlag_[index]; }
  size_t            getMapPredictorIndexDiff( size_t i, size_t j ) { return mapPredictorIndexDiff_[i][j]; }
  bool              getMapAbsoluteCodingEnableFlag( size_t i, size_t j ) { return mapAbsoluteCodingEnableFlag_[i][j]; }
  ProfileTierLevel& getProfileTierLevel() { return profileTierLevel_; }
  GeometryInformation&  getGeometryInformation( size_t index ) { return geometryInformation_[index]; }
  OccupancyInformation& getOccupancyInformation( size_t index ) { return occupancyInformation_[index]; }
  AttributeInformation& getAttributeInformation( size_t index ) { return attributeInformation_[index]; }
  bool                  getExtensionPresentFlag() { return extensionPresentFlag_; }
  uint8_t               getExtension8Bits() { return extension8Bits_; }
  size_t                getExtensionLengthMinus1() { return extensionLengthMinus1_; }
  std::vector<uint8_t>& getExtensionDataByte() { return extensionDataByte_; }
  uint8_t               getExtensionDataByte( size_t index ) { return extensionDataByte_[index]; }
  VpsVpccExtension&     getVpsVpccExtension() { return vpsVpccExtension_; }

  void setV3CParameterSetId( uint32_t value ) { v3cParameterSetId_ = value; }
  void setAtlasCountMinus1( uint32_t value ) { atlasCountMinus1_ = value; }
  void setAtlasId( size_t index, uint16_t value ) { atlasId_[index] = value; }
  void setFrameWidth( size_t index, uint16_t value ) { frameWidth_[index] = value; }
  void setFrameHeight( size_t index, uint16_t value ) { frameHeight_[index] = value; }
  void setMapCountMinus1( size_t index, uint32_t value ) { mapCountMinus1_[index] = value; }
  void setMultipleMapStreamsPresentFlag( size_t index, bool value ) { multipleMapStreamsPresentFlag_[index] = value; }
  void setAuxiliaryVideoPresentFlag( size_t index, bool value ) { auxiliaryVideoPresentFlag_[index] = value; }
  void setOccupancyVideoPresentFlag( size_t index, bool value ) { occupancyVideoPresentFlag_[index] = value; }
  void setGeometryVideoPresentFlag( size_t index, bool value ) { geometryVideoPresentFlag_[index] = value; }
  void setAttributeVideoPresentFlag( size_t index, bool value ) { attributeVideoPresentFlag_[index] = value; }
  void setGeometryInformation( size_t index, GeometryInformation value ) { geometryInformation_[index] = value; }
  void setOccupancyInformation( size_t index, OccupancyInformation value ) { occupancyInformation_[index] = value; }
  void setMapAbsoluteCodingEnableFlag( size_t i, size_t j, bool value ) {
    if ( mapAbsoluteCodingEnableFlag_[i].size() < j + 1 ) { mapAbsoluteCodingEnableFlag_[i].resize( j + 1, 1 ); }
    mapAbsoluteCodingEnableFlag_[i][j] = value;
  }
  void setMapPredictorIndexDiff( size_t i, size_t j, bool value ) {
    if ( mapPredictorIndexDiff_[i].size() < j + 1 ) { mapPredictorIndexDiff_[i].resize( j + 1 ); }
    mapPredictorIndexDiff_[i][j] = value;
  }
  void addMapAbsoluteCodingEnableFlag( size_t index, bool value ) {
    mapAbsoluteCodingEnableFlag_[index].push_back( value );
  }
  void addMapPredictorIndexDiff( size_t index, bool value ) { mapPredictorIndexDiff_[index].push_back( value ); }
  void setExtensionDataByte( size_t index, uint8_t value ) { extensionDataByte_[index] = value; }
  void setExtensionPresentFlag( bool value ) { extensionPresentFlag_ = value; }
  void setExtension8Bits( uint8_t value ) { extension8Bits_ = value; }
  void setExtensionLengthMinus1( size_t value ) { extensionLengthMinus1_ = value; }

 private:
  ProfileTierLevel                  profileTierLevel_;
  uint32_t                          v3cParameterSetId_;
  uint32_t                          atlasCountMinus1_;
  std::vector<uint16_t>             atlasId_;
  std::vector<uint16_t>             frameWidth_;
  std::vector<uint16_t>             frameHeight_;
  std::vector<uint8_t>              mapCountMinus1_;
  std::vector<bool>                 multipleMapStreamsPresentFlag_;
  std::vector<std::vector<bool>>    mapAbsoluteCodingEnableFlag_;
  std::vector<std::vector<size_t>>  mapPredictorIndexDiff_;
  std::vector<bool>                 auxiliaryVideoPresentFlag_;
  std::vector<bool>                 occupancyVideoPresentFlag_;
  std::vector<bool>                 geometryVideoPresentFlag_;
  std::vector<bool>                 attributeVideoPresentFlag_;
  std::vector<OccupancyInformation> occupancyInformation_;
  std::vector<GeometryInformation>  geometryInformation_;
  std::vector<AttributeInformation> attributeInformation_;
  bool                              extensionPresentFlag_;
  uint8_t                           extension8Bits_;
  size_t                            extensionLengthMinus1_;
  std::vector<uint8_t>              extensionDataByte_;
  VpsVpccExtension                  vpsVpccExtension_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_VPCCPARAMETERSET_H
