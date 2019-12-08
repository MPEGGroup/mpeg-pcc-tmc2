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
#ifndef PCCContext_h
#define PCCContext_h

#include "PCCCommon.h"
#include "PCCVideo.h"
#include "PCCMetadata.h"  //jkei: when it is disabled, PCCVector3 is not found!
#include "PCCVideoBitstream.h"
#if VPCCUNIT_DATA_BITSTREAM
#include "PCCBitstream.h"
#endif
#include <map>

namespace pcc {

class PCCGroupOfFrames;
class PCCFrameContext;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoTexture;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoOccupancyMap;

class PCCPatch;
typedef std::map<size_t, PCCPatch>                 unionPatch;     // [TrackIndex, PatchUnion]
typedef std::pair<size_t, size_t>                  GlobalPatch;    // [FrameIndex, PatchIndex]
typedef std::map<size_t, std::vector<GlobalPatch>> GlobalPatches;  // [TrackIndex, <GlobalPatch>]
typedef std::pair<size_t, size_t>                  SubContext;     // [start, end)

class PCCBitstreamStat;

// Annex E: Supplemental enhancement information
// E.2.1	General SEI message syntax  <=> 7.3.8	Supplemental enhancement information message syntax
class SEI {
 public:
  SEI() {}
  virtual ~SEI() {}
  virtual SeiPayloadType getPayloadType() = 0;
  uint8_t                getPayloadSize() { return payloadSize_; }
  void                   setPayloadSize( uint8_t value ) { payloadSize_ = value; }

 private:
  uint8_t payloadSize_;
};

// E.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
class SEIUserDataRegisteredItuTT35 : public SEI {
 public:
  SEIUserDataRegisteredItuTT35() : ituTT35CountryCode_( 0 ), ituTT35CountryCodeExtensionByte_( 0 ) {}

  ~SEIUserDataRegisteredItuTT35() { ituTT35PayloadByte_.clear(); }
  SEIUserDataRegisteredItuTT35& operator=( const SEIUserDataRegisteredItuTT35& ) = default;

  SeiPayloadType getPayloadType() { return USER_DATAREGISTERED_ITUTT35; }

  uint8_t               getItuTT35CountryCode() { return ituTT35CountryCode_; }
  uint8_t               getItuTT35CountryCodeExtensionByte() { return ituTT35CountryCodeExtensionByte_; }
  std::vector<uint8_t>& getItuTT35PayloadByte() { return ituTT35PayloadByte_; }
  void                  setItuTT35CountryCode( uint8_t value ) { ituTT35CountryCode_ = value; }
  void setItuTT35CountryCodeExtensionByte( uint8_t value ) { ituTT35CountryCodeExtensionByte_ = value; }

 private:
  uint8_t              ituTT35CountryCode_;
  uint8_t              ituTT35CountryCodeExtensionByte_;
  std::vector<uint8_t> ituTT35PayloadByte_;
};

// E.2.4  User data unregistered SEI message syntax
class SEIUserDataUnregistered : public SEI {
 public:
  SEIUserDataUnregistered() {
    for ( size_t i = 0; i < 16; i++ ) { uuidIsoIec11578_[i] = 0; }
  }

  ~SEIUserDataUnregistered() { userDataPayloadByte_.clear(); }
  SEIUserDataUnregistered& operator=( const SEIUserDataUnregistered& ) = default;

  SeiPayloadType getPayloadType() { return USER_DATA_UNREGISTERED; }

  uint8_t*              getUuidIsoIec11578() { return uuidIsoIec11578_; }
  uint8_t               getUuidIsoIec11578( size_t index ) { return uuidIsoIec11578_[index]; }
  std::vector<uint8_t>& getUserDataPayloadByte() { return userDataPayloadByte_; }
  uint8_t&              getUserDataPayloadByte( size_t index ) { return userDataPayloadByte_[index]; }
  void                  setUuidIsoIec11578( size_t index, uint8_t value ) { uuidIsoIec11578_[index] = value; }
  void                  setUserDataPayloadByte( size_t index, uint8_t value ) { userDataPayloadByte_[index] = value; }

 private:
  uint8_t              uuidIsoIec11578_[16];
  std::vector<uint8_t> userDataPayloadByte_;
};

// E.2.5  Recovery point SEI message syntax
class SEIRecoveryPoint : public SEI {
 public:
  SEIRecoveryPoint() : recoveryAfocCnt_( 0 ), exactMatchFlag_( 0 ), brokenLinkFlag_( 0 ) {}

  ~SEIRecoveryPoint() {}
  SEIRecoveryPoint& operator=( const SEIRecoveryPoint& ) = default;

  SeiPayloadType getPayloadType() { return RECOVERY_POINT; }

  uint8_t getRecoveryAfocCnt() { return recoveryAfocCnt_; }
  uint8_t getExactMatchFlag() { return exactMatchFlag_; }
  uint8_t getBrokenLinkFlag() { return brokenLinkFlag_; }
  void    setRecoveryAfocCnt( uint8_t value ) { recoveryAfocCnt_ = value; }
  void    setExactMatchFlag( uint8_t value ) { exactMatchFlag_ = value; }
  void    setBrokenLinkFlag( uint8_t value ) { brokenLinkFlag_ = value; }

 private:
  int32_t recoveryAfocCnt_;
  uint8_t exactMatchFlag_;
  uint8_t brokenLinkFlag_;
};

// E.2.6  No display SEI message syntax
class SEINoDisplay : public SEI {
 public:
  SEINoDisplay() {}

  ~SEINoDisplay() {}
  SEINoDisplay& operator=( const SEINoDisplay& ) = default;

  SeiPayloadType getPayloadType() { return NO_DISPLAY; }

 private:
};

// E.2.7  Reserved SEI message syntax
class SEIReservedSeiMessage : public SEI {
 public:
  SEIReservedSeiMessage() {}

  ~SEIReservedSeiMessage() { reservedSeiMessagePayloadByte_.clear(); }
  SEIReservedSeiMessage& operator=( const SEIReservedSeiMessage& ) = default;

  SeiPayloadType getPayloadType() { return RESERVED_SEI_MESSAGE; }

  std::vector<uint8_t>& getReservedSeiMessagePayloadByte() { return reservedSeiMessagePayloadByte_; }
  uint8_t getReservedSeiMessagePayloadByte( size_t index ) { return reservedSeiMessagePayloadByte_[index]; }

  void setReservedSeiMessagePayloadByte( size_t index, uint8_t value ) {
    reservedSeiMessagePayloadByte_[index] = value;
  }

 private:
  std::vector<uint8_t> reservedSeiMessagePayloadByte_;
};

// E.2.8  SEI manifest SEI message syntax
class SEIManifest : public SEI {
 public:
  SEIManifest() : manifestNumSeiMsgTypes_( 0 ) {}
  ~SEIManifest() {
    manifestSeiPayloadType_.clear();

    manifestSeiDescription_.clear();
  }
  SEIManifest& operator=( const SEIManifest& ) = default;

  SeiPayloadType getPayloadType() { return SEI_MANIFEST; }

  void allocate() {
    manifestSeiPayloadType_.resize( manifestNumSeiMsgTypes_, 0 );
    manifestSeiDescription_.resize( manifestNumSeiMsgTypes_, 0 );
  }
  uint16_t               getManifestNumSeiMsgTypes() { return manifestNumSeiMsgTypes_; }
  std::vector<uint16_t>& getManifestSeiPayloadType() { return manifestSeiPayloadType_; }
  std::vector<uint8_t>&  getManifestSeiDescription() { return manifestSeiDescription_; }
  uint16_t               getManifestSeiPayloadType( size_t index ) { return manifestSeiPayloadType_[index]; }
  uint8_t                getManifestSeiDescription( size_t index ) { return manifestSeiDescription_[index]; }
  void                   setManifestNumSeiMsgTypes( uint16_t value ) { manifestNumSeiMsgTypes_ = value; }
  void setManifestSeiPayloadType( size_t index, uint16_t value ) { manifestSeiPayloadType_[index] = value; }
  void setManifestSeiDescription( size_t index, uint8_t value ) { manifestSeiDescription_[index] = value; }

 private:
  uint16_t              manifestNumSeiMsgTypes_;
  std::vector<uint16_t> manifestSeiPayloadType_;
  std::vector<uint8_t>  manifestSeiDescription_;
};

// E.2.9  SEI prefix indication SEI message syntax
class SEIPrefixIndication : public SEI {
 public:
  SEIPrefixIndication() : prefixSeiPayloadType_( 0 ), numSeiPrefixIndicationsMinus1_( 0 ) {}

  ~SEIPrefixIndication() {
    numBitsInPrefixIndicationMinus1_.clear();
    for ( auto& element : seiPrefixDataBit_ ) { element.clear(); }
    seiPrefixDataBit_.clear();
  }
  SEIPrefixIndication& operator=( const SEIPrefixIndication& ) = default;

  SeiPayloadType getPayloadType() { return SEI_PREFIX_INDICATION; }

  uint16_t                        getPrefixSeiPayloadType() { return prefixSeiPayloadType_; }
  uint8_t                         getNumSeiPrefixIndicationsMinus1() { return numSeiPrefixIndicationsMinus1_; }
  std::vector<uint16_t>&          getNumBitsInPrefixIndicationMinus1() { return numBitsInPrefixIndicationMinus1_; }
  std::vector<std::vector<bool>>& getSeiPrefixDataBit() { return seiPrefixDataBit_; }
  uint16_t getNumBitsInPrefixIndicationMinus1( size_t index ) { return numBitsInPrefixIndicationMinus1_[index]; }
  std::vector<bool> getSeiPrefixDataBit( size_t i ) { return seiPrefixDataBit_[i]; }
  bool              getSeiPrefixDataBit( size_t i, size_t j ) { return seiPrefixDataBit_[i][j]; }

  void setPrefixSeiPayloadType( uint16_t value ) { prefixSeiPayloadType_ = value; }
  void setNumSeiPrefixIndicationsMinus1( uint8_t value ) { numSeiPrefixIndicationsMinus1_ = value; }
  void setNumBitsInPrefixIndicationMinus1( size_t index, uint16_t value ) {
    numBitsInPrefixIndicationMinus1_[index] = value;
  }
  void setSeiPrefixDataBit( size_t i, size_t j, bool value ) { seiPrefixDataBit_[i][j] = value; }

 private:
  uint16_t                       prefixSeiPayloadType_;
  uint8_t                        numSeiPrefixIndicationsMinus1_;
  std::vector<uint16_t>          numBitsInPrefixIndicationMinus1_;
  std::vector<std::vector<bool>> seiPrefixDataBit_;
};

// E.2.10  Geometry transformation parameters SEI message syntax
class SEIGeometryTransformationParams : public SEI {
 public:
  SEIGeometryTransformationParams() :
      gtpCancelFlag_( false ),
      gtpScaleEnabledFlag_( false ),
      gtpOffsetEnabledFlag_( false ),
      gtpRotationEnabledFlag_( false ),
      gtpRotationQx_( 0 ),
      gtpRotationQy_( 0 ),
      gtpRotationQz_( 0 ) {
    for ( size_t i = 0; i < 3; i++ ) {
      gtpGeometryScaleOnAxis_[i]  = 0;
      gtpGeometryOffsetOnAxis_[i] = 0;
    }
  }

  ~SEIGeometryTransformationParams() {}
  SEIGeometryTransformationParams& operator=( const SEIGeometryTransformationParams& ) = default;

  SeiPayloadType getPayloadType() { return SEI_PREFIX_INDICATION; }

  bool     getGtpCancelFlag() { return gtpCancelFlag_; }
  bool     getGtpScaleEnabledFlag() { return gtpScaleEnabledFlag_; }
  bool     getGtpOffsetEnabledFlag() { return gtpOffsetEnabledFlag_; }
  bool     getGtpRotationEnabledFlag() { return gtpRotationEnabledFlag_; }
  uint32_t getGtpGeometryScaleOnAxis( size_t index ) { return gtpGeometryScaleOnAxis_[index]; }
  int32_t  getGtpGeometryOffsetOnAxis( size_t index ) { return gtpGeometryOffsetOnAxis_[index]; }
  int16_t  getGtpRotationQx() { return gtpRotationQx_; }
  int16_t  getGtpRotationQy() { return gtpRotationQy_; }
  int16_t  getGtpRotationQz() { return gtpRotationQz_; }

  void setGtpCancelFlag( bool value ) { gtpCancelFlag_ = value; }
  void setGtpScaleEnabledFlag( bool value ) { gtpScaleEnabledFlag_ = value; }
  void setGtpOffsetEnabledFlag( bool value ) { gtpOffsetEnabledFlag_ = value; }
  void setGtpRotationEnabledFlag( bool value ) { gtpRotationEnabledFlag_ = value; }
  void setGtpGeometryScaleOnAxis( size_t index, uint32_t value ) { gtpGeometryScaleOnAxis_[index] = value; }
  void setGtpGeometryOffsetOnAxis( size_t index, int32_t value ) { gtpGeometryOffsetOnAxis_[index] = value; }
  void setGtpRotationQx( uint16_t value ) { gtpRotationQx_ = value; }
  void setGtpRotationQy( uint16_t value ) { gtpRotationQy_ = value; }
  void setGtpRotationQz( uint16_t value ) { gtpRotationQz_ = value; }

 private:
  bool     gtpCancelFlag_;
  bool     gtpScaleEnabledFlag_;
  bool     gtpOffsetEnabledFlag_;
  bool     gtpRotationEnabledFlag_;
  uint32_t gtpGeometryScaleOnAxis_[3];
  int32_t  gtpGeometryOffsetOnAxis_[3];
  int16_t  gtpRotationQx_;
  int16_t  gtpRotationQy_;
  int16_t  gtpRotationQz_;
};

// E.2.11  Attribute transformation parameters SEI message syntax
class SEIAttributeTransformationParams : public SEI {
 public:
  SEIAttributeTransformationParams() : atpCancelFlag_( false ), atpNumAttributeUpdates_( 0 ) {}

  ~SEIAttributeTransformationParams() {
    atpAttributeIdx_.clear();
    atpDimensionMinus1_.clear();
    for ( auto& element : atpScaleParamsEnabledFlag_ ) { element.clear(); }
    for ( auto& element : atpOffsetParamsEnabledFlag_ ) { element.clear(); }
    for ( auto& element : atpAttributeScale_ ) { element.clear(); }
    for ( auto& element : atpAttributeOffset_ ) { element.clear(); }
    atpScaleParamsEnabledFlag_.clear();
    atpOffsetParamsEnabledFlag_.clear();
    atpAttributeScale_.clear();
    atpAttributeOffset_.clear();
  }
  SEIAttributeTransformationParams& operator=( const SEIAttributeTransformationParams& ) = default;

  SeiPayloadType getPayloadType() { return ATTRIBUTE_TRANSFORMATION_PARAMS; }
  void           allocate() {
    atpAttributeIdx_.resize( atpNumAttributeUpdates_ + 1, 0 );
    atpDimensionMinus1_.resize( 256 + 1, 0 );
    atpScaleParamsEnabledFlag_.resize( 256 );
    atpOffsetParamsEnabledFlag_.resize( 256 );
    atpAttributeScale_.resize( 256 );
    atpAttributeOffset_.resize( 256 );
  }
  void allocate( size_t index ) {
    atpScaleParamsEnabledFlag_[index].resize( atpDimensionMinus1_[index] + 1, false );
    atpOffsetParamsEnabledFlag_[index].resize( atpDimensionMinus1_[index] + 1, false );
    atpAttributeScale_[index].resize( atpDimensionMinus1_[index] + 1, 0 );
    atpAttributeOffset_[index].resize( atpDimensionMinus1_[index] + 1, 0 );
  }
  bool                                getAtpCancelFlag() { return atpCancelFlag_; }
  int32_t                             getAtpNumAttributeUpdates() { return atpNumAttributeUpdates_; }
  std::vector<uint8_t>&               atpAttributeIdx() { return atpAttributeIdx_; }
  std::vector<uint8_t>&               getAtpDimensionMinus() { return atpDimensionMinus1_; }
  std::vector<std::vector<bool>>&     getAtpScaleParamsEnabledFlag() { return atpScaleParamsEnabledFlag_; }
  std::vector<std::vector<bool>>&     getAtpOffsetParamsEnabledFlag() { return atpOffsetParamsEnabledFlag_; }
  std::vector<std::vector<uint32_t>>& getAtpAttributeScale() { return atpAttributeScale_; }
  std::vector<std::vector<int32_t>>&  getAtpAttributeOffset() { return atpAttributeOffset_; }

  uint8_t  getAtpAttributeIdx( size_t i ) { return atpAttributeIdx_[i]; }
  uint8_t  getAtpDimensionMinus1( size_t i ) { return atpDimensionMinus1_[i]; }
  bool     getAtpScaleParamsEnabledFlag( size_t i, size_t j ) { return atpScaleParamsEnabledFlag_[i][j]; }
  bool     getAtpOffsetParamsEnabledFlag( size_t i, size_t j ) { return atpOffsetParamsEnabledFlag_[i][j]; }
  uint32_t getAtpAttributeScale( size_t i, size_t j ) { return atpAttributeScale_[i][j]; }
  int32_t  getAtpAttributeOffset( size_t i, size_t j ) { return atpAttributeOffset_[i][j]; }

  void setAtpCancelFlag( bool value ) { atpCancelFlag_ = value; }
  void setAtpNumAttributeUpdates( int32_t value ) { atpNumAttributeUpdates_ = value; }
  void setAtpAttributeIdx( size_t i, uint8_t value ) { atpAttributeIdx_[i] = value; }
  void setAtpDimensionMinus1( size_t i, uint8_t value ) { atpDimensionMinus1_[i] = value; }
  void setAtpScaleParamsEnabledFlag( size_t i, size_t j, bool value ) { atpScaleParamsEnabledFlag_[i][j] = value; }
  void setAtpOffsetParamsEnabledFlag( size_t i, size_t j, bool value ) { atpOffsetParamsEnabledFlag_[i][j] = value; }
  void setAtpAttributeScale( size_t i, size_t j, uint32_t value ) { atpAttributeScale_[i][j] = value; }
  void setAtpAttributeOffset( size_t i, size_t j, int32_t value ) { atpAttributeOffset_[i][j] = value; }

 private:
  bool                               atpCancelFlag_;
  int32_t                            atpNumAttributeUpdates_;
  std::vector<uint8_t>               atpAttributeIdx_;
  std::vector<uint8_t>               atpDimensionMinus1_;
  std::vector<std::vector<bool>>     atpScaleParamsEnabledFlag_;
  std::vector<std::vector<bool>>     atpOffsetParamsEnabledFlag_;
  std::vector<std::vector<uint32_t>> atpAttributeScale_;
  std::vector<std::vector<int32_t>>  atpAttributeOffset_;
};

// E.2.12  Active substreams SEI message syntax
class SEIActiveSubstreams : public SEI {
 public:
  SEIActiveSubstreams() :
      activeAttributesChangesFlag_( 0 ),
      activeMapsChangesFlag_( 0 ),
      rawPointsSubstreamsActiveFlag_( 0 ),
      allAttributesActiveFlag_( 0 ),
      allMapsActiveFlag_( 0 ),
      activeAttributeCountMinus1_( 0 ),
      activeMapCountMinus1_( 0 ) {}
  ~SEIActiveSubstreams() {
    activeAttributeIdx_.clear();
    activeMapIdx_.clear();
  }
  SEIActiveSubstreams& operator=( const SEIActiveSubstreams& ) = default;

  SeiPayloadType        getPayloadType() { return ACTIVE_SUBSTREAMS; }
  bool                  getActiveAttributesChangesFlag() { return activeAttributesChangesFlag_; }
  bool                  getActiveMapsChangesFlag() { return activeMapsChangesFlag_; }
  bool                  getRawPointsSubstreamsActiveFlag() { return rawPointsSubstreamsActiveFlag_; }
  bool                  getAllAttributesActiveFlag() { return allAttributesActiveFlag_; }
  bool                  getAllMapsActiveFlag() { return allMapsActiveFlag_; }
  uint8_t               getActiveAttributeCountMinus1() { return activeAttributeCountMinus1_; }
  uint8_t               getActiveMapCountMinus1() { return activeMapCountMinus1_; }
  std::vector<uint8_t>& getActiveAttributeIdx() { return activeAttributeIdx_; }
  std::vector<uint8_t>& getActiveMapIdx() { return activeMapIdx_; }
  uint8_t               getActiveAttributeIdx( size_t index ) { return activeAttributeIdx_[index]; }
  uint8_t               getActiveMapIdx( size_t index ) { return activeMapIdx_[index]; }

  void setActiveAttributesChangesFlag( bool value ) { activeAttributesChangesFlag_ = value; }
  void setActiveMapsChangesFlag( bool value ) { activeMapsChangesFlag_ = value; }
  void setRawPointsSubstreamsActiveFlag( bool value ) { rawPointsSubstreamsActiveFlag_ = value; }
  void setAllAttributesActiveFlag( bool value ) { allAttributesActiveFlag_ = value; }
  void setAllMapsActiveFlag( bool value ) { allMapsActiveFlag_ = value; }
  void setActiveAttributeCountMinus1( uint8_t value ) { activeAttributeCountMinus1_ = value; }
  void setActiveMapCountMinus1( uint8_t value ) { activeMapCountMinus1_ = value; }
  void setActiveAttributeIdx( size_t index, uint8_t value ) { activeAttributeIdx_[index] = value; }
  void setActiveMapIdx( size_t index, uint8_t value ) { activeMapIdx_[index] = value; }

 private:
  bool                 activeAttributesChangesFlag_;
  bool                 activeMapsChangesFlag_;
  bool                 rawPointsSubstreamsActiveFlag_;
  bool                 allAttributesActiveFlag_;
  bool                 allMapsActiveFlag_;
  uint8_t              activeAttributeCountMinus1_;
  uint8_t              activeMapCountMinus1_;
  std::vector<uint8_t> activeAttributeIdx_;
  std::vector<uint8_t> activeMapIdx_;
};

// E.2.13  Component codec mapping SEI message syntax
class SEIComponentCodecMapping : public SEI {
 public:
  SEIComponentCodecMapping() : ccmCodecMappingsCountMinus1_( 0 ) {}
  ~SEIComponentCodecMapping() {
    ccmCodecId_.clear();
    ccmCodec4cc_.clear();
  }
  SEIComponentCodecMapping& operator=( const SEIComponentCodecMapping& ) = default;

  SeiPayloadType getPayloadType() { return COMPONENT_CODEC_MAPPING; }

  void allocate() {
    ccmCodecId_.resize( ccmCodecMappingsCountMinus1_ + 1, 0 );
    ccmCodec4cc_.resize( 256 );
  }
  bool                      getCcmCodecMappingsCountMinus1() { return ccmCodecMappingsCountMinus1_; }
  std::vector<uint8_t>&     getCcmCodecId() { return ccmCodecId_; }
  std::vector<std::string>& getCcmCodec4cc() { return ccmCodec4cc_; }
  uint8_t                   getCcmCodecId( size_t index ) { return ccmCodecId_[index]; }
  std::string               getCcmCodec4cc( size_t index ) { return ccmCodec4cc_[index]; }

  void setCcmCodecMappingsCountMinus1( uint8_t value ) { ccmCodecMappingsCountMinus1_ = value; }
  void setCcmCodecId( size_t index, uint8_t value ) { ccmCodecId_[index] = value; }
  void setCcmCodec4cc( size_t index, std::string value ) { ccmCodec4cc_[index] = value; }

 private:
  uint8_t                  ccmCodecMappingsCountMinus1_;
  std::vector<uint8_t>     ccmCodecId_;
  std::vector<std::string> ccmCodec4cc_;
};

// E.2.14  Volumetric Tiling SEI message syntax

// E.2.14.2  Volumetric Tiling Info Labels
class VolumetricTilingInfoLabels {
 public:
  VolumetricTilingInfoLabels() : vtiObjectLabelLanguagePresentFlag_( false ), vtiNumObjectLabelUpdates_( 0 ) {}
  ~VolumetricTilingInfoLabels() {
    vtiLabelIdx.clear();
    vtiLabel_.clear();
  }
  VolumetricTilingInfoLabels& operator=( const VolumetricTilingInfoLabels& ) = default;
  void                        allocate() { vtiLabelIdx.resize( vtiNumObjectLabelUpdates_, 0 ); }
  bool                        getVtiObjectLabelLanguagePresentFlag() { return vtiObjectLabelLanguagePresentFlag_; }
  uint32_t                    getVtiNumObjectLabelUpdates() { return vtiNumObjectLabelUpdates_; }
  std::string&                getVtiObjectLabelLanguage() { return vtiObjectLabelLanguage_; }
  std::vector<uint8_t>&       getVtiLabelIdx() { return vtiLabelIdx; }
  std::vector<std::string>&   getVtiLabel() { return vtiLabel_; }
  uint8_t&                    getVtiLabelIdx( size_t index ) { return vtiLabelIdx[index]; }
  std::string&                getVtiLabel( size_t index ) {
    if ( vtiLabel_.size() < index ) { vtiLabel_.resize( index ); }
    return vtiLabel_[index];
  }

  void setVtiObjectLabelLanguage( std::string value ) { vtiObjectLabelLanguage_ = value; }
  void setVtiObjectLabelLanguagePresentFlag( bool value ) { vtiObjectLabelLanguagePresentFlag_ = value; }
  void setVtiNumObjectLabelUpdates( uint32_t value ) { vtiNumObjectLabelUpdates_ = value; }
  void setVtiLabelIdx( size_t index, uint8_t value ) { vtiLabelIdx[index] = value; }
  void setVtiLabel( size_t index, std::string value ) {
    if ( vtiLabel_.size() < index ) { vtiLabel_.resize( index + 1 ); }
    vtiLabel_[index] = value;
  }

 private:
  bool                     vtiObjectLabelLanguagePresentFlag_;
  uint32_t                 vtiNumObjectLabelUpdates_;
  std::string              vtiObjectLabelLanguage_;
  std::vector<uint8_t>     vtiLabelIdx;
  std::vector<std::string> vtiLabel_;
};

// E.2.14.3  Volumetric Tiling Info Objects
class VolumetricTilingInfoObjects {
 public:
  VolumetricTilingInfoObjects() : vtiNumObjectUpdates_( 0 ) {}
  ~VolumetricTilingInfoObjects() {
    vtiObjectIdx_.clear();
    vtiObjectCancelFlag_.clear();
    vtiBoundingBoxUpdateFlag_.clear();
    vti3dBoundingBoxUpdateFlag_.clear();
    vtiObjectHiddenFlag_.clear();
    vtiObjectPriorityUpdateFlag_.clear();
    vtiObjectLabelUpdateFlag_.clear();
    vtiObjectCollisionShapeUpdateFlag_.clear();
    vtiObjectDependencyUpdateFlag_.clear();
    vtiBoundingBoxTop_.clear();
    vtiBoundingBoxLeft_.clear();
    vtiBoundingBoxWidth_.clear();
    vtiBoundingBoxHeight_.clear();
    vti3dBoundingBoxX_.clear();
    vti3dBoundingBoxY_.clear();
    vti3dBoundingBoxZ_.clear();
    vti3dBoundingBoxDeltaX_.clear();
    vti3dBoundingBoxDeltaY_.clear();
    vti3dBoundingBoxDeltaZ_.clear();
    vtiObjectPriorityValue_.clear();
    vtiObjectLabelIdx_.clear();
    vtiObjectCollisionShapeId_.clear();
    vtiObjectNumDependencies_.clear();
    vtiObjectDependencyIdx_.clear();
  }
  VolumetricTilingInfoObjects& operator=( const VolumetricTilingInfoObjects& ) = default;
  void                         allocate() { vtiObjectIdx_.resize( vtiNumObjectUpdates_ ); }
  void                         allocate( size_t size ) {
    if ( vtiObjectCancelFlag_.size() < size ) {
      vtiObjectCancelFlag_.resize( size );
      vtiBoundingBoxUpdateFlag_.resize( size );
      vti3dBoundingBoxUpdateFlag_.resize( size );
      vtiObjectHiddenFlag_.resize( size );
      vtiObjectPriorityUpdateFlag_.resize( size );
      vtiObjectLabelUpdateFlag_.resize( size );
      vtiObjectCollisionShapeUpdateFlag_.resize( size );
      vtiObjectDependencyUpdateFlag_.resize( size );
      vtiBoundingBoxTop_.resize( size );
      vtiBoundingBoxLeft_.resize( size );
      vtiBoundingBoxWidth_.resize( size );
      vtiBoundingBoxHeight_.resize( size );
      vti3dBoundingBoxX_.resize( size );
      vti3dBoundingBoxY_.resize( size );
      vti3dBoundingBoxZ_.resize( size );
      vti3dBoundingBoxDeltaX_.resize( size );
      vti3dBoundingBoxDeltaY_.resize( size );
      vti3dBoundingBoxDeltaZ_.resize( size );
      vtiObjectPriorityValue_.resize( size );
      vtiObjectLabelIdx_.resize( size );
      vtiObjectCollisionShapeId_.resize( size );
      vtiObjectNumDependencies_.resize( size );
      vtiObjectDependencyIdx_.resize( size );
    }
  }
  uint32_t               getVtiNumObjectUpdates() { return vtiNumObjectUpdates_; }
  std::vector<uint8_t>&  getVtiObjectIdx() { return vtiObjectIdx_; }
  std::vector<bool>&     getVtiObjectCancelFlag() { return vtiObjectCancelFlag_; }
  std::vector<bool>&     getVtiBoundingBoxUpdateFlag() { return vtiBoundingBoxUpdateFlag_; }
  std::vector<bool>&     getVti3dBoundingBoxUpdateFlag() { return vti3dBoundingBoxUpdateFlag_; }
  std::vector<bool>&     getVtiObjectHiddenFlag() { return vtiObjectHiddenFlag_; }
  std::vector<bool>&     getVtiObjectPriorityUpdateFlag() { return vtiObjectPriorityUpdateFlag_; }
  std::vector<bool>&     getVtiObjectLabelUpdateFlag() { return vtiObjectLabelUpdateFlag_; }
  std::vector<bool>&     getVtiObjectCollisionShapeUpdateFlag() { return vtiObjectCollisionShapeUpdateFlag_; }
  std::vector<bool>&     getVtiObjectDependencyUpdateFlag() { return vtiObjectDependencyUpdateFlag_; }
  std::vector<uint32_t>& getVtiBoundingBoxTop() { return vtiBoundingBoxTop_; }
  std::vector<uint32_t>& getVtiBoundingBoxLeft() { return vtiBoundingBoxLeft_; }
  std::vector<uint32_t>& getVtiBoundingBoxWidth() { return vtiBoundingBoxWidth_; }
  std::vector<uint32_t>& getVtiBoundingBoxHeight() { return vtiBoundingBoxHeight_; }
  std::vector<uint32_t>& getVti3dBoundingBoxX() { return vti3dBoundingBoxX_; }
  std::vector<uint32_t>& getVti3dBoundingBoxY() { return vti3dBoundingBoxY_; }
  std::vector<uint32_t>& getVti3dBoundingBoxZ() { return vti3dBoundingBoxZ_; }
  std::vector<uint32_t>& getVti3dBoundingBoxDeltaX() { return vti3dBoundingBoxDeltaX_; }
  std::vector<uint32_t>& getVti3dBoundingBoxDeltaY() { return vti3dBoundingBoxDeltaY_; }
  std::vector<uint32_t>& getVti3dBoundingBoxDeltaZ() { return vti3dBoundingBoxDeltaZ_; }
  std::vector<uint32_t>& getVtiObjectPriorityValue() { return vtiObjectPriorityValue_; }
  std::vector<uint32_t>& getVtiObjectLabelIdx() { return vtiObjectLabelIdx_; }
  std::vector<uint32_t>& getVtiObjectCollisionShapeId() { return vtiObjectCollisionShapeId_; }
  std::vector<uint32_t>& getVtiObjectNumDependencies() { return vtiObjectNumDependencies_; }
  std::vector<std::vector<uint32_t>>& getVtiObjectDependencyIdx() { return vtiObjectDependencyIdx_; }

  uint8_t  getVtiObjectIdx( size_t i ) { return vtiObjectIdx_[i]; }
  bool     getVtiObjectCancelFlag( size_t i ) { return vtiObjectCancelFlag_[i]; }
  bool     getVtiBoundingBoxUpdateFlag( size_t i ) { return vtiBoundingBoxUpdateFlag_[i]; }
  bool     getVti3dBoundingBoxUpdateFlag( size_t i ) { return vti3dBoundingBoxUpdateFlag_[i]; }
  bool     getVtiObjectHiddenFlag( size_t i ) { return vtiObjectHiddenFlag_[i]; }
  bool     getVtiObjectPriorityUpdateFlag( size_t i ) { return vtiObjectPriorityUpdateFlag_[i]; }
  bool     getVtiObjectLabelUpdateFlag( size_t i ) { return vtiObjectLabelUpdateFlag_[i]; }
  bool     getVtiObjectCollisionShapeUpdateFlag( size_t i ) { return vtiObjectCollisionShapeUpdateFlag_[i]; }
  bool     getVtiObjectDependencyUpdateFlag( size_t i ) { return vtiObjectDependencyUpdateFlag_[i]; }
  uint32_t getVtiBoundingBoxTop( size_t i ) { return vtiBoundingBoxTop_[i]; }
  uint32_t getVtiBoundingBoxLeft( size_t i ) { return vtiBoundingBoxLeft_[i]; }
  uint32_t getVtiBoundingBoxWidth( size_t i ) { return vtiBoundingBoxWidth_[i]; }
  uint32_t getVtiBoundingBoxHeight( size_t i ) { return vtiBoundingBoxHeight_[i]; }
  uint32_t getVti3dBoundingBoxX( size_t i ) { return vti3dBoundingBoxX_[i]; }
  uint32_t getVti3dBoundingBoxY( size_t i ) { return vti3dBoundingBoxY_[i]; }
  uint32_t getVti3dBoundingBoxZ( size_t i ) { return vti3dBoundingBoxZ_[i]; }
  uint32_t getVti3dBoundingBoxDeltaX( size_t i ) { return vti3dBoundingBoxDeltaX_[i]; }
  uint32_t getVti3dBoundingBoxDeltaY( size_t i ) { return vti3dBoundingBoxDeltaY_[i]; }
  uint32_t getVti3dBoundingBoxDeltaZ( size_t i ) { return vti3dBoundingBoxDeltaZ_[i]; }
  uint32_t getVtiObjectPriorityValue( size_t i ) { return vtiObjectPriorityValue_[i]; }
  uint32_t getVtiObjectLabelIdx( size_t i ) { return vtiObjectLabelIdx_[i]; }
  uint32_t getVtiObjectCollisionShapeId( size_t i ) { return vtiObjectCollisionShapeId_[i]; }
  uint32_t getVtiObjectNumDependencies( size_t i ) { return vtiObjectNumDependencies_[i]; }
  uint32_t getVtiObjectDependencyIdx( size_t i, size_t j ) { return vtiObjectDependencyIdx_[i][j]; }

  void setVtiNumObjectUpdates( uint32_t value ) { vtiNumObjectUpdates_ = value; }
  void setVtiObjectIdx( size_t i, uint8_t value ) { vtiObjectIdx_[i] = value; }
  void setVtiObjectCancelFlag( size_t i, bool value ) { vtiObjectCancelFlag_[i] = value; }
  void setVtiBoundingBoxUpdateFlag( size_t i, bool value ) { vtiBoundingBoxUpdateFlag_[i] = value; }
  void setVti3dBoundingBoxUpdateFlag( size_t i, bool value ) { vti3dBoundingBoxUpdateFlag_[i] = value; }
  void setVtiObjectHiddenFlag( size_t i, bool value ) { vtiObjectHiddenFlag_[i] = value; }
  void setVtiObjectPriorityUpdateFlag( size_t i, bool value ) { vtiObjectPriorityUpdateFlag_[i] = value; }
  void setVtiObjectLabelUpdateFlag( size_t i, bool value ) { vtiObjectLabelUpdateFlag_[i] = value; }
  void setVtiObjectCollisionShapeUpdateFlag( size_t i, bool value ) { vtiObjectCollisionShapeUpdateFlag_[i] = value; }
  void setVtiObjectDependencyUpdateFlag( size_t i, bool value ) { vtiObjectDependencyUpdateFlag_[i] = value; }
  void setVtiBoundingBoxTop( size_t i, uint32_t value ) { vtiBoundingBoxTop_[i] = value; }
  void setVtiBoundingBoxLeft( size_t i, uint32_t value ) { vtiBoundingBoxLeft_[i] = value; }
  void setVtiBoundingBoxWidth( size_t i, uint32_t value ) { vtiBoundingBoxWidth_[i] = value; }
  void setVtiBoundingBoxHeight( size_t i, uint32_t value ) { vtiBoundingBoxHeight_[i] = value; }
  void setVti3dBoundingBoxX( size_t i, uint32_t value ) { vti3dBoundingBoxX_[i] = value; }
  void setVti3dBoundingBoxY( size_t i, uint32_t value ) { vti3dBoundingBoxY_[i] = value; }
  void setVti3dBoundingBoxZ( size_t i, uint32_t value ) { vti3dBoundingBoxZ_[i] = value; }
  void setVti3dBoundingBoxDeltaX( size_t i, uint32_t value ) { vti3dBoundingBoxDeltaX_[i] = value; }
  void setVti3dBoundingBoxDeltaY( size_t i, uint32_t value ) { vti3dBoundingBoxDeltaY_[i] = value; }
  void setVti3dBoundingBoxDeltaZ( size_t i, uint32_t value ) { vti3dBoundingBoxDeltaZ_[i] = value; }
  void setVtiObjectPriorityValue( size_t i, uint32_t value ) { vtiObjectPriorityValue_[i] = value; }
  void setVtiObjectLabelIdx( size_t i, uint32_t value ) { vtiObjectLabelIdx_[i] = value; }
  void setVtiObjectCollisionShapeId( size_t i, uint32_t value ) { vtiObjectCollisionShapeId_[i] = value; }
  void setVtiObjectNumDependencies( size_t i, uint32_t value ) { vtiObjectNumDependencies_[i] = value; }
  void setVtiObjectDependencyIdx( size_t i, size_t j, uint32_t value ) { vtiObjectDependencyIdx_[i][j] = value; }

 private:
  uint32_t                           vtiNumObjectUpdates_;
  std::vector<uint8_t>               vtiObjectIdx_;
  std::vector<bool>                  vtiObjectCancelFlag_;
  std::vector<bool>                  vtiBoundingBoxUpdateFlag_;
  std::vector<bool>                  vti3dBoundingBoxUpdateFlag_;
  std::vector<bool>                  vtiObjectHiddenFlag_;
  std::vector<bool>                  vtiObjectPriorityUpdateFlag_;
  std::vector<bool>                  vtiObjectLabelUpdateFlag_;
  std::vector<bool>                  vtiObjectCollisionShapeUpdateFlag_;
  std::vector<bool>                  vtiObjectDependencyUpdateFlag_;
  std::vector<uint32_t>              vtiBoundingBoxTop_;
  std::vector<uint32_t>              vtiBoundingBoxLeft_;
  std::vector<uint32_t>              vtiBoundingBoxWidth_;
  std::vector<uint32_t>              vtiBoundingBoxHeight_;
  std::vector<uint32_t>              vti3dBoundingBoxX_;
  std::vector<uint32_t>              vti3dBoundingBoxY_;
  std::vector<uint32_t>              vti3dBoundingBoxZ_;
  std::vector<uint32_t>              vti3dBoundingBoxDeltaX_;
  std::vector<uint32_t>              vti3dBoundingBoxDeltaY_;
  std::vector<uint32_t>              vti3dBoundingBoxDeltaZ_;
  std::vector<uint32_t>              vtiObjectPriorityValue_;
  std::vector<uint32_t>              vtiObjectLabelIdx_;
  std::vector<uint32_t>              vtiObjectCollisionShapeId_;
  std::vector<uint32_t>              vtiObjectNumDependencies_;
  std::vector<std::vector<uint32_t>> vtiObjectDependencyIdx_;
};

// E.2.14.1  General
class SEIVolumetricTilingInfo : public SEI {
 public:
  SEIVolumetricTilingInfo() :
      vtiCancelFlag_( false ),
      vtiObjectLabelPresentFlag_( false ),
      vti3dBoundingBoxPresentFlag_( false ),
      vtiObjectPriorityPresentFlag_( false ),
      vtiObjectHiddenPresentFlag_( false ),
      vtiObjectCollisionShapePresentFlag_( false ),
      vtiObjectDependencyPresentFlag_( false ),
      vtiBoundingBoxScaleLog2_( 0 ),
      vti3dBoundingBoxScaleLog2_( 0 ),
      vti3dBoundingBoxPrecisionMinus8_( 0 ) {}
  ~SEIVolumetricTilingInfo() {}
  SEIVolumetricTilingInfo& operator=( const SEIVolumetricTilingInfo& ) = default;

  SeiPayloadType getPayloadType() { return VOLUMETRIC_TILING_INFO; }

  bool    getVtiCancelFlag() { return vtiCancelFlag_; }
  bool    getVtiObjectLabelPresentFlag() { return vtiObjectLabelPresentFlag_; }
  bool    getVti3dBoundingBoxPresentFlag() { return vti3dBoundingBoxPresentFlag_; }
  bool    getVtiObjectPriorityPresentFlag() { return vtiObjectPriorityPresentFlag_; }
  bool    getVtiObjectHiddenPresentFlag() { return vtiObjectHiddenPresentFlag_; }
  bool    getVtiObjectCollisionShapePresentFlag() { return vtiObjectCollisionShapePresentFlag_; }
  bool    getVtiObjectDependencyPresentFlag() { return vtiObjectDependencyPresentFlag_; }
  uint8_t getVtiBoundingBoxScaleLog2() { return vtiBoundingBoxScaleLog2_; }
  uint8_t getVti3dBoundingBoxScaleLog2() { return vti3dBoundingBoxScaleLog2_; }
  uint8_t getVti3dBoundingBoxPrecisionMinus8() { return vti3dBoundingBoxPrecisionMinus8_; }
  void    setVtiCancelFlag( bool value ) { vtiCancelFlag_ = value; }
  void    setVtiObjectLabelPresentFlag( bool value ) { vtiObjectLabelPresentFlag_ = value; }
  void    setVti3dBoundingBoxPresentFlag( bool value ) { vti3dBoundingBoxPresentFlag_ = value; }
  void    setVtiObjectPriorityPresentFlag( bool value ) { vtiObjectPriorityPresentFlag_ = value; }
  void    setVtiObjectHiddenPresentFlag( bool value ) { vtiObjectHiddenPresentFlag_ = value; }
  void    setVtiObjectCollisionShapePresentFlag( bool value ) { vtiObjectCollisionShapePresentFlag_ = value; }
  void    setVtiObjectDependencyPresentFlag( bool value ) { vtiObjectDependencyPresentFlag_ = value; }
  void    setVtiBoundingBoxScaleLog2( uint8_t value ) { vtiBoundingBoxScaleLog2_ = value; }
  void    setVti3dBoundingBoxScaleLog2( uint8_t value ) { vti3dBoundingBoxScaleLog2_ = value; }
  void    setVti3dBoundingBoxPrecisionMinus8( uint8_t value ) { vti3dBoundingBoxPrecisionMinus8_ = value; }

  VolumetricTilingInfoLabels&  getVolumetricTilingInfoLabels() { return volumetricTilingInfoLabels_; }
  VolumetricTilingInfoObjects& getVolumetricTilingInfoObjects() { return volumetricTilingInfoObjects_; }

 private:
  bool                        vtiCancelFlag_;
  bool                        vtiObjectLabelPresentFlag_;
  bool                        vti3dBoundingBoxPresentFlag_;
  bool                        vtiObjectPriorityPresentFlag_;
  bool                        vtiObjectHiddenPresentFlag_;
  bool                        vtiObjectCollisionShapePresentFlag_;
  bool                        vtiObjectDependencyPresentFlag_;
  uint8_t                     vtiBoundingBoxScaleLog2_;
  uint8_t                     vti3dBoundingBoxScaleLog2_;
  uint8_t                     vti3dBoundingBoxPrecisionMinus8_;
  VolumetricTilingInfoLabels  volumetricTilingInfoLabels_;
  VolumetricTilingInfoObjects volumetricTilingInfoObjects_;
};

// E.2.15  Buffering period SEI message syntax
class SEIBufferingPeriod : public SEI {
 public:
  SEIBufferingPeriod() :
      bpIrapCabParamsPresentFlag_( false ),
      bpConcatenationFlag_( false ),
      bpAtlasSequenceParameterSetId_( 0 ),
      bpCabDelayOffset_( 0 ),
      bpDabDelayOffset_( 0 ),
      bpAtlasCabRemovalDelayDeltaMinus1_( 0 ),
      bpMaxSubLayersMinus1_( 0 ) {}
  ~SEIBufferingPeriod() {
    for ( auto& element : bpNalInitialCabRemovalDelay_ ) { element.clear(); }
    for ( auto& element : bpNalInitialCabRemovalOffset_ ) { element.clear(); }
    for ( auto& element : bpAclInitialCabRemovalDelay_ ) { element.clear(); }
    for ( auto& element : bpAclInitialCabRemovalOffset_ ) { element.clear(); }
    bpNalInitialCabRemovalDelay_.clear();
    bpNalInitialCabRemovalOffset_.clear();
    bpAclInitialCabRemovalDelay_.clear();
    bpAclInitialCabRemovalOffset_.clear();
    bpNalInitialAltCabRemovalDelay_.clear();
    bpNalInitialAltCabRemovalOffset_.clear();
    bpAclInitialAltCabRemovalDelay_.clear();
    bpAclInitialAltCabRemovalOffset_.clear();
  }
  SEIBufferingPeriod& operator=( const SEIBufferingPeriod& ) = default;

  SeiPayloadType getPayloadType() { return BUFFERING_PERIOD; }
  void           allocate() {
    bpNalInitialCabRemovalDelay_.resize( bpMaxSubLayersMinus1_ + 1 );
    bpNalInitialCabRemovalOffset_.resize( bpMaxSubLayersMinus1_ + 1 );
    bpAclInitialCabRemovalDelay_.resize( bpMaxSubLayersMinus1_ + 1 );
    bpAclInitialCabRemovalOffset_.resize( bpMaxSubLayersMinus1_ + 1 );
    bpNalInitialAltCabRemovalDelay_.resize( bpMaxSubLayersMinus1_ + 1 );
    bpNalInitialAltCabRemovalOffset_.resize( bpMaxSubLayersMinus1_ + 1 );
    bpAclInitialAltCabRemovalDelay_.resize( bpMaxSubLayersMinus1_ + 1 );
    bpAclInitialAltCabRemovalOffset_.resize( bpMaxSubLayersMinus1_ + 1 );
  }
  bool     getBpIrapCabParamsPresentFlag() { return bpIrapCabParamsPresentFlag_; }
  bool     getBpConcatenationFlag() { return bpConcatenationFlag_; }
  uint8_t  getBpAtlasSequenceParameterSetId() { return bpAtlasSequenceParameterSetId_; }
  uint32_t getBpCabDelayOffset() { return bpCabDelayOffset_; }
  uint32_t getBpDabDelayOffset() { return bpDabDelayOffset_; }
  uint32_t getBpAtlasCabRemovalDelayDeltaMinus1() { return bpAtlasCabRemovalDelayDeltaMinus1_; }
  uint32_t getBpMaxSubLayersMinus1() { return bpMaxSubLayersMinus1_; }
  std::vector<std::vector<uint32_t>>& getBpNalInitialCabRemovalDelay() { return bpNalInitialCabRemovalDelay_; }
  std::vector<std::vector<uint32_t>>& getBpNalInitialCabRemovalOffse() { return bpNalInitialCabRemovalOffset_; }
  std::vector<uint32_t>&              getBpNalInitialAltCabRemovalDelay() { return bpNalInitialAltCabRemovalDelay_; }
  std::vector<uint32_t>&              getBpNalInitialAltCabRemovalOffset() { return bpNalInitialAltCabRemovalOffset_; }
  std::vector<std::vector<uint32_t>>& getBpAclInitialCabRemovalDelay() { return bpAclInitialCabRemovalDelay_; }
  std::vector<std::vector<uint32_t>>& getBpAclInitialCabRemovalOffset() { return bpAclInitialCabRemovalOffset_; }
  std::vector<uint32_t>&              getBpAclInitialAltCabRemovalDelay() { return bpAclInitialAltCabRemovalDelay_; }
  std::vector<uint32_t>&              getBpAclInitialAltCabRemovalOffset() { return bpAclInitialAltCabRemovalOffset_; }
  std::vector<uint32_t>& getBpNalInitialCabRemovalDelay( size_t index ) { return bpNalInitialCabRemovalDelay_[index]; }
  std::vector<uint32_t>& getBpNalInitialCabRemovalOffsey( size_t index ) {
    return bpNalInitialCabRemovalOffset_[index];
  }
  std::vector<uint32_t>& getBpAclInitialCabRemovalDelay( size_t index ) { return bpAclInitialCabRemovalDelay_[index]; }
  std::vector<uint32_t>& getBpAclInitialCabRemovalOffset( size_t index ) {
    return bpAclInitialCabRemovalOffset_[index];
  }
  uint32_t getBpNalInitialAltCabRemovalDelay( size_t i ) { return bpNalInitialAltCabRemovalDelay_[i]; }
  uint32_t getBpNalInitialAltCabRemovalOffset( size_t i ) { return bpNalInitialAltCabRemovalOffset_[i]; }
  uint32_t getBpAclInitialAltCabRemovalDelay( size_t i ) { return bpAclInitialAltCabRemovalDelay_[i]; }
  uint32_t getBpAclInitialAltCabRemovalOffset( size_t i ) { return bpAclInitialAltCabRemovalOffset_[i]; }
  uint32_t getBpNalInitialCabRemovalDelay( size_t i, size_t j ) { return bpNalInitialCabRemovalDelay_[i][j]; }
  uint32_t getBpNalInitialCabRemovalOffset( size_t i, size_t j ) { return bpNalInitialCabRemovalOffset_[i][j]; }
  uint32_t getBpAclInitialCabRemovalDelay( size_t i, size_t j ) { return bpAclInitialCabRemovalDelay_[i][j]; }
  uint32_t getBpAclInitialCabRemovalOffset( size_t i, size_t j ) { return bpAclInitialCabRemovalOffset_[i][j]; }

  void setBpIrapCabParamsPresentFlag( bool value ) { bpIrapCabParamsPresentFlag_; }
  void setBpConcatenationFlag( bool value ) { bpConcatenationFlag_ = value; }
  void setBpAtlasSequenceParameterSetId( uint8_t value ) { bpAtlasSequenceParameterSetId_ = value; }
  void setBpCabDelayOffset( uint32_t value ) { bpCabDelayOffset_ = value; }
  void setBpDabDelayOffset( uint32_t value ) { bpDabDelayOffset_ = value; }
  void setBpAtlasCabRemovalDelayDeltaMinus1( uint32_t value ) { bpAtlasCabRemovalDelayDeltaMinus1_ = value; }
  void setBpMaxSubLayersMinus1( uint32_t value ) { bpMaxSubLayersMinus1_ = value; }
  void setBpNalInitialAltCabRemovalDelay( size_t i, uint32_t value ) { bpNalInitialAltCabRemovalDelay_[i] = value; }
  void setBpNalInitialAltCabRemovalOffset( size_t i, uint32_t value ) { bpNalInitialAltCabRemovalOffset_[i] = value; }
  void setBpAclInitialAltCabRemovalDelay( size_t i, uint32_t value ) { bpAclInitialAltCabRemovalDelay_[i] = value; }
  void setBpAclInitialAltCabRemovalOffset( size_t i, uint32_t value ) { bpAclInitialAltCabRemovalOffset_[i] = value; }
  void setBpNalInitialCabRemovalDelay( size_t i, size_t j, uint32_t value ) {
    bpNalInitialCabRemovalDelay_[i][j] = value;
  }
  void setBpNalInitialCabRemovalOffset( size_t i, size_t j, uint32_t value ) {
    bpNalInitialCabRemovalOffset_[i][j] = value;
  }
  void setBpAclInitialCabRemovalDelay( size_t i, size_t j, uint32_t value ) {
    bpAclInitialCabRemovalDelay_[i][j] = value;
  }
  void setBpAclInitialCabRemovalOffset( size_t i, size_t j, uint32_t value ) {
    bpAclInitialCabRemovalOffset_[i][j] = value;
  }

 private:
  bool                               bpIrapCabParamsPresentFlag_;
  bool                               bpConcatenationFlag_;
  uint8_t                            bpAtlasSequenceParameterSetId_;
  uint32_t                           bpCabDelayOffset_;
  uint32_t                           bpDabDelayOffset_;
  uint32_t                           bpAtlasCabRemovalDelayDeltaMinus1_;
  uint32_t                           bpMaxSubLayersMinus1_;
  std::vector<std::vector<uint32_t>> bpNalInitialCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> bpNalInitialCabRemovalOffset_;
  std::vector<uint32_t>              bpNalInitialAltCabRemovalDelay_;
  std::vector<uint32_t>              bpNalInitialAltCabRemovalOffset_;
  std::vector<std::vector<uint32_t>> bpAclInitialCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> bpAclInitialCabRemovalOffset_;
  std::vector<uint32_t>              bpAclInitialAltCabRemovalDelay_;
  std::vector<uint32_t>              bpAclInitialAltCabRemovalOffset_;
};

// E.2.16  Atlas frame timing SEI message syntax
class SEIAtlasFrameTiming : public SEI {
 public:
  SEIAtlasFrameTiming() : aftCabRemovalDelayMinus1_( 0 ), aftDabOutputDelay_( 0 ) {}
  ~SEIAtlasFrameTiming() {}
  SEIAtlasFrameTiming& operator=( const SEIAtlasFrameTiming& ) = default;

  SeiPayloadType getPayloadType() { return ATLAS_FRAME_TIMING; }

  uint32_t getAftCabRemovalDelayMinus1() { return aftCabRemovalDelayMinus1_; }
  uint32_t getAftDabOutputDelay() { return aftDabOutputDelay_; }

  void setAftCabRemovalDelayMinus1( uint32_t value ) { aftCabRemovalDelayMinus1_ = value; }
  void setAftDabOutputDelay( uint32_t value ) { aftDabOutputDelay_ = value; }

 private:
  uint32_t aftCabRemovalDelayMinus1_;
  uint32_t aftDabOutputDelay_;
};

// E.2.17  Presentation inforomation SEI message syntax
class SEIPresentationInformation : public SEI {
 public:
  SEIPresentationInformation() :
      piUnitOfLengthFlag_( false ),
      piOrientationPresentFlag_( false ),
      piPivotPresentFlag_( false ),
      piDimensionPresentFlag_( false ) {
    for ( size_t i = 0; i < 3; i++ ) {
      piUp_[i]        = 0;
      piFront_[i]     = 0;
      piPivot_[i]     = 0;
      piDimension_[i] = 0;
    }
  }
  ~SEIPresentationInformation() {}
  SEIPresentationInformation& operator=( const SEIPresentationInformation& ) = default;

  SeiPayloadType getPayloadType() { return PRESENTATION_INFORMATION; }

  bool    getPiUnitOfLengthFlag() { return piUnitOfLengthFlag_; }
  bool    getPiOrientationPresentFlag() { return piOrientationPresentFlag_; }
  bool    getPiPivotPresentFlag() { return piPivotPresentFlag_; }
  bool    getPiDimensionPresentFlag() { return piDimensionPresentFlag_; }
  int32_t getPiUp( size_t index ) { return piUp_[index]; }
  int32_t getPiFront( size_t index ) { return piFront_[index]; }
  int64_t getPiPivot( size_t index ) { return piPivot_[index]; }
  int64_t getPiDimension( size_t index ) { return piDimension_[index]; }
  void    setPiUnitOfLengthFlag( bool value ) { piUnitOfLengthFlag_ = value; }
  void    setPiOrientationPresentFlag( bool value ) { piOrientationPresentFlag_ = value; }
  void    setPiPivotPresentFlag( bool value ) { piPivotPresentFlag_ = value; }
  void    setPiDimensionPresentFlag( bool value ) { piDimensionPresentFlag_ = value; }
  void    setPiUp( size_t index, int32_t value ) { piUp_[index] = value; }
  void    setPiFront( size_t index, int32_t value ) { piFront_[index] = value; }
  void    setPiPivot( size_t index, int32_t value ) { piPivot_[index] = value; }
  void    setPiDimension( size_t index, int32_t value ) { piDimension_[index] = value; }

 private:
  bool    piUnitOfLengthFlag_;
  bool    piOrientationPresentFlag_;
  bool    piPivotPresentFlag_;
  bool    piDimensionPresentFlag_;
  int32_t piUp_[3];
  int32_t piFront_[3];
  int64_t piPivot_[3];
  int64_t piDimension_[3];
};

// E.2.18  Smoothing parameters SEI message syntax
class SEISmoothingParameters : public SEI {
 public:
  SEISmoothingParameters() :
      spGeometryCancelFlag_( true ),
      spAttributeCancelFlag_( true ),
      spGeometrySmoothingEnabledFlag_( false ),
      spGeometrySmoothingId_( 0 ),
      spGeometrySmoothingGridSizeMinus2_( 0 ),
      spGeometrySmoothingThreshold_( 0 ),
      spGeometryPatchBlockFilteringLog2ThresholdMinus1_( 0 ),
      spGeometryPatchBlockFilteringPassesCountMinus1_( 0 ),
      spGeometryPatchBlockFilteringFilterSizeMinus1_( 0 ),
      spNumAttributeUpdates_( 0 ) {}

  ~SEISmoothingParameters() {
    for ( auto& element : spAttrSmoothingParamsEnabledFlag_ ) { element.clear(); }
    for ( auto& element : spAttrSmoothingGridSizeMinus2_ ) { element.clear(); }
    for ( auto& element : spAttrSmoothingThreshold_ ) { element.clear(); }
    for ( auto& element : spAttrSmoothingLocalEntropyThreshold_ ) { element.clear(); }
    for ( auto& element : spAttrSmoothingThresholdVariation_ ) { element.clear(); }
    for ( auto& element : spAttrSmoothingThresholdDifference_ ) { element.clear(); }
    spAttributeIdx_.clear();
    spDimensionMinus1_.clear();
    spAttrSmoothingParamsEnabledFlag_.clear();
    spAttrSmoothingGridSizeMinus2_.clear();
    spAttrSmoothingThreshold_.clear();
    spAttrSmoothingLocalEntropyThreshold_.clear();
    spAttrSmoothingThresholdVariation_.clear();
    spAttrSmoothingThresholdDifference_.clear();
  }
  SEISmoothingParameters& operator=( const SEISmoothingParameters& ) = default;

  SeiPayloadType getPayloadType() { return SMOOTHING_PARAMETERS; }

  void allocate() { spAttributeIdx_.resize( spNumAttributeUpdates_ ); }
  void allocate( size_t size, size_t dimension ) {
    if ( spDimensionMinus1_.size() < size ) {
      spDimensionMinus1_.resize( size );
      spAttrSmoothingParamsEnabledFlag_.resize( size );
      spAttrSmoothingGridSizeMinus2_.resize( size );
      spAttrSmoothingThreshold_.resize( size );
      spAttrSmoothingLocalEntropyThreshold_.resize( size );
      spAttrSmoothingThresholdVariation_.resize( size );
      spAttrSmoothingThresholdDifference_.resize( size );
    }
    spAttrSmoothingParamsEnabledFlag_[size - 1].resize( dimension );
    spAttrSmoothingGridSizeMinus2_[size - 1].resize( dimension );
    spAttrSmoothingThreshold_[size - 1].resize( dimension );
    spAttrSmoothingLocalEntropyThreshold_[size - 1].resize( dimension );
    spAttrSmoothingThresholdVariation_[size - 1].resize( dimension );
    spAttrSmoothingThresholdDifference_[size - 1].resize( dimension );
  }
  bool    getSpGeometryCancelFlag() { return spGeometryCancelFlag_; }
  bool    getSpAttributeCancelFlag() { return spAttributeCancelFlag_; }
  bool    getSpGeometrySmoothingEnabledFlag() { return spGeometrySmoothingEnabledFlag_; }
  uint8_t getSpGeometrySmoothingId() { return spGeometrySmoothingId_; }
  uint8_t getSpGeometrySmoothingGridSizeMinus2() { return spGeometrySmoothingGridSizeMinus2_; }
  uint8_t getSpGeometrySmoothingThreshold() { return spGeometrySmoothingThreshold_; }
  uint8_t getSpGeometryPatchBlockFilteringLog2ThresholdMinus1() {
    return spGeometryPatchBlockFilteringLog2ThresholdMinus1_;
  }
  uint8_t getSpGeometryPatchBlockFilteringPassesCountMinus1() {
    return spGeometryPatchBlockFilteringPassesCountMinus1_;
  }
  uint8_t  getSpGeometryPatchBlockFilteringFilterSizeMinus1() { return spGeometryPatchBlockFilteringFilterSizeMinus1_; }
  uint32_t getSpNumAttributeUpdates() { return spNumAttributeUpdates_; }
  std::vector<uint32_t>&          getSpAttributeIdx() { return spAttributeIdx_; }
  std::vector<uint32_t>&          getSpDimensionMinus1() { return spDimensionMinus1_; }
  std::vector<std::vector<bool>>& getSpAttrSmoothingParamsEnabledFlag() { return spAttrSmoothingParamsEnabledFlag_; }
  std::vector<std::vector<uint32_t>>& getSpAttrSmoothingGridSizeMinus2() { return spAttrSmoothingGridSizeMinus2_; }
  std::vector<std::vector<uint32_t>>& getSpAttrSmoothingThreshold() { return spAttrSmoothingThreshold_; }
  std::vector<std::vector<uint32_t>>& getSpAttrSmoothingLocalEntropyThreshold() {
    return spAttrSmoothingLocalEntropyThreshold_;
  }
  std::vector<std::vector<uint32_t>>& getSpAttrSmoothingThresholdVariation() {
    return spAttrSmoothingThresholdVariation_;
  }
  std::vector<std::vector<uint32_t>>& getSpAttrSmoothingThresholdDifference() {
    return spAttrSmoothingThresholdDifference_;
  }

  uint32_t getSpAttributeIdx( size_t i ) { return spAttributeIdx_[i]; }
  uint32_t getSpDimensionMinus1( size_t i ) { return spDimensionMinus1_[i]; }
  bool     getSpAttrSmoothingParamsEnabledFlag( size_t i, size_t j ) { return spAttrSmoothingParamsEnabledFlag_[i][j]; }
  uint32_t getSpAttrSmoothingGridSizeMinus2( size_t i, size_t j ) { return spAttrSmoothingGridSizeMinus2_[i][j]; }
  uint32_t getSpAttrSmoothingThreshold( size_t i, size_t j ) { return spAttrSmoothingThreshold_[i][j]; }
  uint32_t getSpAttrSmoothingLocalEntropyThreshold( size_t i, size_t j ) {
    return spAttrSmoothingLocalEntropyThreshold_[i][j];
  }
  uint32_t getSpAttrSmoothingThresholdVariation( size_t i, size_t j ) {
    return spAttrSmoothingThresholdVariation_[i][j];
  }
  uint32_t getSpAttrSmoothingThresholdDifference( size_t i, size_t j ) {
    return spAttrSmoothingThresholdDifference_[i][j];
  }

  void setSpGeometryCancelFlag( bool value ) { spGeometryCancelFlag_ = value; }
  void setSpAttributeCancelFlag( bool value ) { spAttributeCancelFlag_ = value; }
  void setSpGeometrySmoothingEnabledFlag( bool value ) { spGeometrySmoothingEnabledFlag_ = value; }
  void setSpGeometrySmoothingId( uint8_t value ) { spGeometrySmoothingId_ = value; }
  void setSpGeometrySmoothingGridSizeMinus2( uint8_t value ) { spGeometrySmoothingGridSizeMinus2_ = value; }
  void setSpGeometrySmoothingThreshold( uint8_t value ) { spGeometrySmoothingThreshold_ = value; }
  void setSpNumAttributeUpdates( uint32_t value ) { spNumAttributeUpdates_ = value; }
  void setSpGeometryPatchBlockFilteringLog2ThresholdMinus1( uint8_t value ) {
    spGeometryPatchBlockFilteringLog2ThresholdMinus1_ = value;
  }
  void setSpGeometryPatchBlockFilteringPassesCountMinus1( uint8_t value ) {
    spGeometryPatchBlockFilteringPassesCountMinus1_ = value;
  }
  void setSpGeometryPatchBlockFilteringFilterSizeMinus1( uint8_t value ) {
    spGeometryPatchBlockFilteringFilterSizeMinus1_ = value;
  }
  void setSpAttributeIdx( size_t i, uint32_t value ) { spAttributeIdx_[i] = value; }
  void setSpDimensionMinus1( size_t i, uint32_t value ) { spDimensionMinus1_[i] = value; }
  void setSpAttrSmoothingParamsEnabledFlag( size_t i, size_t j, bool value ) {
    spAttrSmoothingParamsEnabledFlag_[i][j] = value;
  }
  void setSpAttrSmoothingGridSizeMinus2( size_t i, size_t j, uint32_t value ) {
    spAttrSmoothingGridSizeMinus2_[i][j] = value;
  }
  void setSpAttrSmoothingThreshold( size_t i, size_t j, uint32_t value ) { spAttrSmoothingThreshold_[i][j] = value; }
  void setSpAttrSmoothingLocalEntropyThreshold( size_t i, size_t j, uint32_t value ) {
    spAttrSmoothingLocalEntropyThreshold_[i][j] = value;
  }
  void setSpAttrSmoothingThresholdVariation( size_t i, size_t j, uint32_t value ) {
    spAttrSmoothingThresholdVariation_[i][j] = value;
  }
  void setSpAttrSmoothingThresholdDifference( size_t i, size_t j, uint32_t value ) {
    spAttrSmoothingThresholdDifference_[i][j] = value;
  }

 private:
  bool    spGeometryCancelFlag_;
  bool    spAttributeCancelFlag_;
  bool    spGeometrySmoothingEnabledFlag_;
  uint8_t spGeometrySmoothingGridSizeMinus2_;
  uint8_t spGeometrySmoothingThreshold_;
  uint8_t spGeometrySmoothingId_;
  uint8_t spGeometryPatchBlockFilteringLog2ThresholdMinus1_;
  uint8_t spGeometryPatchBlockFilteringPassesCountMinus1_;
  uint8_t spGeometryPatchBlockFilteringFilterSizeMinus1_;

  uint32_t                           spNumAttributeUpdates_;
  std::vector<uint32_t>              spAttributeIdx_;
  std::vector<uint32_t>              spDimensionMinus1_;
  std::vector<std::vector<bool>>     spAttrSmoothingParamsEnabledFlag_;
  std::vector<std::vector<uint32_t>> spAttrSmoothingGridSizeMinus2_;
  std::vector<std::vector<uint32_t>> spAttrSmoothingThreshold_;
  std::vector<std::vector<uint32_t>> spAttrSmoothingLocalEntropyThreshold_;
  std::vector<std::vector<uint32_t>> spAttrSmoothingThresholdVariation_;
  std::vector<std::vector<uint32_t>> spAttrSmoothingThresholdDifference_;
};

// F.2  VUI syntax
// F.2.3  Sub-layer HRD parameters syntax
class HrdSubLayerParameters {
 public:
  HrdSubLayerParameters() {}
  ~HrdSubLayerParameters() {
    hrdBitRateValueMinus1_.clear();
    hrdCabSizeValueMinus1_.clear();
    hrdCbrFlag_.clear();
  }
  HrdSubLayerParameters& operator=( const HrdSubLayerParameters& ) = default;
  void                   allocate( size_t size ) {
    hrdBitRateValueMinus1_.resize( size, 0 );
    hrdCabSizeValueMinus1_.resize( size, 0 );
    hrdCbrFlag_.resize( size, false );
  }
  size_t   size() { return hrdBitRateValueMinus1_.size(); }
  bool     getHrdCbrFlag( size_t index ) { return hrdCbrFlag_[index]; }
  uint32_t getHrdBitRateValueMinus1( size_t index ) { return hrdBitRateValueMinus1_[index]; }
  uint32_t getHrdCabSizeValueMinus1( size_t index ) { return hrdCabSizeValueMinus1_[index]; }
  void     setHrdCbrFlag( size_t index, bool value ) { hrdCbrFlag_[index] = value; }
  void     setHrdBitRateValueMinus1( size_t index, uint32_t value ) { hrdBitRateValueMinus1_[index] = value; }
  void     setHrdCabSizeValueMinus1( size_t index, uint32_t value ) { hrdCabSizeValueMinus1_[index] = value; }

 private:
  std::vector<uint32_t> hrdBitRateValueMinus1_;
  std::vector<uint32_t> hrdCabSizeValueMinus1_;
  std::vector<bool>     hrdCbrFlag_;
};

// F.2.2  HRD parameters syntax
class HrdParameters {
 public:
  HrdParameters() :
      hrdNalParametersPresentFlag_( 0 ),
      hrdAclParametersPresentFlag_( 0 ),
      hrdBitRateScale_( 0 ),
      hrdCabSizeScale_( 0 ),
      hrdInitialCabRemovalDelayLengthMinus1_( 0 ),
      hrdAuCabRemovalDelayLengthMinus1_( 0 ),
      hrdDabOutputDelayLengthMinus1_( 0 ) {
    for ( size_t i = 0; i <= maxNumSubLayersMinus1_; i++ ) {
      hrdFixedAtlasRateGeneralFlag_[i]   = 0;
      hrdFixedAtlasRateWithinCasFlag_[i] = 0;
      hrdLowDelayFlag[i]                 = 0;
    }
  }
  ~HrdParameters() {}
  HrdParameters& operator=( const HrdParameters& ) = default;

  uint8_t getMaxNumSubLayersMinus1() { return maxNumSubLayersMinus1_; }
  bool    getHrdNalParametersPresentFlag() { return hrdNalParametersPresentFlag_; }
  bool    getHrdAclParametersPresentFlag() { return hrdAclParametersPresentFlag_; }
  uint8_t getHrdBitRateScale() { return hrdBitRateScale_; }
  uint8_t getHrdCabSizeScale() { return hrdCabSizeScale_; }
  uint8_t getHrdInitialCabRemovalDelayLengthMinus1() { return hrdInitialCabRemovalDelayLengthMinus1_; }
  uint8_t getHrdAuCabRemovalDelayLengthMinus1() { return hrdAuCabRemovalDelayLengthMinus1_; }
  uint8_t getHrdDabOutputDelayLengthMinus1() { return hrdDabOutputDelayLengthMinus1_; }
  bool    getHrdFixedAtlasRateGeneralFlag( size_t index ) { return hrdFixedAtlasRateGeneralFlag_[index]; }
  bool    getHrdFixedAtlasRateWithinCasFlag( size_t index ) { return hrdFixedAtlasRateWithinCasFlag_[index]; }
  bool    getHrdLowDelayFlag( size_t index ) { return hrdLowDelayFlag[index]; }
  bool    getHrdElementalDurationInTcMinus1( size_t index ) { return hrdFixedAtlasRateWithinCasFlag_[index]; }
  bool    getHrdCabCntMinus1( size_t index ) { return hrdLowDelayFlag[index]; }

  HrdSubLayerParameters& getHrdSubLayerParameters( size_t i, size_t j ) { return hrdSubLayerParameters_[i][j]; }

  void setHrdNalParametersPresentFlag( bool value ) { hrdNalParametersPresentFlag_ = value; }
  void setHrdAclParametersPresentFlag( bool value ) { hrdAclParametersPresentFlag_ = value; }
  void setHrdBitRateScale( uint8_t value ) { hrdBitRateScale_ = value; }
  void setHrdCabSizeScale( uint8_t value ) { hrdCabSizeScale_ = value; }
  void setHrdInitialCabRemovalDelayLengthMinus1( uint8_t value ) { hrdInitialCabRemovalDelayLengthMinus1_ = value; }
  void setHrdAuCabRemovalDelayLengthMinus1( uint8_t value ) { hrdAuCabRemovalDelayLengthMinus1_ = value; }
  void setHrdDabOutputDelayLengthMinus1( uint8_t value ) { hrdDabOutputDelayLengthMinus1_ = value; }
  void setHrdFixedAtlasRateGeneralFlag( size_t index, bool value ) { hrdFixedAtlasRateGeneralFlag_[index] = value; }
  void setHrdFixedAtlasRateWithinCasFlag( size_t index, bool value ) { hrdFixedAtlasRateWithinCasFlag_[index] = value; }
  void setHrdLowDelayFlag( size_t index, bool value ) { hrdLowDelayFlag[index] = value; }
  void setHrdElementalDurationInTcMinus1( size_t index, uint32_t value ) {
    hrdFixedAtlasRateWithinCasFlag_[index] = value;
  }
  void setHrdCabCntMinus1( size_t index, uint32_t value ) { hrdLowDelayFlag[index] = value; }

 private:
  static const uint8_t  maxNumSubLayersMinus1_ = 0;
  bool                  hrdNalParametersPresentFlag_;
  bool                  hrdAclParametersPresentFlag_;
  uint8_t               hrdBitRateScale_;
  uint8_t               hrdCabSizeScale_;
  uint8_t               hrdInitialCabRemovalDelayLengthMinus1_;
  uint8_t               hrdAuCabRemovalDelayLengthMinus1_;
  uint8_t               hrdDabOutputDelayLengthMinus1_;
  bool                  hrdFixedAtlasRateGeneralFlag_[maxNumSubLayersMinus1_ + 1];
  bool                  hrdFixedAtlasRateWithinCasFlag_[maxNumSubLayersMinus1_ + 1];
  bool                  hrdLowDelayFlag[maxNumSubLayersMinus1_ + 1];
  uint32_t              hrdElementalDurationInTcMinus1_;
  uint32_t              hrdCabCntMinus1_;
  HrdSubLayerParameters hrdSubLayerParameters_[2][maxNumSubLayersMinus1_ + 1];
};

// F.2.1  VUI parameters syntax
class VUIParameters {
 public:
  VUIParameters() :
      vuiTimingInfoPresentFlag_( false ),
      vuiPocProportionalToTimingFlag_( false ),
      vuiHrdParametersPresentFlag_( false ),
      vuiNumUnitsInTick_( 1001 ),
      vuiTimeScale_( 60000 ),
      vuiNumTicksPocDiffOneMinus1_( 0 ) {}
  ~VUIParameters() {}
  VUIParameters& operator=( const VUIParameters& ) = default;

  bool           getVuiTimingInfoPresentFlag() { return vuiTimingInfoPresentFlag_; }
  bool           getVuiPocProportionalToTimingFlag() { return vuiPocProportionalToTimingFlag_; }
  bool           getVuiHrdParametersPresentFlag() { return vuiHrdParametersPresentFlag_; }
  uint32_t       getVuiNumUnitsInTick() { return vuiNumUnitsInTick_; }
  uint32_t       getVuiTimeScale() { return vuiTimeScale_; }
  uint32_t       getVuiNumTicksPocDiffOneMinus1() { return vuiNumTicksPocDiffOneMinus1_; }
  HrdParameters& getHrdParameters() { return hrdParameters_; }

  void setVuiTimingInfoPresentFlag( bool value ) { vuiTimingInfoPresentFlag_ = value; }
  void setVuiPocProportionalToTimingFlag( bool value ) { vuiPocProportionalToTimingFlag_ = value; }
  void setVuiHrdParametersPresentFlag( bool value ) { vuiHrdParametersPresentFlag_ = value; }
  void setVuiNumUnitsInTick( uint32_t value ) { vuiNumUnitsInTick_ = value; }
  void setVuiTimeScale( uint32_t value ) { vuiTimeScale_ = value; }
  void setVuiNumTicksPocDiffOneMinus1( bool value ) { vuiNumTicksPocDiffOneMinus1_ = value; }

 private:
  bool          vuiTimingInfoPresentFlag_;
  bool          vuiPocProportionalToTimingFlag_;
  bool          vuiHrdParametersPresentFlag_;
  uint32_t      vuiNumUnitsInTick_;
  uint32_t      vuiTimeScale_;
  uint32_t      vuiNumTicksPocDiffOneMinus1_;
  HrdParameters hrdParameters_;
};

// 7.3.2 V-PCC unit syntax
class VpccUnit {
 public:
  VpccUnit() {}
  ~VpccUnit() {
#if !VPCCUNIT_DATA_BITSTREAM
    vpccUnitData_.clear();
#endif
  }

  void setVpccUnitSize( size_t value ) { vpccUnitSize_ = value; }
#if VPCCUNIT_DATA_BITSTREAM
  void setVpccUnitSize() { vpccUnitSize_ = vpccUnitDataBitstream_.size(); }
#endif
  size_t getVpccUnitSize() { return vpccUnitSize_; }

#if VPCCUNIT_DATA_BITSTREAM
  void setVpccUnitDataBitstream( PCCBitstream&& bitstream, VPCCUnitType unitType ) {
    vpccUnitDataBitstream_ = std::move(bitstream);
    vpccUnitSize_          = bitstream.size();
    vpccUnitType_          = unitType;
  }
  PCCBitstream& getVpccUnitDataBitstream() {
    vpccUnitDataBitstream_.beginning();
    return vpccUnitDataBitstream_;
  }
  void allocate() { vpccUnitDataBitstream_.initialize( vpccUnitSize_ ); }
#else
  void                  setVpccUnitData( size_t index, uint8_t data ) { vpccUnitData_[index] = data; }
  uint8_t               getVpccUnitData( size_t index ) { return vpccUnitData_[index]; }
  std::vector<uint8_t>& getVpccUnitData() { return vpccUnitData_; }

  void allocate() { vpccUnitData_.resize( vpccUnitSize_, 0 ); }
  void initialize( size_t size, uint8_t* data, VPCCUnitType unitType ) {
    vpccUnitSize_ = size;
    if ( size > 0 ) {
      vpccUnitData_.resize( size, 0 );
      memcpy( vpccUnitData_.data(), data, size * sizeof( uint8_t ) );
    }
    vpccUnitType_ = unitType;
  }

#endif
  void         getVpccUnitHeader() {}  // read 32 bits
  void         getVpccUnitPayload() {}
  VPCCUnitType getVpccUnitType() { return vpccUnitType_; }

  void setVpccUnitType( size_t value ) { vpccUnitType_ = (VPCCUnitType)value; }
  void setVpccUnitType( VPCCUnitType value ) { vpccUnitType_ = value; }

 private:
  VPCCUnitType vpccUnitType_;  // jkei: just for humane reason..
  size_t       vpccUnitSize_;
#if VPCCUNIT_DATA_BITSTREAM
  PCCBitstream vpccUnitDataBitstream_;
#else
  std::vector<uint8_t> vpccUnitData_;  // jkei: do we really need to do like this? ;o;
#endif
};

// 7.3.5 NAL unit syntax
class NalUnit {
 public:
  NalUnit( NalUnitType nalUnitType = NAL_TRAIL, uint8_t layerId = 0, uint8_t temporalyIdPlus1 = 0 ) :
      nalUnitType_( nalUnitType ),
      layerId_( layerId ),
      temporalyIdPlus1_( temporalyIdPlus1 ),
      nalUnitSize_( 0 ) {}
  ~NalUnit() { nalUnitData_.clear(); }
  void                  allocate() { nalUnitData_.resize( nalUnitSize_ - 2, 0 ); }
  NalUnitType           getNalUnitType() { return nalUnitType_; }
  uint8_t               getLayerId() { return layerId_; }
  uint8_t               getTemporalyIdPlus1() { return temporalyIdPlus1_; }
  size_t                getNalUnitSize() { return nalUnitSize_; }
  uint8_t               getNalUnitData( size_t index ) { return nalUnitData_[index]; }
  std::vector<uint8_t>& getNalUnitData() { return nalUnitData_; }

  void setNalUnitType( NalUnitType value ) { nalUnitType_ = value; }
  void setLayerId( uint8_t value ) { layerId_ = value; }
  void setTemporalyIdPlus1( uint8_t value ) { temporalyIdPlus1_ = value; }
  void setNalUnitSize( size_t value ) { nalUnitSize_ = value; }
  void setNalUnitData( size_t index, uint8_t data ) { nalUnitData_[index] = data; }

  void initialize( NalUnitType nalUnitType, uint8_t layerId, uint8_t temporalyIdPlus1 ) {
    nalUnitType_      = nalUnitType;
    layerId_          = layerId;
    temporalyIdPlus1_ = temporalyIdPlus1;
  }
  void initialize( NalUnitType nalUnitType, uint8_t layerId, uint8_t temporalyIdPlus1, size_t size, uint8_t* data ) {
    nalUnitType_      = nalUnitType;
    layerId_          = layerId;
    temporalyIdPlus1_ = temporalyIdPlus1;
    nalUnitSize_      = size;
    if ( size > 0 ) {
      nalUnitData_.resize( size, 0 );
      memcpy( nalUnitData_.data(), data, size * sizeof( uint8_t ) );
    }
  }

 private:
  NalUnitType          nalUnitType_;
  uint8_t              layerId_;
  uint8_t              temporalyIdPlus1_;
  size_t               nalUnitSize_;
  std::vector<uint8_t> nalUnitData_;
};

// C.2 Sample stream NAL unit syntax and semantics
class SampleStreamNalUnit {
 public:
  SampleStreamNalUnit() : unitSizePrecisionBytesMinus1_( 0 ) {}
  ~SampleStreamNalUnit() { nalUnit_.clear(); }
  NalUnit& addNalUnit() {
    nalUnit_.resize( nalUnit_.size() + 1 );
    return nalUnit_.back();
  }

  void                  popFront() { nalUnit_.erase( nalUnit_.begin(), nalUnit_.begin() + 1 ); }
  NalUnit&              front() { return *( nalUnit_.begin() ); }
  std::vector<NalUnit>& getNalUnit() { return nalUnit_; }
  NalUnit&              getNalUnit( size_t index ) { return nalUnit_[index]; }
  size_t                getNalUnitCount() { return nalUnit_.size(); }
  uint8_t               getUnitSizePrecisionBytesMinus1() { return unitSizePrecisionBytesMinus1_; }
  void                  setUnitSizePrecisionBytesMinus1( uint8_t value ) { unitSizePrecisionBytesMinus1_ = value; }
  NalUnit&              getLastNalUnit() { return nalUnit_.back(); }

  // NalUnit& getNalUnit() { return nalUnit_; }
  // void     getNalUnit( NalUnit nalu ) { nalUnit_ = nalu; }  // jkei: doesn't it work?

 private:
  uint8_t              unitSizePrecisionBytesMinus1_;
  std::vector<NalUnit> nalUnit_;  // jkei: let's remove this since one nalUnit per one sampleStreamNalUnit??
  // NalUnit            nalUnit_;
};

// B.2 Sample stream V-PCC unit syntax and semantics
// 6.1 V-PCC bistreams format (previously)
// jkei: lets do something to match this one to the spec. otherwise I don't know how to read NAL_Atlas group layer
// repeatly...
class SampleStreamVpccUnit {
 public:
  SampleStreamVpccUnit() : ssvhUnitSizePrecisionBytesMinus1_( 0 ) {}
  ~SampleStreamVpccUnit() { vpccUnits_.clear(); }
  VpccUnit& addVpccUnit() {
    vpccUnits_.resize( vpccUnits_.size() + 1 );
    return vpccUnits_.back();
  }
  void                   popFront() { vpccUnits_.erase( vpccUnits_.begin(), vpccUnits_.begin() + 1 ); }
  VpccUnit&              front() { return *( vpccUnits_.begin() ); }
  std::vector<VpccUnit>& getVpccUnit() { return vpccUnits_; }
  size_t                 getVpccUnitCount() { return vpccUnits_.size(); }
  VpccUnit&              getLastVpccUnit() { return vpccUnits_.back(); }
  uint32_t               getSsvhUnitSizePrecisionBytesMinus1() { return ssvhUnitSizePrecisionBytesMinus1_; }
  void setSsvhUnitSizePrecisionBytesMinus1( uint32_t value ) { ssvhUnitSizePrecisionBytesMinus1_ = value; }

 private:
  std::vector<VpccUnit> vpccUnits_;  // jkei: really need to be list?
  uint32_t              ssvhUnitSizePrecisionBytesMinus1_;
};

// // 7.3.8	Supplemental enhancement information message syntax
// class SEIMessage {
//  public:
//   SEIMessage() : payloadTypeByte_( 0 ), payloadSizeByte_( 0 ) { sei_.clear(); }
//   ~SEIMessage() { sei_.clear(); }
//   SEIMessage& operator=( const SEIMessage& ) = default;
//   uint8_t     getPayloadTypeByte() { return payloadTypeByte_; }
//   uint8_t     getPayloadSizeByte() { return payloadSizeByte_; }
//   void        setPayloadTypeByte( uint8_t value ) { payloadTypeByte_ = value; }
//   void        setPayloadSizeByte( uint8_t value ) { payloadSizeByte_ = value; }
//   SEI& getSEI(){ return sei_;}
//  private:
//   uint8_t payloadTypeByte_;
//   uint8_t payloadSizeByte_;
//   SEI     sei_;
// };

// 7.3.7.9 Point local reconstruction data syntax : jkei: need to be updated!
class PointLocalReconstructionData {
 public:
  PointLocalReconstructionData() :
      blockToPatchMapHeight_( 0 ),
      blockToPatchMapWidth_( 0 ),
      levelFlag_( 0 ),
      presentFlag_( false ),
      modeMinus1_( 0 ) {
    blockPresentFlag_.clear();
    blockModeMinus1_.clear();
  };
  ~PointLocalReconstructionData() {
    blockPresentFlag_.clear();
    blockModeMinus1_.clear();
  };

  PointLocalReconstructionData& operator=( const PointLocalReconstructionData& ) = default;

  void allocate( size_t blockToPatchMapWidth, size_t blockToPatchMapHeight ) {
    blockToPatchMapWidth_  = blockToPatchMapWidth;
    blockToPatchMapHeight_ = blockToPatchMapHeight;
    blockPresentFlag_.resize( blockToPatchMapWidth_ * blockToPatchMapHeight_, false );
    blockModeMinus1_.resize( blockToPatchMapWidth_ * blockToPatchMapHeight_, 0 );
  }
  size_t  getBlockToPatchMapHeight() { return blockToPatchMapHeight_; }
  size_t  getBlockToPatchMapWidth() { return blockToPatchMapWidth_; }
  bool    getLevelFlag() { return levelFlag_; }
  bool    getPresentFlag() { return presentFlag_; }
  uint8_t getModeMinus1() { return modeMinus1_; }
  bool    getBlockPresentFlag( size_t index ) { return blockPresentFlag_[index]; }
  uint8_t getBlockModeMinus1( size_t index ) { return blockModeMinus1_[index]; }

  void setLevelFlag( bool value ) { levelFlag_ = value; }
  void setPresentFlag( bool value ) { presentFlag_ = value; }
  void setModeMinus1( uint8_t value ) { modeMinus1_ = value; }
  void setBlockPresentFlag( size_t index, bool value ) { blockPresentFlag_[index] = value; }
  void setBlockModeMinus1( size_t index, uint8_t value ) { blockModeMinus1_[index] = value; }

 private:
  size_t               blockToPatchMapHeight_;
  size_t               blockToPatchMapWidth_;
  bool                 levelFlag_;
  bool                 presentFlag_;
  uint8_t              modeMinus1_;
  std::vector<bool>    blockPresentFlag_;
  std::vector<uint8_t> blockModeMinus1_;
};

// jkei : Here to ln1420(AtlasFrameParameterSetRbsp) updated (please check PST.oct29th)
// 7.3.7.8  EOM patch data unit syntax
class EOMPatchDataUnit {  // jkei: EOM or Eom?
 public:
  EOMPatchDataUnit() :
      epdu2dPosX_( 0 ),
      epdu2dPosY_( 0 ),
      epdu2dDeltaSizeX_( 0 ),
      epdu2dDeltaSizeY_( 0 ),
      epduAssociatedPatcheCountMinus1_( 0 ),
      epduPatchIndex_( 0 ),
      epduFrameIndex_( 0 ) {
    epduAssociatedPatches_.clear();
    epduEomPointsPerPatch_.clear();
  };
  ~EOMPatchDataUnit(){};
  EOMPatchDataUnit& operator=( const EOMPatchDataUnit& ) = default;

  size_t               getEpdu2dPosX() { return epdu2dPosX_; }
  size_t               getEpdu2dPosY() { return epdu2dPosY_; }
  int64_t              getEpdu2dDeltaSizeX() { return epdu2dDeltaSizeX_; }
  int64_t              getEpdu2dDeltaSizeY() { return epdu2dDeltaSizeY_; }
  int64_t              getEpduAssociatedPatchesCountMinus1() { return epduAssociatedPatcheCountMinus1_; }
  std::vector<size_t>& getEpduAssociatedPatches() { return epduAssociatedPatches_; }
  size_t               getEpduAssociatedPatches( size_t index ) { return epduAssociatedPatches_[index]; }
  std::vector<size_t>  getEpduEomPointsPerPatch() { return epduEomPointsPerPatch_; }
  size_t               getEpduEomPointsPerPatch( size_t index ) { return epduEomPointsPerPatch_[index]; }
  size_t               getPatchIndex() { return epduPatchIndex_; }
  size_t               getFrameIndex() { return epduFrameIndex_; }
  void                 setPatchIndex( size_t value ) { epduPatchIndex_ = value; }
  void                 setFrameIndex( size_t value ) { epduFrameIndex_ = value; }

  void setEpduAssociatedPatchesCountMinus1( uint32_t value ) {
    epduAssociatedPatcheCountMinus1_ = value;
    epduAssociatedPatches_.resize( epduAssociatedPatcheCountMinus1_ + 1 );
    epduEomPointsPerPatch_.resize( epduAssociatedPatcheCountMinus1_ + 1 );
  }
  void setEpduAssociatedPatches( size_t value, size_t index ) { epduAssociatedPatches_[index] = value; }
  void setEpduEomPointsPerPatch( size_t value, size_t index ) { epduEomPointsPerPatch_[index] = value; }

  void setEpdu2dPosX( size_t value ) { epdu2dPosX_ = value; }
  void setEpdu2dPosY( size_t value ) { epdu2dPosY_ = value; }
  void setEpdu2dDeltaSizeX( int64_t value ) { epdu2dDeltaSizeX_ = value; }
  void setEpdu2dDeltaSizeY( int64_t value ) { epdu2dDeltaSizeY_ = value; }

 private:
  size_t              epdu2dPosX_;
  size_t              epdu2dPosY_;
  int64_t             epdu2dDeltaSizeX_;
  int64_t             epdu2dDeltaSizeY_;
  size_t              epduAssociatedPatcheCountMinus1_;
  size_t              epduPatchIndex_;
  size_t              epduFrameIndex_;
  std::vector<size_t> epduAssociatedPatches_;
  std::vector<size_t> epduEomPointsPerPatch_;
};

// 7.3.7.7  raw patch data unit syntax (jkei:updated)
class RawPatchDataUnit {
 public:
  RawPatchDataUnit() :
      rpduPatchInRawVideoFlag_( false ),
      rpdu2dPosX_( 0 ),
      rpdu2dPosY_( 0 ),
      rpdu2dDeltaSizeX_( 0 ),
      rpdu2dDeltaSizeY_( 0 ),
      rpdu3dPosX_( 0 ),
      rpdu3dPosY_( 0 ),
      rpdu3dPosZ_( 0 ),
      rpduRawPoints_( 0 ),
      rpduPatchIndex_( 0 ),
      rpduFrameIndex_( 0 ) {}
  ~RawPatchDataUnit() {}
  RawPatchDataUnit& operator=( const RawPatchDataUnit& ) = default;

  bool     getRpduPatchInRawVideoFlag() { return rpduPatchInRawVideoFlag_; }
  size_t   getRpdu2dPosX() { return rpdu2dPosX_; }
  size_t   getRpdu2dPosY() { return rpdu2dPosY_; }
  int64_t  getRpdu2dDeltaSizeX() { return rpdu2dDeltaSizeX_; }
  int64_t  getRpdu2dDeltaSizeY() { return rpdu2dDeltaSizeY_; }
  size_t   getRpdu3dPosX() { return rpdu3dPosX_; }
  size_t   getRpdu3dPosY() { return rpdu3dPosY_; }
  size_t   getRpdu3dPosZ() { return rpdu3dPosZ_; }
  uint32_t getRpduRawPoints() { return rpduRawPoints_; }
  size_t   getPatchIndex() { return rpduPatchIndex_; }
  size_t   getFrameIndex() { return rpduFrameIndex_; }
  void     setPatchIndex( size_t value ) { rpduPatchIndex_ = value; }
  void     setFrameIndex( size_t value ) { rpduFrameIndex_ = value; }
  void     setRpduPatchInRawVideoFlag( bool value ) { rpduPatchInRawVideoFlag_ = value; }
  void     setRpdu2dPosX( size_t value ) { rpdu2dPosX_ = value; }
  void     setRpdu2dPosY( size_t value ) { rpdu2dPosY_ = value; }
  void     setRpdu2dDeltaSizeX( int64_t value ) { rpdu2dDeltaSizeX_ = value; }
  void     setRpdu2dDeltaSizeY( int64_t value ) { rpdu2dDeltaSizeY_ = value; }
  void     setRpdu3dPosX( size_t value ) { rpdu3dPosX_ = value; }
  void     setRpdu3dPosY( size_t value ) { rpdu3dPosY_ = value; }
  void     setRpdu3dPosZ( size_t value ) { rpdu3dPosZ_ = value; }
  void     setRpduRawPoints( uint32_t value ) { rpduRawPoints_ = value; }

 private:
  bool     rpduPatchInRawVideoFlag_;
  size_t   rpdu2dPosX_;
  size_t   rpdu2dPosY_;
  int64_t  rpdu2dDeltaSizeX_;
  int64_t  rpdu2dDeltaSizeY_;
  size_t   rpdu3dPosX_;
  size_t   rpdu3dPosY_;
  size_t   rpdu3dPosZ_;
  uint32_t rpduRawPoints_;
  size_t   rpduPatchIndex_;
  size_t   rpduFrameIndex_;
};

// 7.3.7.6  Inter patch data unit syntax(jkei: newly added)
class InterPatchDataUnit {
 public:
  InterPatchDataUnit() :
      ipduRefIndex_( 0 ),
      ipduRefpatchIndex_( 0 ),
      ipdu2dPosX_( 0 ),
      ipdu2dPosY_( 0 ),
      ipdu2dDeltaSizeX_( 0 ),
      ipdu2dDeltaSizeY_( 0 ),
      ipdu3dPosX_( 0 ),
      ipdu3dPosY_( 0 ),
      ipdu3dPosMinZ_( 0 ),
      ipdu3dPosDeltaMaxZ_( 0 ),
      ipduPatchIndex_( 0 ),
      ipduFrameIndex_( 0 ){};
  ~InterPatchDataUnit(){};
  InterPatchDataUnit& operator=( const InterPatchDataUnit& ) = default;

  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }
  int64_t                       getIpduRefIndex() { return ipduRefIndex_; }
  int64_t                       getIpduRefPatchIndex() { return ipduRefpatchIndex_; }
  int64_t                       getIpdu2dPosX() { return ipdu2dPosX_; }
  int64_t                       getIpdu2dPosY() { return ipdu2dPosY_; }
  int64_t                       getIpdu2dDeltaSizeX() { return ipdu2dDeltaSizeX_; }
  int64_t                       getIpdu2dDeltaSizeY() { return ipdu2dDeltaSizeY_; }
  int64_t                       getIpdu3dPosX() { return ipdu3dPosX_; }
  int64_t                       getIpdu3dPosY() { return ipdu3dPosY_; }
  int64_t                       getIpdu3dPosMinZ() { return ipdu3dPosMinZ_; }
  int64_t                       getIpdu3dPosDeltaMaxZ() { return ipdu3dPosDeltaMaxZ_; }

  void setIpduRefIndex( int64_t value ) { ipduRefIndex_ = value; }
  void setIpduRefPatchIndex( int64_t value ) { ipduRefpatchIndex_ = value; }
  void setIpdu2dPosX( int64_t value ) { ipdu2dPosX_ = value; }
  void setIpdu2dPosY( int64_t value ) { ipdu2dPosY_ = value; }
  void setIpdu2dDeltaSizeX( int64_t value ) { ipdu2dDeltaSizeX_ = value; }
  void setIpdu2dDeltaSizeY( int64_t value ) { ipdu2dDeltaSizeY_ = value; }
  void setIpdu3dPosX( int64_t value ) { ipdu3dPosX_ = value; }
  void setIpdu3dPosY( int64_t value ) { ipdu3dPosY_ = value; }
  void setIpdu3dPosMinZ( int64_t value ) { ipdu3dPosMinZ_ = value; }
  void setIpdu3dPosDeltaMaxZ( int64_t value ) { ipdu3dPosDeltaMaxZ_ = value; }

  size_t getPatchIndex() { return ipduPatchIndex_; }
  size_t getFrameIndex() { return ipduFrameIndex_; }
  void   setPatchIndex( size_t value ) { ipduPatchIndex_ = value; }
  void   setFrameIndex( size_t value ) { ipduFrameIndex_ = value; }

  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }

 private:
  int64_t                      ipduRefIndex_;
  int64_t                      ipduRefpatchIndex_;
  int64_t                      ipdu2dPosX_;
  int64_t                      ipdu2dPosY_;
  int64_t                      ipdu2dDeltaSizeX_;
  int64_t                      ipdu2dDeltaSizeY_;
  int64_t                      ipdu3dPosX_;
  int64_t                      ipdu3dPosY_;
  int64_t                      ipdu3dPosMinZ_;
  int64_t                      ipdu3dPosDeltaMaxZ_;
  size_t                       ipduPatchIndex_;
  size_t                       ipduFrameIndex_;
  PointLocalReconstructionData pointLocalReconstructionData_;
};

// 7.3.7.5  Merge patch data unit syntax(jkei: newly added)
class MergePatchDataUnit {
 public:
  MergePatchDataUnit() :
      mpduOverride2dParamsFlag_( 0 ),
      mpduOverride3dParamsFlag_( 0 ),
      mpduRefIndex_( 0 ),
      mpdu2dPosX_( 0 ),
      mpdu2dPosY_( 0 ),
      mpdu2dDeltaSizeX_( 0 ),
      mpdu2dDeltaSizeY_( 0 ),
      mpdu3dPosX_( 0 ),
      mpdu3dPosY_( 0 ),
      mpdu3dPosMinZ_( 0 ),
      mpdu3dPosDeltaMaxZ_( 0 ),
      mpduOverridePlrFlag_( 0 ),
      mpduPatchIndex_( 0 ),
      mpduFrameIndex_( 0 ){};
  ~MergePatchDataUnit(){};
  MergePatchDataUnit& operator=( const MergePatchDataUnit& ) = default;

  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }
  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }
  size_t getPatchIndex() { return mpduPatchIndex_; }
  size_t getFrameIndex() { return mpduFrameIndex_; }
  void   setPatchIndex( size_t value ) { mpduPatchIndex_ = value; }
  void   setFrameIndex( size_t value ) { mpduFrameIndex_ = value; }

  bool    getMpduOverride2dParamsFlag() { return mpduOverride2dParamsFlag_; }
  bool    getMpduOverride3dParamsFlag() { return mpduOverride3dParamsFlag_; }
  int64_t getMpduRefIndex() { return mpduRefIndex_; }
  int64_t getMpdu2dPosX() { return mpdu2dPosX_; }
  int64_t getMpdu2dPosY() { return mpdu2dPosY_; }
  int64_t getMpdu2dDeltaSizeX() { return mpdu2dDeltaSizeX_; }
  int64_t getMpdu2dDeltaSizeY() { return mpdu2dDeltaSizeY_; }
  int64_t getMpdu3dPosX() { return mpdu3dPosX_; }
  int64_t getMpdu3dPosY() { return mpdu3dPosY_; }
  int64_t getMpdu3dPosMinZ() { return mpdu3dPosMinZ_; }
  int64_t getMpdu3dPosDeltaMaxZ() { return mpdu3dPosDeltaMaxZ_; }
  int64_t getMpduOverridePlrFlag() { return mpduOverridePlrFlag_; }

  void setMpduOverride2dParamsFlag( bool value ) { mpduOverride2dParamsFlag_ = value; }
  void setMpduOverride3dParamsFlag( bool value ) { mpduOverride3dParamsFlag_ = value; }
  void setMpduRefIndex( int64_t value ) { mpduRefIndex_ = value; }
  void setMpdu2dPosX( int64_t value ) { mpdu2dPosX_ = value; }
  void setMpdu2dPosY( int64_t value ) { mpdu2dPosY_ = value; }
  void setMpdu2dDeltaSizeX( int64_t value ) { mpdu2dDeltaSizeX_ = value; }
  void setMpdu2dDeltaSizeY( int64_t value ) { mpdu2dDeltaSizeY_ = value; }
  void setMpdu3dPosX( int64_t value ) { mpdu3dPosX_ = value; }
  void setMpdu3dPosY( int64_t value ) { mpdu3dPosY_ = value; }
  void setMpdu3dPosMinZ( int64_t value ) { mpdu3dPosMinZ_ = value; }
  void setMpdu3dPosDeltaMaxZ( int64_t value ) { mpdu3dPosDeltaMaxZ_ = value; }
  void setMpduOverridePlrFlag( int64_t value ) { mpduOverridePlrFlag_ = value; }

 private:
  bool                         mpduOverride2dParamsFlag_;
  bool                         mpduOverride3dParamsFlag_;
  int64_t                      mpduRefIndex_;
  int64_t                      mpdu2dPosX_;
  int64_t                      mpdu2dPosY_;
  int64_t                      mpdu2dDeltaSizeX_;
  int64_t                      mpdu2dDeltaSizeY_;
  int64_t                      mpdu3dPosX_;
  int64_t                      mpdu3dPosY_;
  int64_t                      mpdu3dPosMinZ_;
  int64_t                      mpdu3dPosDeltaMaxZ_;
  int64_t                      mpduOverridePlrFlag_;
  size_t                       mpduPatchIndex_;
  size_t                       mpduFrameIndex_;
  PointLocalReconstructionData pointLocalReconstructionData_;
};

// 7.3.7.4  Skip patch data unit syntax(jkei: newly added)
class SkipPatchDataUnit {
 public:
  SkipPatchDataUnit() : spduPatchIndex_( 0 ), spduFrameIndex_( 0 ){};
  ~SkipPatchDataUnit(){};
  SkipPatchDataUnit& operator=( const SkipPatchDataUnit& ) = default;
  size_t             getPatchIndex() { return spduPatchIndex_; }
  size_t             getFrameIndex() { return spduFrameIndex_; }
  void               setPatchIndex( size_t value ) { spduPatchIndex_ = value; }
  void               setFrameIndex( size_t value ) { spduFrameIndex_ = value; }

 private:
  size_t spduPatchIndex_;
  size_t spduFrameIndex_;
};

// 7.3.7.3  Patch data unit syntax (jkei: updated)
class PatchDataUnit {
 public:
  PatchDataUnit() :
      pdu2dPosX_( 0 ),
      pdu2dPosY_( 0 ),
      pdu2dDeltaSizeX_( 0 ),
      pdu2dDeltaSizeY_( 0 ),
      pdu3dPosX_( 0 ),
      pdu3dPosY_( 0 ),
      pdu3dPosMinZ_( 0 ),
      pdu3dPosDeltaMaxZ_( 0 ),
      pduProjectionId_( 0 ),
      pduOrientationIndex_( 0 ),
      pduLodEnableFlag_( false ),
      pduLodScaleXminus1_( 0 ),
      pduLodScaleY_( 0 ),
      pduPatchIndex_( 0 ),
      pduFrameIndex_( 0 ){
          // jkei: remove these ones later..
          //    pdu45DegreeProjectionPresentFlag_  = 0;
          //    pdu45DegreeProjectionRotationAxis_ = 0;
          /////////////
      };
  ~PatchDataUnit(){};

  PatchDataUnit&                operator=( const PatchDataUnit& ) = default;
  size_t                        getPdu2dPosX() { return pdu2dPosX_; }
  size_t                        getPdu2dPosY() { return pdu2dPosY_; }
  int64_t                       getPdu2dDeltaSizeX() { return pdu2dDeltaSizeX_; }
  int64_t                       getPdu2dDeltaSizeY() { return pdu2dDeltaSizeY_; }
  size_t                        getPdu3dPosX() { return pdu3dPosX_; }
  size_t                        getPdu3dPosY() { return pdu3dPosY_; }
  size_t                        getPdu3dPosMinZ() { return pdu3dPosMinZ_; }
  size_t                        getPdu3dPosDeltaMaxZ() { return pdu3dPosDeltaMaxZ_; }
  size_t                        getPduProjectionId() { return pduProjectionId_; }
  size_t                        getPduOrientationIndex() { return pduOrientationIndex_; }
  bool                          getLodEnableFlag() { return pduLodEnableFlag_; }
  uint8_t                       getLodScaleXminus1() { return pduLodScaleXminus1_; }
  uint8_t                       getLodScaleY() { return pduLodScaleY_; }
  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }

  size_t getPatchIndex() { return pduPatchIndex_; }
  size_t getFrameIndex() { return pduFrameIndex_; }
  void   setPatchIndex( size_t value ) { pduPatchIndex_ = value; }
  void   setFrameIndex( size_t value ) { pduFrameIndex_ = value; }
  void   setPdu2dPosX( size_t value ) { pdu2dPosX_ = value; }
  void   setPdu2dPosY( size_t value ) { pdu2dPosY_ = value; }
  void   setPdu2dDeltaSizeX( int64_t value ) { pdu2dDeltaSizeX_ = value; }
  void   setPdu2dDeltaSizeY( int64_t value ) { pdu2dDeltaSizeY_ = value; }
  void   setPdu3dPosX( size_t value ) { pdu3dPosX_ = value; }
  void   setPdu3dPosY( size_t value ) { pdu3dPosY_ = value; }
  void   setPdu3dPosMinZ( size_t value ) { pdu3dPosMinZ_ = value; }
  void   setPdu3dPosDeltaMaxZ( size_t value ) { pdu3dPosDeltaMaxZ_ = value; }
  void   setPduProjectionId( size_t value ) { pduProjectionId_ = value; }
  void   setPduOrientationIndex( size_t value ) { pduOrientationIndex_ = value; }

  void setLodEnableFlag( bool value ) { pduLodEnableFlag_ = value; }
  void setLodScaleXminus1( uint8_t value ) { pduLodScaleXminus1_ = value; }
  void setLodScaleY( uint8_t value ) { pduLodScaleY_ = value; }
  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }

  // jkei: remove these ones later..
  //   size_t                        get2DShiftU() { return pdu2dPosX_; }
  //   size_t                        get2DShiftV() { return pdu2dPosY_; }
  //   int64_t                       get2DDeltaSizeU() { return pdu2dDeltaSizeX_; }
  //   int64_t                       get2DDeltaSizeV() { return pdu2dDeltaSizeY_; }
  //   size_t                        get3DShiftTangentAxis() { return pdu3dPosX_; }
  //   size_t                        get3DShiftBiTangentAxis() { return pdu3dPosY_; }
  //   size_t                        get3DShiftMinNormalAxis() { return pdu3dPosMinZ_; }
  //   size_t                        get3DShiftDeltaMaxNormalAxis() { return pdu3dPosDeltaMaxZ_; }
  //  size_t  getProjectPlane() { return pduProjectionId_; }
  //  bool    get45DegreeProjectionPresentFlag() { return pdu45DegreeProjectionPresentFlag_; }
  //  uint8_t get45DegreeProjectionRotationAxis() { return pdu45DegreeProjectionRotationAxis_; }
  //   size_t                        getPduPatchIndex() { return pduPatchIndex_; }
  //   size_t                        getPduFrameIndex() { return pduFrameIndex_; }
  //   void                          setPduPatchIndex( size_t value ) { pduPatchIndex_ = value; }
  //   void                          setPduFrameIndex( size_t value ) { pduFrameIndex_ = value; }
  //   void                          set2DShiftU( size_t value ) { pdu2dPosX_ = value; }
  //   void                          set2DShiftV( size_t value ) { pdu2dPosY_ = value; }
  //   void                          set2DDeltaSizeU( int64_t value ) { pdu2dDeltaSizeX_ = value; }
  //   void                          set2DDeltaSizeV( int64_t value ) { pdu2dDeltaSizeY_ = value; }
  //   void                          set3DShiftTangentAxis( size_t value ) { pdu3dPosX_ = value; }
  //   void                          set3DShiftBiTangentAxis( size_t value ) { pdu3dPosY_ = value; }
  //   void                          set3DShiftMinNormalAxis( size_t value ) { pdu3dPosMinZ_ = value; }
  //   void                          set3DShiftDeltaMaxNormalAxis( size_t value ) { pdu3dPosDeltaMaxZ_ = value; }
  //  void setProjectPlane( size_t value ) { pduProjectionId_ = value; }
  //  void set45DegreeProjectionPresentFlag( bool value ) { pdu45DegreeProjectionPresentFlag_ = value; }
  //  void set45DegreeProjectionRotationAxis( uint8_t value ) { pdu45DegreeProjectionRotationAxis_ = value; }
  //////////////////

 private:
  size_t                       pdu2dPosX_;
  size_t                       pdu2dPosY_;
  int64_t                      pdu2dDeltaSizeX_;
  int64_t                      pdu2dDeltaSizeY_;
  size_t                       pdu3dPosX_;
  size_t                       pdu3dPosY_;
  size_t                       pdu3dPosMinZ_;
  size_t                       pdu3dPosDeltaMaxZ_;
  size_t                       pduProjectionId_;
  size_t                       pduOrientationIndex_;
  bool                         pduLodEnableFlag_;
  uint8_t                      pduLodScaleXminus1_;
  uint8_t                      pduLodScaleY_;
  PointLocalReconstructionData pointLocalReconstructionData_;
  size_t                       pduPatchIndex_;
  size_t                       pduFrameIndex_;
  // jkei: remove these ones later..
  //  bool    pdu45DegreeProjectionPresentFlag_;
  //  uint8_t pdu45DegreeProjectionRotationAxis_;
  /////////////
};

// 7.3.7.2  Patch information data syntax (pid) : jkei : updated
class PatchInformationData {
 public:
  PatchInformationData(){};
  ~PatchInformationData(){};
  PatchInformationData& operator=( const PatchInformationData& ) = default;

  uint8_t             getPatchMode() { return patchMode_; }
  PatchDataUnit&      getPatchDataUnit() { return patchDataUnit_; }
  InterPatchDataUnit& getInterPatchDataUnit() { return interPatchDataUnit_; }
  MergePatchDataUnit& getMergePatchDataUnit() { return mergePatchDataUnit_; }
  SkipPatchDataUnit&  getSkipPatchDataUnit() { return skipPatchDataUnit_; }
  RawPatchDataUnit&   getRawPatchDataUnit() { return rawPatchyDataUnit_; }
  EOMPatchDataUnit&   getEomPatchDataUnit() { return eomPatchDataUnit_; }

  void setPatchMode( uint8_t value ) { patchMode_ = value; }
  void setPatchDataUnit( PatchDataUnit& value ) { patchDataUnit_ = value; }
  void setInterPatchDataUnit( InterPatchDataUnit& value ) { interPatchDataUnit_ = value; }
  void setMergePatchDataUnit( MergePatchDataUnit& value ) { mergePatchDataUnit_ = value; }
  void setSkipPatchDataUnit( SkipPatchDataUnit& value ) { skipPatchDataUnit_ = value; }
  void setRawPatchDataUnit( RawPatchDataUnit& value ) { rawPatchyDataUnit_ = value; }
  void setEomPatchDataUnit( EOMPatchDataUnit& value ) { eomPatchDataUnit_ = value; }

  size_t getFrameIndex() { return frameIndex_; }
  size_t getPatchIndex() { return patchIndex_; }
  void   setFrameIndex( size_t value ) { frameIndex_ = value; }
  void   setPatchIndex( size_t value ) { patchIndex_ = value; }

 private:
  size_t             frameIndex_;
  size_t             patchIndex_;
  uint8_t            patchMode_;
  PatchDataUnit      patchDataUnit_;
  InterPatchDataUnit interPatchDataUnit_;
  MergePatchDataUnit mergePatchDataUnit_;
  SkipPatchDataUnit  skipPatchDataUnit_;  // jkei : do we need this??
  RawPatchDataUnit   rawPatchyDataUnit_;
  EOMPatchDataUnit   eomPatchDataUnit_;
};

// 7.3.7.1  General atlas tile group data unit syntax (jkei: newly added, same as ptgdu but atgduPatchMode_. do we need
// atgduPatchMode_?)
class AtlasTileGroupDataUnit {
 public:
  AtlasTileGroupDataUnit() {}
  ~AtlasTileGroupDataUnit() { patchInformationData_.clear(); }
  AtlasTileGroupDataUnit& operator=( const AtlasTileGroupDataUnit& ) = default;

  void init() { patchInformationData_.clear(); }
  void allocate( size_t size ) { patchInformationData_.resize( size ); }

  void addPatchInformationData( PatchInformationData& value ) { patchInformationData_.push_back( value ); }
  PatchInformationData& addPatchInformationData( uint8_t patchMode ) {
    PatchInformationData pid;
    pid.setPatchMode( patchMode );
    patchInformationData_.push_back( pid );
    return patchInformationData_.back();
  }

  size_t  getFrameIndex() { return frameIndex_; }
  uint8_t getPatchMode( size_t index ) { return patchInformationData_[index].getPatchMode(); }
  uint8_t getPatchCount() { return patchInformationData_.size(); }
  size_t  getAtgduPatchMode() { return atgduPatchMode_; }

  PatchInformationData&              getPatchInformationData( size_t index ) { return patchInformationData_[index]; }
  std::vector<PatchInformationData>& getPatchInformationData() { return patchInformationData_; }
  size_t                             getMatchedPatchCount() {
    size_t matchedPatchCount = 0;
    for ( auto& v : patchInformationData_ ) {
      if ( v.getPatchMode() == PATCH_MODE_P_INTER ) { matchedPatchCount++; }
    }
    return matchedPatchCount;
  }

  void setFrameIndex( size_t value ) { frameIndex_ = value; }
  void setPatchCount( size_t value ) { patchCount_ = value; }
  void setAtgduPatchMode( size_t value ) { atgduPatchMode_ = value; }
  void setPatchInformationData( size_t index, PatchInformationData& value ) { patchInformationData_[index] = value; }

 private:
  size_t                            frameIndex_;
  size_t                            patchCount_;
  size_t                            atgduPatchMode_;
  std::vector<PatchInformationData> patchInformationData_;
};

// 7.3.6.12  Reference list structure syntax (jkei : table number, name updated, encoder/decoder updated accordingly)
class RefListStruct {
 public:
  RefListStruct() : numRefEntries_( 0 ) {
    absDeltaAfocSt_.clear();
    afocLsbLt_.clear();
    stRefAtlasFrameFlag_.clear();
    strpfEntrySignFlag_.clear();
  }
  ~RefListStruct() {
    absDeltaAfocSt_.clear();
    afocLsbLt_.clear();
    stRefAtlasFrameFlag_.clear();
    strpfEntrySignFlag_.clear();
  }
  RefListStruct& operator=( const RefListStruct& ) = default;

  void allocate() {
    absDeltaAfocSt_.resize( numRefEntries_, 0 );
    afocLsbLt_.resize( numRefEntries_, 0 );
    stRefAtlasFrameFlag_.resize( numRefEntries_, false );
    strpfEntrySignFlag_.resize( numRefEntries_, false );
  }

  void addAbsDeltaAfocSt( uint8_t value ) { absDeltaAfocSt_.push_back( value ); }
  void addStrpfEntrySignFlag( bool value ) { strpfEntrySignFlag_.push_back( value ); }
  void addAfocLsbLt( uint8_t value ) { afocLsbLt_.push_back( value ); }
  void addStRefAtalsFrameFlag( bool value ) { stRefAtlasFrameFlag_.push_back( value ); }

  uint8_t getNumRefEntries() { return numRefEntries_; }
  uint8_t getAbsDeltaAfocSt( uint16_t index ) { return absDeltaAfocSt_[index]; }
  bool    getStrpfEntrySignFlag( uint16_t index ) { return strpfEntrySignFlag_[index]; }
  uint8_t getAfocLsbLt( uint16_t index ) { return afocLsbLt_[index]; }
  bool    getStRefAtalsFrameFlag( uint16_t index ) { return stRefAtlasFrameFlag_[index]; }

  void setNumRefEntries( uint8_t value ) { numRefEntries_ = value; }
  void setAbsDeltaAfocSt( uint16_t index, uint8_t value ) { absDeltaAfocSt_[index] = value; }
  void setStrpfEntrySignFlag( uint16_t index, bool value ) { strpfEntrySignFlag_[index] = value; }
  void setAfocLsbLt( uint16_t index, uint8_t value ) { afocLsbLt_[index] = value; }
  void setStRefAtalsFrameFlag( uint16_t index, bool value ) { stRefAtlasFrameFlag_[index] = value; }

 private:
  uint8_t              numRefEntries_;
  std::vector<uint8_t> absDeltaAfocSt_;
  std::vector<uint8_t> afocLsbLt_;
  std::vector<bool>    stRefAtlasFrameFlag_;
  std::vector<bool>    strpfEntrySignFlag_;
};

// 7.3.6.11  Atlas tile group header syntax (jkei : newly added. )
class AtlasTileGroupHeader {
 public:
  AtlasTileGroupHeader() :
      atghFrameIndex_( 0 ),
      atghAtlasFrameParameterSetId_( 0 ),
      atghAddress_( 0 ),
      atghType_( PCCTILEGROUP( 0 ) ),
      atghAtlasFrmOrderCntLsb_( 0 ),
      atghRefAtlasFrameListSpsFlag_( 0 ),
      atghRefAtlasFrameListIdx_( 0 ),
      atghPosMinZQuantizer_( 0 ),
      atghPosDeltaMaxZQuantizer_( 0 ),
      atghPatchSizeXinfoQuantizer_( 0 ),
      atghPatchSizeYinfoQuantizer_( 0 ),
      atghRaw3dPosAxisBitCountMinus1_( 0 ),
      atghNumRefIdxActiveOverrideFlag_( 0 ),
      atghNumRefdxActiveMinus1_( 0 ) {
    atghAdditionalAfocLsbPresentFlag_.resize( 1, 0 );
    atghAdditionalAfocLsbVal_.resize( 1, 0 );
  };
  ~AtlasTileGroupHeader() {
    atghAdditionalAfocLsbPresentFlag_.clear();
    atghAdditionalAfocLsbVal_.clear();
  };
  AtlasTileGroupHeader& operator=( const AtlasTileGroupHeader& ) = default;

  // void           allocate( size_t size ) { refListStruct_.resize( size ); }
  RefListStruct& getRefListStruct() { return refListStruct_; }  // only 1 refList
  void           setRefListStruct( RefListStruct value ) { refListStruct_ = value; }
  // uint8_t        getRefListStructSize() { return refListStruct_.size(); }
  // void           setRefListStruct( uint8_t index, RefListStruct value ) { refListStruct_[index] = value; }
  // void           addRefListStruct( RefListStruct value ) { refListStruct_.push_back( value ); }
  //  RefListStruct& addRefListStruct() {
  //    RefListStruct refListStruct;
  //    refListStruct_.push_back( refListStruct );
  //    return refListStruct_.back();
  //  }

  uint8_t      getFrameIndex() { return atghFrameIndex_; }
  void         setFrameIndex( uint8_t value ) { atghFrameIndex_ = value; }
  uint8_t      getAtghAtlasFrameParameterSetId() { return atghAtlasFrameParameterSetId_; }
  uint32_t     getAtghAddress() { return atghAddress_; }
  PCCTILEGROUP getAtghType() { return atghType_; }
  uint8_t      getAtghAtlasFrmOrderCntLsb() { return atghAtlasFrmOrderCntLsb_; }
  bool         getAtghRefAtlasFrameListSpsFlag() { return atghRefAtlasFrameListSpsFlag_; }
  uint8_t      getAtghRefAtlasFrameListIdx() { return atghRefAtlasFrameListIdx_; }
  uint8_t      getAtghPosMinZQuantizer() { return atghPosMinZQuantizer_; }
  uint8_t      getAtghPosDeltaMaxZQuantizer() { return atghPosDeltaMaxZQuantizer_; }
  uint8_t      getAtghPatchSizeXinfoQuantizer() { return atghPatchSizeXinfoQuantizer_; }
  uint8_t      getAtghPatchSizeYinfoQuantizer() { return atghPatchSizeYinfoQuantizer_; }

  uint8_t               getAtghRaw3dPosAxisBitCountMinus1() { return atghRaw3dPosAxisBitCountMinus1_; }
  bool                  getAtghNumRefIdxActiveOverrideFlag() { return atghNumRefIdxActiveOverrideFlag_; }
  uint8_t               getAtghNumRefdxActiveMinus1() { return atghNumRefdxActiveMinus1_; }
  std::vector<bool>&    getAtghAdditionalAfocLsbPresentFlag() { return atghAdditionalAfocLsbPresentFlag_; }
  std::vector<uint8_t>& getAtghAdditionalAfocLsbVal() { return atghAdditionalAfocLsbVal_; }
  bool    getAtghAdditionalAfocLsbPresentFlag( size_t idx ) { return atghAdditionalAfocLsbPresentFlag_[idx]; }
  uint8_t getAtghAdditionalAfocLsbVal( size_t idx ) { return atghAdditionalAfocLsbVal_[idx]; }

  void setAtghAtlasFrameParameterSetId( uint8_t value ) { atghAtlasFrameParameterSetId_ = value; }
  void setAtghAddress( uint32_t value ) { atghAddress_ = value; }
  void setAtghType( PCCTILEGROUP value ) { atghType_ = value; }
  void setAtghAtlasFrmOrderCntLsb( uint8_t value ) { atghAtlasFrmOrderCntLsb_ = value; }
  void setAtghRefAtlasFrameListSpsFlag( bool value ) { atghRefAtlasFrameListSpsFlag_ = value; }
  void setAtghRefAtlasFrameListIdx( uint8_t value ) { atghRefAtlasFrameListIdx_ = value; }
  void setAtghPosMinZQuantizer( uint8_t value ) { atghPosMinZQuantizer_ = value; }
  void setAtghPosDeltaMaxZQuantizer( uint8_t value ) { atghPosDeltaMaxZQuantizer_ = value; }
  void setAtghPatchSizeXinfoQuantizer( uint8_t value ) { atghPatchSizeXinfoQuantizer_ = value; }
  void setAtghPatchSizeYinfoQuantizer( uint8_t value ) { atghPatchSizeYinfoQuantizer_ = value; }

  void setAtghRaw3dPosAxisBitCountMinus1( uint8_t value ) { atghRaw3dPosAxisBitCountMinus1_ = value; }
  void setAtghNumRefIdxActiveOverrideFlag( bool value ) { atghNumRefIdxActiveOverrideFlag_ = value; }
  void setAtghNumRefdxActiveMinus1( uint8_t value ) { atghNumRefdxActiveMinus1_ = value; }
  void setAtghAdditionalAfocLsbPresentFlag( std::vector<bool>& value ) { atghAdditionalAfocLsbPresentFlag_ = value; }
  void setAtghAdditionalAfocLsbVal( std::vector<uint8_t>& value ) { atghAdditionalAfocLsbVal_ = value; }
  void setAtghAdditionalAfocLsbPresentFlag( size_t idx, bool value ) { atghAdditionalAfocLsbPresentFlag_[idx] = value; }
  void setAtghAdditionalAfocLsbVal( size_t idx, uint8_t value ) { atghAdditionalAfocLsbVal_[idx] = value; }

 private:
  uint8_t              atghFrameIndex_;
  uint8_t              atghAtlasFrameParameterSetId_;
  uint32_t             atghAddress_;
  PCCTILEGROUP         atghType_;
  uint8_t              atghAtlasFrmOrderCntLsb_;
  bool                 atghRefAtlasFrameListSpsFlag_;
  uint8_t              atghRefAtlasFrameListIdx_;
  uint8_t              atghPosMinZQuantizer_;
  uint8_t              atghPosDeltaMaxZQuantizer_;
  uint8_t              atghPatchSizeXinfoQuantizer_;
  uint8_t              atghPatchSizeYinfoQuantizer_;
  uint8_t              atghRaw3dPosAxisBitCountMinus1_;
  bool                 atghNumRefIdxActiveOverrideFlag_;
  uint8_t              atghNumRefdxActiveMinus1_;
  std::vector<bool>    atghAdditionalAfocLsbPresentFlag_;
  std::vector<uint8_t> atghAdditionalAfocLsbVal_;
  RefListStruct        refListStruct_;  // jkei: one ref list
};

// 7.3.6.10  Atlas tile group layer RBSP syntax (jkei: newly added, replace PatchTileGroupLayerUnit with this)
class AtlasTileGroupLayerRbsp {
 public:
  AtlasTileGroupLayerRbsp() : frameIndex_( 0 ) {}
  ~AtlasTileGroupLayerRbsp() {}

  AtlasTileGroupLayerRbsp& operator=( const AtlasTileGroupLayerRbsp& ) = default;

  uint8_t                 getFrameIndex() { return frameIndex_; }
  AtlasTileGroupHeader&   getAtlasTileGroupHeader() { return atlasTileGroupHeader_; }
  AtlasTileGroupDataUnit& getAtlasTileGroupDataUnit() { return atlasTileGroupDataUnit_; }

  void setFrameIndex( uint8_t value ) { frameIndex_ = value; }
  void setAtlasTileGroupHeader( AtlasTileGroupHeader value ) { atlasTileGroupHeader_ = value; }
  void setAtlasTileGroupDataUnit( AtlasTileGroupDataUnit value ) { atlasTileGroupDataUnit_ = value; }

 private:
  uint8_t                frameIndex_;
  AtlasTileGroupHeader   atlasTileGroupHeader_;
  AtlasTileGroupDataUnit atlasTileGroupDataUnit_;
};

// jkei: do we need 7.3.6.5~7.3.6.9?
// 7.3.6.9  Filler data RBSP syntax
class FillerDataRbsp {
 public:
  FillerDataRbsp() {}
  ~FillerDataRbsp() {}
  FillerDataRbsp& operator=( const FillerDataRbsp& ) = default;

 private:
};

// 7.3.6.8  End of bitstream RBSP syntax
class EndOfBitstreamRbsp {
 public:
  EndOfBitstreamRbsp() {}
  ~EndOfBitstreamRbsp() {}
  EndOfBitstreamRbsp& operator=( const EndOfBitstreamRbsp& ) = default;

 private:
};

// 7.3.6.7  End of sequence RBSP syntax
class EndOfSequenceRbsp {
 public:
  EndOfSequenceRbsp() {}
  ~EndOfSequenceRbsp() {}
  EndOfSequenceRbsp& operator=( const EndOfSequenceRbsp& ) = default;

 private:
};

// 7.3.6.6  Access unit delimiter RBSP syntax
class AccessUnitDelimiterRbsp {
 public:
  AccessUnitDelimiterRbsp() : aframeType_( 0 ) {}
  ~AccessUnitDelimiterRbsp() {}
  AccessUnitDelimiterRbsp& operator=( const AccessUnitDelimiterRbsp& ) = default;
  uint8_t                  getAframeType() { return aframeType_; }
  void                     setAframeType( uint8_t value ) { aframeType_ = value; }

 private:
  uint8_t aframeType_;
};

// 7.3.6.5  Supplemental enhancement information RBSP syntax
class SupplementalEnhancementInformationRbsp {
 public:
  SupplementalEnhancementInformationRbsp() {}
  ~SupplementalEnhancementInformationRbsp() {}
  SupplementalEnhancementInformationRbsp& operator=( const SupplementalEnhancementInformationRbsp& ) = default;

 private:
  std::vector<SEI> sei_;
};

// 7.3.6.4  Atlas frame tile information syntax (jkei: PatchFrameTileInformation is renamed )
class AtlasFrameTileInformation {
 public:
  AtlasFrameTileInformation() :
      singleTileInAtlasFrameFlag_( 0 ),
      uniformTileSpacingFlag_( 0 ),
      numTileColumnsMinus1_( 0 ),
      numTileRowsMinus1_( 0 ),
      singleTilePerTileGroupFlag_( 0 ),
      numTileGroupsInAtlasFrameMinus1_( 0 ),
      signalledTileGroupIdFlag_( 0 ),
      signalledTileGroupIdLengthMinus1_( 0 ) {
    tileColumnWidthMinus1_.resize( 1, 0 );
    tileRowHeightMinus1_.resize( 1, 0 );
    topLeftTileIdx_.resize( 1, 0 );
    bottomRightTileIdxDelta_.resize( 1, 0 );
    tileGroupId_.resize( 1, 0 );
  };
  ~AtlasFrameTileInformation() {
    tileColumnWidthMinus1_.clear();
    tileRowHeightMinus1_.clear();
    topLeftTileIdx_.clear();
    bottomRightTileIdxDelta_.clear();
    tileGroupId_.clear();
  };

  AtlasFrameTileInformation& operator=( const AtlasFrameTileInformation& ) = default;

  bool     getSingleTileInAtlasFrameFlag() { return singleTileInAtlasFrameFlag_; }
  bool     getUniformTileSpacingFlag() { return uniformTileSpacingFlag_; }
  uint32_t getNumTileColumnsMinus1() { return numTileColumnsMinus1_; }
  uint32_t getNumTileRowsMinus1() { return numTileRowsMinus1_; }
  uint32_t getSingleTilePerTileGroupFlag() { return singleTilePerTileGroupFlag_; }
  uint32_t getNumTileGroupsInAtlasFrameMinus1() { return numTileGroupsInAtlasFrameMinus1_; }
  bool     getSignalledTileGroupIdFlag() { return signalledTileGroupIdFlag_; }
  uint32_t getSignalledTileGroupIdLengthMinus1() { return signalledTileGroupIdLengthMinus1_; }
  uint32_t getTileColumnWidthMinus1( size_t index ) { return tileColumnWidthMinus1_[index]; }
  uint32_t getTileRowHeightMinus1( size_t index ) { return tileRowHeightMinus1_[index]; }
  uint32_t getTopLeftTileIdx( size_t index ) { return topLeftTileIdx_[index]; }
  uint32_t getBottomRightTileIdxDelta( size_t index ) { return bottomRightTileIdxDelta_[index]; }
  uint32_t getTileGroupId( size_t index ) { return tileGroupId_[index]; }

  void setSingleTileInAtlasFrameFlag( bool value ) { singleTileInAtlasFrameFlag_ = value; }
  void setUniformTileSpacingFlag( bool value ) { uniformTileSpacingFlag_ = value; }
  void setNumTileColumnsMinus1( uint32_t value ) { numTileColumnsMinus1_ = value; }
  void setNumTileRowsMinus1( uint32_t value ) { numTileRowsMinus1_ = value; }
  void setSingleTilePerTileGroupFlag( uint32_t value ) { singleTilePerTileGroupFlag_ = value; }
  void setNumTileGroupsInAtlasFrameMinus1( uint32_t value ) { numTileGroupsInAtlasFrameMinus1_ = value; }
  void setSignalledTileGroupIdFlag( bool value ) { signalledTileGroupIdFlag_ = value; }
  void setSignalledTileGroupIdLengthMinus1( uint32_t value ) { signalledTileGroupIdLengthMinus1_ = value; }

  void setTileColumnWidthMinus1( size_t index, uint32_t value ) {
    if ( index == ( tileColumnWidthMinus1_.size() ) )
      tileColumnWidthMinus1_.resize( tileColumnWidthMinus1_.size() + 1 );
    else if ( index > tileColumnWidthMinus1_.size() )
      assert( 0 );
    tileColumnWidthMinus1_[index] = value;
  }
  void setTileRowHeightMinus1( size_t index, uint32_t value ) {
    if ( index == ( tileRowHeightMinus1_.size() ) )
      tileRowHeightMinus1_.resize( tileRowHeightMinus1_.size() + 1 );
    else if ( index > tileRowHeightMinus1_.size() )
      assert( 0 );
    tileRowHeightMinus1_[index] = value;
  }
  void setTopLeftTileIdx( size_t index, uint32_t value ) {
    if ( index == ( topLeftTileIdx_.size() ) )
      topLeftTileIdx_.resize( topLeftTileIdx_.size() + 1 );
    else if ( index > topLeftTileIdx_.size() )
      assert( 0 );
    topLeftTileIdx_[index] = value;
  }
  void setBottomRightTileIdxDelta( size_t index, uint32_t value ) {
    if ( index == ( bottomRightTileIdxDelta_.size() ) )
      bottomRightTileIdxDelta_.resize( bottomRightTileIdxDelta_.size() + 1 );
    else if ( index > bottomRightTileIdxDelta_.size() )
      assert( 0 );
    bottomRightTileIdxDelta_[index] = value;
  }
  void setTileGroupId( size_t index, uint32_t value ) {
    if ( index == ( tileGroupId_.size() ) )
      tileGroupId_.resize( tileGroupId_.size() + 1 );
    else if ( index > tileGroupId_.size() )
      assert( 0 );
    tileGroupId_[index] = value;
  }

 private:
  bool singleTileInAtlasFrameFlag_;
  bool uniformTileSpacingFlag_;
  //  uint32_t              tileColsWidthMinus1_;
  //  uint32_t              tileRowsHeightMinus1_;
  uint32_t              numTileColumnsMinus1_;
  uint32_t              numTileRowsMinus1_;
  uint32_t              singleTilePerTileGroupFlag_;
  uint32_t              numTileGroupsInAtlasFrameMinus1_;
  bool                  signalledTileGroupIdFlag_;
  uint32_t              signalledTileGroupIdLengthMinus1_;
  std::vector<uint32_t> tileColumnWidthMinus1_;
  std::vector<uint32_t> tileRowHeightMinus1_;
  std::vector<uint32_t> topLeftTileIdx_;
  std::vector<uint32_t> bottomRightTileIdxDelta_;
  std::vector<uint32_t> tileGroupId_;
};

// 7.3.6.3  Atlas frame parameter set RBSP syntax
class AtlasFrameParameterSetRbsp {
 public:
  AtlasFrameParameterSetRbsp() :
      afpsAtlasFrameParameterSetId_( 0 ),
      afpsAtlasSequenceParameterSetId_( 0 ),
      afpsNumRefIdxDefaultActiveMinus1_( 0 ),
      afpsAdditionalLtAfocLsbLen_( 0 ),
      afps2dPosXBitCountMinus1_( 0 ),
      afps2dPosYBitCountMinus1_( 0 ),
      afps3dPosXBitCountMinus1_( 0 ),
      afps3dPosYBitCountMinus1_( 0 ),
      afpsLodModeEnableFlag_( false ),
      afpsOverrideEomForDepthFlag_( 0 ),
      afpsEomNumberOfPatchBitCountMinus1_( 0 ),
      afpsEomMaxBitCountMinus1_( 0 ),
      afpsRaw3dPosBitCountExplicitModeFlag_( 0 ),
      afpsExtensionPresentFlag_( 0 ),
      afpsExtensionDataFlag_( 0 ) {}

  ~AtlasFrameParameterSetRbsp() {}
  AtlasFrameParameterSetRbsp& operator=( const AtlasFrameParameterSetRbsp& ) = default;
  void                        copyFrom( AtlasFrameParameterSetRbsp& refAfps ) {
    afpsAtlasSequenceParameterSetId_      = refAfps.getAtlasSequenceParameterSetId();
    afpsNumRefIdxDefaultActiveMinus1_     = refAfps.getAfpsNumRefIdxDefaultActiveMinus1();
    afpsAdditionalLtAfocLsbLen_           = refAfps.getAfpsAdditionalLtAfocLsbLen();
    afps2dPosXBitCountMinus1_             = refAfps.getAfps2dPosXBitCountMinus1();
    afps2dPosYBitCountMinus1_             = refAfps.getAfps2dPosYBitCountMinus1();
    afps3dPosXBitCountMinus1_             = refAfps.getAfps3dPosXBitCountMinus1();
    afps3dPosYBitCountMinus1_             = refAfps.getAfps3dPosYBitCountMinus1();
    afpsLodModeEnableFlag_                = refAfps.getLodModeEnableFlag();
    afpsOverrideEomForDepthFlag_          = refAfps.getAfpsOverrideEomForDepthFlag();
    afpsEomNumberOfPatchBitCountMinus1_   = refAfps.getAfpsEomNumberOfPatchBitCountMinus1();
    afpsEomMaxBitCountMinus1_             = refAfps.getAfpsEomMaxBitCountMinus1();
    afpsRaw3dPosBitCountExplicitModeFlag_ = refAfps.getAfpsRaw3dPosBitCountExplicitModeFlag();
    afpsExtensionPresentFlag_             = refAfps.getAfpsExtensionPresentFlag();
    afpsExtensionDataFlag_                = refAfps.getAfpsExtensionDataFlag();
    atlasFrameTileInformation_ = refAfps.getAtlasFrameTileInformation();  // jkei: does it work?
  }

  bool getLodModeEnableFlag() { return afpsLodModeEnableFlag_; }
  void setLodModeEnableFlag( bool value ) { afpsLodModeEnableFlag_ = value; }

  uint8_t getAtlasFrameParameterSetId() { return afpsAtlasFrameParameterSetId_; }
  uint8_t getAtlasSequenceParameterSetId() { return afpsAtlasSequenceParameterSetId_; }
  uint8_t getAfpsNumRefIdxDefaultActiveMinus1() { return afpsNumRefIdxDefaultActiveMinus1_; }
  uint8_t getAfpsAdditionalLtAfocLsbLen() { return afpsAdditionalLtAfocLsbLen_; }
  size_t  getAfps2dPosXBitCountMinus1() { return afps2dPosXBitCountMinus1_; }
  size_t  getAfps2dPosYBitCountMinus1() { return afps2dPosYBitCountMinus1_; }
  size_t  getAfps3dPosXBitCountMinus1() { return afps3dPosXBitCountMinus1_; }
  size_t  getAfps3dPosYBitCountMinus1() { return afps3dPosYBitCountMinus1_; }
  bool    getAfpsOverrideEomForDepthFlag() { return afpsOverrideEomForDepthFlag_; }
  uint8_t getAfpsEomNumberOfPatchBitCountMinus1() { return afpsEomNumberOfPatchBitCountMinus1_; }
  uint8_t getAfpsEomMaxBitCountMinus1() { return afpsEomMaxBitCountMinus1_; }
  bool    getAfpsRaw3dPosBitCountExplicitModeFlag() { return afpsRaw3dPosBitCountExplicitModeFlag_; }
  uint8_t getAfpsExtensionPresentFlag() { return afpsExtensionPresentFlag_; }
  bool    getAfpsExtensionDataFlag() { return afpsExtensionDataFlag_; }

  void setAtlasFrameParameterSetId( uint8_t value ) { afpsAtlasFrameParameterSetId_ = value; }
  void setAtlasSequenceParameterSetId( uint8_t value ) { afpsAtlasSequenceParameterSetId_ = value; }
  void setAfpsNumRefIdxDefaultActiveMinus1( uint8_t value ) { afpsNumRefIdxDefaultActiveMinus1_ = value; }
  void setAfpsAdditionalLtAfocLsbLen( uint8_t value ) { afpsAdditionalLtAfocLsbLen_ = value; }
  void setAfps2dPosXBitCountMinus1( uint8_t value ) { afps2dPosXBitCountMinus1_ = value; }
  void setAfps2dPosYBitCountMinus1( uint8_t value ) { afps2dPosYBitCountMinus1_ = value; }
  void setAfps3dPosXBitCountMinus1( uint8_t value ) { afps3dPosXBitCountMinus1_ = value; }
  void setAfps3dPosYBitCountMinus1( uint8_t value ) { afps3dPosYBitCountMinus1_ = value; }
  void setAfpsOverrideEomForDepthFlag( bool value ) { afpsOverrideEomForDepthFlag_ = value; }
  void setAfpsEomNumberOfPatchBitCountMinus1( uint8_t value ) { afpsEomNumberOfPatchBitCountMinus1_ = value; }
  void setAfpsEomMaxBitCountMinus1( uint8_t value ) { afpsEomMaxBitCountMinus1_ = value; }
  void setAfpsRaw3dPosBitCountExplicitModeFlag( bool value ) { afpsRaw3dPosBitCountExplicitModeFlag_ = value; }
  void setAfpsExtensionPresentFlag( uint8_t value ) { afpsExtensionPresentFlag_ = value; }
  void setAfpsExtensionDataFlag( bool value ) { afpsExtensionDataFlag_ = value; }

  AtlasFrameTileInformation& getAtlasFrameTileInformation() { return atlasFrameTileInformation_; }
  void setAtlasFrameTileInformation( AtlasFrameTileInformation value ) { atlasFrameTileInformation_ = value; }

 private:
  uint8_t                   afpsAtlasFrameParameterSetId_;
  uint8_t                   afpsAtlasSequenceParameterSetId_;
  AtlasFrameTileInformation atlasFrameTileInformation_;
  uint8_t                   afpsNumRefIdxDefaultActiveMinus1_;
  uint8_t                   afpsAdditionalLtAfocLsbLen_;
  size_t                    afps2dPosXBitCountMinus1_;
  size_t                    afps2dPosYBitCountMinus1_;
  size_t                    afps3dPosXBitCountMinus1_;
  size_t                    afps3dPosYBitCountMinus1_;
  bool                      afpsLodModeEnableFlag_;  // TODO: remove?
  // uint8_t                   afpsLodBitCount_; //DBG added
  bool    afpsOverrideEomForDepthFlag_;
  uint8_t afpsEomNumberOfPatchBitCountMinus1_;
  uint8_t afpsEomMaxBitCountMinus1_;
  bool    afpsRaw3dPosBitCountExplicitModeFlag_;
  uint8_t afpsExtensionPresentFlag_;
  bool    afpsExtensionDataFlag_;
};

// 7.3.6.2 Point local reconstruction information syntax
class PointLocalReconstructionInformation {
 public:
  PointLocalReconstructionInformation() : numberOfModesMinus1_( 0 ), blockThresholdPerPatchMinus1_( 0 ) {
    minimumDepth_.clear();
    neighbourMinus1_.clear();
    interpolateFlag_.clear();
    fillingFlag_.clear();
  };
  ~PointLocalReconstructionInformation() {
    minimumDepth_.clear();
    neighbourMinus1_.clear();
    interpolateFlag_.clear();
    fillingFlag_.clear();
  };

  PointLocalReconstructionInformation& operator=( const PointLocalReconstructionInformation& ) = default;

  void allocate() {
    minimumDepth_.resize( numberOfModesMinus1_ + 1, 0 );
    neighbourMinus1_.resize( numberOfModesMinus1_ + 1, 0 );
    interpolateFlag_.resize( numberOfModesMinus1_ + 1, false );
    fillingFlag_.resize( numberOfModesMinus1_ + 1, false );
  }
  bool    getMapEnabledFlag() { return mapEnabledFlag_; }
  uint8_t getNumberOfModesMinus1() { return numberOfModesMinus1_; }
  uint8_t getBlockThresholdPerPatchMinus1() { return blockThresholdPerPatchMinus1_; }
  uint8_t getMinimumDepth( size_t index ) { return minimumDepth_[index]; }
  uint8_t getNeighbourMinus1( size_t index ) { return neighbourMinus1_[index]; }
  bool    getInterpolateFlag( size_t index ) { return interpolateFlag_[index]; }
  bool    getFillingFlag( size_t index ) { return fillingFlag_[index]; }
  void    setMapEnabledFlag( bool value ) { mapEnabledFlag_ = value; }
  void    setNumberOfModesMinus1( uint8_t value ) { numberOfModesMinus1_ = value; }
  void    setBlockThresholdPerPatchMinus1( uint8_t value ) { blockThresholdPerPatchMinus1_ = value; }
  void    setMinimumDepth( size_t index, uint8_t value ) { minimumDepth_[index] = value; }
  void    setNeighbourMinus1( size_t index, uint8_t value ) { neighbourMinus1_[index] = value; }
  void    setInterpolateFlag( size_t index, bool value ) { interpolateFlag_[index] = value; }
  void    setFillingFlag( size_t index, bool value ) { fillingFlag_[index] = value; }

 private:
  bool                 mapEnabledFlag_;
  uint8_t              numberOfModesMinus1_;
  std::vector<bool>    interpolateFlag_;
  std::vector<bool>    fillingFlag_;
  std::vector<uint8_t> minimumDepth_;
  std::vector<uint8_t> neighbourMinus1_;
  uint8_t              blockThresholdPerPatchMinus1_;
};

// 7.3.6.1 Atlas sequence parameter set RBSP
class AtlasSequenceParameterSetRBSP {
 public:
  AtlasSequenceParameterSetRBSP() :
      altasSequenceParameterSetId_( 0 ),
      frameWidth_( 0 ),
      frameHeight_( 0 ),
      log2PatchPackingBlockSize_( 0 ),
      log2MaxAtlasFrameOrderCntLsbMinus4_( 4 ),
      maxDecAtlasFrameBufferingMinus1_( 0 ),
      longTermRefAtlasFramesFlag_( false ),
      numRefAtlasFrameListsInAsps_( 0 ),
      useEightOrientationsFlag_( false ),
      degree45ProjectionPatchPresentFlag_( false ),
      normalAxisLimitsQuantizationEnabledFlag_( true ),
      normalAxisMaxDeltaValueEnabledFlag_( false ),
      removeDuplicatePointEnabledFlag_( false ),
      pixelDeinterleavingFlag_( false ),
      patchPrecedenceOrderFlag_( false ),
      patchSizeQuantizerPresentFlag_( false ),
      enhancedOccupancyMapForDepthFlag_( false ),
      pointLocalReconstructionEnabledFlag_( false ),
      mapCountMinus1_( 0 ),
      enhancedOccupancyMapFixBitCountMinus1_( 1 ),
      surfaceThicknessMinus1_( 3 ),
      vuiParametersPresentFlag_( false ),
      extensionPresentFlag_( false ),
      extensionDataFlag_( false ) {}
  ~AtlasSequenceParameterSetRBSP() {
    refListStruct_.clear();
    pointLocalReconstructionInformation_.clear();
  }

  AtlasSequenceParameterSetRBSP& operator=( const AtlasSequenceParameterSetRBSP& ) = default;

  void allocateRefListStruct() { refListStruct_.resize( numRefAtlasFrameListsInAsps_ ); }
  void allocatePointLocalReconstructionInformation() {
    pointLocalReconstructionInformation_.resize( mapCountMinus1_ + 1 );
  }
  uint8_t        getAltasSequenceParameterSetId() { return altasSequenceParameterSetId_; }
  uint8_t        getFrameWidth() { return frameWidth_; }
  uint8_t        getFrameHeight() { return frameHeight_; }
  uint8_t        getLog2PatchPackingBlockSize() { return log2PatchPackingBlockSize_; }
  uint8_t        getLog2MaxAtlasFrameOrderCntLsbMinus4() { return log2MaxAtlasFrameOrderCntLsbMinus4_; }
  uint8_t        getMaxDecAtlasFrameBufferingMinus1() { return maxDecAtlasFrameBufferingMinus1_; }
  uint8_t        getNumRefAtlasFrameListsInAsps() { return numRefAtlasFrameListsInAsps_; }
  uint8_t        getMapCountMinus1() { return mapCountMinus1_; }
  uint8_t        getEnhancedOccupancyMapFixBitCountMinus1() { return enhancedOccupancyMapFixBitCountMinus1_; }
  uint8_t        getSurfaceThicknessMinus1() { return surfaceThicknessMinus1_; }
  bool           getLongTermRefAtlasFramesFlag() { return longTermRefAtlasFramesFlag_; }
  bool           getUseEightOrientationsFlag() { return useEightOrientationsFlag_; }
  bool           get45DegreeProjectionPatchPresentFlag() { return degree45ProjectionPatchPresentFlag_; }
  bool           getNormalAxisLimitsQuantizationEnabledFlag() { return normalAxisLimitsQuantizationEnabledFlag_; }
  bool           getNormalAxisMaxDeltaValueEnabledFlag() { return normalAxisMaxDeltaValueEnabledFlag_; }
  bool           getRemoveDuplicatePointEnabledFlag() { return removeDuplicatePointEnabledFlag_; }
  bool           getPixelDeinterleavingFlag() { return pixelDeinterleavingFlag_; }
  bool           getPatchPrecedenceOrderFlag() { return patchPrecedenceOrderFlag_; }
  bool           getPatchSizeQuantizerPresentFlag() { return patchSizeQuantizerPresentFlag_; }
  bool           getEnhancedOccupancyMapForDepthFlag() { return enhancedOccupancyMapForDepthFlag_; }
  bool           getPointLocalReconstructionEnabledFlag() { return pointLocalReconstructionEnabledFlag_; }
  bool           getVuiParametersPresentFlag() { return vuiParametersPresentFlag_; }
  bool           getExtensionPresentFlag() { return extensionPresentFlag_; }
  bool           getExtensionDataFlag() { return extensionDataFlag_; }
  VUIParameters& getVuiParameters() { return vuiParameters_; }

  void setAltasSequenceParameterSetId( uint8_t value ) { altasSequenceParameterSetId_ = value; }
  void setFrameWidth( uint8_t value ) { frameWidth_ = value; }
  void setFrameHeight( uint8_t value ) { frameHeight_ = value; }
  void setLog2PatchPackingBlockSize( uint8_t value ) { log2PatchPackingBlockSize_ = value; }
  void setLog2MaxAtlasFrameOrderCntLsbMinus4( uint8_t value ) { log2MaxAtlasFrameOrderCntLsbMinus4_ = value; }
  void setMaxDecAtlasFrameBufferingMinus1( uint8_t value ) { maxDecAtlasFrameBufferingMinus1_ = value; }
  void setNumRefAtlasFrameListsInAsps( uint8_t value ) { numRefAtlasFrameListsInAsps_ = value; }
  void setMapCountMinus1( uint8_t value ) { mapCountMinus1_ = value; }
  void setEnhancedOccupancyMapFixBitCountMinus1( uint8_t value ) { enhancedOccupancyMapFixBitCountMinus1_ = value; }
  void setSurfaceThicknessMinus1( uint8_t value ) { surfaceThicknessMinus1_ = value; }
  void setLongTermRefAtlasFramesFlag( bool value ) { longTermRefAtlasFramesFlag_ = value; }
  void setUseEightOrientationsFlag( bool value ) { useEightOrientationsFlag_ = value; }
  void set45DegreeProjectionPatchPresentFlag( bool value ) { degree45ProjectionPatchPresentFlag_ = value; }
  void setNormalAxisLimitsQuantizationEnabledFlag( bool value ) { normalAxisLimitsQuantizationEnabledFlag_ = value; }
  void setNormalAxisMaxDeltaValueEnabledFlag( bool value ) { normalAxisMaxDeltaValueEnabledFlag_ = value; }
  void setRemoveDuplicatePointEnabledFlag( bool value ) { removeDuplicatePointEnabledFlag_ = value; }
  void setPixelDeinterleavingFlag( bool value ) { pixelDeinterleavingFlag_ = value; }
  void setPatchPrecedenceOrderFlag( bool value ) { patchPrecedenceOrderFlag_ = value; }
  void setPatchSizeQuantizerPresentFlag( bool value ) { patchSizeQuantizerPresentFlag_ = value; }
  void setEnhancedOccupancyMapForDepthFlag( bool value ) { enhancedOccupancyMapForDepthFlag_ = value; }
  void setPointLocalReconstructionEnabledFlag( bool value ) { pointLocalReconstructionEnabledFlag_ = value; }
  void setVuiParametersPresentFlag( bool value ) { vuiParametersPresentFlag_ = value; }
  void setExtensionPresentFlag( bool value ) { extensionPresentFlag_ = value; }
  void setExtensionDataFlag( bool value ) { extensionDataFlag_ = value; }

  RefListStruct& getRefListStruct( uint8_t index ) { return refListStruct_[index]; }
  void           addRefListStruct( RefListStruct value ) { refListStruct_.push_back( value ); }
  RefListStruct& addRefListStruct() {
    RefListStruct refListStruct;
    refListStruct_.push_back( refListStruct );
    return refListStruct_.back();
  }
  PointLocalReconstructionInformation& getPointLocalReconstructionInformation( uint8_t index ) {
    return pointLocalReconstructionInformation_[index];
  }
  void addPointLocalReconstructionInformation( PointLocalReconstructionInformation value ) {
    pointLocalReconstructionInformation_.push_back( value );
  }
  PointLocalReconstructionInformation& addPointLocalReconstructionInformation() {
    PointLocalReconstructionInformation plri;
    pointLocalReconstructionInformation_.push_back( plri );
    return pointLocalReconstructionInformation_.back();
  }

 private:
  uint8_t                                          altasSequenceParameterSetId_;
  uint16_t                                         frameWidth_;
  uint16_t                                         frameHeight_;
  uint8_t                                          log2PatchPackingBlockSize_;
  uint8_t                                          log2MaxAtlasFrameOrderCntLsbMinus4_;
  uint8_t                                          maxDecAtlasFrameBufferingMinus1_;
  bool                                             longTermRefAtlasFramesFlag_;
  uint8_t                                          numRefAtlasFrameListsInAsps_;
  std::vector<RefListStruct>                       refListStruct_;
  bool                                             useEightOrientationsFlag_;
  bool                                             degree45ProjectionPatchPresentFlag_;
  bool                                             normalAxisLimitsQuantizationEnabledFlag_;
  bool                                             normalAxisMaxDeltaValueEnabledFlag_;
  bool                                             removeDuplicatePointEnabledFlag_;
  bool                                             pixelDeinterleavingFlag_;
  bool                                             patchPrecedenceOrderFlag_;
  bool                                             patchSizeQuantizerPresentFlag_;
  bool                                             enhancedOccupancyMapForDepthFlag_;
  bool                                             pointLocalReconstructionEnabledFlag_;
  uint8_t                                          mapCountMinus1_;
  uint8_t                                          enhancedOccupancyMapFixBitCountMinus1_;
  std::vector<PointLocalReconstructionInformation> pointLocalReconstructionInformation_;
  uint8_t                                          surfaceThicknessMinus1_;
  bool                                             vuiParametersPresentFlag_;
  bool                                             extensionPresentFlag_;
  bool                                             extensionDataFlag_;

  VUIParameters vuiParameters_;
};

// 7.3.6	Atlas sequence, frame, and tile group parameter set syntax

// 7.3.4.5 Attribute information Syntax
class AttributeInformation {
 public:
  AttributeInformation() :
      attributeCount_( 0 ),
      attributeMSBAlignFlag_( false )
      // JR TODO: remove
      ,
      attributeParamsEnabledFlag_( false ),
      attributePatchParamsEnabledFlag_( false ) {}
  ~AttributeInformation() {
    attributeTypeId_.clear();
    attributeCodecId_.clear();
    rawAttributeCodecId_.clear();
    attributeDimensionMinus1_.clear();
    attributeDimensionPartitionsMinus1_.clear();
    attributeNominal2dBitdepthMinus1_.clear();
    for ( auto& value : attributePartitionChannelsMinus1_ ) { value.clear(); }
    attributePartitionChannelsMinus1_.clear();
    for ( auto& value : attributeMapAbsoluteCodingEnabledFlagList_ ) { value.clear(); }
  }
  AttributeInformation& operator=( const AttributeInformation& ) = default;

  void allocate() {
    attributeTypeId_.resize( attributeCount_, 0 );
    attributeCodecId_.resize( attributeCount_, 0 );
    rawAttributeCodecId_.resize( attributeCount_, 0 );
    attributeDimensionMinus1_.resize( attributeCount_, 0 );
    attributeDimensionPartitionsMinus1_.resize( attributeCount_, 0 );
    attributeNominal2dBitdepthMinus1_.resize( attributeCount_, 0 );
    attributePartitionChannelsMinus1_.resize( attributeCount_ );
    attributeMapAbsoluteCodingEnabledFlagList_.resize( attributeCount_ );
  }
  uint8_t getAttributeCount() { return attributeCount_; }
  bool    getAttributeMSBAlignFlag() { return attributeMSBAlignFlag_; }
  uint8_t getAttributeTypeId( uint32_t index ) { return attributeTypeId_[index]; }
  uint8_t getAttributeCodecId( uint32_t index ) { return attributeCodecId_[index]; };
  uint8_t getRawAttributeCodecId( uint32_t index ) { return rawAttributeCodecId_[index]; }
  uint8_t getAttributeDimensionMinus1( uint32_t index ) { return attributeDimensionMinus1_[index]; }
  uint8_t getAttributeDimensionPartitionsMinus1( uint32_t index ) { return attributeDimensionPartitionsMinus1_[index]; }
  uint8_t getAttributeNominal2dBitdepthMinus1( uint32_t index ) { return attributeNominal2dBitdepthMinus1_[index]; }
  uint8_t getAttributePartitionChannelsMinus1( uint32_t index, uint32_t j ) {
    if ( j >= attributePartitionChannelsMinus1_[index].size() ) {
      attributePartitionChannelsMinus1_[index].resize( j + 1, 0 );
    }
    return attributePartitionChannelsMinus1_[index][j];
  }
  std::vector<std::vector<bool>>& getAttributeMapAbsoluteCodingEnabledFlagList() {
    return attributeMapAbsoluteCodingEnabledFlagList_;
  }
  std::vector<bool>& getAttributeMapAbsoluteCodingEnabledFlagList( size_t attIdx ) {
    return attributeMapAbsoluteCodingEnabledFlagList_[attIdx];
  }
  uint8_t getAttributeMapAbsoluteCodingEnabledFlag( size_t attIdx, size_t mapIdx ) {
    return (uint8_t)attributeMapAbsoluteCodingEnabledFlagList_[attIdx][mapIdx];
  }
  void setAttributeCount( uint8_t value ) { attributeCount_ = value; }
  void setAttributeMSBAlignFlag( bool value ) { attributeMSBAlignFlag_ = value; }
  void setAttributeTypeId( uint32_t index, uint8_t value ) { attributeTypeId_[index] = value; }
  void setAttributeCodecId( uint32_t index, uint8_t value ) { attributeCodecId_[index] = value; };
  void setRawAttributeCodecId( uint32_t index, uint8_t value ) { rawAttributeCodecId_[index] = value; }
  void setAttributeDimensionMinus1( uint32_t index, uint8_t value ) { attributeDimensionMinus1_[index] = value; }
  void setAttributeDimensionPartitionsMinus1( uint32_t index, uint8_t value ) {
    attributeDimensionPartitionsMinus1_[index] = value;
  }
  void setAttributeNominal2dBitdepthMinus1( uint32_t index, uint8_t value ) {
    attributeNominal2dBitdepthMinus1_[index] = value;
  }
  void setAttributePartitionChannelsMinus1( uint32_t index, uint32_t j, uint8_t value ) {
    if ( j >= attributePartitionChannelsMinus1_[index].size() ) {
      attributePartitionChannelsMinus1_[index].resize( j + 1, 0 );
    }
    attributePartitionChannelsMinus1_[index][j] = value;
  }
  bool setAttributeMapAbsoluteCodingEnabledFlag( size_t attIdx, size_t mapIdx, bool value ) {
    bool bSizeAdjusted = false;
    if ( attributeMapAbsoluteCodingEnabledFlagList_[attIdx].size() <= mapIdx ) {
      attributeMapAbsoluteCodingEnabledFlagList_[attIdx].resize( mapIdx + 1 );
      bSizeAdjusted = true;
    }

    attributeMapAbsoluteCodingEnabledFlagList_[attIdx][mapIdx] = value;
    return bSizeAdjusted;
  }
  void addAttributeMapAbsoluteCodingEnabledFlag( size_t attIdx, bool value ) {
    attributeMapAbsoluteCodingEnabledFlagList_[attIdx].push_back( value );
  }
  // JR TODO: remove
  uint8_t getAttributeParamsEnabledFlag() { return attributeParamsEnabledFlag_; }
  uint8_t getAttributePatchParamsEnabledFlag() { return attributePatchParamsEnabledFlag_; }
  void    setAttributeParamsEnabledFlag( bool value ) { attributeParamsEnabledFlag_ = value; }
  void    setAttributePatchParamsEnabledFlag( bool value ) { attributePatchParamsEnabledFlag_ = value; }

 private:
  uint8_t                           attributeCount_;
  std::vector<uint8_t>              attributeTypeId_;
  std::vector<uint8_t>              attributeCodecId_;
  std::vector<uint8_t>              rawAttributeCodecId_;
  std::vector<std::vector<bool>>    attributeMapAbsoluteCodingEnabledFlagList_;
  std::vector<uint8_t>              attributeDimensionMinus1_;
  std::vector<uint8_t>              attributeDimensionPartitionsMinus1_;
  std::vector<std::vector<uint8_t>> attributePartitionChannelsMinus1_;
  std::vector<uint8_t>              attributeNominal2dBitdepthMinus1_;
  bool                              attributeMSBAlignFlag_;
  // JR TODO: remove
  bool attributeParamsEnabledFlag_;       // TODO: remove?
  bool attributePatchParamsEnabledFlag_;  // TODO: remove?
};

// 7.3.4.4 Geometry information Syntax
class GeometryInformation {
 public:
  GeometryInformation() :
      geometryCodecId_( 0 ),
      geometryNominal2dBitdepthMinus1_( 10 ),
      geometryMSBAlignFlag_( false ),
      geometry3dCoordinatesBitdepthMinus1_( 9 ),
      rawGeometryCodecId_( 0 ),
      // geometryPatchBlockFilteringEnableFlag_( 0 ),
      // geometryPatchBlockFilteringPassesCountMinus1_( 0 ),
      // geometryPatchBlockFilteringFilterSizeMinus1_( 0 ),
      // geometryPatchBlockFilteringLog2ThresholdMinus1_( 0 ),
      geometryParamsEnabledFlag_( false ),
      geometryPatchParamsEnabledFlag_( false ) {}
  ~GeometryInformation() {}
  GeometryInformation& operator=( const GeometryInformation& ) = default;

  void init( uint8_t codecId,
             uint8_t nominal2dBitdepthMinus1,
             bool    msbAlignFlag,
             uint8_t coordinatesBitdepthMinus1,
             uint8_t rawGeometryCodecId,
             bool    paramsEnabledFlag,
             bool    patchParamsEnabledFlag ) {
    geometryCodecId_                     = codecId;
    geometryNominal2dBitdepthMinus1_     = nominal2dBitdepthMinus1;
    geometryMSBAlignFlag_                = msbAlignFlag;
    geometry3dCoordinatesBitdepthMinus1_ = coordinatesBitdepthMinus1;
    rawGeometryCodecId_                  = rawGeometryCodecId;
    geometryParamsEnabledFlag_           = paramsEnabledFlag;
    geometryPatchParamsEnabledFlag_      = patchParamsEnabledFlag;
  }
  uint8_t getGeometryCodecId() { return geometryCodecId_; }
  uint8_t getGeometryNominal2dBitdepthMinus1() { return geometryNominal2dBitdepthMinus1_; }
  bool    getGeometryMSBAlignFlag() { return geometryMSBAlignFlag_; }
  uint8_t getGeometry3dCoordinatesBitdepthMinus1() { return geometry3dCoordinatesBitdepthMinus1_; }
  uint8_t getRawGeometryCodecId() { return rawGeometryCodecId_; }
  void    setGeometryCodecId( uint8_t value ) { geometryCodecId_ = value; }
  void    setGeometryNominal2dBitdepthMinus1( uint8_t value ) { geometryNominal2dBitdepthMinus1_ = value; }
  void    setGeometryMSBAlignFlag( bool value ) { geometryMSBAlignFlag_ = value; }
  void    setGeometry3dCoordinatesBitdepthMinus1( uint8_t value ) { geometry3dCoordinatesBitdepthMinus1_ = value; }
  void    setRawGeometryCodecId( uint8_t value ) { rawGeometryCodecId_ = value; }

  // JR TODO: remove
  void setGeometryParamsEnabledFlag( bool value ) { geometryParamsEnabledFlag_ = value; }
  void setGeometryPatchParamsEnabledFlag( bool value ) { geometryPatchParamsEnabledFlag_ = value; }
  bool getGeometryParamsEnabledFlag() { return geometryParamsEnabledFlag_; }
  bool getGeometryPatchParamsEnabledFlag() { return geometryPatchParamsEnabledFlag_; }

  // bool     getGeometryPatchBlockFilteringEnableFlag() { return geometryPatchBlockFilteringEnableFlag_; }
  // uint32_t getGeometryPatchBlockFilteringPassesCountMinus1() { return geometryPatchBlockFilteringPassesCountMinus1_;
  // } uint32_t getGeometryPatchBlockFilteringFilterSizeMinus1() { return geometryPatchBlockFilteringFilterSizeMinus1_;
  // } uint32_t getGeometryPatchBlockFilteringLog2ThresholdMinus1() {
  //   return geometryPatchBlockFilteringLog2ThresholdMinus1_;
  // }
  // void setGeometryPatchBlockFilteringEnableFlag( bool value ) { geometryPatchBlockFilteringEnableFlag_ = value; }
  // void setGeometryPatchBlockFilteringPassesCountMinus1( uint32_t value ) {
  //   geometryPatchBlockFilteringPassesCountMinus1_ = value;
  // }
  // void setGeometryPatchBlockFilteringFilterSizeMinus1( uint32_t value ) {
  //   geometryPatchBlockFilteringFilterSizeMinus1_ = value;
  // }
  // void setGeometryPatchBlockFilteringLog2ThresholdMinus1( uint32_t value ) {
  //   geometryPatchBlockFilteringLog2ThresholdMinus1_ = value;
  // }

 private:
  uint8_t geometryCodecId_;
  uint8_t geometryNominal2dBitdepthMinus1_;
  bool    geometryMSBAlignFlag_;
  uint8_t geometry3dCoordinatesBitdepthMinus1_;
  uint8_t rawGeometryCodecId_;

  // // jkei: here or sei?
  // bool    geometryPatchBlockFilteringEnableFlag_;
  // uint8_t geometryPatchBlockFilteringPassesCountMinus1_;
  // uint8_t geometryPatchBlockFilteringFilterSizeMinus1_;
  // uint8_t geometryPatchBlockFilteringLog2ThresholdMinus1_;

  // JR TODO: remove
  bool geometryParamsEnabledFlag_;
  bool geometryPatchParamsEnabledFlag_;
};

// 7.3.4.3 Occupancy information Set Syntax
class OccupancyInformation {
 public:
  OccupancyInformation() :
      occupancyCodecId_( 0 ),
      lossyOccupancyMapCompressionThreshold_( 0 ),
      occupancyNominal2DBitdepthMinus1_( 10 ),
      occupancyMSBAlignFlag_( false ) {}
  ~OccupancyInformation() {}
  OccupancyInformation& operator=( const OccupancyInformation& ) = default;
  void                  init( uint8_t codecId, uint8_t threshold, uint8_t nominal2DBitdepth, bool msbAlignFlag ) {
    occupancyCodecId_                      = codecId;
    lossyOccupancyMapCompressionThreshold_ = threshold;
    occupancyNominal2DBitdepthMinus1_      = nominal2DBitdepth;
    occupancyMSBAlignFlag_                 = msbAlignFlag;
  }
  uint8_t getOccupancyCodecId() { return occupancyCodecId_; }
  uint8_t getLossyOccupancyMapCompressionThreshold() { return lossyOccupancyMapCompressionThreshold_; }
  uint8_t getOccupancyNominal2DBitdepthMinus1() { return occupancyNominal2DBitdepthMinus1_; }
  bool    getOccupancyMSBAlignFlag() { return occupancyMSBAlignFlag_; }
  void    setOccupancyCodecId( uint8_t value ) { occupancyCodecId_ = value; }
  void    setLossyOccupancyMapCompressionThreshold( uint8_t value ) { lossyOccupancyMapCompressionThreshold_ = value; }
  void    setOccupancyNominal2DBitdepthMinus1( uint8_t value ) { occupancyNominal2DBitdepthMinus1_ = value; }
  void    setOccupancyMSBAlignFlag( bool value ) { occupancyMSBAlignFlag_ = value; }

 private:
  uint8_t occupancyCodecId_;
  uint8_t lossyOccupancyMapCompressionThreshold_;
  uint8_t occupancyNominal2DBitdepthMinus1_;
  bool    occupancyMSBAlignFlag_;
};

// 7.3.4.2 Profile, Tier and Level Syntax
class ProfileTierLevel {
 public:
  ProfileTierLevel() :
      tierFlag_( false ),
      profileCodecGroupIdc_( 0 ),
      profilePccToolsetIdc_( 0 ),
      profileReconctructionIdc_( 0 ),
      levelIdc_( 0 ) {}
  ~ProfileTierLevel() {}
  ProfileTierLevel& operator=( const ProfileTierLevel& ) = default;
  bool              getTierFlag() { return tierFlag_; }
  uint8_t           getProfileCodecGroupIdc() { return profileCodecGroupIdc_; }
  uint8_t           getProfilePccToolsetIdc() { return profilePccToolsetIdc_; }
  uint8_t           getProfileReconctructionIdc() { return profileReconctructionIdc_; }
  uint8_t           getLevelIdc() { return levelIdc_; }
  void              setTierFlag( bool value ) { tierFlag_ = value; }
  void              setProfileCodecGroupIdc( uint8_t value ) { profileCodecGroupIdc_ = value; }
  void              setProfilePccToolsetIdc( uint8_t value ) { profilePccToolsetIdc_ = value; }
  void              setProfileReconctructionIdc( uint8_t value ) { profileReconctructionIdc_ = value; }
  void              setLevelIdc( uint8_t value ) { levelIdc_ = value; }

 private:
  bool    tierFlag_;
  uint8_t profileCodecGroupIdc_;
  uint8_t profilePccToolsetIdc_;
  uint8_t profileReconctructionIdc_;
  uint8_t levelIdc_;
};

// 7.3.4.1  General V-PCC Sequence parameter set syntax
class VpccParameterSet {
 public:
  VpccParameterSet() :
      vpccParameterSetId_( 0 ),
      atlasCountMinus1_( 0 ),
      extensionPresentFlag_( false ),
      extensionLength_( 0 )

      // JR: remove
      ,
      avgFrameRate_( 0 ),
      avgFrameRatePresentFlag_( false ) {}
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

    // remove
    avgFrameRatePresentFlag_ = avgFrameRatePresentFlag;
    avgFrameRate_            = avgFrameRate;
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

  // JR: remove
  bool     getAvgFrameRatePresentFlag() { return avgFrameRatePresentFlag_; }
  uint16_t getAvgFrameRate() { return avgFrameRate_; }
  // PointLocalReconstructionInformation& getPointLocalReconstructionInformation() {
  //   return pointLocalReconstructionInformation_;
  // }

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

  // JR: Remove
  void setAvgFrameRate( size_t atlasIndex, uint16_t value ) { avgFrameRate_ = value; }
  void setAvgFrameRatePresentFlag( bool value ) { avgFrameRatePresentFlag_ = value; }
  void setProfileTierLevel( ProfileTierLevel value ) { profileTierLevel_ = value; }

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

  // TODO: syntax elements not present in DIS text (JR: must me remove)
  uint16_t avgFrameRate_;             // TODO: remove?
  bool     avgFrameRatePresentFlag_;  // TODO: remove?
  //  bool                                enhancedOccupancyMapForDepthFlag_;     // TODO: remove?
  //  size_t                              EOMFixBitCount_;                       // TODO: remove?
  // bool                                patchInterPredictionEnabledFlag_;      // TODO: remove?
  // bool                                pixelDeinterleavingFlag_;              // TODO: remove?
  // bool                                pointLocalReconstructionEnabledFlag_;  // TODO: remove?
  //  bool                                removeDuplicatePointEnabledFlag_;      // TODO: remove?
  //  bool                                projection45degreeEnabledFlag_;        // TODO: remove?
  //  bool                                patchPrecedenceOrderFlag_;             // TODO: remove?
  // PointLocalReconstructionInformation pointLocalReconstructionInformation_;  // TODO: remove?

  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
 public:
  bool    getLosslessGeo444() { return losslessGeo444_; }
  bool    getLosslessGeo() { return losslessGeo_; }
  uint8_t getMinLevel() { return minLevel_; }
  size_t  getSurfaceThickness() { return surfaceThickness_; }
  void    setLosslessGeo444( bool losslessGeo444 ) { losslessGeo444_ = losslessGeo444; }
  void    setLosslessGeo( bool losslessGeo ) { losslessGeo_ = losslessGeo; }
  void    setMinLevel( uint8_t minLevel ) { minLevel_ = minLevel; }
  void    setSurfaceThickness( size_t surfaceThickness ) { surfaceThickness_ = surfaceThickness; }

 private:
  bool    losslessGeo444_;
  bool    losslessGeo_;
  size_t  surfaceThickness_;
  uint8_t minLevel_;
  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
};

// 7.3.2.2  V-PCC unit header syntax
class vpccUnitPayloadHeader {
 public:
  vpccUnitPayloadHeader() :
      unitType_( 0 ),
      sequenceParamterSetId_( 0 ),
      atlasId_( 0 ),
      attributeIndex_( 0 ),
      attributeDimensionIndex_( 0 ),
      mapIndex_( 0 ),
      rawVideoFlag_( false ) {}

  ~vpccUnitPayloadHeader() {}
  vpccUnitPayloadHeader& operator=( const vpccUnitPayloadHeader& ) = default;
  uint8_t                getVpccUnitType() { return unitType_; }
  uint8_t                getVpccParameterSetId() { return sequenceParamterSetId_; }
  uint8_t                getAtlasId() { return atlasId_; }
  uint8_t                getAttributeIndex() { return attributeIndex_; }
  uint8_t                getAttributeDimensionIndex() { return attributeDimensionIndex_; }
  uint8_t                getMapIndex() { return mapIndex_; }
  bool                   getRawVideoFlag() { return rawVideoFlag_; }
  void                   setVpccUnitType( uint8_t value ) { unitType_ = value; }
  void                   setVpccParameterSetId( uint8_t value ) { sequenceParamterSetId_ = value; }
  void                   setAtlasId( uint8_t value ) { atlasId_ = value; }
  void                   setAttributeIndex( uint8_t value ) { attributeIndex_ = value; }
  void                   setAttributeDimensionIndex( uint8_t value ) { attributeDimensionIndex_ = value; }
  void                   setMapIndex( uint8_t value ) { mapIndex_ = value; }
  void                   setRawVideoFlag( bool value ) { rawVideoFlag_ = value; }

 private:
  uint8_t unitType_;
  uint8_t sequenceParamterSetId_;
  uint8_t atlasId_;
  uint8_t attributeIndex_;
  uint8_t attributeDimensionIndex_;
  uint8_t mapIndex_;
  bool    rawVideoFlag_;
};

typedef struct PointLocalReconstructionMode {
  bool    interpolate_;
  bool    filling_;
  uint8_t minD1_;
  uint8_t neighbor_;
} PointLocalReconstructionMode;

// // 7.3.7.1  Patch tile group header syntax (ptgh) (jkei: replce this with Atlas tile group header) -> TODO: REMOVE
// class PatchTileGroupHeader {
//  public:
//   PatchTileGroupHeader() :
//       frameIndex_( 0 ),
//       patchFrameParameterSetId_( 0 ),
//       type_( 0 ),
//       address_( 0 ),
//       patchFrameOrderCntLsb_( 0 ),
//       refPatchFrameListIdx_( 0 ),
//       refPatchFrameListSpsFlag_( 0 ),
//       numRefIdxActiveOverrideFlag_( false ),
//       numRefIdxActiveMinus1_( 0 ),
//       interPredictPatch2dShiftUBitCountMinus1_( 0 ),
//       interPredictPatch2dShiftVBitCountMinus1_( 0 ),
//       interPredictPatch2dDeltaSizeDBitCountMinus1_( 0 ),
//       interPredictPatch3dShiftTangentAxisBitCountMinus1_( 0 ),
//       interPredictPatch3dShiftBitangentAxisBitCountMinus1_( 0 ),
//       interPredictPatch3dShiftNormalAxisBitCountMinus1_( 0 ),
//       interPredictPatchLodBitCount_( 0 ),
//       interPredictPatchBitCountFlag_( false ),
//       interPredictPatch2dShiftUBitCountFlag_( false ),
//       interPredictPatch2dShiftVBitCountFlag_( false ),
//       interPredictPatch3dShiftTangentAxisBitCountFlag_( false ),
//       interPredictPatch3dShiftBitangentAxisBitCountFlag_( false ),
//       interPredictPatch3dShiftNormalAxisBitCountFlag_( false ),
//       interPredictPatchLodBitCountFlag_( false ),
//       raw3dShiftAxisBitCountMinus1_( 9 ),
//       raw3dShiftBitCountPresentFlag_( true ),
//       eomPatchNbPatchBitCountMinus1_( 0 ),
//       eomPatchMaxEPBitCountMinus1_( 0 ) {
//     additionalPfocLsbPresentFlag_.resize( 1, 0 );
//     additionalPfocLsbVal_.resize( 1, 0 );
//   }

//   ~PatchTileGroupHeader() {
//     additionalPfocLsbPresentFlag_.clear();
//     additionalPfocLsbVal_.clear();
//   }

//   PatchTileGroupHeader& operator=( const PatchTileGroupHeader& ) = default;

//   uint8_t  getFrameIndex() { return frameIndex_; }
//   uint8_t  getPatchFrameParameterSetId() { return patchFrameParameterSetId_; }
//   uint32_t getAddress() { return address_; }
//   uint8_t  getType() { return type_; }
//   uint8_t  getPatchFrameOrderCntLsb() { return patchFrameOrderCntLsb_; }
//   bool     getRefPatchFrameListSpsFlag() { return refPatchFrameListSpsFlag_; }
//   uint8_t  getRefPatchFrameListIdx() { return refPatchFrameListIdx_; }
//   bool     getAdditionalPfocLsbPresentFlag( size_t index ) { return additionalPfocLsbPresentFlag_[index]; }
//   uint32_t getAdditionalPfocLsbVal( size_t index ) { return additionalPfocLsbVal_[index]; }
//   bool     getNumRefIdxActiveOverrideFlag() { return numRefIdxActiveOverrideFlag_; }
//   uint8_t  getNumRefIdxActiveMinus1() { return numRefIdxActiveMinus1_; }
//   bool     getInterPredictPatchBitCountFlag() { return interPredictPatchBitCountFlag_; }
//   bool     getInterPredictPatch2dShiftUBitCountFlag() { return interPredictPatch2dShiftUBitCountFlag_; }
//   bool     getInterPredictPatch2dShiftVBitCountFlag() { return interPredictPatch2dShiftVBitCountFlag_; }
//   bool getInterPredictPatch3dShiftTangentAxisBitCountFlag() { return
//   interPredictPatch3dShiftTangentAxisBitCountFlag_; } bool getInterPredictPatch3dShiftBitangentAxisBitCountFlag() {
//     return interPredictPatch3dShiftBitangentAxisBitCountFlag_;
//   }
//   bool getInterPredictPatch3dShiftNormalAxisBitCountFlag() { return interPredictPatch3dShiftNormalAxisBitCountFlag_;
//   } bool getInterPredictPatchLodBitCountFlag() { return interPredictPatchLodBitCountFlag_; } uint8_t
//   getInterPredictPatch2dShiftUBitCountMinus1() { return interPredictPatch2dShiftUBitCountMinus1_; } uint8_t
//   getInterPredictPatch2dShiftVBitCountMinus1() { return interPredictPatch2dShiftVBitCountMinus1_; } uint8_t
//   getInterPredictPatch2dDeltaSizeDBitCountMinus1() { return interPredictPatch2dDeltaSizeDBitCountMinus1_; } uint8_t
//   getInterPredictPatch3dShiftTangentAxisBitCountMinus1() {
//     return interPredictPatch3dShiftTangentAxisBitCountMinus1_;
//   }
//   uint8_t getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() {
//     return interPredictPatch3dShiftBitangentAxisBitCountMinus1_;
//   }
//   uint8_t getInterPredictPatch3dShiftNormalAxisBitCountMinus1() {
//     return interPredictPatch3dShiftNormalAxisBitCountMinus1_;
//   }
//   uint8_t getRaw3dShiftAxisBitCountMinus1() { return raw3dShiftAxisBitCountMinus1_; }
//   bool    getRaw3dShiftBitCountPresentFlag() { return raw3dShiftBitCountPresentFlag_; }
//   uint8_t getInterPredictPatchLodBitCount() { return interPredictPatchLodBitCount_; }
//   uint8_t getNormalAxisMinValueQuantizer() { return normalAxisMinValueQuantizer_; }
//   uint8_t getNormalAxisMaxDeltaValueQuantizer() { return normalAxisMaxDeltaValueQuantizer_; }

//   void setFrameIndex( uint8_t index ) { frameIndex_ = index; }
//   void setPatchFrameParameterSetId( uint8_t value ) { patchFrameParameterSetId_ = value; }
//   void setAddress( uint8_t value ) { address_ = value; }
//   void setType( uint8_t value ) { type_ = value; }
//   void setPatchFrameOrderCntLsb( uint8_t value ) { patchFrameOrderCntLsb_ = value; }
//   void setRefPatchFrameListSpsFlag( bool value ) { refPatchFrameListSpsFlag_ = value; }
//   void setRefPatchFrameListIdx( uint8_t value ) { refPatchFrameListIdx_ = value; }
//   void setAdditionalPfocLsbPresentFlag( size_t index, bool value ) { additionalPfocLsbPresentFlag_[index] = value; }
//   void setAdditionalPfocLsbVal( size_t index, uint32_t value ) { additionalPfocLsbVal_[index] = value; }
//   void setNumRefIdxActiveOverrideFlag( bool value ) { numRefIdxActiveOverrideFlag_ = value; }
//   void setNumRefIdxActiveMinus1( uint8_t value ) { numRefIdxActiveMinus1_ = value; }
//   void setInterPredictPatch2dDeltaSizeDBitCountMinus1( uint8_t value ) {
//     interPredictPatch2dDeltaSizeDBitCountMinus1_ = value;
//   }
//   void setInterPredictPatch2dShiftUBitCountMinus1( uint8_t value ) { interPredictPatch2dShiftUBitCountMinus1_ =
//   value; } void setInterPredictPatch2dShiftVBitCountMinus1( uint8_t value ) {
//   interPredictPatch2dShiftVBitCountMinus1_ = value; } void setInterPredictPatch3dShiftTangentAxisBitCountMinus1(
//   uint8_t value ) {
//     interPredictPatch3dShiftTangentAxisBitCountMinus1_ = value;
//   }
//   void setInterPredictPatch3dShiftBitangentAxisBitCountMinus1( uint8_t value ) {
//     interPredictPatch3dShiftBitangentAxisBitCountMinus1_ = value;
//   }
//   void setInterPredictPatch3dShiftNormalAxisBitCountMinus1( uint8_t value ) {
//     interPredictPatch3dShiftNormalAxisBitCountMinus1_ = value;
//   }
//   void setRaw3dShiftAxisBitCountMinus1( uint8_t value ) { raw3dShiftAxisBitCountMinus1_ = value; }
//   void setRaw3dShiftBitCountPresentFlag( uint8_t value ) { raw3dShiftBitCountPresentFlag_ = value; }
//   void setInterPredictPatchLodBitCount( uint8_t value ) { interPredictPatchLodBitCount_ = value; }
//   void setInterPredictPatchBitCountFlag( bool value ) { interPredictPatchBitCountFlag_ = value; }
//   void setInterPredictPatch2dShiftUBitCountFlag( bool value ) { interPredictPatch2dShiftUBitCountFlag_ = value; }
//   void setInterPredictPatch2dShiftVBitCountFlag( bool value ) { interPredictPatch2dShiftVBitCountFlag_ = value; }
//   void setInterPredictPatch3dShiftTangentAxisBitCountFlag( bool value ) {
//     interPredictPatch3dShiftTangentAxisBitCountFlag_ = value;
//   }
//   void setInterPredictPatch3dShiftBitangentAxisBitCountFlag( bool value ) {
//     interPredictPatch3dShiftBitangentAxisBitCountFlag_ = value;
//   }
//   void setInterPredictPatch3dShiftNormalAxisBitCountFlag( bool value ) {
//     interPredictPatch3dShiftNormalAxisBitCountFlag_ = value;
//   }
//   void setInterPredictPatchLodBitCountFlag( bool value ) { interPredictPatchLodBitCountFlag_ = value; }
//   void setNormalAxisMinValueQuantizer( uint8_t value ) { normalAxisMinValueQuantizer_ = value; }
//   void setNormalAxisMaxDeltaValueQuantizer( uint8_t value ) { normalAxisMaxDeltaValueQuantizer_ = value; }

//   uint8_t getEOMPatchNbPatchBitCountMinus1() { return eomPatchNbPatchBitCountMinus1_; }
//   uint8_t getEOMPatchMaxEPBitCountMinus1() { return eomPatchMaxEPBitCountMinus1_; }

//   void setEOMPatchNbPatchBitCountMinus1( uint8_t value ) { eomPatchNbPatchBitCountMinus1_ = value; }
//   void setEOMPatchMaxEPBitCountMinus1( uint8_t value ) { eomPatchMaxEPBitCountMinus1_ = value; }

//  private:
//   uint8_t               frameIndex_;
//   uint8_t               patchFrameParameterSetId_;
//   uint8_t               type_;
//   uint32_t              address_;
//   uint8_t               patchFrameOrderCntLsb_;
//   uint8_t               refPatchFrameListIdx_;
//   bool                  refPatchFrameListSpsFlag_;
//   std::vector<bool>     additionalPfocLsbPresentFlag_;
//   std::vector<uint32_t> additionalPfocLsbVal_;
//   bool                  numRefIdxActiveOverrideFlag_;
//   uint8_t               numRefIdxActiveMinus1_;
//   uint8_t               normalAxisMinValueQuantizer_;
//   uint8_t               normalAxisMaxDeltaValueQuantizer_;
//   uint8_t               interPredictPatch2dShiftUBitCountMinus1_;
//   uint8_t               interPredictPatch2dShiftVBitCountMinus1_;
//   uint8_t               interPredictPatch2dDeltaSizeDBitCountMinus1_;
//   uint8_t               interPredictPatch3dShiftTangentAxisBitCountMinus1_;
//   uint8_t               interPredictPatch3dShiftBitangentAxisBitCountMinus1_;
//   uint8_t               interPredictPatch3dShiftNormalAxisBitCountMinus1_;
//   uint8_t               interPredictPatchLodBitCount_;
//   bool                  interPredictPatchBitCountFlag_;
//   bool                  interPredictPatch2dShiftUBitCountFlag_;
//   bool                  interPredictPatch2dShiftVBitCountFlag_;
//   bool                  interPredictPatch3dShiftTangentAxisBitCountFlag_;
//   bool                  interPredictPatch3dShiftBitangentAxisBitCountFlag_;
//   bool                  interPredictPatch3dShiftNormalAxisBitCountFlag_;
//   bool                  interPredictPatchLodBitCountFlag_;
//   uint8_t               raw3dShiftAxisBitCountMinus1_;
//   bool                  raw3dShiftBitCountPresentFlag_;
//   uint8_t               eomPatchNbPatchBitCountMinus1_;
//   uint8_t               eomPatchMaxEPBitCountMinus1_;
// };

// // 7.3.6  Patch frame data unit syntax (ptgdu) : jkei : replace this one with ATGDU -> TODO: REMOVE
// class PatchTileGroupDataUnit {
//  public:
//   PatchTileGroupDataUnit() {}
//   ~PatchTileGroupDataUnit() { patchInformationData_.clear(); }
//   PatchTileGroupDataUnit& operator=( const PatchTileGroupDataUnit& ) = default;

//   void init() { patchInformationData_.clear(); }
//   void allocate( size_t size ) { patchInformationData_.resize( size ); }

//   void addPatchInformationData( PatchInformationData& value ) { patchInformationData_.push_back( value ); }
//   PatchInformationData& addPatchInformationData( uint8_t patchMode ) {
//     PatchInformationData pid;
//     pid.setPatchMode( patchMode );
//     patchInformationData_.push_back( pid );
//     return patchInformationData_.back();
//   }
//   uint8_t               getPatchMode( size_t index ) { return patchInformationData_[index].getPatchMode(); }
//   uint8_t               getPatchCount() { return patchInformationData_.size(); }
//   PatchInformationData& getPatchInformationData( size_t index ) { return patchInformationData_[index]; }
//   std::vector<PatchInformationData>& getPatchInformationData() { return patchInformationData_; }
//   size_t                             getMatchedPatchCount() {
//     size_t matchedPatchCount = 0;
//     for ( auto& v : patchInformationData_ ) {
//       if ( v.getPatchMode() == PATCH_MODE_P_INTER ) { matchedPatchCount++; }
//     }
//     return matchedPatchCount;
//   }

//   size_t getFrameIndex() { return frameIndex_; }
//   void   setFrameIndex( size_t value ) { frameIndex_ = value; }
//   void   setPatchCount( size_t value ) { patchCount_ = value; }
//   void   setPatchInformationData( size_t index, PatchInformationData& value ) { patchInformationData_[index] = value;
//   }

//  private:
//   size_t                            frameIndex_;
//   size_t                            patchCount_;
//   std::vector<PatchInformationData> patchInformationData_;
// };

// // 7.3.5.14  Patch tile group layer unit syntax (jkei: remove and replace with AtlasTileGroupLayerRbsp) -> TODO:
// REMOVE class PatchTileGroupLayerUnit {
//  public:
//   PatchTileGroupLayerUnit() : frameIndex_( 0 ) {}
//   ~PatchTileGroupLayerUnit() {}

//   PatchTileGroupLayerUnit& operator=( const PatchTileGroupLayerUnit& ) = default;

//   uint8_t                 getFrameIndex() { return frameIndex_; }
//   PatchTileGroupHeader&   getPatchTileGroupHeader() { return patchTileGroupHeader_; }
//   PatchTileGroupDataUnit& getPatchTileGroupDataUnit() { return patchTileGroupDataUnit_; }

//   void setFrameIndex( uint8_t value ) { frameIndex_ = value; }
//   void setPatchTileGroupHeader( PatchTileGroupHeader value ) { patchTileGroupHeader_ = value; }
//   void setPatchTileGroupDataUnit( PatchTileGroupDataUnit value ) { patchTileGroupDataUnit_ = value; }

//  private:
//   uint8_t                frameIndex_;
//   PatchTileGroupHeader   patchTileGroupHeader_;
//   PatchTileGroupDataUnit patchTileGroupDataUnit_;
// };

// 7.3.5.12  Patch frame parameter set syntax (jkei: remove and replace it with AtlasFrameParameterSetRbsp -> TODO:
// REMOVE

// class PatchFrameParameterSet {
//  public:
//   PatchFrameParameterSet() :
//       patchFrameParameterSetId_( 0 ),
//       patchVpccParameterSetId_( 0 ),
//       geometryPatchFrameParameterSetId_( 0 ),
//       additionalLtPfocLsbLen_( 0 ),
//       localOverrideGeometryPatchEnableFlag_( false ),
//       projection45DegreeEnableFlag_( false ),
//       lodModeEnableFlag_( false ) {}
//   ~PatchFrameParameterSet() {
//     localOverrideAttributePatchEnableFlag_.clear();
//     attributePatchFrameParameterSetId_.clear();
//   }
//   PatchFrameParameterSet& operator=( const PatchFrameParameterSet& ) = default;

//   void allocate( size_t size = 255 ) {
//     localOverrideAttributePatchEnableFlag_.resize( size, false );
//     attributePatchFrameParameterSetId_.resize( size, 0 );
//   }
//   bool getLodModeEnableFlag() { return lodModeEnableFlag_; }
//   void setLodModeEnableFlag( bool value ) { lodModeEnableFlag_ = value; }

//   uint8_t getPatchFrameParameterSetId() { return patchFrameParameterSetId_; }
//   uint8_t getPatchVpccParameterSetId() { return patchVpccParameterSetId_; }
//   uint8_t getGeometryPatchFrameParameterSetId() { return geometryPatchFrameParameterSetId_; }
//   uint8_t getAttributePatchFrameParameterSetId( size_t index ) { return attributePatchFrameParameterSetId_[index]; }
//   uint8_t getAdditionalLtPfocLsbLen() { return additionalLtPfocLsbLen_; }
//   bool    getLocalOverrideAttributePatchEnableFlag( size_t index ) {
//     return localOverrideAttributePatchEnableFlag_[index];
//   }
//   bool                       getLocalOverrideGeometryPatchEnableFlag() { return
//   localOverrideGeometryPatchEnableFlag_; } bool                       getProjection45DegreeEnableFlag() { return
//   projection45DegreeEnableFlag_; } AtlasFrameTileInformation& getAtlasFrameTileInformation() { return
//   atlasFrameTileInformation_; } void                       setPatchFrameParameterSetId( uint8_t value ) {
//   patchFrameParameterSetId_ = value; } void                       setPatchVpccParameterSetId( uint8_t value ) {
//   patchVpccParameterSetId_ = value; } void setGeometryPatchFrameParameterSetId( uint8_t value ) {
//   geometryPatchFrameParameterSetId_ = value; } void setAttributePatchFrameParameterSetId( size_t index, uint8_t value
//   ) {
//     attributePatchFrameParameterSetId_[index] = value;
//   }
//   void setAdditionalLtPfocLsbLen( uint8_t value ) { additionalLtPfocLsbLen_ = value; }
//   void setLocalOverrideAttributePatchEnableFlag( size_t index, bool value ) {
//     localOverrideAttributePatchEnableFlag_[index] = value;
//   }
//   void setLocalOverrideGeometryPatchEnableFlag( bool value ) { localOverrideGeometryPatchEnableFlag_ = value; }
//   void setProjection45DegreeEnableFlag( bool value ) { projection45DegreeEnableFlag_ = value; }
//   void setAtlasFrameTileInformation( AtlasFrameTileInformation value ) { atlasFrameTileInformation_ = value; }

//  private:
//   uint8_t                   patchFrameParameterSetId_;
//   uint8_t                   patchVpccParameterSetId_;
//   uint8_t                   geometryPatchFrameParameterSetId_;
//   std::vector<uint8_t>      attributePatchFrameParameterSetId_;
//   uint8_t                   additionalLtPfocLsbLen_;
//   bool                      localOverrideGeometryPatchEnableFlag_;
//   std::vector<bool>         localOverrideAttributePatchEnableFlag_;
//   bool                      projection45DegreeEnableFlag_;
//   bool                      lodModeEnableFlag_;
//   AtlasFrameTileInformation atlasFrameTileInformation_;
// };

// // 7.3.5.11 Attribute patch params syntax (apps) -> TODO: REMOVE
// class AttributePatchParams {
//  public:
//   AttributePatchParams() :
//       attributePatchScaleParamsPresentFlag_( false ),
//       attributePatchOffsetParamsPresentFlag_( false ) {
//     attributePatchScale_.clear();
//     attributePatchOffset_.clear();
//   }
//   ~AttributePatchParams() {
//     attributePatchScale_.clear();
//     attributePatchOffset_.clear();
//   }
//   void allocate( size_t size = 255 ) {
//     attributePatchScale_.resize( size, 0 );
//     attributePatchOffset_.resize( size, 0 );
//   }
//   AttributePatchParams& operator=( const AttributePatchParams& ) = default;
//   bool                  getAttributePatchScaleParamsPresentFlag() { return attributePatchScaleParamsPresentFlag_; }
//   bool                  getAttributePatchOffsetParamsPresentFlag() { return attributePatchOffsetParamsPresentFlag_; }
//   uint32_t              getAttributePatchScale( size_t index ) { return attributePatchScale_[index]; }
//   int32_t               getAttributePatchOffset( size_t index ) { return attributePatchOffset_[index]; }

//   void setAttributePatchScaleParamsPresentFlag( bool value ) { attributePatchScaleParamsPresentFlag_ = value; }
//   void setAttributePatchOffsetParamsPresentFlag( bool value ) { attributePatchOffsetParamsPresentFlag_ = value; }
//   void setAttributePatchScale( size_t index, uint32_t value ) { attributePatchScale_[index] = value; }
//   void setAttributePatchOffset( size_t index, int32_t value ) { attributePatchOffset_[index] = value; }

//  private:
//   bool                  attributePatchScaleParamsPresentFlag_;
//   std::vector<uint32_t> attributePatchScale_;
//   bool                  attributePatchOffsetParamsPresentFlag_;
//   std::vector<int32_t>  attributePatchOffset_;
// };

// // 7.3.5.10 Attribute Patch Parameter Set syntax (appss) -> TODO: REMOVE
// class AttributePatchParameterSet {
//  public:
//   AttributePatchParameterSet() :
//       attributePatchParameterSetId_( 0 ),
//       patchFrameAttributeParameterSetId_( 0 ),
//       attributeDimensionMinus1_( 0 ),
//       attributePatchParamsPresentFlag_( false ) {}
//   ~AttributePatchParameterSet() {}
//   AttributePatchParameterSet& operator=( const AttributePatchParameterSet& ) = default;

//   uint8_t               getAttributePatchParameterSetId() { return attributePatchParameterSetId_; }
//   uint8_t               getPatchFrameAttributeParameterSetId() { return patchFrameAttributeParameterSetId_; }
//   bool                  getAttributePatchParamsPresentFlag() { return attributePatchParamsPresentFlag_; }
//   AttributePatchParams& getAttributePatchParams() { return attributePatchParams_; }
//   uint8_t               getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
//   void                  setAttributePatchParameterSetId( uint8_t value ) { attributePatchParameterSetId_ = value; }
//   void setPatchFrameAttributeParameterSetId( uint8_t value ) { patchFrameAttributeParameterSetId_ = value; }
//   void setAttributePatchParamsPresentFlag( bool value ) { attributePatchParamsPresentFlag_ = value; }
//   void setAttributePatchParams( AttributePatchParams value ) { attributePatchParams_ = value; }
//   void setAttributeDimensionMinus1( uint8_t value ) { attributeDimensionMinus1_ = value; }

//  private:
//   uint8_t              attributePatchParameterSetId_;
//   uint8_t              patchFrameAttributeParameterSetId_;
//   uint8_t              attributeDimensionMinus1_;
//   bool                 attributePatchParamsPresentFlag_;
//   AttributePatchParams attributePatchParams_;
// };

// // 7.3.5.9 Geometry patch params syntax -> TODO: REMOVE
// class GeometryPatchParams {
//  public:
//   GeometryPatchParams() :
//       geometryPatchScaleParamsPresentFlag_( false ),
//       geometryPatchOffsetParamsPresentFlag_( false ),
//       geometryPatchRotationParamsPresentFlag_( false ),
//       geometryPatchPointSizeInfoPresentFlag_( false ),
//       geometryPatchPointShapeInfoPresentFlag_( false ),
//       geometryPatchPointSizeInfo_( 0 ),
//       geometryPatchPointShapeInfo_( 0 ) {
//     for ( size_t d = 0; d < 3; d++ ) {
//       geometryPatchScaleOnAxis_[d]  = 0;  // u(32)
//       geometryPatchOffsetOnAxis_[d] = 0;  // i(32)
//     }
//     for ( size_t d = 0; d < 4; d++ ) {
//       geometryPatchRotationXYZW_[d] = 0;  // i(32)
//     }
//   }
//   ~GeometryPatchParams() {}

//   GeometryPatchParams& operator=( const GeometryPatchParams& ) = default;

//   bool     getGeometryPatchScaleParamsPresentFlag() { return geometryPatchScaleParamsPresentFlag_; }
//   bool     getGeometryPatchOffsetParamsPresentFlag() { return geometryPatchOffsetParamsPresentFlag_; }
//   bool     getGeometryPatchRotationParamsPresentFlag() { return geometryPatchRotationParamsPresentFlag_; }
//   bool     getGeometryPatchPointSizeInfoPresentFlag() { return geometryPatchPointSizeInfoPresentFlag_; }
//   bool     getGeometryPatchPointShapeInfoPresentFlag() { return geometryPatchPointShapeInfoPresentFlag_; }
//   uint32_t getGeometryPatchScaleOnAxis( size_t index ) { return geometryPatchScaleOnAxis_[index]; }
//   int32_t  getGeometryPatchOffsetOnAxis( size_t index ) { return geometryPatchOffsetOnAxis_[index]; }
//   int32_t  getGeometryPatchRotationQuaternion( size_t index ) { return geometryPatchRotationXYZW_[index]; }
//   uint16_t getGeometryPatchPointSizeInfo() { return geometryPatchPointSizeInfo_; }
//   uint32_t getGeometryPatchPointShapeInfo() { return geometryPatchPointShapeInfo_; }

//   void setGeometryPatchScaleParamsPresentFlag( bool value ) { geometryPatchScaleParamsPresentFlag_ = value; }
//   void setGeometryPatchOffsetParamsPresentFlag( bool value ) { geometryPatchOffsetParamsPresentFlag_ = value; }
//   void setGeometryPatchRotationParamsPresentFlag( bool value ) { geometryPatchRotationParamsPresentFlag_ = value; }
//   void setGeometryPatchPointSizeInfoPresentFlag( bool value ) { geometryPatchPointSizeInfoPresentFlag_ = value; }
//   void setGeometryPatchPointShapeInfoPresentFlag( bool value ) { geometryPatchPointShapeInfoPresentFlag_ = value; }
//   void setGeometryPatchScaleOnAxis( size_t index, uint32_t value ) { geometryPatchScaleOnAxis_[index] = value; }
//   void setGeometryPatchOffsetOnAxis( size_t index, int32_t value ) { geometryPatchOffsetOnAxis_[index] = value; }
//   void setGeometryPatchRotationQuaternion( size_t index, int32_t value ) { geometryPatchRotationXYZW_[index] = value;
//   } void setGeometryPatchPointSizeInfo( uint16_t value ) { geometryPatchPointSizeInfo_ = value; } void
//   setGeometryPatchPointShapeInfo( uint32_t value ) { geometryPatchPointShapeInfo_ = value; }

//  private:
//   bool     geometryPatchScaleParamsPresentFlag_;
//   bool     geometryPatchOffsetParamsPresentFlag_;
//   bool     geometryPatchRotationParamsPresentFlag_;
//   bool     geometryPatchPointSizeInfoPresentFlag_;
//   bool     geometryPatchPointShapeInfoPresentFlag_;
//   uint32_t geometryPatchScaleOnAxis_[3];
//   int32_t  geometryPatchOffsetOnAxis_[3];
//   int32_t  geometryPatchRotationXYZW_[4];
//   uint16_t geometryPatchPointSizeInfo_;
//   uint32_t geometryPatchPointShapeInfo_;
// };
// // 7.3.5.8 Geometry patch parameter set syntax -> TODO: REMOVE
// class GeometryPatchParameterSet {
//  public:
//   GeometryPatchParameterSet() :
//       geometryPatchParameterSetId_( 0 ),
//       patchFrameGeometryParameterSetId_( 0 ),
//       geometryPatchParamsPresentFlag_( false ) {}
//   ~GeometryPatchParameterSet() {}
//   GeometryPatchParameterSet& operator=( const GeometryPatchParameterSet& ) = default;

//   uint8_t              getGeometryPatchParameterSetId() { return geometryPatchParameterSetId_; }
//   uint8_t              getPatchFrameGeometryParameterSetId() { return patchFrameGeometryParameterSetId_; }
//   bool                 getGeometryPatchParamsPresentFlag() { return geometryPatchParamsPresentFlag_; }
//   GeometryPatchParams& getGeometryPatchParams() { return geometryPatchParams_; }

//   void setGeometryPatchParameterSetId( uint8_t value ) { geometryPatchParameterSetId_ = value; }
//   void setPatchFrameGeometryParameterSetId( uint8_t value ) { patchFrameGeometryParameterSetId_ = value; }
//   void setGeometryPatchParamsPresentFlag( bool value ) { geometryPatchParamsPresentFlag_ = value; }
//   void setGeometryPatchParams( GeometryPatchParams value ) { geometryPatchParams_ = value; }

//  private:
//   uint8_t             geometryPatchParameterSetId_;
//   uint8_t             patchFrameGeometryParameterSetId_;
//   bool                geometryPatchParamsPresentFlag_;
//   GeometryPatchParams geometryPatchParams_;
// };

// // 7.3.5.7 Attribute frame paramS syntax (afp) -> TODO: REMOVE
// class AttributeFrameParams {
//  public:
//   AttributeFrameParams() :
//       attributeSmoothingParamsPresentFlag_( false ),
//       attributeScaleParamsPresentFlag_( false ),
//       attributeOffsetParamsPresentFlag_( false ) {
//     attributeScale_.clear();
//     attributeOffset_.clear();
//     attributeSmoothingParamsPresentFlag_.clear();
//     attributeSmoothingGridSizeMinus2_.clear();
//     attributeSmoothingThreshold_.clear();
//     attributeSmoothingThresholdAttributeDifference_.clear();
//     attributeSmoothingThresholdAttributeVariation_.clear();
//     attributeSmoothingLocalEntropyThreshold_.clear();
//   }
//   ~AttributeFrameParams() {
//     attributeScale_.clear();
//     attributeOffset_.clear();
//     attributeSmoothingParamsPresentFlag_.clear();
//     attributeSmoothingGridSizeMinus2_.clear();
//     attributeSmoothingThreshold_.clear();
//     attributeSmoothingThresholdAttributeDifference_.clear();
//     attributeSmoothingThresholdAttributeVariation_.clear();
//     attributeSmoothingLocalEntropyThreshold_.clear();
//   }

//   AttributeFrameParams& operator=( const AttributeFrameParams& ) = default;

//   void allocate( size_t size ) {
//     attributeScale_.resize( size, 0 );
//     attributeOffset_.resize( size, 0 );
//     attributeSmoothingParamsPresentFlag_.resize( size, 0 );
//     attributeSmoothingGridSizeMinus2_.resize( size, 0 );
//     attributeSmoothingThreshold_.resize( size, 0 );
//     attributeSmoothingThresholdAttributeDifference_.resize( size, 0 );
//     attributeSmoothingThresholdAttributeVariation_.resize( size, 0 );
//     attributeSmoothingLocalEntropyThreshold_.resize( size, 0 );
//   }

//   bool    getAttributeSmoothingParamsPresentFlag( size_t index ) { return
//   attributeSmoothingParamsPresentFlag_[index]; } uint8_t getAttributeSmoothingGridSizeMinus2( size_t index ) { return
//   attributeSmoothingGridSizeMinus2_[index]; } uint8_t getAttributeSmoothingThreshold( size_t index ) { return
//   attributeSmoothingThreshold_[index]; } uint32_t getAttributeSmoothingLocalEntropyThreshold( size_t index ) {
//     return attributeSmoothingLocalEntropyThreshold_[index];
//   }
//   uint8_t getAttributeSmoothingThresholdAttributeDifference( size_t index ) {
//     return attributeSmoothingThresholdAttributeDifference_[index];
//   }
//   uint8_t getAttributeSmoothingThresholdAttributeVariation( size_t index ) {
//     return attributeSmoothingThresholdAttributeVariation_[index];
//   }
//   bool     getAttributeScaleParamsPresentFlag() { return attributeScaleParamsPresentFlag_; }
//   bool     getAttributeOffsetParamsPresentFlag() { return attributeOffsetParamsPresentFlag_; }
//   uint32_t getAttributeScale( size_t index ) { return attributeScale_[index]; }
//   int32_t  getAttributeOffset( size_t index ) { return attributeOffset_[index]; }

//   void setAttributeSmoothingParamsPresentFlag( std::vector<bool> value ) {
//     attributeSmoothingParamsPresentFlag_ = value;
//   }
//   void setAttributeSmoothingGridSize( std::vector<uint8_t> value ) { attributeSmoothingGridSizeMinus2_ = value; }
//   void setAttributeSmoothingThreshold( std::vector<uint8_t> value ) { attributeSmoothingThreshold_ = value; }
//   void setAttributeSmoothingLocalEntropyThreshold( std::vector<uint32_t> value ) {
//     attributeSmoothingLocalEntropyThreshold_ = value;
//   }
//   void setAttributeSmoothingThresholdAttributeDifference( std::vector<uint8_t> value ) {
//     attributeSmoothingThresholdAttributeDifference_ = value;
//   }
//   void setAttributeSmoothingThresholdAttributeVariation( std::vector<uint8_t> value ) {
//     attributeSmoothingThresholdAttributeVariation_ = value;
//   }

//   void setAttributeSmoothingParamsPresentFlag( size_t index, bool value ) {
//     attributeSmoothingParamsPresentFlag_[index] = value;
//   }
//   void setAttributeSmoothingGridSizeMinus2( size_t index, uint8_t value ) {
//     attributeSmoothingGridSizeMinus2_[index] = value;
//   }
//   void setAttributeSmoothingThreshold( size_t index, uint8_t value ) { attributeSmoothingThreshold_[index] = value; }
//   void setAttributeSmoothingLocalEntropyThreshold( size_t index, uint32_t value ) {
//     attributeSmoothingLocalEntropyThreshold_[index] = value;
//   }
//   void setAttributeSmoothingThresholdAttributeDifference( size_t index, uint8_t value ) {
//     attributeSmoothingThresholdAttributeDifference_[index] = value;
//   }
//   void setAttributeSmoothingThresholdAttributeVariation( size_t index, uint8_t value ) {
//     attributeSmoothingThresholdAttributeVariation_[index] = value;
//   }
//   void setAttributeScaleParamsPresentFlag( bool value ) { attributeScaleParamsPresentFlag_ = value; }
//   void setAttributeOffsetParamsPresentFlag( bool value ) { attributeOffsetParamsPresentFlag_ = value; }
//   void setAttributeScale( size_t index, uint32_t value ) { attributeScale_[index] = value; }
//   void setAttributeOffset( size_t index, int32_t value ) { attributeOffset_[index] = value; }

//  private:
//   std::vector<bool>     attributeSmoothingParamsPresentFlag_;
//   std::vector<uint8_t>  attributeSmoothingGridSizeMinus2_;
//   std::vector<uint8_t>  attributeSmoothingThreshold_;
//   std::vector<uint32_t> attributeSmoothingLocalEntropyThreshold_;
//   std::vector<uint8_t>  attributeSmoothingThresholdAttributeVariation_;
//   std::vector<uint8_t>  attributeSmoothingThresholdAttributeDifference_;
//   bool                  attributeScaleParamsPresentFlag_;
//   bool                  attributeOffsetParamsPresentFlag_;
//   std::vector<uint32_t> attributeScale_;
//   std::vector<int32_t>  attributeOffset_;
// };

// // 7.3.5.6 Patch frame attribute parameter set syntax -> TODO: REMOVE
// class PatchFrameAttributeParameterSet {
//  public:
//   PatchFrameAttributeParameterSet() :
//       patchFrameAttributeParameterSetId_( 0 ),
//       patchSequencParameterSetId_( 0 ),
//       attributeDimensionMinus1_( 3 ),
//       attributePatchScaleParamsEnabledFlag_( false ),
//       attributePatchOffsetParamsEnabledFlag_( false ) {}

//   ~PatchFrameAttributeParameterSet() {}

//   PatchFrameAttributeParameterSet& operator=( const PatchFrameAttributeParameterSet& ) = default;

//   uint8_t               getPatchFrameAttributeParameterSetId() { return patchFrameAttributeParameterSetId_; }
//   uint8_t               getPatchSequencParameterSetId() { return patchSequencParameterSetId_; }
//   uint8_t               getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
//   bool                  getAttributePatchScaleParamsEnabledFlag() { return attributePatchScaleParamsEnabledFlag_; }
//   bool                  getAttributePatchOffsetParamsEnabledFlag() { return attributePatchOffsetParamsEnabledFlag_; }
//   AttributeFrameParams& getAttributeFrameParams() { return attributeFrameParams_; }

//   void setPatchFrameAttributeParameterSetId( uint8_t value ) { patchFrameAttributeParameterSetId_ = value; }
//   void setPatchSequencParameterSetId( uint8_t value ) { patchSequencParameterSetId_ = value; }
//   void setAttributeDimensionMinus1( uint8_t value ) { attributeDimensionMinus1_ = value; }
//   void setAttributePatchScaleParamsEnabledFlag( bool value ) { attributePatchScaleParamsEnabledFlag_ = value; }
//   void setAttributePatchOffsetParamsEnabledFlag( bool value ) { attributePatchOffsetParamsEnabledFlag_ = value; }
//   void setAttributeFrameParams( AttributeFrameParams value ) { attributeFrameParams_ = value; }

//  private:
//   uint8_t              patchFrameAttributeParameterSetId_;
//   uint8_t              patchSequencParameterSetId_;
//   uint8_t              attributeDimensionMinus1_;
//   bool                 attributePatchScaleParamsEnabledFlag_;
//   bool                 attributePatchOffsetParamsEnabledFlag_;
//   AttributeFrameParams attributeFrameParams_;
// };

// // 7.3.5.5 Geometry frame params syntax -> TODO: REMOVE
// class GeometryFrameParams {
//  public:
//   GeometryFrameParams() :
//       geometrySmoothingParamsPresentFlag_( false ),
//       geometryScaleParamsPresentFlag_( false ),
//       geometryOffsetParamsPresentFlag_( false ),
//       geometryRotationParamsPresentFlag_( false ),
//       geometryPointSizeInfoPresentFlag_( false ),
//       geometryPointShapeInfoPresentFlag_( false ),
//       geometrySmoothingEnabledFlag_( false ),
//       geometryPatchBlockFilteringEnableFlag_( false ),
//       geometrySmoothingGridSizeMinus2_( 0 ),
//       geometrySmoothingThreshold_( 0 ),
//       geometryPointSizeInfo_( 0 ),
//       geometryPointShapeInfo_( 0 ),
//       geometryPatchBlockFilteringPassesCountMinus1_( 0 ),
//       geometryPatchBlockFilteringFilterSizeMinus1_( 0 ),
//       geometryPatchBlockFilteringLog2ThresholdMinus1_( 0 ) {
//     for ( uint8_t i = 0; i < 3; i++ ) {
//       geometryScaleOnAxis_[i]  = 0;
//       geometryOffsetOnAxis_[i] = 0;
//     }
//     for ( uint8_t i = 0; i < 4; i++ ) { geometryRotationXYZW_[i] = 0; }
//   }
//   ~GeometryFrameParams() {}
//   GeometryFrameParams& operator=( const GeometryFrameParams& ) = default;

//   bool     getGeometrySmoothingParamsPresentFlag() { return geometrySmoothingParamsPresentFlag_; }
//   bool     getGeometryScaleParamsPresentFlag() { return geometryScaleParamsPresentFlag_; }
//   bool     getGeometryOffsetParamsPresentFlag() { return geometryOffsetParamsPresentFlag_; }
//   bool     getGeometryRotationParamsPresentFlag() { return geometryRotationParamsPresentFlag_; }
//   bool     getGeometryPointSizeInfoPresentFlag() { return geometryPointSizeInfoPresentFlag_; }
//   bool     getGeometryPointShapeInfoPresentFlag() { return geometryPointShapeInfoPresentFlag_; }
//   bool     getGeometrySmoothingEnabledFlag() { return geometrySmoothingEnabledFlag_; }
//   bool     getGeometryPatchBlockFilteringEnableFlag() { return geometryPatchBlockFilteringEnableFlag_; }
//   uint8_t  getGeometrySmoothingGridSizeMinus2() { return geometrySmoothingGridSizeMinus2_; }
//   uint8_t  getGeometrySmoothingThreshold() { return geometrySmoothingThreshold_; }
//   uint32_t getGeometryScaleOnAxis( size_t index ) { return geometryScaleOnAxis_[index]; }
//   int32_t  getGeometryOffsetOnAxis( size_t index ) { return geometryOffsetOnAxis_[index]; }
//   int32_t  getGeometryRotationQuaternion( size_t index ) { return geometryRotationXYZW_[index]; }
//   uint16_t getGeometryPointSizeInfo() { return geometryPointSizeInfo_; }
//   uint32_t getGeometryPointShapeInfo() { return geometryPointShapeInfo_; }
//   uint32_t getGeometryPatchBlockFilteringPassesCountMinus1() { return geometryPatchBlockFilteringPassesCountMinus1_;
//   } uint32_t getGeometryPatchBlockFilteringFilterSizeMinus1() { return geometryPatchBlockFilteringFilterSizeMinus1_;
//   } uint32_t getGeometryPatchBlockFilteringLog2ThresholdMinus1() {
//     return geometryPatchBlockFilteringLog2ThresholdMinus1_;
//   }

//   void setGeometrySmoothingParamsPresentFlag( bool value ) { geometrySmoothingParamsPresentFlag_ = value; }
//   void setGeometryScaleParamsPresentFlag( bool value ) { geometryScaleParamsPresentFlag_ = value; }
//   void setGeometryOffsetParamsPresentFlag( bool value ) { geometryOffsetParamsPresentFlag_ = value; }
//   void setGeometryRotationParamsPresentFlag( bool value ) { geometryRotationParamsPresentFlag_ = value; }
//   void setGeometryPointSizeInfoPresentFlag( bool value ) { geometryPointSizeInfoPresentFlag_ = value; }
//   void setGeometryPointShapeInfoPresentFlag( bool value ) { geometryPointShapeInfoPresentFlag_ = value; }
//   void setGeometrySmoothingEnabledFlag( bool value ) { geometrySmoothingEnabledFlag_ = value; }
//   void setGeometryPatchBlockFilteringEnableFlag( bool value ) { geometryPatchBlockFilteringEnableFlag_ = value; }
//   void setGeometrySmoothingGridSizeMinus2( uint8_t value ) { geometrySmoothingGridSizeMinus2_ = value; }
//   void setGeometrySmoothingThreshold( uint8_t value ) { geometrySmoothingThreshold_ = value; }
//   void setGeometryScaleOnAxis( size_t index, uint32_t value ) { geometryScaleOnAxis_[index] = value; }
//   void setGeometryOffsetOnAxis( size_t index, int32_t value ) { geometryOffsetOnAxis_[index] = value; }
//   void setGeometryRotationQuaternion( size_t index, int32_t value ) { geometryRotationXYZW_[index] = value; }
//   void setGeometryPointSizeInfo( uint16_t value ) { geometryPointSizeInfo_ = value; }
//   void setGeometryPointShapeInfo( uint32_t value ) { geometryPointShapeInfo_ = value; }
//   void setGeometryPatchBlockFilteringPassesCountMinus1( uint8_t value ) {
//     geometryPatchBlockFilteringPassesCountMinus1_ = value;
//   }
//   void setGeometryPatchBlockFilteringFilterSizeMinus1( uint8_t value ) {
//     geometryPatchBlockFilteringFilterSizeMinus1_ = value;
//   }
//   void setGeometryPatchBlockFilteringLog2ThresholdMinus1( uint8_t value ) {
//     geometryPatchBlockFilteringLog2ThresholdMinus1_ = value;
//   }

//  private:
//   bool     geometrySmoothingParamsPresentFlag_;
//   bool     geometryScaleParamsPresentFlag_;
//   bool     geometryOffsetParamsPresentFlag_;
//   bool     geometryRotationParamsPresentFlag_;
//   bool     geometryPointSizeInfoPresentFlag_;
//   bool     geometryPointShapeInfoPresentFlag_;
//   bool     geometrySmoothingEnabledFlag_;
//   bool     geometryPatchBlockFilteringEnableFlag_;
//   uint8_t  geometrySmoothingGridSizeMinus2_;
//   uint8_t  geometrySmoothingThreshold_;
//   uint32_t geometryScaleOnAxis_[3];
//   int32_t  geometryOffsetOnAxis_[3];
//   int32_t  geometryRotationXYZW_[4];
//   uint16_t geometryPointSizeInfo_;
//   uint32_t geometryPointShapeInfo_;
//   uint8_t  geometryPatchBlockFilteringPassesCountMinus1_;
//   uint8_t  geometryPatchBlockFilteringFilterSizeMinus1_;
//   uint8_t  geometryPatchBlockFilteringLog2ThresholdMinus1_;
// };

// // 7.3.5.4 Patch frame geometry parameter set syntax -> TODO: REMOVE
// class PatchFrameGeometryParameterSet {
//  public:
//   PatchFrameGeometryParameterSet() :
//       patchFrameGeometryParameterSetId_( 0 ),
//       patchVpccParameterSetId_( 0 ),
//       geometryPatchParamsEnabledFlag_( false ),
//       geometryPatchScaleParamsEnabledFlag_( false ),
//       geometryPatchOffsetParamsEnabledFlag_( false ),
//       geometryPatchRotationParamsEnabledFlag_( false ),
//       geometryPatchPointSizeInfoEnabledFlag_( false ),
//       geometryPatchPointShapeInfoEnabledFlag_( false ) {}

//   ~PatchFrameGeometryParameterSet() {}
//   PatchFrameGeometryParameterSet& operator=( const PatchFrameGeometryParameterSet& ) = default;

//   uint8_t              getPatchFrameGeometryParameterSetId() { return patchFrameGeometryParameterSetId_; }
//   uint8_t              getPatchVpccParameterSetId() { return patchVpccParameterSetId_; }
//   bool                 getGeometryPatchParamsEnabledFlag() { return geometryPatchParamsEnabledFlag_; }
//   bool                 getOverrideGeometryPatchParamsFlag() { return overrideGeometryPatchParamsFlag_; }
//   bool                 getGeometryPatchScaleParamsEnabledFlag() { return geometryPatchScaleParamsEnabledFlag_; }
//   bool                 getGeometryPatchOffsetParamsEnabledFlag() { return geometryPatchOffsetParamsEnabledFlag_; }
//   bool                 getGeometryPatchRotationParamsEnabledFlag() { return geometryPatchRotationParamsEnabledFlag_;
//   } bool                 getGeometryPatchPointSizeInfoEnabledFlag() { return geometryPatchPointSizeInfoEnabledFlag_;
//   } bool                 getGeometryPatchPointShapeInfoEnabledFlag() { return
//   geometryPatchPointShapeInfoEnabledFlag_; } GeometryFrameParams& getGeometryFrameParams() { return
//   geometryFrameParams_; }

//   void setPatchFrameGeometryParameterSetId( uint8_t value ) { patchFrameGeometryParameterSetId_ = value; }
//   void setPatchVpccParameterSetId( uint8_t value ) { patchVpccParameterSetId_ = value; }
//   void setGeometryPatchParamsEnabledFlag( bool value ) { geometryPatchParamsEnabledFlag_ = value; }
//   void setOverrideGeometryPatchParamsFlag( bool value ) { overrideGeometryPatchParamsFlag_ = value; }
//   void setGeometryPatchScaleParamsEnabledFlag( bool value ) { geometryPatchScaleParamsEnabledFlag_ = value; }
//   void setGeometryPatchOffsetParamsEnabledFlag( bool value ) { geometryPatchOffsetParamsEnabledFlag_ = value; }
//   void setGeometryPatchRotationParamsEnabledFlag( bool value ) { geometryPatchRotationParamsEnabledFlag_ = value; }
//   void setGeometryPatchPointSizeInfoEnabledFlag( bool value ) { geometryPatchPointSizeInfoEnabledFlag_ = value; }
//   void setGeometryPatchPointShapeInfoEnabledFlag( bool value ) { geometryPatchPointShapeInfoEnabledFlag_ = value; }
//   void setGeometryFrameParams( GeometryFrameParams value ) { geometryFrameParams_ = value; }

//  private:
//   uint8_t             patchFrameGeometryParameterSetId_;
//   uint8_t             patchVpccParameterSetId_;
//   bool                geometryPatchParamsEnabledFlag_;
//   bool                overrideGeometryPatchParamsFlag_;
//   bool                geometryPatchScaleParamsEnabledFlag_;
//   bool                geometryPatchOffsetParamsEnabledFlag_;
//   bool                geometryPatchRotationParamsEnabledFlag_;
//   bool                geometryPatchPointSizeInfoEnabledFlag_;
//   bool                geometryPatchPointShapeInfoEnabledFlag_;
//   GeometryFrameParams geometryFrameParams_;
// };

// // 7.3.5.3  Patch sequence parameter set syntax (psps) -> TODO: REMOVE
// class PatchVpccParameterSet {
//  public:
//   PatchVpccParameterSet() :
//       patchVpccParameterSetId_( 0 ),
//       log2PatchPackingBlockSize_( 0 ),
//       log2MaxPatchFrameOrderCntLsb_( 0 ),
//       maxDecPatchFrameBufferingMinus1_( 0 ),
//       numRefPatchFrameListsInPsps_( 0 ),
//       longTermRefPatchFramesFlag_( false ),
//       useEightOrientationsFlag_( false ),
//       normalAxisLimitsQuantizationEnableFlag_( false ),
//       normalAxisMaxDeltaValueEnableFlag_( false ) {
//     refListStruct_.clear();
//   }
//   ~PatchVpccParameterSet() { refListStruct_.clear(); }

//   PatchVpccParameterSet& operator=( const PatchVpccParameterSet& ) = default;

//   void           allocate( size_t size ) { refListStruct_.resize( size ); }
//   uint8_t        getPatchVpccParameterSetId() { return patchVpccParameterSetId_; }
//   uint8_t        getLog2PatchPackingBlockSize() { return log2PatchPackingBlockSize_; }
//   uint8_t        getLog2MaxPatchFrameOrderCntLsbMinus4() { return log2MaxPatchFrameOrderCntLsb_; }
//   uint8_t        getMaxDecPatchFrameBufferingMinus1() { return maxDecPatchFrameBufferingMinus1_; }
//   uint8_t        getNumRefPatchFrameListsInPsps() { return numRefPatchFrameListsInPsps_; }
//   bool           getLongTermRefPatchFramesFlag() { return longTermRefPatchFramesFlag_; }
//   bool           getUseEightOrientationsFlag() { return useEightOrientationsFlag_; }
//   bool           getNormalAxisLimitsQuantizationEnableFlag() { return normalAxisLimitsQuantizationEnableFlag_; }
//   bool           getNormalAxisMaxDeltaValueEnableFlag() { return normalAxisMaxDeltaValueEnableFlag_; }
//   RefListStruct& getRefListStruct( uint8_t index ) { return refListStruct_[index]; }
//   uint8_t        getRefListStructSize() { return refListStruct_.size(); }

//   void setPatchVpccParameterSetId( uint8_t value ) { patchVpccParameterSetId_ = value; }
//   void setLog2PatchPackingBlockSize( uint8_t value ) { log2PatchPackingBlockSize_ = value; }
//   void setLog2MaxPatchFrameOrderCntLsbMinus4( uint8_t value ) { log2MaxPatchFrameOrderCntLsb_ = value; }
//   void setMaxDecPatchFrameBufferingMinus1( uint8_t value ) { maxDecPatchFrameBufferingMinus1_ = value; }
//   void setNumRefPatchFrameListsInPsps( uint8_t value ) { numRefPatchFrameListsInPsps_ = value; }
//   void setLongTermRefPatchFramesFlag( bool value ) { longTermRefPatchFramesFlag_ = value; }
//   void setUseEightOrientationsFlag( bool value ) { useEightOrientationsFlag_ = value; }
//   void setNormalAxisLimitsQuantizationEnableFlag( bool value ) { normalAxisLimitsQuantizationEnableFlag_ = value; }
//   void setNormalAxisMaxDeltaValueEnableFlag( bool value ) { normalAxisMaxDeltaValueEnableFlag_ = value; }
//   void setRefListStruct( uint8_t index, RefListStruct value ) { refListStruct_[index] = value; }
//   void addRefListStruct( RefListStruct value ) { refListStruct_.push_back( value ); }
//   RefListStruct& addRefListStruct() {
//     RefListStruct refListStruct;
//     refListStruct_.push_back( refListStruct );
//     return refListStruct_.back();
//   }

//  private:
//   uint8_t patchVpccParameterSetId_;
//   uint8_t log2PatchPackingBlockSize_;
//   uint8_t log2MaxPatchFrameOrderCntLsb_;
//   uint8_t maxDecPatchFrameBufferingMinus1_;
//   uint8_t numRefPatchFrameListsInPsps_;
//   bool    longTermRefPatchFramesFlag_;
//   bool    useEightOrientationsFlag_;
//   bool    normalAxisLimitsQuantizationEnableFlag_;
//   bool    normalAxisMaxDeltaValueEnableFlag_;

//   std::vector<RefListStruct> refListStruct_;
// };

// // 7.3.5.2  Patch data group unit payload syntax (pdgup) -> this should be removed -> TODO: REMOVE
// class PatchDataGroup {
//  public:
//   PatchDataGroup() {}
//   ~PatchDataGroup() {
//     patchTileGroupLayerUnit_.clear();
//     // seiMessageSuffix_.clear();
//     // seiMessagePrefix_.clear();
//   }
//   PatchDataGroup&             operator=( const PatchDataGroup& ) = default;
//   PatchVpccParameterSet&      getPatchVpccParameterSet( size_t index ) { return patchVpccParameterSet_[index]; }
//   GeometryPatchParameterSet&  getGeometryPatchParameterSet( size_t index ) { return
//   geometryPatchParameterSet_[index]; } AttributePatchParameterSet& getAttributePatchParameterSet( size_t index ) {
//     return attributePatchParameterSet_[index];
//   }
//   PatchFrameParameterSet&          getPatchFrameParameterSet( size_t index ) { return patchFrameParameterSet_[index];
//   } PatchFrameAttributeParameterSet& getPatchFrameAttributeParameterSet( size_t index ) {
//     return patchFrameAttributeParameterSet_[index];
//   }
//   PatchFrameGeometryParameterSet& getPatchFrameGeometryParameterSet( size_t index ) {
//     return patchFrameGeometryParameterSet_[index];
//   }
//   PatchTileGroupLayerUnit& getPatchTileGroupLayerUnit( size_t index ) { return patchTileGroupLayerUnit_[index]; }
//   // std::vector<SeiMessage>& getSeiMessagePrefix() { return seiMessagePrefix_; }
//   // std::vector<SeiMessage>& getSeiMessageSuffix() { return seiMessageSuffix_; }
//   // SeiMessage&              getSeiMessagePrefix( size_t index ) { return seiMessagePrefix_[index]; }
//   // SeiMessage&              getSeiMessageSuffix( size_t index ) { return seiMessageSuffix_[index]; }
//   size_t getPatchTileGroupLayerUnitSize() { return patchTileGroupLayerUnit_.size(); }
//   // size_t                   getSeiMessagePrefixSize() { return seiMessagePrefix_.size(); }
//   // size_t                   getSeiMessageSuffixSize() { return seiMessageSuffix_.size(); }
//   size_t getPatchVpccParameterSetSize() { return patchVpccParameterSetSize_; }
//   size_t getGeometryPatchParameterSetSize() { return geometryPatchParameterSetSize_; }
//   size_t getAttributePatchParameterSetSize() { return attributePatchParameterSetSize_; }
//   size_t getPatchFrameParameterSetSize() { return patchFrameParameterSetSize_; }
//   size_t getPatchFrameAttributeParameterSetSize() { return patchFrameAttributeParameterSetSize_; }
//   size_t getPatchFrameGeometryParameterSetSize() { return patchFrameGeometryParameterSetSize_; }
//   void   setPatchVpccParameterSetSize( size_t value ) { patchVpccParameterSetSize_ = value; }
//   void   setGeometryPatchParameterSetSize( size_t value ) { geometryPatchParameterSetSize_ = value; }
//   void   setAttributePatchParameterSetSize( size_t value ) { attributePatchParameterSetSize_ = value; }
//   void   setPatchFrameParameterSetSize( size_t value ) { patchFrameParameterSetSize_ = value; }
//   void   setPatchFrameAttributeParameterSetSize( size_t value ) { patchFrameAttributeParameterSetSize_ = value; }
//   void   setPatchFrameGeometryParameterSetSize( size_t value ) { patchFrameGeometryParameterSetSize_ = value; }
//   void setPatchVpccParameterSet( size_t index, PatchVpccParameterSet value ) { patchVpccParameterSet_[index] = value;
//   } void setGeometryPatchParameterSet( size_t index, GeometryPatchParameterSet value ) {
//     geometryPatchParameterSet_[index] = value;
//   }
//   void setAttributePatchParameterSet( size_t index, AttributePatchParameterSet& value ) {
//     attributePatchParameterSet_[index] = value;
//   }
//   void setPatchFrameParameterSet( size_t index, PatchFrameParameterSet value ) {
//     patchFrameParameterSet_[index] = value;
//   }
//   void setPatchFrameAttributeParameterSet( size_t index, PatchFrameAttributeParameterSet& value ) {
//     patchFrameAttributeParameterSet_[index] = value;
//   }
//   void setPatchFrameGeometryParameterSet( size_t index, PatchFrameGeometryParameterSet value ) {
//     patchFrameGeometryParameterSet_[index] = value;
//   }
//   PatchTileGroupLayerUnit& addPatchTileGroupLayerUnit() {
//     PatchTileGroupLayerUnit patchTileGroupLayerUnit;
//     patchTileGroupLayerUnit_.push_back( patchTileGroupLayerUnit );
//     return patchTileGroupLayerUnit_.back();
//   }

// std::shared_ptr<SEI>& addSei( std::vector<std::shared_ptr<SEI>>& list, SeiPayloadType payloadType ) {
//   switch ( payloadType ) {
//     case BUFFERING_PERIOD: list.push_back( std::make_shared<SEIBufferingPeriod>() ); break;
//     case ATLAS_FRAME_TIMING: list.push_back( std::make_shared<SEIAtlasFrameTiming>() ); break;
//     case FILLER_PAYLOAD: break;
//     case USER_DATAREGISTERED_ITUTT35: list.push_back( std::make_shared<SEIUserDataRegisteredItuTT35>() ); break;
//     case USER_DATA_UNREGISTERED: list.push_back( std::make_shared<SEIUserDataUnregistered>() ); break;
//     case RECOVERY_POINT: list.push_back( std::make_shared<SEIRecoveryPoint>() ); break;
//     case NO_DISPLAY: list.push_back( std::make_shared<SEINoDisplay>() ); break;
//     case TIME_CODE: break;
//     case REGIONAL_NESTING: break;
//     case SEI_MANIFEST: list.push_back( std::make_shared<SEIManifest>() ); break;
//     case SEI_PREFIX_INDICATION: list.push_back( std::make_shared<SEIPrefixIndication>() ); break;
//     case GEOMETRY_TRANSFORMATION_PARAMS: list.push_back( std::make_shared<SEIGeometryTransformationParams>() );
//     break; case ATTRIBUTE_TRANSFORMATION_PARAMS: list.push_back(
//     std::make_shared<SEIAttributeTransformationParams>() ); break; case ACTIVE_SUBSTREAMS: list.push_back(
//     std::make_shared<SEIActiveSubstreams>() ); break; case COMPONENT_CODEC_MAPPING: list.push_back(
//     std::make_shared<SEIComponentCodecMapping>() ); break; case VOLUMETRIC_TILING_INFO: list.push_back(
//     std::make_shared<SEIVolumetricTilingInfo>() ); break; case PRESENTATION_INFORMATION: list.push_back(
//     std::make_shared<SEIPresentationInformation>() ); break; case SMOOTHING_PARAMETERS: list.push_back(
//     std::make_shared<SEISmoothingParameters>() ); break; case RESERVED_SEI_MESSAGE: list.push_back(
//     std::make_shared<SEIReservedSeiMessage>() ); break; default:
//       fprintf( stderr, "SEI payload type not supported \n" );
//       exit( -1 );
//       break;
//   }
//   return list.back();
// }

// std::shared_ptr<SEI>& addSeiPrefix( SeiPayloadType payloadType ) { return addSei( seiPrefix_, payloadType ); }
// std::shared_ptr<SEI>& addSeiSuffix( SeiPayloadType payloadType ) { return addSei( seiSuffix_, payloadType ); }
// std::vector<std::shared_ptr<SEI>>& getSeiPrefix() { return seiPrefix_; }
// std::vector<std::shared_ptr<SEI>>& getSeiSuffix() { return seiSuffix_; }
// std::vector<std::shared_ptr<SEI>>    seiPrefix_;
// std::vector<std::shared_ptr<SEI>>    seiSuffix_;
//  private:
//   PatchVpccParameterSet                patchVpccParameterSet_[16];
//   GeometryPatchParameterSet            geometryPatchParameterSet_[64];
//   AttributePatchParameterSet           attributePatchParameterSet_[64];
//   PatchFrameParameterSet               patchFrameParameterSet_[64];
//   PatchFrameAttributeParameterSet      patchFrameAttributeParameterSet_[64];
//   PatchFrameGeometryParameterSet       patchFrameGeometryParameterSet_[64];
//   std::vector<PatchTileGroupLayerUnit> patchTileGroupLayerUnit_;

//   size_t patchVpccParameterSetSize_;
//   size_t geometryPatchParameterSetSize_;
//   size_t attributePatchParameterSetSize_;
//   size_t patchFrameParameterSetSize_;
//   size_t patchFrameAttributeParameterSetSize_;
//   size_t patchFrameGeometryParameterSetSize_;
// };

// used for context handling
class PCCContext {
 public:
  PCCContext();
  ~PCCContext();

  std::vector<PCCFrameContext>::iterator begin() { return frames_.begin(); }
  std::vector<PCCFrameContext>::iterator end() { return frames_.end(); }

  void resize( size_t size );

  const size_t                  size() { return frames_.size(); }  // jkei: lets rename this
  std::vector<PCCFrameContext>& getFrames() { return frames_; }
  PCCFrameContext&              getFrame( int16_t index ) { return frames_[index]; }
  PCCFrameContext&              operator[]( int index ) { return frames_[index]; }
  PCCVideoGeometry&             getVideoGeometry() { return videoGeometry_; }
  PCCVideoGeometry&             getVideoGeometryD1() { return videoGeometryD1_; }
  PCCVideoTexture&              getVideoTexture() { return videoTexture_; }
  PCCVideoTexture&              getVideoTextureT1() { return videoTextureT1_; }
  PCCVideoOccupancyMap&         getVideoOccupancyMap() { return videoOccupancyMap_; }
  PCCVideoGeometry&             getVideoMPsGeometry() { return videoMPsGeometry_; }
  PCCVideoTexture&              getVideoMPsTexture() { return videoMPsTexture_; }
  uint8_t                       getOccupancyPrecision() { return occupancyPrecision_; }
  uint8_t                       getOccupancyPackingBlockSize() { return occupancyPackingBlockSize_; }
  uint8_t                       getLog2PatchQuantizerSizeX() { return log2PatchQuantizerSizeX_; }
  uint8_t                       getLog2PatchQuantizerSizeY() { return log2PatchQuantizerSizeY_; }
  bool                          getEnablePatchSizeQuantization() { return enablePatchSizeQuantization_; }
  float                         getModelScale() { return modelScale_; }
  PCCVector3<float>&            getModelOrigin() { return modelOrigin_; }
  size_t                        getMPGeoWidth() { return MPGeoWidth_; }
  size_t                        getMPGeoHeight() { return MPGeoHeight_; }
  size_t                        getMPAttWidth() { return MPAttWidth_; }
  size_t                        getMPAttHeight() { return MPAttHeight_; }
  std::vector<SubContext>&      getSubContexts() { return subContexts_; }
  std::vector<unionPatch>&      getUnionPatch() { return unionPatch_; }
  bool&                         getPrefilterLossyOM() { return prefilterLossyOM_; }
  size_t&                       getOffsetLossyOM() { return offsetLossyOM_; }
  size_t                        getGeometry3dCoordinatesBitdepth() { return geometry3dCoordinatesBitdepth_; }
  void                          setOccupancyPrecision( uint8_t value ) { occupancyPrecision_ = value; }
  void                          setOccupancyPackingBlockSize( uint8_t value ) { occupancyPackingBlockSize_ = value; }

  void                          setLog2PatchQuantizerSizeX( uint8_t value ) { log2PatchQuantizerSizeX_=value; }
  void                          setLog2PatchQuantizerSizeY( uint8_t value ) { log2PatchQuantizerSizeY_=value; }
  void                          setEnablePatchSizeQuantization( bool value ){ enablePatchSizeQuantization_=value; }
  void                          setModelScale( float value ) { modelScale_ = value; }
  void                          setModelOrigin( PCCVector3<float>& value ) { modelOrigin_ = value; }
  void                          setMPGeoWidth( size_t value ) { MPGeoWidth_ = value; }
  void                          setMPGeoHeight( size_t value ) { MPGeoHeight_ = value; }
  void                          setMPAttWidth( size_t value ) { MPAttWidth_ = value; }
  void                          setMPAttHeight( size_t value ) { MPAttHeight_ = value; }
  void                          setPrefilterLossyOM( bool value ) { prefilterLossyOM_ = value; }
  void                          setOffsetLossyOM( size_t value ) { offsetLossyOM_ = value; }
  void setGeometry3dCoordinatesBitdepth( size_t value ) { geometry3dCoordinatesBitdepth_ = value; }

  PCCVideoBitstream& createVideoBitstream( PCCVideoType type ) {
    videoBitstream_.push_back( PCCVideoBitstream( type ) );
    return videoBitstream_.back();
  }
  bool& getSingleLayerMode() { return singleLayerMode_; }
  // size_t&            getEOMFixBitCount() { return EOMFixBitCount_; }
  size_t             getVideoBitstreamCount() { return videoBitstream_.size(); }
  PCCVideoBitstream& getVideoBitstream( size_t index ) { return videoBitstream_[index]; }
  PCCVideoBitstream& getVideoBitstream( PCCVideoType type ) {
    for ( auto& value : videoBitstream_ ) {
      if ( value.type() == type ) { return value; }
    }
    printf( "ERROR: can't get video bitstream of type %s \n", toString( type ).c_str() );
    fflush( stdout );
    assert( 0 );
    exit( -1 );
  }

  void                   allocOneLayerData();
  void                   printVideoBitstream();
  void                   printBlockToPatch( const size_t occupancyResolution );
  vpccUnitPayloadHeader& getVpccUnitHeaderAVD() { return vpccUnitHeader_[size_t( VPCC_AVD ) - 1]; }  // 3
  vpccUnitPayloadHeader& getVpccUnitHeaderGVD() { return vpccUnitHeader_[size_t( VPCC_GVD ) - 1]; }  // 2
  vpccUnitPayloadHeader& getVpccUnitHeaderOVD() { return vpccUnitHeader_[size_t( VPCC_OVD ) - 1]; }  // 1
  vpccUnitPayloadHeader& getVpccUnitHeaderAD() { return vpccUnitHeader_[size_t( VPCC_AD ) - 1]; }    // 0
  vpccUnitPayloadHeader& getVpccUnitHeader( int index ) { return vpccUnitHeader_[index]; }

  VpccParameterSet&              getSps() { return vpccParameterSets_[0]; }
  VpccParameterSet&              getSps( size_t index ) { return vpccParameterSets_[index]; }
  std::vector<VpccParameterSet>& getSpsList() { return vpccParameterSets_; }

  VpccParameterSet& addVpccParameterSet( uint8_t index ) {
    VpccParameterSet sps;
    sps.setVpccParameterSetId( index );
    vpccParameterSets_.push_back( sps );
    return vpccParameterSets_.back();
  }

  VpccParameterSet& addVpccParameterSet() {
    uint8_t          index = vpccParameterSets_.size();
    VpccParameterSet sps;
    sps.setVpccParameterSetId( index );
    vpccParameterSets_.push_back( sps );
    return vpccParameterSets_.back();
  }
  AtlasSequenceParameterSetRBSP& getAtlasSequenceParameterSet( size_t setId ) {
    return atlasSequenceParameterSet_[setId];
  }
  std::vector<AtlasSequenceParameterSetRBSP>& getAtlasSequenceParameterSetList() { return atlasSequenceParameterSet_; }

  AtlasFrameParameterSetRbsp& getAtlasFrameParameterSet( size_t setId ) { return atlasFrameParameterSet_[setId]; }
  std::vector<AtlasFrameParameterSetRbsp>& getAtlasFrameParameterSetList() { return atlasFrameParameterSet_; }

  // size_t addAtlasSequenceParameterSet() {
  //   size_t                        setId = atlasSequenceParameterSet_.size();
  //   AtlasSequenceParameterSetRBSP asps;
  //   asps.setAltasSequenceParameterSetId( setId );
  //   atlasSequenceParameterSet_.resize( setId + 1 );  // jkei: setId doesnot need to be sequential
  //   atlasSequenceParameterSet_[setId] = asps;        // jkei: makes sens?
  //   return setId;
  // }
  AtlasSequenceParameterSetRBSP& addAtlasSequenceParameterSet() {
    AtlasSequenceParameterSetRBSP asps;
    asps.setAltasSequenceParameterSetId( atlasSequenceParameterSet_.size() );
    atlasSequenceParameterSet_.push_back( asps );
    return atlasSequenceParameterSet_.back();
  }
  AtlasSequenceParameterSetRBSP& addAtlasSequenceParameterSet( uint8_t setId ) {
    AtlasSequenceParameterSetRBSP asps;
    asps.setAltasSequenceParameterSetId( setId );
    if ( atlasSequenceParameterSet_.size() < setId + 1 ) {
      atlasSequenceParameterSet_.resize( setId + 1 );  // jkei: setId doesnot need to be sequential
    }
    atlasSequenceParameterSet_[setId] = asps;  // jkei: makes sens?
    return atlasSequenceParameterSet_[setId];
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet() {
    AtlasFrameParameterSetRbsp afps;
    afps.setAtlasFrameParameterSetId( atlasFrameParameterSet_.size() );
    atlasFrameParameterSet_.push_back( afps );
    return atlasFrameParameterSet_.back();
  }
  size_t addAtlasFrameParameterSet( AtlasFrameParameterSetRbsp& refAfps ) {
    size_t                     setId = atlasFrameParameterSet_.size();
    AtlasFrameParameterSetRbsp afps;
    // AtlasFrameParameterSetRbsp(refAfps) afps; //jkei: can we do this??
    afps.copyFrom( refAfps );
    afps.setAtlasFrameParameterSetId( setId );
    atlasFrameParameterSet_.resize( setId + 1 );
    atlasFrameParameterSet_[setId] = afps;
    return setId;
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet( uint8_t setId ) {
    AtlasFrameParameterSetRbsp afps;
    afps.setAtlasFrameParameterSetId( setId );
    if ( atlasFrameParameterSet_.size() < setId + 1 ) {
      atlasFrameParameterSet_.resize( setId + 1 );  // jkei: setId doesnot need to be sequential
    }
    atlasFrameParameterSet_[setId] = afps;  // jkei: makes sens?
    return atlasFrameParameterSet_[setId];
  }

  AtlasTileGroupLayerRbsp& addAtlasTileGroupLayer() {
    size_t                  frameIdx = atlasTileGroupLayer_.size();
    AtlasTileGroupLayerRbsp atgl;
    atgl.setFrameIndex( frameIdx );
    atgl.getAtlasTileGroupDataUnit().setFrameIndex( frameIdx );
    atlasTileGroupLayer_.push_back( atgl );
    return atlasTileGroupLayer_.back();
  }

  AtlasTileGroupLayerRbsp& addAtlasTileGroupLayer( size_t frameIdx ) {
    AtlasTileGroupLayerRbsp atgl;
    atlasTileGroupLayer_.resize( frameIdx + 1 );
    atlasTileGroupLayer_[frameIdx] = atgl;
    return atlasTileGroupLayer_[frameIdx];
  }

  std::vector<AtlasTileGroupLayerRbsp>& getAtlasTileGroupLayerList() { return atlasTileGroupLayer_; }
  AtlasTileGroupLayerRbsp&              getAtlasTileGroupLayer( size_t index ) { return atlasTileGroupLayer_[index]; }

  // PatchDataGroup& getPatchDataGroup() { return patchSequenceDataUnit_; }

  void addPointLocalReconstructionMode( const PointLocalReconstructionMode& mode ) {
    pointLocalReconstructionMode_.push_back( mode );
  }
  PointLocalReconstructionMode& getPointLocalReconstructionMode( size_t index ) {
    return pointLocalReconstructionMode_[index];
  }
  size_t getPointLocalReconstructionModeNumber() { return pointLocalReconstructionMode_.size(); }

  void              setBitstreamStat( PCCBitstreamStat& bitstreamStat ) { bitstreamStat_ = &bitstreamStat; }
  PCCBitstreamStat& getBitstreamStat() { return *bitstreamStat_; }

  SEI& addSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    std::shared_ptr<SEI> sharedPtr;
    switch ( payloadType ) {
      case BUFFERING_PERIOD: sharedPtr = std::make_shared<SEIBufferingPeriod>(); break;
      case ATLAS_FRAME_TIMING: sharedPtr = std::make_shared<SEIAtlasFrameTiming>(); break;
      case FILLER_PAYLOAD: break;
      case USER_DATAREGISTERED_ITUTT35: sharedPtr = std::make_shared<SEIUserDataRegisteredItuTT35>(); break;
      case USER_DATA_UNREGISTERED: sharedPtr = std::make_shared<SEIUserDataUnregistered>(); break;
      case RECOVERY_POINT: sharedPtr = std::make_shared<SEIRecoveryPoint>(); break;
      case NO_DISPLAY: sharedPtr = std::make_shared<SEINoDisplay>(); break;
      case TIME_CODE: break;
      case REGIONAL_NESTING: break;
      case SEI_MANIFEST: sharedPtr = std::make_shared<SEIManifest>(); break;
      case SEI_PREFIX_INDICATION: sharedPtr = std::make_shared<SEIPrefixIndication>(); break;
      case GEOMETRY_TRANSFORMATION_PARAMS: sharedPtr = std::make_shared<SEIGeometryTransformationParams>(); break;
      case ATTRIBUTE_TRANSFORMATION_PARAMS: sharedPtr = std::make_shared<SEIAttributeTransformationParams>(); break;
      case ACTIVE_SUBSTREAMS: sharedPtr = std::make_shared<SEIActiveSubstreams>(); break;
      case COMPONENT_CODEC_MAPPING: sharedPtr = std::make_shared<SEIComponentCodecMapping>(); break;
      case VOLUMETRIC_TILING_INFO: sharedPtr = std::make_shared<SEIVolumetricTilingInfo>(); break;
      case PRESENTATION_INFORMATION: sharedPtr = std::make_shared<SEIPresentationInformation>(); break;
      case SMOOTHING_PARAMETERS: sharedPtr = std::make_shared<SEISmoothingParameters>(); break;
      case RESERVED_SEI_MESSAGE: sharedPtr = std::make_shared<SEIReservedSeiMessage>(); break;
      default:
        fprintf( stderr, "SEI payload type not supported \n" );
        exit( -1 );
        break;
    }
    if ( nalUnitType == NAL_PREFIX_SEI ) {
      seiPrefix_.push_back( sharedPtr );
      return *( seiPrefix_.back().get() );
    } else if ( nalUnitType == NAL_SUFFIX_SEI ) {
      seiSuffix_.push_back( sharedPtr );
      return *( seiSuffix_.back().get() );
    } else {
      fprintf( stderr, "Nal unit type of SEI not correct\n" );
      exit( -1 );
    }
    return *( sharedPtr );
  }
  bool seiIsPresent( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    if ( nalUnitType != NAL_PREFIX_SEI && nalUnitType != NAL_SUFFIX_SEI ) { return false; }
    for ( auto& sei : nalUnitType == NAL_PREFIX_SEI ? seiPrefix_ : seiSuffix_ ) {
      if ( sei->getPayloadType() == payloadType ) { return true; }
    }
    return false;
  }

  SEI& getSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    for ( auto& sei : nalUnitType == NAL_PREFIX_SEI ? seiPrefix_ : seiSuffix_ ) {
      if ( sei->getPayloadType() == payloadType ) { return *sei; }
    }
  }
  SEI& addSeiPrefix( SeiPayloadType payloadType ) { return addSei( NAL_PREFIX_SEI, payloadType ); }
  SEI& addSeiSuffix( SeiPayloadType payloadType ) { return addSei( NAL_SUFFIX_SEI, payloadType ); }
  std::vector<std::shared_ptr<SEI>>& getSeiPrefix() { return seiPrefix_; }
  std::vector<std::shared_ptr<SEI>>& getSeiSuffix() { return seiSuffix_; }
  SEI&                               getSeiPrefix( size_t index ) { return *( seiPrefix_[index] ); }
  SEI&                               getSeiSuffix( size_t index ) { return *( seiSuffix_[index] ); }

 private:
  std::vector<PCCFrameContext>   frames_;
  PCCVideoGeometry               videoGeometry_;
  PCCVideoGeometry               videoGeometryD1_;
  PCCVideoTexture                videoTexture_;
  PCCVideoTexture                videoTextureT1_;
  PCCVideoOccupancyMap           videoOccupancyMap_;
  PCCVideoGeometry               videoMPsGeometry_;
  PCCVideoTexture                videoMPsTexture_;
  std::vector<PCCVideoBitstream> videoBitstream_;
  vpccUnitPayloadHeader          vpccUnitHeader_[5];
  // PatchDataGroup                 patchSequenceDataUnit_;
  // // jkei: should use vector? or only 1 asps is allowed?
  // std::vector<AtlasSequenceParameterSetRBSP> atlasSequenceParameterSetList_;
  // std::vector<AtlasFrameParameterSetRbsp>    atalsFrameParameterSetList_;
  // std::vector<AtlasTileGroupLayerRbsp>       atlasTileGroupLayerList_;  // jkei: each tileGroupLayer means a frame...

  std::vector<AtlasSequenceParameterSetRBSP> atlasSequenceParameterSet_;
  std::vector<AtlasFrameParameterSetRbsp>    atlasFrameParameterSet_;
  std::vector<AtlasTileGroupLayerRbsp>       atlasTileGroupLayer_;

  std::vector<std::shared_ptr<SEI>> seiPrefix_;
  std::vector<std::shared_ptr<SEI>> seiSuffix_;

  std::vector<VpccParameterSet>             vpccParameterSets_;
  uint8_t                                   occupancyPrecision_;
  uint8_t                                   occupancyPackingBlockSize_;
  uint8_t                                   log2PatchQuantizerSizeX_;
  uint8_t                                   log2PatchQuantizerSizeY_;
  bool                                      enablePatchSizeQuantization_;
  size_t                                    MPGeoWidth_;
  size_t                                    MPGeoHeight_;
  size_t                                    MPAttWidth_;
  size_t                                    MPAttHeight_;
  float                                     modelScale_;
  PCCVector3<float>                         modelOrigin_;
  std::vector<SubContext>                   subContexts_;
  std::vector<unionPatch>                   unionPatch_;
  std::vector<PointLocalReconstructionMode> pointLocalReconstructionMode_;
  bool                                      prefilterLossyOM_;
  size_t                                    offsetLossyOM_;
  size_t                                    geometry3dCoordinatesBitdepth_;
  bool                                      singleLayerMode_;
  PCCBitstreamStat*                         bitstreamStat_;
  // size_t                                    EOMFixBitCount_;
  // std::vector<int16_t> deltaAfocSt_; //ref, -1,-2,-3,-4
};
};  // namespace pcc

#endif /* PCCContext_h */
