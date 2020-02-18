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
#ifndef PCC_BITSTREAM_SEI_H
#define PCC_BITSTREAM_SEI_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// Annex E: Supplemental enhancement information
// E.2.1	General SEI message syntax  <=> 7.3.8	Supplemental enhancement information message syntax
class SEI {
 public:
  SEI() {payloadSize_=0;}
  virtual ~SEI() {}
  virtual SeiPayloadType getPayloadType() = 0;
  size_t                 getPayloadSize() { return payloadSize_; }
  void                   setPayloadSize( size_t value ) { payloadSize_ = value; }
 private:
  size_t payloadSize_;
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
      gtpNumCameraInfoMinus1_( 0 ),
      gtpRotationQx_( 0 ),
      gtpRotationQy_( 0 ),
      gtpRotationQz_( 0 ) {
    for ( size_t i = 0; i < 3; i++ ) {
      gtpGeometryScaleOnAxis_[i]  = 0;
      gtpGeometryOffsetOnAxis_[i] = 0;
    }
  }

  ~SEIGeometryTransformationParams() {
    gtpCameraOffsetOnAxis_[3].clear();
    gtpCameraOrientationOnAxis_[3].clear();
  }
  SEIGeometryTransformationParams& operator=( const SEIGeometryTransformationParams& ) = default;

  SeiPayloadType getPayloadType() { return GEOMETRY_TRANSFORMATION_PARAMS; }

  void allocate() {
    for (uint8_t d = 0; d < 3; d++) {
      gtpCameraOffsetOnAxis_[d].resize(8);
      gtpCameraOrientationOnAxis_[d].resize(8);
    }
  }

  bool     getGtpCancelFlag() { return gtpCancelFlag_; }
  bool     getGtpScaleEnabledFlag() { return gtpScaleEnabledFlag_; }
  bool     getGtpOffsetEnabledFlag() { return gtpOffsetEnabledFlag_; }
  bool     getGtpRotationEnabledFlag() { return gtpRotationEnabledFlag_; }
  uint32_t getGtpGeometryScaleOnAxis( size_t index ) { return gtpGeometryScaleOnAxis_[index]; }
  int32_t  getGtpGeometryOffsetOnAxis( size_t index ) { return gtpGeometryOffsetOnAxis_[index]; }
  int16_t  getGtpRotationQx() { return gtpRotationQx_; }
  int16_t  getGtpRotationQy() { return gtpRotationQy_; }
  int16_t  getGtpRotationQz() { return gtpRotationQz_; }
  uint8_t                   getGtpNumCameraInfoMinus1() { return gtpNumCameraInfoMinus1_; }
  std::vector<int16_t>&     getGtpCameraOffsetOnAxis() { return gtpCameraOffsetOnAxis_[3]; }
  std::vector<int16_t>&     getGtpCameraOrientationOnAxis() { return gtpCameraOrientationOnAxis_[3]; }
  int16_t                   getGtpCameraOffsetOnAxis(uint8_t cameraId, size_t axisId) { return gtpCameraOffsetOnAxis_[cameraId][axisId]; }
  int16_t                   getGtpCameraOrientationOnAxis(uint8_t cameraId, size_t axisId) { return gtpCameraOrientationOnAxis_[cameraId][axisId]; }

  void setGtpCancelFlag( bool value ) { gtpCancelFlag_ = value; }
  void setGtpScaleEnabledFlag( bool value ) { gtpScaleEnabledFlag_ = value; }
  void setGtpOffsetEnabledFlag( bool value ) { gtpOffsetEnabledFlag_ = value; }
  void setGtpRotationEnabledFlag( bool value ) { gtpRotationEnabledFlag_ = value; }
  void setGtpGeometryScaleOnAxis( size_t index, uint32_t value ) { gtpGeometryScaleOnAxis_[index] = value; }
  void setGtpGeometryOffsetOnAxis( size_t index, int32_t value ) { gtpGeometryOffsetOnAxis_[index] = value; }
  void setGtpRotationQx( uint16_t value ) { gtpRotationQx_ = value; }
  void setGtpRotationQy( uint16_t value ) { gtpRotationQy_ = value; }
  void setGtpRotationQz( uint16_t value ) { gtpRotationQz_ = value; }
  void setGtpNumCameraInfoMinus1(uint8_t value) { gtpNumCameraInfoMinus1_ = value; }
  void setGtpCameraOffsetOnAxis(uint8_t cameraId, uint8_t axisId, int16_t value) { gtpCameraOffsetOnAxis_[cameraId][axisId] = value; }
  void setGtpCameraOrientationOnAxis(uint8_t cameraId, uint8_t axisId, int16_t value) { gtpCameraOrientationOnAxis_[cameraId][axisId] = value; }

 private:
  bool                      gtpCancelFlag_;
  bool                      gtpScaleEnabledFlag_;
  bool                      gtpOffsetEnabledFlag_;
  bool                      gtpRotationEnabledFlag_;
  uint32_t                  gtpGeometryScaleOnAxis_[3];
  int32_t                   gtpGeometryOffsetOnAxis_[3];
  int16_t                   gtpRotationQx_;
  int16_t                   gtpRotationQy_;
  int16_t                   gtpRotationQz_;
  uint8_t                   gtpNumCameraInfoMinus1_;
  std::vector<int16_t>      gtpCameraOffsetOnAxis_[3];
  std::vector<int16_t>      gtpCameraOrientationOnAxis_[3];
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

  void setBpIrapCabParamsPresentFlag( bool value ) { bpIrapCabParamsPresentFlag_ = value; }
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
      spGeometrySmoothingGridSizeMinus2_( 0 ),
      spGeometrySmoothingThreshold_( 0 ),
      spGeometrySmoothingId_( 0 ),
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

};  // namespace pcc

#endif //~PCC_BITSTREAM_SEI_H
