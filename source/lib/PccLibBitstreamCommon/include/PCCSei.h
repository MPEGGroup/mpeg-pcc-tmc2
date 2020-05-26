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
// F.2.1	General SEI message syntax  <=> 7.3.8	Supplemental enhancement
// information message syntax
class SEI {
 public:
  SEI() { payloadSize_ = 0; }
  virtual ~SEI() {}
  virtual SeiPayloadType getPayloadType() = 0;
  size_t                 getPayloadSize() { return payloadSize_; }
  void                   setPayloadSize( size_t value ) { payloadSize_ = value; }

 private:
  size_t payloadSize_;
};

// F.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
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

// F.2.4  User data unregistered SEI message syntax
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

// F.2.5  Recovery point SEI message syntax
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

// F.2.6  No display SEI message syntax
class SEINoDisplay : public SEI {
 public:
  SEINoDisplay() {}

  ~SEINoDisplay() {}
  SEINoDisplay& operator=( const SEINoDisplay& ) = default;

  SeiPayloadType getPayloadType() { return NO_DISPLAY; }

 private:
};

// F.2.7  Reserved SEI message syntax
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

// F.2.8  SEI manifest SEI message syntax
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

// F.2.9  SEI prefix indication SEI message syntax
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
/*
// F.2.10  Geometry transformation parameters SEI message syntax
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
    for ( uint8_t d = 0; d < 3; d++ ) {
      gtpCameraOffsetOnAxis_[d].resize( 8 );
      gtpCameraOrientationOnAxis_[d].resize( 8 );
    }
  }

  bool                  getGtpCancelFlag() { return gtpCancelFlag_; }
  bool                  getGtpScaleEnabledFlag() { return gtpScaleEnabledFlag_; }
  bool                  getGtpOffsetEnabledFlag() { return gtpOffsetEnabledFlag_; }
  bool                  getGtpRotationEnabledFlag() { return gtpRotationEnabledFlag_; }
  uint32_t              getGtpGeometryScaleOnAxis( size_t index ) { return gtpGeometryScaleOnAxis_[index]; }
  int32_t               getGtpGeometryOffsetOnAxis( size_t index ) { return gtpGeometryOffsetOnAxis_[index]; }
  int16_t               getGtpRotationQx() { return gtpRotationQx_; }
  int16_t               getGtpRotationQy() { return gtpRotationQy_; }
  int16_t               getGtpRotationQz() { return gtpRotationQz_; }
  uint8_t               getGtpNumCameraInfoMinus1() { return gtpNumCameraInfoMinus1_; }
  std::vector<int16_t>& getGtpCameraOffsetOnAxis( size_t index ) { return gtpCameraOffsetOnAxis_[index]; }
  std::vector<int16_t>& getGtpCameraOrientationOnAxis( size_t index ) { return gtpCameraOrientationOnAxis_[index]; }
  int16_t               getGtpCameraOffsetOnAxis( uint8_t cameraId, size_t axisId ) {
    return gtpCameraOffsetOnAxis_[cameraId][axisId];
  }
  int16_t getGtpCameraOrientationOnAxis( uint8_t cameraId, size_t axisId ) {
    return gtpCameraOrientationOnAxis_[cameraId][axisId];
  }

  void setGtpCancelFlag( bool value ) { gtpCancelFlag_ = value; }
  void setGtpScaleEnabledFlag( bool value ) { gtpScaleEnabledFlag_ = value; }
  void setGtpOffsetEnabledFlag( bool value ) { gtpOffsetEnabledFlag_ = value; }
  void setGtpRotationEnabledFlag( bool value ) { gtpRotationEnabledFlag_ = value; }
  void setGtpGeometryScaleOnAxis( size_t index, uint32_t value ) { gtpGeometryScaleOnAxis_[index] = value; }
  void setGtpGeometryOffsetOnAxis( size_t index, int32_t value ) { gtpGeometryOffsetOnAxis_[index] = value; }
  void setGtpRotationQx( uint16_t value ) { gtpRotationQx_ = value; }
  void setGtpRotationQy( uint16_t value ) { gtpRotationQy_ = value; }
  void setGtpRotationQz( uint16_t value ) { gtpRotationQz_ = value; }
  void setGtpNumCameraInfoMinus1( uint8_t value ) { gtpNumCameraInfoMinus1_ = value; }
  void setGtpCameraOffsetOnAxis( uint8_t cameraId, uint8_t axisId, int16_t value ) {
    gtpCameraOffsetOnAxis_[cameraId][axisId] = value;
  }
  void setGtpCameraOrientationOnAxis( uint8_t cameraId, uint8_t axisId, int16_t value ) {
    gtpCameraOrientationOnAxis_[cameraId][axisId] = value;
  }

 private:
  bool                 gtpCancelFlag_;
  bool                 gtpScaleEnabledFlag_;
  bool                 gtpOffsetEnabledFlag_;
  bool                 gtpRotationEnabledFlag_;
  uint32_t             gtpGeometryScaleOnAxis_[3];
  int32_t              gtpGeometryOffsetOnAxis_[3];
  uint8_t              gtpNumCameraInfoMinus1_;
  int16_t              gtpRotationQx_;
  int16_t              gtpRotationQy_;
  int16_t              gtpRotationQz_;
  std::vector<int16_t> gtpCameraOffsetOnAxis_[3];
  std::vector<int16_t> gtpCameraOrientationOnAxis_[3];
};
*/

// F.2.10  Attribute transformation parameters SEI message syntax
class SEIAttributeTransformationParams : public SEI {
 public:
  SEIAttributeTransformationParams() :
      atpCancelFlag_( false ),
      atpNumAttributeUpdates_( 0 ),
      atpPersistenceFlag_( false ) {}

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
  bool                                getAtpPersistenceFlag() { return atpPersistenceFlag_; }
  uint8_t                             getAtpAttributeIdx( size_t i ) { return atpAttributeIdx_[i]; }
  uint8_t                             getAtpDimensionMinus1( size_t i ) { return atpDimensionMinus1_[i]; }
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
  void setAtpPersistenceFlag( bool value ) { atpPersistenceFlag_ = value; }

 private:
  bool                               atpCancelFlag_;
  int32_t                            atpNumAttributeUpdates_;
  std::vector<uint8_t>               atpAttributeIdx_;
  std::vector<uint8_t>               atpDimensionMinus1_;
  std::vector<std::vector<bool>>     atpScaleParamsEnabledFlag_;
  std::vector<std::vector<bool>>     atpOffsetParamsEnabledFlag_;
  std::vector<std::vector<uint32_t>> atpAttributeScale_;
  std::vector<std::vector<int32_t>>  atpAttributeOffset_;
  bool                               atpPersistenceFlag_;
};

// F.2.11  Active substreams SEI message syntax
class SEIActiveSubBitstreams : public SEI {
 public:
  SEIActiveSubBitstreams() :
      activeAttributesChangesFlag_( 0 ),
      activeMapsChangesFlag_( 0 ),
      auxiliaryPointsSubstreamsActiveFlag_( 0 ),
      allAttributesActiveFlag_( 0 ),
      allMapsActiveFlag_( 0 ),
      activeAttributeCountMinus1_( 0 ),
      activeMapCountMinus1_( 0 ) {}
  ~SEIActiveSubBitstreams() {
    activeAttributeIdx_.clear();
    activeMapIdx_.clear();
  }
  SEIActiveSubBitstreams& operator=( const SEIActiveSubBitstreams& ) = default;

  SeiPayloadType        getPayloadType() { return ACTIVE_SUB_BITSTREAMS; }
  bool                  getActiveAttributesChangesFlag() { return activeAttributesChangesFlag_; }
  bool                  getActiveMapsChangesFlag() { return activeMapsChangesFlag_; }
  bool                  getAuxiliaryPointsSubstreamsActiveFlag() { return auxiliaryPointsSubstreamsActiveFlag_; }
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
  void setAuxiliaryPointsSubstreamsActiveFlag( bool value ) { auxiliaryPointsSubstreamsActiveFlag_ = value; }
  void setAllAttributesActiveFlag( bool value ) { allAttributesActiveFlag_ = value; }
  void setAllMapsActiveFlag( bool value ) { allMapsActiveFlag_ = value; }
  void setActiveAttributeCountMinus1( uint8_t value ) { activeAttributeCountMinus1_ = value; }
  void setActiveMapCountMinus1( uint8_t value ) { activeMapCountMinus1_ = value; }
  void setActiveAttributeIdx( size_t index, uint8_t value ) { activeAttributeIdx_[index] = value; }
  void setActiveMapIdx( size_t index, uint8_t value ) { activeMapIdx_[index] = value; }

 private:
  bool                 activeAttributesChangesFlag_;
  bool                 activeMapsChangesFlag_;
  bool                 auxiliaryPointsSubstreamsActiveFlag_;
  bool                 allAttributesActiveFlag_;
  bool                 allMapsActiveFlag_;
  uint8_t              activeAttributeCountMinus1_;
  uint8_t              activeMapCountMinus1_;
  std::vector<uint8_t> activeAttributeIdx_;
  std::vector<uint8_t> activeMapIdx_;
};

// F.2.12  Component codec mapping SEI message syntax
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
  uint8_t                   getCcmCodecMappingsCountMinus1() { return ccmCodecMappingsCountMinus1_; }
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

// F.2.14  Volumetric Tiling SEI message syntax
// m52705
class SEISceneObjectInformation : public SEI {
 public:
  SEISceneObjectInformation() {}
  ~SEISceneObjectInformation() {}
  SEISceneObjectInformation& operator=( const SEISceneObjectInformation& ) = default;
  void                       allocateObjectIdx() { soiObjectIdx_.resize( soiNumObjectUpdates_ ); }
  void                       allocateObjectNumDependencies( size_t index, size_t objectNumDependencies ) {
    soiObjectDependencyIdx_[index].resize( objectNumDependencies );
  }
  void allocate( size_t size ) {
    if ( soiObjectCancelFlag_.size() < size ) {
      soiObjectCancelFlag_.resize( size );
      soiObjectLabelUpdateFlag_.resize( size );
      soiObjectLabelIdx_.resize( size );
      soiPriorityUpdateFlag_.resize( size );
      soiPriorityValue_.resize( size );
      soiObjectHiddenFlag_.resize( size );
      soiObjectDependencyUpdateFlag_.resize( size );
      soiObjectNumDependencies_.resize( size );
      soiVisibilityConesUpdateFlag_.resize( size );
      soiDirectionX_.resize( size );
      soiDirectionY_.resize( size );
      soiDirectionZ_.resize( size );
      soiAngle_.resize( size );
      soi3dBoundingBoxUpdateFlag_.resize( size );
      soi3dBoundingBoxX_.resize( size );
      soi3dBoundingBoxY_.resize( size );
      soi3dBoundingBoxZ_.resize( size );
      soi3dBoundingBoxDeltaX_.resize( size );
      soi3dBoundingBoxDeltaY_.resize( size );
      soi3dBoundingBoxDeltaZ_.resize( size );
      soiCollisionShapeUpdateFlag_.resize( size );
      soiCollisionShapeId_.resize( size );
      soiPointStyleUpdateFlag_.resize( size );
      soiPointShapeId_.resize( size );
      soiPointSize_.resize( size );
      soiMaterialIdUpdateFlag_.resize( size );
      soiMaterialId_.resize( size );

      soiObjectDependencyIdx_.resize( size );  // 16
    }
  }

  SeiPayloadType         getPayloadType() { return SCENE_OBJECT_INFORMATION; }
  bool                   getSoiCancelFlag() { return soiCancelFlag_; }
  uint32_t               getSoiNumObjectUpdates() { return soiNumObjectUpdates_; }
  bool                   getSoiSimpleObjectsFlag() { return soiSimpleObjectsFlag_; }
  bool                   getSoiObjectLabelPresentFlag() { return soiObjectLabelPresentFlag_; }
  bool                   getSoiPriorityPresentFlag() { return soiPriorityPresentFlag_; }
  bool                   getSoiObjectHiddenPresentFlag() { return soiObjectHiddenPresentFlag_; }
  bool                   getSoiObjectDependencyPresentFlag() { return soiObjectDependencyPresentFlag_; }
  bool                   getSoiVisibilityConesPresentFlag() { return soiVisibilityConesPresentFlag_; }
  bool                   getSoi3dBoundingBoxPresentFlag() { return soi3dBoundingBoxPresentFlag_; }
  bool                   getSoiCollisionShapePresentFlag() { return soiCollisionShapePresentFlag_; }
  bool                   getSoiPointStylePresentFlag() { return soiPointStylePresentFlag_; }
  bool                   getSoiMaterialIdPresentFlag() { return soiMaterialIdPresentFlag_; }
  bool                   getSoiExtensionPresentFlag() { return soiExtensionPresentFlag_; }
  uint8_t                getSoi3dBoundingBoxScaleLog2() { return soi3dBoundingBoxScaleLog2_; }
  uint8_t                getSoi3dBoundingBoxPrecisionMinus8() { return soi3dBoundingBoxPrecisionMinus8_; }
  uint8_t                getSoiLog2MaxObjectIdxUpdated() { return soiLog2MaxObjectIdxUpdated_; }
  uint8_t                getSoiLog2MaxObjectDependencyIdx() { return soiLog2MaxObjectDependencyIdx_; }
  std::vector<uint32_t>& getSoiObjectIdx() { return soiObjectIdx_; }
  uint32_t               getSoiObjectIdx( size_t index ) { return soiObjectIdx_[index]; }
  bool                   getSoiObjectCancelFlag( size_t index ) { return soiObjectCancelFlag_[index]; }
  bool                   getSoiObjectLabelUpdateFlag( size_t index ) { return soiObjectLabelUpdateFlag_[index]; }
  uint32_t               getSoiObjectLabelIdx( size_t index ) { return soiObjectLabelIdx_[index]; }
  bool                   getSoiPriorityUpdateFlag( size_t index ) { return soiPriorityUpdateFlag_[index]; }
  uint8_t                getSoiPriorityValue( size_t index ) { return soiPriorityValue_[index]; }
  bool                   getSoiObjectHiddenFlag( size_t index ) { return soiObjectHiddenFlag_[index]; }
  bool     getSoiObjectDependencyUpdateFlag( size_t index ) { return soiObjectDependencyUpdateFlag_[index]; }
  uint8_t  getSoiObjectNumDependencies( size_t index ) { return soiObjectNumDependencies_[index]; }
  uint32_t getSoiObjectDependencyIdx( size_t index, size_t objnum ) { return soiObjectDependencyIdx_[index][objnum]; }
  bool     getSoiVisibilityConesUpdateFlag( size_t index ) { return soiVisibilityConesUpdateFlag_[index]; }
  uint32_t getSoiDirectionX( size_t index ) { return soiDirectionX_[index]; }
  uint32_t getSoiDirectionY( size_t index ) { return soiDirectionY_[index]; }
  uint32_t getSoiDirectionZ( size_t index ) { return soiDirectionZ_[index]; }
  uint16_t getSoiAngle( size_t index ) { return soiAngle_[index]; }
  bool     getSoi3dBoundingBoxUpdateFlag( size_t index ) { return soi3dBoundingBoxUpdateFlag_[index]; }
  uint32_t getSoi3dBoundingBoxX( size_t index ) { return soi3dBoundingBoxX_[index]; }
  uint32_t getSoi3dBoundingBoxY( size_t index ) { return soi3dBoundingBoxY_[index]; }
  uint32_t getSoi3dBoundingBoxZ( size_t index ) { return soi3dBoundingBoxZ_[index]; }
  uint32_t getSoi3dBoundingBoxDeltaX( size_t index ) { return soi3dBoundingBoxDeltaX_[index]; }
  uint32_t getSoi3dBoundingBoxDeltaY( size_t index ) { return soi3dBoundingBoxDeltaY_[index]; }
  uint32_t getSoi3dBoundingBoxDeltaZ( size_t index ) { return soi3dBoundingBoxDeltaZ_[index]; }
  bool     getSoiCollisionShapeUpdateFlag( size_t index ) { return soiCollisionShapeUpdateFlag_[index]; }
  uint16_t getSoiCollisionShapeId( size_t index ) { return soiCollisionShapeId_[index]; }
  bool     getSoiPointStyleUpdateFlag( size_t index ) { return soiPointStyleUpdateFlag_[index]; }
  uint8_t  getSoiPointShapeId( size_t index ) { return soiPointShapeId_[index]; }
  uint16_t getSoiPointSize( size_t index ) { return soiPointSize_[index]; }
  bool     getSoiMaterialIdUpdateFlag( size_t index ) { return soiMaterialIdUpdateFlag_[index]; }
  uint16_t getSoiMaterialId( size_t index ) { return soiMaterialId_[index]; }

  void setSoiCancelFlag( bool value ) { soiCancelFlag_ = value; }
  void setSoiNumObjectUpdates( uint32_t value ) { soiNumObjectUpdates_ = value; }
  void setSoiSimpleObjectsFlag( bool value ) { soiSimpleObjectsFlag_ = value; }
  void setSoiObjectLabelPresentFlag( bool value ) { soiObjectLabelPresentFlag_ = value; }
  void setSoiPriorityPresentFlag( bool value ) { soiPriorityPresentFlag_ = value; }
  void setSoiObjectHiddenPresentFlag( bool value ) { soiObjectHiddenPresentFlag_ = value; }
  void setSoiObjectDependencyPresentFlag( bool value ) { soiObjectDependencyPresentFlag_ = value; }
  void setSoiVisibilityConesPresentFlag( bool value ) { soiVisibilityConesPresentFlag_ = value; }
  void setSoi3dBoundingBoxPresentFlag( bool value ) { soi3dBoundingBoxPresentFlag_ = value; }
  void setSoiCollisionShapePresentFlag( bool value ) { soiCollisionShapePresentFlag_ = value; }
  void setSoiPointStylePresentFlag( bool value ) { soiPointStylePresentFlag_ = value; }
  void setSoiMaterialIdPresentFlag( bool value ) { soiMaterialIdPresentFlag_ = value; }
  void setSoiExtensionPresentFlag( bool value ) { soiExtensionPresentFlag_ = value; }
  void setSoi3dBoundingBoxScaleLog2( uint8_t value ) { soi3dBoundingBoxScaleLog2_ = value; }
  void setSoi3dBoundingBoxPrecisionMinus8( uint8_t value ) { soi3dBoundingBoxPrecisionMinus8_ = value; }
  void setSoiLog2MaxObjectIdxUpdated( uint8_t value ) { soiLog2MaxObjectIdxUpdated_ = value; }
  void setSoiLog2MaxObjectDependencyIdx( uint8_t value ) { soiLog2MaxObjectDependencyIdx_ = value; }
  void setSoiObjectIdx( size_t index, uint32_t value ) { soiObjectIdx_[index] = value; }
  void setSoiObjectCancelFlag( size_t index, bool value ) { soiObjectCancelFlag_[index] = value; }
  void setSoiObjectLabelUpdateFlag( size_t index, bool value ) { soiObjectLabelUpdateFlag_[index] = value; }
  void setSoiObjectLabelIdx( size_t index, uint32_t value ) { soiObjectLabelIdx_[index] = value; }
  void setSoiPriorityUpdateFlag( size_t index, bool value ) { soiPriorityUpdateFlag_[index] = value; }
  void setSoiPriorityValue( size_t index, uint8_t value ) { soiPriorityValue_[index] = value; }
  void setSoiObjectHiddenFlag( size_t index, bool value ) { soiObjectHiddenFlag_[index] = value; }
  void setSoiObjectDependencyUpdateFlag( size_t index, bool value ) { soiObjectDependencyUpdateFlag_[index] = value; }
  void setSoiObjectNumDependencies( size_t index, uint8_t value ) { soiObjectNumDependencies_[index] = value; }
  void setSoiObjectDependencyIdx( size_t index, size_t objnum, uint32_t value ) {
    soiObjectDependencyIdx_[index][objnum] = value;
  }
  void setSoiVisibilityConesUpdateFlag( size_t index, bool value ) { soiVisibilityConesUpdateFlag_[index] = value; }
  void setSoiDirectionX( size_t index, uint32_t value ) { soiDirectionX_[index] = value; }
  void setSoiDirectionY( size_t index, uint32_t value ) { soiDirectionY_[index] = value; }
  void setSoiDirectionZ( size_t index, uint32_t value ) { soiDirectionZ_[index] = value; }
  void setSoiAngle( size_t index, uint16_t value ) { soiAngle_[index] = value; }
  void setSoi3dBoundingBoxUpdateFlag( size_t index, bool value ) { soi3dBoundingBoxUpdateFlag_[index] = value; }
  void setSoi3dBoundingBoxX( size_t index, uint32_t value ) { soi3dBoundingBoxX_[index] = value; }
  void setSoi3dBoundingBoxY( size_t index, uint32_t value ) { soi3dBoundingBoxY_[index] = value; }
  void setSoi3dBoundingBoxZ( size_t index, uint32_t value ) { soi3dBoundingBoxZ_[index] = value; }
  void setSoi3dBoundingBoxDeltaX( size_t index, uint32_t value ) { soi3dBoundingBoxDeltaX_[index] = value; }
  void setSoi3dBoundingBoxDeltaY( size_t index, uint32_t value ) { soi3dBoundingBoxDeltaY_[index] = value; }
  void setSoi3dBoundingBoxDeltaZ( size_t index, uint32_t value ) { soi3dBoundingBoxDeltaZ_[index] = value; }
  void setSoiCollisionShapeUpdateFlag( size_t index, bool value ) { soiCollisionShapeUpdateFlag_[index] = value; }
  void setSoiCollisionShapeId( size_t index, uint16_t value ) { soiCollisionShapeId_[index] = value; }
  void setSoiPointStyleUpdateFlag( size_t index, bool value ) { soiPointStyleUpdateFlag_[index] = value; }
  void setSoiPointShapeId( size_t index, uint8_t value ) { soiPointShapeId_[index] = value; }
  void setSoiPointSize( size_t index, uint16_t value ) { soiPointSize_[index] = value; }
  void setSoiMaterialIdUpdateFlag( size_t index, bool value ) { soiMaterialIdUpdateFlag_[index] = value; }
  void setSoiMaterialId( size_t index, uint16_t value ) { soiMaterialId_[index] = value; }

 private:
  bool     soiCancelFlag_;
  uint32_t soiNumObjectUpdates_;
  bool     soiSimpleObjectsFlag_;
  bool     soiObjectLabelPresentFlag_;
  bool     soiPriorityPresentFlag_;
  bool     soiObjectHiddenPresentFlag_;
  bool     soiObjectDependencyPresentFlag_;
  bool     soiVisibilityConesPresentFlag_;
  bool     soi3dBoundingBoxPresentFlag_;
  bool     soiCollisionShapePresentFlag_;
  bool     soiPointStylePresentFlag_;
  bool     soiMaterialIdPresentFlag_;
  bool     soiExtensionPresentFlag_;
  uint8_t  soi3dBoundingBoxScaleLog2_;
  uint8_t  soi3dBoundingBoxPrecisionMinus8_;
  uint8_t  soiLog2MaxObjectIdxUpdated_;
  uint8_t  soiLog2MaxObjectDependencyIdx_;
  // max pow(2,32)
  std::vector<uint32_t>              soiObjectIdx_;
  std::vector<bool>                  soiObjectCancelFlag_;
  std::vector<bool>                  soiObjectLabelUpdateFlag_;
  std::vector<uint32_t>              soiObjectLabelIdx_;
  std::vector<bool>                  soiPriorityUpdateFlag_;
  std::vector<uint8_t>               soiPriorityValue_;
  std::vector<bool>                  soiObjectHiddenFlag_;
  std::vector<bool>                  soiObjectDependencyUpdateFlag_;
  std::vector<uint8_t>               soiObjectNumDependencies_;
  std::vector<std::vector<uint32_t>> soiObjectDependencyIdx_;  // 16
  std::vector<bool>                  soiVisibilityConesUpdateFlag_;
  std::vector<uint32_t>              soiDirectionX_;
  std::vector<uint32_t>              soiDirectionY_;
  std::vector<uint32_t>              soiDirectionZ_;
  std::vector<uint16_t>              soiAngle_;
  std::vector<bool>                  soi3dBoundingBoxUpdateFlag_;
  std::vector<uint32_t>              soi3dBoundingBoxX_;
  std::vector<uint32_t>              soi3dBoundingBoxY_;
  std::vector<uint32_t>              soi3dBoundingBoxZ_;
  std::vector<uint32_t>              soi3dBoundingBoxDeltaX_;
  std::vector<uint32_t>              soi3dBoundingBoxDeltaY_;
  std::vector<uint32_t>              soi3dBoundingBoxDeltaZ_;
  std::vector<bool>                  soiCollisionShapeUpdateFlag_;
  std::vector<uint16_t>              soiCollisionShapeId_;
  std::vector<bool>                  soiPointStyleUpdateFlag_;
  std::vector<uint8_t>               soiPointShapeId_;
  std::vector<uint16_t>              soiPointSize_;
  std::vector<bool>                  soiMaterialIdUpdateFlag_;
  std::vector<uint16_t>              soiMaterialId_;

};  // sceneobjectinformation

class SEIObjectLabelInformation : public SEI {
 public:
  SEIObjectLabelInformation() {}
  ~SEIObjectLabelInformation() {}
  SEIObjectLabelInformation& operator=( const SEIObjectLabelInformation& ) = default;

  SeiPayloadType getPayloadType() { return OBJECT_LABEL_INFORMATION; }
  void           allocate() { oliLabelIdx_.resize( oliNumLabelUpdates_, 0 ); }
  bool           getOliCancelFlag() { return oliCancelFlag_; }
  bool           getOliLabelLanguagePresentFlag() { return oliLabelLanguagePresentFlag_; }
  std::string&   getOliLabelLanguage() { return oliLabelLanguage_; }
  uint32_t       getOliNumLabelUpdates() { return oliNumLabelUpdates_; }
  uint32_t       getOliLabelIdx( size_t index ) { return oliLabelIdx_[index]; }
  bool           getOliLabelCancelFlag() { return oliLabelCancelFlag_; }
  bool           getOliBitEqualToZero() { return oliBitEqualToZero_; }
  std::string&   getOliLabel( size_t index ) { return oliLabel_[index]; }
  bool           getOliPersistenceFlag() { return oliPersistenceFlag_; }

  void setOliCancelFlag( bool value ) { oliCancelFlag_ = value; }
  void setOliLabelLanguagePresentFlag( bool value ) { oliLabelLanguagePresentFlag_ = value; }
  void setOliLabelLanguage( std::string value ) { oliLabelLanguage_ = value; }
  void setOliNumLabelUpdates( uint32_t value ) { oliNumLabelUpdates_ = value; }
  void setOliLabelIdx( size_t index, uint32_t value ) { oliLabelIdx_[index] = value; }
  void setOliLabelCancelFlag( bool value ) { oliLabelCancelFlag_ = value; }
  void setOliBitEqualToZero( bool value ) { oliBitEqualToZero_ = value; }
  void setOliLabel( size_t index, std::string value ) {
    if ( oliLabel_.size() < index ) { oliLabel_.resize( index + 1 ); }
    oliLabel_[index] = value;
  }
  void setOliPersistenceFlag( bool value ) { oliPersistenceFlag_ = value; }

 private:
  bool                     oliCancelFlag_;
  bool                     oliLabelLanguagePresentFlag_;
  std::string              oliLabelLanguage_;  // st(v)
  uint32_t                 oliNumLabelUpdates_;
  std::vector<uint32_t>    oliLabelIdx_;
  bool                     oliLabelCancelFlag_;
  bool                     oliBitEqualToZero_;
  std::vector<std::string> oliLabel_;  // st(v)
  bool                     oliPersistenceFlag_;
};  // Object label information

class SEIPatchInformation : public SEI {
 public:
  SEIPatchInformation() {}
  ~SEIPatchInformation() {}
  SEIPatchInformation& operator=( const SEIPatchInformation& ) = default;
  SeiPayloadType       getPayloadType() { return PATCH_INFORMATION; }
  bool                 getPiPersistenceFlag() { return piPersistenceFlag_; }
  bool                 getPiResetFlag() { return piResetFlag_; }
  uint32_t             getPiNumTileUpdates() { return piNumTileUpdates_; }
  uint8_t              getPiLog2MaxObjectIdxTracked() { return piLog2MaxObjectIdxTracked_; }
  uint8_t              getPiLog2MaxPatchIdxUpdated() { return piLog2MaxPatchIdxUpdated_; }
  uint32_t             getPiTileAddress( size_t index ) { return piTileAddress_[index]; }
  bool                 getPiTileCancelFlag( size_t index ) { return piTileCancelFlag_[index]; }
  uint32_t             getPiNumPatchUpdates( size_t index ) { return piNumPatchUpdates_[index]; }
  uint32_t             getPiPatchIdx( size_t index, size_t index2 ) { return piPatchIdx_[index][index2]; }
  bool                 getPiPatchCancelFlag( size_t index, size_t index2 ) { return piPatchCancelFlag_[index][index2]; }
  uint32_t             getPiPatchNumberOfObjectsMinus1( size_t index, size_t index2 ) {
    return piPatchNumberOfObjectsMinus1_[index][index2];
  }
  uint32_t getPiPatchObjectIdx( size_t index, size_t index2, size_t index3 ) {
    return piPatchObjectIdx_[index][index2][index3];
  }
  void setPiPersistenceFlag( bool value ) { piPersistenceFlag_ = value; }
  void setPiResetFlag( bool value ) { piResetFlag_ = value; }
  void setPiNumTileUpdates( uint32_t value ) { piNumTileUpdates_ = value; }
  void setPiLog2MaxObjectIdxTracked( uint8_t value ) { piLog2MaxObjectIdxTracked_ = value; }
  void setPiLog2MaxPatchIdxUpdated( uint8_t value ) { piLog2MaxPatchIdxUpdated_ = value; }
  void setPiTileAddress( size_t index, uint32_t value ) { piTileAddress_[index] = value; }
  void setPiTileCancelFlag( size_t index, bool value ) { piTileCancelFlag_[index] = value; }
  void setPiNumPatchUpdates( size_t index, uint32_t value ) { piNumPatchUpdates_[index] = value; }
  void setPiPatchIdx( size_t index, size_t index2, uint32_t value ) { piPatchIdx_[index][index2] = value; }
  void setPiPatchCancelFlag( size_t index, size_t index2, bool value ) { piPatchCancelFlag_[index][index2] = value; }
  void setPiPatchNumberOfObjectsMinus1( size_t index, size_t index2, uint32_t value ) {
    piPatchNumberOfObjectsMinus1_[index][index2] = value;
  }
  void setPiPatchObjectIdx( size_t index, size_t index2, size_t index3, uint32_t value ) {
    piPatchObjectIdx_[index][index2][index3] = value;
  }

 private:
  bool                                            piPersistenceFlag_;
  bool                                            piResetFlag_;
  uint32_t                                        piNumTileUpdates_;
  uint8_t                                         piLog2MaxObjectIdxTracked_;
  uint8_t                                         piLog2MaxPatchIdxUpdated_;
  std::vector<uint32_t>                           piTileAddress_;
  std::vector<bool>                               piTileCancelFlag_;
  std::vector<uint32_t>                           piNumPatchUpdates_;
  std::vector<std::vector<uint32_t>>              piPatchIdx_;
  std::vector<std::vector<bool>>                  piPatchCancelFlag_;
  std::vector<std::vector<uint32_t>>              piPatchNumberOfObjectsMinus1_;
  std::vector<std::vector<std::vector<uint32_t>>> piPatchObjectIdx_;
};

class SEIVolumetricRectangleInformation : public SEI {
 public:
  SEIVolumetricRectangleInformation() {}
  ~SEIVolumetricRectangleInformation() {}
  SEIVolumetricRectangleInformation& operator=( const SEIVolumetricRectangleInformation& ) = default;
  void                               allocate( size_t size ) {
    vriBoundingBoxUpdateFlag_.resize( size );
    vriBoundingBoxTop_.resize( size );
    vriBoundingBoxLeft_.resize( size );
    vriBoundingBoxWidth_.resize( size );
    vriBoundingBoxHeight_.resize( size );
    vriRectangleObjectIdx_.resize( size );
  }
  void allocateRectangleObjectIdx( size_t index, size_t size ) { vriRectangleObjectIdx_[index].resize( size ); }

  SeiPayloadType getPayloadType() { return VOLUMETRIC_RECTANGLE_INFORMATION; }
  bool           getVriPersistenceFlag() { return vriPersistenceFlag_; }
  bool           getVriResetFlag() { return vriResetFlag_; }
  uint32_t       getVriNumRectanglesUpdates() { return vriNumRectanglesUpdates_; }
  uint8_t        getVriLog2MaxObjectIdxTracked() { return vriLog2MaxObjectIdxTracked_; }
  uint8_t        getVriLog2MaxRectangleIdxUpdated() { return vriLog2MaxRectangleIdxUpdated_; }
  uint32_t       getVriRectangleIdx( size_t index ) { return vriRectangleIdx_[index]; }
  bool           getVriRectangleCancelFlag( size_t index ) { return vriRectangleCancelFlag_[index]; }
  bool           getVriBoundingBoxUpdateFlag( size_t index ) { return vriBoundingBoxUpdateFlag_[index]; }
  uint32_t       getVriBoundingBoxTop( size_t index ) { return vriBoundingBoxTop_[index]; }
  uint32_t       getVriBoundingBoxLeft( size_t index ) { return vriBoundingBoxLeft_[index]; }
  uint32_t       getVriBoundingBoxWidth( size_t index ) { return vriBoundingBoxWidth_[index]; }
  uint32_t       getVriBoundingBoxHeight( size_t index ) { return vriBoundingBoxHeight_[index]; }
  uint32_t getVriRectangleNumberOfObjectsMinus1( size_t index ) { return vriRectangleNumberOfObjectsMinus1_[index]; }
  uint32_t getVriRectangleObjectIdx( size_t index, size_t index2 ) { return vriRectangleObjectIdx_[index][index2]; }

  void setVriPersistenceFlag( bool value ) { vriPersistenceFlag_ = value; }
  void setVriResetFlag( bool value ) { vriResetFlag_ = value; }
  void setVriNumRectanglesUpdates( uint32_t value ) { vriNumRectanglesUpdates_ = value; }
  void setVriLog2MaxObjectIdxTracked( uint8_t value ) { vriLog2MaxObjectIdxTracked_ = value; }
  void setVriLog2MaxRectangleIdxUpdated( uint8_t value ) { vriLog2MaxRectangleIdxUpdated_ = value; }
  void setVriRectangleIdx( size_t index, uint32_t value ) { vriRectangleIdx_[index] = value; }
  void setVriRectangleCancelFlag( size_t index, bool value ) { vriRectangleCancelFlag_[index] = value; }
  void setVriBoundingBoxUpdateFlag( size_t index, bool value ) { vriBoundingBoxUpdateFlag_[index] = value; }
  void setVriBoundingBoxTop( size_t index, uint32_t value ) { vriBoundingBoxTop_[index] = value; }
  void setVriBoundingBoxLeft( size_t index, uint32_t value ) { vriBoundingBoxLeft_[index] = value; }
  void setVriBoundingBoxWidth( size_t index, uint32_t value ) { vriBoundingBoxWidth_[index] = value; }
  void setVriBoundingBoxHeight( size_t index, uint32_t value ) { vriBoundingBoxHeight_[index] = value; }
  void setVriRectangleNumberOfObjectsMinus1( size_t index, uint32_t value ) {
    vriRectangleNumberOfObjectsMinus1_[index] = value;
  }
  void setVriRectangleObjectIdx( size_t index, size_t index2, uint32_t value ) {
    vriRectangleObjectIdx_[index][index2] = value;
  }

 private:
  bool                               vriPersistenceFlag_;
  bool                               vriResetFlag_;
  uint32_t                           vriNumRectanglesUpdates_;
  uint8_t                            vriLog2MaxObjectIdxTracked_;
  uint8_t                            vriLog2MaxRectangleIdxUpdated_;
  std::vector<uint32_t>              vriRectangleIdx_;
  std::vector<bool>                  vriRectangleCancelFlag_;
  std::vector<bool>                  vriBoundingBoxUpdateFlag_;
  std::vector<uint32_t>              vriBoundingBoxTop_;
  std::vector<uint32_t>              vriBoundingBoxLeft_;
  std::vector<uint32_t>              vriBoundingBoxWidth_;
  std::vector<uint32_t>              vriBoundingBoxHeight_;
  std::vector<uint32_t>              vriRectangleNumberOfObjectsMinus1_;
  std::vector<std::vector<uint32_t>> vriRectangleObjectIdx_;
};

// F.2.15  Buffering period SEI message syntax
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

// F.2.16  Atlas frame timing SEI message syntax
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

class SEIAtlasInformation : public SEI {
 public:
  SEIAtlasInformation() :
      aiPersistenceFlag_( false ),
      aiResetFlag_( false ),
      aiNumAtlasesMinus1_( 0 ),
      aiNumUpdates_( 0 ),
      aiLog2MaxObjectIdxTracked_( 0 ) {}
  ~SEIAtlasInformation() {
    aiObjectIdx_.clear();
    for ( auto& element : aiObjectInAtlasPresentFlag_ ) { element.clear(); }
    aiObjectInAtlasPresentFlag_.clear();
  }
  SEIAtlasInformation& operator=( const SEIAtlasInformation& ) = default;

  SeiPayloadType getPayloadType() { return ATLAS_INFORMATION; }

  void allocate() {
    aiObjectIdx_.resize( aiNumUpdates_, 0 );
    aiObjectInAtlasPresentFlag_.resize( aiNumUpdates_ );
    for ( auto& element : aiObjectInAtlasPresentFlag_ ) { element.resize( aiNumAtlasesMinus1_ + 1, false ); }
  }

  bool     getPersistenceFlag() { return aiPersistenceFlag_; }
  bool     getResetFlag() { return aiResetFlag_; }
  uint32_t getNumAtlasesMinus1() { return aiNumAtlasesMinus1_; }
  uint32_t getNumUpdates() { return aiNumUpdates_; }
  uint32_t getLog2MaxObjectIdxTracked() { return aiLog2MaxObjectIdxTracked_; }
  uint32_t getObjectIdx( size_t i ) { return aiObjectIdx_[i]; }
  bool     getObjectInAtlasPresentFlag( size_t i, size_t j ) { return aiObjectInAtlasPresentFlag_[i][j]; }

  void setPersistenceFlag( bool value ) { aiPersistenceFlag_ = value; }
  void setResetFlag( bool value ) { aiResetFlag_ = value; }
  void setNumAtlasesMinus1( uint32_t value ) { aiNumAtlasesMinus1_ = value; }
  void setNumUpdates( uint32_t value ) { aiNumUpdates_ = value; }
  void setLog2MaxObjectIdxTracked( uint32_t value ) { aiLog2MaxObjectIdxTracked_ = value; }
  void setObjectIdx( size_t i, uint32_t value ) { aiObjectIdx_[i] = value; }
  void setObjectInAtlasPresentFlag( size_t i, size_t j, bool value ) { aiObjectInAtlasPresentFlag_[i][j] = value; }

 private:
  bool                           aiPersistenceFlag_;
  bool                           aiResetFlag_;
  uint32_t                       aiNumAtlasesMinus1_;
  uint32_t                       aiNumUpdates_;
  uint32_t                       aiLog2MaxObjectIdxTracked_;
  std::vector<uint32_t>          aiObjectIdx_;
  std::vector<std::vector<bool>> aiObjectInAtlasPresentFlag_;
};

class SEIViewportCameraParameters : public SEI {
 public:
  SEIViewportCameraParameters() :
      vcpCameraId_( 0 ),
      vcpCancelFlag_( false ),
      vcpPersistenceFlag_( false ),
      vcpCameraType_( 0.f ),
      vcpErpHorizontalFov_( 0.f ),
      vcpErpVerticalFov_( 0.f ),
      vcpPerspectiveAspectRatio_( 0.f ),
      vcpPerspectiveHorizontalFov_( 0.f ),
      vcpOrthoAspectRatio_( 0.f ),
      vcpOrthoHorizontalSize_( 0.f ),
      vcpClippingNearPlane_( 0.f ),
      vcpClippingFarPlane_( 0.f ) {}
  ~SEIViewportCameraParameters() {}
  SEIViewportCameraParameters& operator=( const SEIViewportCameraParameters& ) = default;

  SeiPayloadType getPayloadType() { return VIEWPORT_CAMERA_PARAMETERS; }

  uint32_t getCameraId() { return vcpCameraId_; }
  bool     getCancelFlag() { return vcpCancelFlag_; }
  bool     getPersistenceFlag() { return vcpPersistenceFlag_; }
  uint8_t  getCameraType() { return vcpCameraType_; }
  float    getErpHorizontalFov() { return vcpErpHorizontalFov_; }
  float    getErpVerticalFov() { return vcpErpVerticalFov_; }
  float    getPerspectiveAspectRatio() { return vcpPerspectiveAspectRatio_; }
  float    getPerspectiveHorizontalFov() { return vcpPerspectiveHorizontalFov_; }
  float    getOrthoAspectRatio() { return vcpOrthoAspectRatio_; }
  float    getOrthoHorizontalSize() { return vcpOrthoHorizontalSize_; }
  float    getClippingNearPlane() { return vcpClippingNearPlane_; }
  float    getClippingFarPlane() { return vcpClippingFarPlane_; }

  void setCameraId( uint32_t value ) { vcpCameraId_ = value; }
  void setCancelFlag( bool value ) { vcpCancelFlag_ = value; }
  void setPersistenceFlag( bool value ) { vcpPersistenceFlag_ = value; }
  void setCameraType( uint8_t value ) { vcpCameraType_ = value; }
  void setErpHorizontalFov( float value ) { vcpErpHorizontalFov_ = value; }
  void setErpVerticalFov( float value ) { vcpErpVerticalFov_ = value; }
  void setPerspectiveAspectRatio( float value ) { vcpPerspectiveAspectRatio_ = value; }
  void setPerspectiveHorizontalFov( float value ) { vcpPerspectiveHorizontalFov_ = value; }
  void setOrthoAspectRatio( float value ) { vcpOrthoAspectRatio_ = value; }
  void setOrthoHorizontalSize( float value ) { vcpOrthoHorizontalSize_ = value; }
  void setClippingNearPlane( float value ) { vcpClippingNearPlane_ = value; }
  void setClippingFarPlane( float value ) { vcpClippingFarPlane_ = value; }

 private:
  uint32_t vcpCameraId_;
  bool     vcpCancelFlag_;
  bool     vcpPersistenceFlag_;
  uint8_t  vcpCameraType_;
  float    vcpErpHorizontalFov_;
  float    vcpErpVerticalFov_;
  float    vcpPerspectiveAspectRatio_;
  float    vcpPerspectiveHorizontalFov_;
  float    vcpOrthoAspectRatio_;
  float    vcpOrthoHorizontalSize_;
  float    vcpClippingNearPlane_;
  float    vcpClippingFarPlane_;
};

class SEIViewportPosition : public SEI {
 public:
  SEIViewportPosition() :
      vpViewportId_( 0 ),
      vpCameraParametersPresentFlag_( false ),
      vpVcpCameraId_( 0 ),
      vpCancelFlag_( false ),
      vpPersistenceFlag_( false ),
      vpPosition_{0.0f, 0.0f, 0.0f},
      vpQuaternionX_( 0.0f ),
      vpQuaternionY_( 0.0f ),
      vpQuaternionZ_( 0.0f ),
      vpCenterViewFlag_( false ),
      vpLeftViewFlag_( false ) {}
  ~SEIViewportPosition() {}
  SEIViewportPosition& operator=( const SEIViewportPosition& ) = default;

  SeiPayloadType getPayloadType() { return VIEWPORT_POSITION; }

  uint32_t getViewportId() { return vpViewportId_; }
  bool     getCameraParametersPresentFlag() { return vpCameraParametersPresentFlag_; }
  uint8_t  getVcpCameraId() { return vpVcpCameraId_; }
  bool     getCancelFlag() { return vpCancelFlag_; }
  bool     getPersistenceFlag() { return vpPersistenceFlag_; }
  float    getPosition( size_t i ) { return vpPosition_[i]; }
  float    getQuaternionX() { return vpQuaternionX_; }
  float    getQuaternionY() { return vpQuaternionY_; }
  float    getQuaternionZ() { return vpQuaternionZ_; }
  bool     getCenterViewFlag() { return vpCenterViewFlag_; }
  bool     getLeftViewFlag() { return vpLeftViewFlag_; }

  void setViewportId( uint32_t value ) { vpViewportId_ = value; }
  void setCameraParametersPresentFlag( bool value ) { vpCameraParametersPresentFlag_ = value; }
  void setVcpCameraId( uint8_t value ) { vpVcpCameraId_ = value; }
  void setCancelFlag( bool value ) { vpCancelFlag_ = value; }
  void setPersistenceFlag( bool value ) { vpPersistenceFlag_ = value; }
  void setPosition( size_t i, float value ) { vpPosition_[i] = value; }
  void setQuaternionX( float value ) { vpQuaternionX_ = value; }
  void setQuaternionY( float value ) { vpQuaternionY_ = value; }
  void setQuaternionZ( float value ) { vpQuaternionZ_ = value; }
  void setCenterViewFlag( bool value ) { vpCenterViewFlag_ = value; }
  void setLeftViewFlag( bool value ) { vpLeftViewFlag_ = value; }

 private:
  uint32_t vpViewportId_;
  bool     vpCameraParametersPresentFlag_;
  uint8_t  vpVcpCameraId_;
  bool     vpCancelFlag_;
  bool     vpPersistenceFlag_;
  float    vpPosition_[3];
  float    vpQuaternionX_;
  float    vpQuaternionY_;
  float    vpQuaternionZ_;
  bool     vpCenterViewFlag_;
  bool     vpLeftViewFlag_;
};

class SEIGeometrySmoothing : public SEI {
 public:
  SEIGeometrySmoothing() :
      gsSmoothingPersistenceFlag_( false ),
      gsSmoothingResetFlag_( false ),
      gsSmoothingInstancesUpdated_( 0 ) {}
  ~SEIGeometrySmoothing() {
    gsSmoothingInstanceIndex_.clear();
    gsSmoothingInstanceCancelFlag_.clear();
    gsSmoothingMethodType_.clear();
    gsSmoothingGridSizeMinus2_.clear();
    gsSmoothingThreshold_.clear();
    gsPbfLog2ThresholdMinus1_.clear();
    gsPbfPassesCountMinus1_.clear();
    gsPbfFilterSizeMinus1_.clear();
  }
  SEIGeometrySmoothing& operator=( const SEIGeometrySmoothing& ) = default;

  SeiPayloadType getPayloadType() { return GEOMETRY_SMOOTHING; }
  void           allocate() {
    gsSmoothingInstanceIndex_.resize( gsSmoothingInstancesUpdated_, 0 );
    gsSmoothingInstanceCancelFlag_.resize( gsSmoothingInstancesUpdated_, 0 );
    gsSmoothingMethodType_.resize( gsSmoothingInstancesUpdated_, 0 );
    gsSmoothingGridSizeMinus2_.resize( gsSmoothingInstancesUpdated_, 0 );
    gsSmoothingThreshold_.resize( gsSmoothingInstancesUpdated_, 0 );
    gsPbfLog2ThresholdMinus1_.resize( gsSmoothingInstancesUpdated_, 0 );
    gsPbfPassesCountMinus1_.resize( gsSmoothingInstancesUpdated_, 0 );
    gsPbfFilterSizeMinus1_.resize( gsSmoothingInstancesUpdated_, 0 );
  }

  bool    getSmoothingPersistenceFlag() { return gsSmoothingPersistenceFlag_; }
  bool    getSmoothingResetFlag() { return gsSmoothingResetFlag_; }
  uint8_t getSmoothingInstancesUpdated() { return gsSmoothingInstancesUpdated_; }
  uint8_t getSmoothingInstanceIndex( size_t i ) { return gsSmoothingInstanceIndex_[i]; }
  bool    getSmoothingInstanceCancelFlag( size_t i ) { return gsSmoothingInstanceCancelFlag_[i]; }
  uint8_t getSmoothingMethodType( size_t i ) { return gsSmoothingMethodType_[i]; }
  uint8_t getSmoothingGridSizeMinus2( size_t i ) { return gsSmoothingGridSizeMinus2_[i]; }
  uint8_t getSmoothingThreshold( size_t i ) { return gsSmoothingThreshold_[i]; }
  uint8_t getPbfLog2ThresholdMinus1( size_t i ) { return gsPbfLog2ThresholdMinus1_[i]; }
  uint8_t getPbfPassesCountMinus1( size_t i ) { return gsPbfPassesCountMinus1_[i]; }
  uint8_t getPbfFilterSizeMinus1( size_t i ) { return gsPbfFilterSizeMinus1_[i]; }

  void setSmoothingPersistenceFlag( bool value ) { gsSmoothingPersistenceFlag_ = value; }
  void setSmoothingResetFlag( bool value ) { gsSmoothingResetFlag_ = value; }
  void setSmoothingInstancesUpdated( uint8_t value ) { gsSmoothingInstancesUpdated_ = value; }
  void setSmoothingInstanceIndex( size_t i, uint8_t value ) { gsSmoothingInstanceIndex_[i] = value; }
  void setSmoothingInstanceCancelFlag( size_t i, bool value ) { gsSmoothingInstanceCancelFlag_[i] = value; }
  void setSmoothingMethodType( size_t i, uint8_t value ) { gsSmoothingMethodType_[i] = value; }
  void setSmoothingGridSizeMinus2( size_t i, uint8_t value ) { gsSmoothingGridSizeMinus2_[i] = value; }
  void setSmoothingThreshold( size_t i, uint8_t value ) { gsSmoothingThreshold_[i] = value; }
  void setPbfLog2ThresholdMinus1( size_t i, uint8_t value ) { gsPbfLog2ThresholdMinus1_[i] = value; }
  void setPbfPassesCountMinus1( size_t i, uint8_t value ) { gsPbfPassesCountMinus1_[i] = value; }
  void setPbfFilterSizeMinus1( size_t i, uint8_t value ) { gsPbfFilterSizeMinus1_[i] = value; }

 private:
  bool                 gsSmoothingPersistenceFlag_;
  bool                 gsSmoothingResetFlag_;
  uint8_t              gsSmoothingInstancesUpdated_;
  std::vector<uint8_t> gsSmoothingInstanceIndex_;
  std::vector<bool>    gsSmoothingInstanceCancelFlag_;
  std::vector<uint8_t> gsSmoothingMethodType_;
  std::vector<uint8_t> gsSmoothingGridSizeMinus2_;
  std::vector<uint8_t> gsSmoothingThreshold_;
  std::vector<uint8_t> gsPbfLog2ThresholdMinus1_;
  std::vector<uint8_t> gsPbfPassesCountMinus1_;
  std::vector<uint8_t> gsPbfFilterSizeMinus1_;
};

class SEIAttributeSmoothing : public SEI {
 public:
  SEIAttributeSmoothing() :
      asSmoothingPersistenceFlag_( false ),
      asSmoothingResetFlag_( false ),
      asNumAttributesUpdated_( 0 ) {}
  ~SEIAttributeSmoothing() {
    asAttributeIdx_.clear();
    asAttributeSmoothingCancelFlag_.clear();
    asSmoothingInstancesUpdated_.clear();
    for ( auto& element : asSmoothingInstanceIndex_ ) { element.clear(); }
    for ( auto& element : asSmoothingInstanceCancelFlag_ ) { element.clear(); }
    for ( auto& element : asSmoothingMethodType_ ) { element.clear(); }
    for ( auto& element : asSmoothingGridSizeMinus2_ ) { element.clear(); }
    for ( auto& element : asSmoothingThreshold_ ) { element.clear(); }
    for ( auto& element : asSmoothingThresholdVariation_ ) { element.clear(); }
    for ( auto& element : asSmoothingThresholdDifference_ ) { element.clear(); }
    asSmoothingInstanceIndex_.clear();
    asSmoothingInstanceCancelFlag_.clear();
    asSmoothingMethodType_.clear();
    asSmoothingGridSizeMinus2_.clear();
    asSmoothingThreshold_.clear();
    asSmoothingThresholdVariation_.clear();
    asSmoothingThresholdDifference_.clear();
  }
  SEIAttributeSmoothing& operator=( const SEIAttributeSmoothing& ) = default;

  SeiPayloadType getPayloadType() { return ATTRIBUTE_SMOOTHING; }

  void allocate() {
    asAttributeIdx_.resize( asNumAttributesUpdated_, 0 );
    asAttributeSmoothingCancelFlag_.resize( asNumAttributesUpdated_, false );
    asSmoothingInstancesUpdated_.resize( asNumAttributesUpdated_, 0 );
  }
  void allocate( size_t size, size_t dimension ) {
    if ( asSmoothingInstanceIndex_.size() < size ) {
      asSmoothingInstanceIndex_.resize( size );
      asSmoothingInstanceCancelFlag_.resize( size );
      asSmoothingMethodType_.resize( size );
      asSmoothingGridSizeMinus2_.resize( size );
      asSmoothingThreshold_.resize( size );
      asSmoothingThresholdVariation_.resize( size );
      asSmoothingThresholdDifference_.resize( size );
    }
    asSmoothingInstanceIndex_[size - 1].resize( dimension );
    asSmoothingInstanceCancelFlag_[size - 1].resize( dimension );
    asSmoothingMethodType_[size - 1].resize( dimension );
    asSmoothingGridSizeMinus2_[size - 1].resize( dimension );
    asSmoothingThreshold_[size - 1].resize( dimension );
    asSmoothingThresholdVariation_[size - 1].resize( dimension );
    asSmoothingThresholdDifference_[size - 1].resize( dimension );
  }

  uint8_t getSmoothingPersistenceFlag() { return asSmoothingPersistenceFlag_; }
  uint8_t getSmoothingResetFlag() { return asSmoothingResetFlag_; }
  uint8_t getNumAttributesUpdated() { return asNumAttributesUpdated_; }
  uint8_t getAttributeIdx( size_t i ) { return asAttributeIdx_[i]; }
  uint8_t getAttributeSmoothingCancelFlag( size_t i ) { return asAttributeSmoothingCancelFlag_[i]; }
  uint8_t getSmoothingInstancesUpdated( size_t i ) { return asSmoothingInstancesUpdated_[i]; }
  uint8_t getSmoothingInstanceIndex( size_t i, size_t j ) { return asSmoothingInstanceIndex_[i][j]; }
  uint8_t getSmoothingInstanceCancelFlag( size_t i, size_t j ) { return asSmoothingInstanceCancelFlag_[i][j]; }
  uint8_t getSmoothingMethodType( size_t i, size_t j ) { return asSmoothingMethodType_[i][j]; }
  uint8_t getSmoothingGridSizeMinus2( size_t i, size_t j ) { return asSmoothingGridSizeMinus2_[i][j]; }
  uint8_t getSmoothingThreshold( size_t i, size_t j ) { return asSmoothingThreshold_[i][j]; }
  uint8_t getSmoothingThresholdVariation( size_t i, size_t j ) { return asSmoothingThresholdVariation_[i][j]; }
  uint8_t getSmoothingThresholdDifference( size_t i, size_t j ) { return asSmoothingThresholdDifference_[i][j]; }

  void setSmoothingPersistenceFlag( uint8_t value ) { asSmoothingPersistenceFlag_ = value; }
  void setSmoothingResetFlag( uint8_t value ) { asSmoothingResetFlag_ = value; }
  void setNumAttributesUpdated( uint8_t value ) { asNumAttributesUpdated_ = value; }
  void setAttributeIdx( size_t i, uint8_t value ) { asAttributeIdx_[i] = value; }
  void setAttributeSmoothingCancelFlag( size_t i, uint8_t value ) { asAttributeSmoothingCancelFlag_[i] = value; }
  void setSmoothingInstancesUpdated( size_t i, uint8_t value ) { asSmoothingInstancesUpdated_[i] = value; }
  void setSmoothingInstanceIndex( size_t i, size_t j, uint8_t value ) { asSmoothingInstanceIndex_[i][j] = value; }
  void setSmoothingInstanceCancelFlag( size_t i, size_t j, uint8_t value ) {
    asSmoothingInstanceCancelFlag_[i][j] = value;
  }
  void setSmoothingMethodType( size_t i, size_t j, uint8_t value ) { asSmoothingMethodType_[i][j] = value; }
  void setSmoothingGridSizeMinus2( size_t i, size_t j, uint8_t value ) { asSmoothingGridSizeMinus2_[i][j] = value; }
  void setSmoothingThreshold( size_t i, size_t j, uint8_t value ) { asSmoothingThreshold_[i][j] = value; }
  void setSmoothingThresholdVariation( size_t i, size_t j, uint8_t value ) {
    asSmoothingThresholdVariation_[i][j] = value;
  }
  void setSmoothingThresholdDifference( size_t i, size_t j, uint8_t value ) {
    asSmoothingThresholdDifference_[i][j] = value;
  }

 private:
  bool                              asSmoothingPersistenceFlag_;
  bool                              asSmoothingResetFlag_;
  uint8_t                           asNumAttributesUpdated_;
  std::vector<uint8_t>              asAttributeIdx_;
  std::vector<bool>                 asAttributeSmoothingCancelFlag_;
  std::vector<uint8_t>              asSmoothingInstancesUpdated_;
  std::vector<std::vector<uint8_t>> asSmoothingInstanceIndex_;
  std::vector<std::vector<uint8_t>> asSmoothingInstanceCancelFlag_;
  std::vector<std::vector<uint8_t>> asSmoothingMethodType_;
  std::vector<std::vector<uint8_t>> asSmoothingGridSizeMinus2_;
  std::vector<std::vector<uint8_t>> asSmoothingThreshold_;
  std::vector<std::vector<uint8_t>> asSmoothingThresholdVariation_;
  std::vector<std::vector<uint8_t>> asSmoothingThresholdDifference_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_SEI_H
