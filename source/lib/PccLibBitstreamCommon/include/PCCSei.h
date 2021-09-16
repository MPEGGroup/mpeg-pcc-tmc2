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
  SEI() {
    payloadSize_ = 0;
    byteStrData_.clear();
  }
  virtual ~SEI() { byteStrData_.clear(); }
  virtual SeiPayloadType getPayloadType() = 0;

  virtual std::vector<uint8_t>& getMD5ByteStrData() { return byteStrData_; }

  size_t getPayloadSize() { return payloadSize_; }
  void   setPayloadSize( size_t value ) { payloadSize_ = value; }

 protected:
  std::vector<uint8_t> byteStrData_;

 private:
  size_t payloadSize_;
};

// F.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
class SEIUserDataRegisteredItuTT35 : public SEI {
 public:
  SEIUserDataRegisteredItuTT35() : countryCode_( 0 ), countryCodeExtensionByte_( 0 ) {}

  ~SEIUserDataRegisteredItuTT35() { payloadByte_.clear(); }
  SEIUserDataRegisteredItuTT35& operator=( const SEIUserDataRegisteredItuTT35& ) = default;

  SeiPayloadType getPayloadType() { return USER_DATAREGISTERED_ITUTT35; }

  uint8_t               getCountryCode() { return countryCode_; }
  uint8_t               getCountryCodeExtensionByte() { return countryCodeExtensionByte_; }
  std::vector<uint8_t>& getPayloadByte() { return payloadByte_; }
  void                  setCountryCode( uint8_t value ) { countryCode_ = value; }
  void                  setCountryCodeExtensionByte( uint8_t value ) { countryCodeExtensionByte_ = value; }

 private:
  uint8_t              countryCode_;
  uint8_t              countryCodeExtensionByte_;
  std::vector<uint8_t> payloadByte_;
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
  uint8_t               getUuidIsoIec11578( size_t i ) { return uuidIsoIec11578_[i]; }
  std::vector<uint8_t>& getUserDataPayloadByte() { return userDataPayloadByte_; }
  uint8_t&              getUserDataPayloadByte( size_t i ) { return userDataPayloadByte_[i]; }
  void                  setUuidIsoIec11578( size_t i, uint8_t value ) { uuidIsoIec11578_[i] = value; }
  void                  setUserDataPayloadByte( size_t i, uint8_t value ) { userDataPayloadByte_[i] = value; }

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

  SeiPayloadType getPayloadType() { return NO_RECONSTRUCTION; }

 private:
};

// F.2.7  Reserved SEI message syntax
class SEIReservedSeiMessage : public SEI {
 public:
  SEIReservedSeiMessage() {}
  ~SEIReservedSeiMessage() { payloadByte_.clear(); }
  SEIReservedSeiMessage& operator=( const SEIReservedSeiMessage& ) = default;

  SeiPayloadType getPayloadType() { return RESERVED_SEI_MESSAGE; }

  std::vector<uint8_t>& getPayloadByte() { return payloadByte_; }
  uint8_t               getPayloadByte( size_t i ) { return payloadByte_[i]; }

  void setPayloadByte( size_t i, uint8_t value ) { payloadByte_[i] = value; }

 private:
  std::vector<uint8_t> payloadByte_;
};

// F.2.8  SEI manifest SEI message syntax
class SEIManifest : public SEI {
 public:
  SEIManifest() : numSeiMsgTypes_( 0 ) {}
  ~SEIManifest() {
    seiPayloadType_.clear();
    seiDescription_.clear();
  }
  SEIManifest& operator=( const SEIManifest& ) = default;

  SeiPayloadType getPayloadType() { return SEI_MANIFEST; }

  void allocate() {
    seiPayloadType_.resize( numSeiMsgTypes_, 0 );
    seiDescription_.resize( numSeiMsgTypes_, 0 );
  }
  uint16_t               getNumSeiMsgTypes() { return numSeiMsgTypes_; }
  std::vector<uint16_t>& getSeiPayloadType() { return seiPayloadType_; }
  std::vector<uint8_t>&  getSeiDescription() { return seiDescription_; }
  uint16_t               getSeiPayloadType( size_t i ) { return seiPayloadType_[i]; }
  uint8_t                getSeiDescription( size_t i ) { return seiDescription_[i]; }
  void                   setNumSeiMsgTypes( uint16_t value ) { numSeiMsgTypes_ = value; }
  void                   setSeiPayloadType( size_t i, uint16_t value ) { seiPayloadType_[i] = value; }
  void                   setSeiDescription( size_t i, uint8_t value ) { seiDescription_[i] = value; }

 private:
  uint16_t              numSeiMsgTypes_;
  std::vector<uint16_t> seiPayloadType_;
  std::vector<uint8_t>  seiDescription_;
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
  uint16_t          getNumBitsInPrefixIndicationMinus1( size_t i ) { return numBitsInPrefixIndicationMinus1_[i]; }
  std::vector<bool> getSeiPrefixDataBit( size_t i ) { return seiPrefixDataBit_[i]; }
  bool              getSeiPrefixDataBit( size_t i, size_t j ) { return seiPrefixDataBit_[i][j]; }

  void setPrefixSeiPayloadType( uint16_t value ) { prefixSeiPayloadType_ = value; }
  void setNumSeiPrefixIndicationsMinus1( uint8_t value ) { numSeiPrefixIndicationsMinus1_ = value; }
  void setNumBitsInPrefixIndicationMinus1( size_t i, uint16_t value ) { numBitsInPrefixIndicationMinus1_[i] = value; }
  void setSeiPrefixDataBit( size_t i, size_t j, bool value ) { seiPrefixDataBit_[i][j] = value; }

 private:
  uint16_t                       prefixSeiPayloadType_;
  uint8_t                        numSeiPrefixIndicationsMinus1_;
  std::vector<uint16_t>          numBitsInPrefixIndicationMinus1_;
  std::vector<std::vector<bool>> seiPrefixDataBit_;
};

// F.2.10  Attribute transformation parameters SEI message syntax
class SEIAttributeTransformationParams : public SEI {
 public:
  SEIAttributeTransformationParams() : cancelFlag_( false ), numAttributeUpdates_( 0 ), persistenceFlag_( false ) {}

  ~SEIAttributeTransformationParams() {
    attributeIdx_.clear();
    dimensionMinus1_.clear();
    for ( auto& element : scaleParamsEnabledFlag_ ) { element.clear(); }
    for ( auto& element : offsetParamsEnabledFlag_ ) { element.clear(); }
    for ( auto& element : attributeScale_ ) { element.clear(); }
    for ( auto& element : attributeOffset_ ) { element.clear(); }
    scaleParamsEnabledFlag_.clear();
    offsetParamsEnabledFlag_.clear();
    attributeScale_.clear();
    attributeOffset_.clear();
  }
  SEIAttributeTransformationParams& operator=( const SEIAttributeTransformationParams& ) = default;

  SeiPayloadType getPayloadType() { return ATTRIBUTE_TRANSFORMATION_PARAMS; }
  void           allocate() {
    attributeIdx_.resize( numAttributeUpdates_ + 1, 0 );
    dimensionMinus1_.resize( 256 + 1, 0 );
    scaleParamsEnabledFlag_.resize( 256 );
    offsetParamsEnabledFlag_.resize( 256 );
    attributeScale_.resize( 256 );
    attributeOffset_.resize( 256 );
  }
  void allocate( size_t i ) {
    scaleParamsEnabledFlag_[i].resize( dimensionMinus1_[i] + 1, false );
    offsetParamsEnabledFlag_[i].resize( dimensionMinus1_[i] + 1, false );
    attributeScale_[i].resize( dimensionMinus1_[i] + 1, 0 );
    attributeOffset_[i].resize( dimensionMinus1_[i] + 1, 0 );
  }
  bool                                getCancelFlag() { return cancelFlag_; }
  int32_t                             getNumAttributeUpdates() { return numAttributeUpdates_; }
  std::vector<uint8_t>&               AttributeIdx() { return attributeIdx_; }
  std::vector<uint8_t>&               getDimensionMinus() { return dimensionMinus1_; }
  std::vector<std::vector<bool>>&     getScaleParamsEnabledFlag() { return scaleParamsEnabledFlag_; }
  std::vector<std::vector<bool>>&     getOffsetParamsEnabledFlag() { return offsetParamsEnabledFlag_; }
  std::vector<std::vector<uint32_t>>& getAttributeScale() { return attributeScale_; }
  std::vector<std::vector<int32_t>>&  getAttributeOffset() { return attributeOffset_; }
  bool                                getPersistenceFlag() { return persistenceFlag_; }
  uint8_t                             getAttributeIdx( size_t i ) { return attributeIdx_[i]; }
  uint8_t                             getDimensionMinus1( size_t i ) { return dimensionMinus1_[i]; }
  bool     getScaleParamsEnabledFlag( size_t i, size_t j ) { return scaleParamsEnabledFlag_[i][j]; }
  bool     getOffsetParamsEnabledFlag( size_t i, size_t j ) { return offsetParamsEnabledFlag_[i][j]; }
  uint32_t getAttributeScale( size_t i, size_t j ) { return attributeScale_[i][j]; }
  int32_t  getAttributeOffset( size_t i, size_t j ) { return attributeOffset_[i][j]; }

  void setCancelFlag( bool value ) { cancelFlag_ = value; }
  void setNumAttributeUpdates( int32_t value ) { numAttributeUpdates_ = value; }
  void setAttributeIdx( size_t i, uint8_t value ) { attributeIdx_[i] = value; }
  void setDimensionMinus1( size_t i, uint8_t value ) { dimensionMinus1_[i] = value; }
  void setScaleParamsEnabledFlag( size_t i, size_t j, bool value ) { scaleParamsEnabledFlag_[i][j] = value; }
  void setOffsetParamsEnabledFlag( size_t i, size_t j, bool value ) { offsetParamsEnabledFlag_[i][j] = value; }
  void setAttributeScale( size_t i, size_t j, uint32_t value ) { attributeScale_[i][j] = value; }
  void setAttributeOffset( size_t i, size_t j, int32_t value ) { attributeOffset_[i][j] = value; }
  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }

 private:
  bool                               cancelFlag_;
  int32_t                            numAttributeUpdates_;
  std::vector<uint8_t>               attributeIdx_;
  std::vector<uint8_t>               dimensionMinus1_;
  std::vector<std::vector<bool>>     scaleParamsEnabledFlag_;
  std::vector<std::vector<bool>>     offsetParamsEnabledFlag_;
  std::vector<std::vector<uint32_t>> attributeScale_;
  std::vector<std::vector<int32_t>>  attributeOffset_;
  bool                               persistenceFlag_;
};

// F.2.10  Active substreams SEI message syntax
class SEIActiveSubBitstreams : public SEI {
 public:
  SEIActiveSubBitstreams() :
      activeSubBitstreamsCancelFlag_( 0 ),
      activeAttributesChangesFlag_( 0 ),
      activeMapsChangesFlag_( 0 ),
      auxiliarySubstreamsActiveFlag_( 0 ),
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
  bool                  getActiveSubBitstreamsCancelFlag() { return activeSubBitstreamsCancelFlag_; }
  bool                  getActiveAttributesChangesFlag() { return activeAttributesChangesFlag_; }
  bool                  getActiveMapsChangesFlag() { return activeMapsChangesFlag_; }
  bool                  getAuxiliarySubstreamsActiveFlag() { return auxiliarySubstreamsActiveFlag_; }
  bool                  getAllAttributesActiveFlag() { return allAttributesActiveFlag_; }
  bool                  getAllMapsActiveFlag() { return allMapsActiveFlag_; }
  uint8_t               getActiveAttributeCountMinus1() { return activeAttributeCountMinus1_; }
  uint8_t               getActiveMapCountMinus1() { return activeMapCountMinus1_; }
  std::vector<uint8_t>& getActiveAttributeIdx() { return activeAttributeIdx_; }
  std::vector<uint8_t>& getActiveMapIdx() { return activeMapIdx_; }
  uint8_t               getActiveAttributeIdx( size_t i ) { return activeAttributeIdx_[i]; }
  uint8_t               getActiveMapIdx( size_t i ) { return activeMapIdx_[i]; }

  void setActiveSubBitstreamsCancelFlag( bool value ) { activeSubBitstreamsCancelFlag_ = value; }
  void setActiveAttributesChangesFlag( bool value ) { activeAttributesChangesFlag_ = value; }
  void setActiveMapsChangesFlag( bool value ) { activeMapsChangesFlag_ = value; }
  void setAuxiliarySubstreamsActiveFlag( bool value ) { auxiliarySubstreamsActiveFlag_ = value; }
  void setAllAttributesActiveFlag( bool value ) { allAttributesActiveFlag_ = value; }
  void setAllMapsActiveFlag( bool value ) { allMapsActiveFlag_ = value; }
  void setActiveAttributeCountMinus1( uint8_t value ) { activeAttributeCountMinus1_ = value; }
  void setActiveMapCountMinus1( uint8_t value ) { activeMapCountMinus1_ = value; }
  void setActiveAttributeIdx( size_t i, uint8_t value ) { activeAttributeIdx_[i] = value; }
  void setActiveMapIdx( size_t i, uint8_t value ) { activeMapIdx_[i] = value; }

 private:
  bool                 activeSubBitstreamsCancelFlag_;
  bool                 activeAttributesChangesFlag_;
  bool                 activeMapsChangesFlag_;
  bool                 auxiliarySubstreamsActiveFlag_;
  bool                 allAttributesActiveFlag_;
  bool                 allMapsActiveFlag_;
  uint8_t              activeAttributeCountMinus1_;
  uint8_t              activeMapCountMinus1_;
  std::vector<uint8_t> activeAttributeIdx_;
  std::vector<uint8_t> activeMapIdx_;
};

// F.2.11  Component codec mapping SEI message syntax
class SEIComponentCodecMapping : public SEI {
 public:
  SEIComponentCodecMapping() : componentCodecCancelFlag_( 0 ), codecMappingsCountMinus1_( 0 ) {}
  ~SEIComponentCodecMapping() {
    codecId_.clear();
    codec4cc_.clear();
  }
  SEIComponentCodecMapping& operator=( const SEIComponentCodecMapping& ) = default;

  SeiPayloadType getPayloadType() { return COMPONENT_CODEC_MAPPING; }

  void allocate() {
    codecId_.resize( codecMappingsCountMinus1_ + 1, 0 );
    codec4cc_.resize( 256 );
  }
  uint8_t     getComponentCodecCancelFlag() { return componentCodecCancelFlag_; }
  uint8_t     getCodecMappingsCountMinus1() { return codecMappingsCountMinus1_; }
  uint8_t     getCodecId( size_t i ) { return codecId_[i]; }
  std::string getCodec4cc( size_t i ) { return codec4cc_[i]; }

  void setComponentCodecCancelFlag( uint8_t value ) { componentCodecCancelFlag_ = value; }
  void setCodecMappingsCountMinus1( uint8_t value ) { codecMappingsCountMinus1_ = value; }
  void setCodecId( size_t i, uint8_t value ) { codecId_[i] = value; }
  void setCodec4cc( size_t i, std::string value ) { codec4cc_[i] = value; }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = componentCodecCancelFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = codecMappingsCountMinus1_ & 0xFF;
    byteStrData_.push_back( val );
    for ( int i = 0; i < codecId_.size(); i++ ) {
      val = codecId_[i] & 0xFF;
      byteStrData_.push_back( val );
    }
    for ( int i = 0; i < codecId_.size(); i++ ) {
      char const* strByte = codec4cc_[i].c_str();
      for ( int j = 0; j < codec4cc_[i].length(); j++ ) {
        val = strByte[j] & 0xFF;
        byteStrData_.push_back( val );
      }
    }
    return byteStrData_;
  }

 private:
  uint8_t                  componentCodecCancelFlag_;
  uint8_t                  codecMappingsCountMinus1_;
  std::vector<uint8_t>     codecId_;
  std::vector<std::string> codec4cc_;
};

// F.2.12.1	Scene object information SEI message syntax
class SEISceneObjectInformation : public SEI {
 public:
  SEISceneObjectInformation() : persistenceFlag_( true ), resetFlag_( true ) {}
  ~SEISceneObjectInformation() {}
  SEISceneObjectInformation& operator=( const SEISceneObjectInformation& ) = default;
  void                       allocateObjectIdx() { objectIdx_.resize( numObjectUpdates_ ); }
  void                       allocateObjectNumDependencies( size_t i, size_t objectNumDependencies ) {
    objectDependencyIdx_[i].resize( objectNumDependencies );
  }
  void allocate( size_t size ) {
    if ( objectCancelFlag_.size() < size ) {
      objectCancelFlag_.resize( size );
      objectLabelUpdateFlag_.resize( size );
      objectLabelIdx_.resize( size );
      priorityUpdateFlag_.resize( size );
      priorityValue_.resize( size );
      objectHiddenFlag_.resize( size );
      objectDependencyUpdateFlag_.resize( size );
      objectNumDependencies_.resize( size );
      visibilityConesUpdateFlag_.resize( size );
      directionX_.resize( size );
      directionY_.resize( size );
      directionZ_.resize( size );
      angle_.resize( size );
      boundingBoxUpdateFlag_.resize( size );
      boundingBoxX_.resize( size );
      boundingBoxY_.resize( size );
      boundingBoxZ_.resize( size );
      boundingBoxDeltaX_.resize( size );
      boundingBoxDeltaY_.resize( size );
      boundingBoxDeltaZ_.resize( size );
      collisionShapeUpdateFlag_.resize( size );
      collisionShapeId_.resize( size );
      pointStyleUpdateFlag_.resize( size );
      pointShapeId_.resize( size );
      pointSize_.resize( size );
      materialIdUpdateFlag_.resize( size );
      materialId_.resize( size );
      objectDependencyIdx_.resize( size );  // 16
    }
  }

  SeiPayloadType         getPayloadType() { return SCENE_OBJECT_INFORMATION; }
  bool                   getPersistenceFlag() { return persistenceFlag_; }
  bool                   getResetFlag() { return resetFlag_; }
  uint32_t               getNumObjectUpdates() { return numObjectUpdates_; }
  bool                   getSimpleObjectsFlag() { return soiSimpleObjectsFlag_; }
  bool                   getObjectLabelPresentFlag() { return objectLabelPresentFlag_; }
  bool                   getPriorityPresentFlag() { return priorityPresentFlag_; }
  bool                   getObjectHiddenPresentFlag() { return objectHiddenPresentFlag_; }
  bool                   getObjectDependencyPresentFlag() { return objectDependencyPresentFlag_; }
  bool                   getVisibilityConesPresentFlag() { return visibilityConesPresentFlag_; }
  bool                   get3dBoundingBoxPresentFlag() { return boundingBoxPresentFlag_; }
  bool                   getCollisionShapePresentFlag() { return collisionShapePresentFlag_; }
  bool                   getPointStylePresentFlag() { return pointStylePresentFlag_; }
  bool                   getMaterialIdPresentFlag() { return materialIdPresentFlag_; }
  bool                   getExtensionPresentFlag() { return extensionPresentFlag_; }
  uint8_t                get3dBoundingBoxScaleLog2() { return boundingBoxScaleLog2_; }
  uint8_t                get3dBoundingBoxPrecisionMinus8() { return boundingBoxPrecisionMinus8_; }
  uint8_t                getLog2MaxObjectIdxUpdated() { return log2MaxObjectIdxUpdated_; }
  uint8_t                getLog2MaxObjectDependencyIdx() { return log2MaxObjectDependencyIdx_; }
  std::vector<uint32_t>& getObjectIdx() { return objectIdx_; }
  uint32_t               getObjectIdx( size_t i ) { return objectIdx_[i]; }
  bool                   getObjectCancelFlag( size_t i ) { return objectCancelFlag_[i]; }
  bool                   getObjectLabelUpdateFlag( size_t i ) { return objectLabelUpdateFlag_[i]; }
  uint32_t               getObjectLabelIdx( size_t i ) { return objectLabelIdx_[i]; }
  bool                   getPriorityUpdateFlag( size_t i ) { return priorityUpdateFlag_[i]; }
  uint8_t                getPriorityValue( size_t i ) { return priorityValue_[i]; }
  bool                   getObjectHiddenFlag( size_t i ) { return objectHiddenFlag_[i]; }
  bool                   getObjectDependencyUpdateFlag( size_t i ) { return objectDependencyUpdateFlag_[i]; }
  uint8_t                getObjectNumDependencies( size_t i ) { return objectNumDependencies_[i]; }
  uint32_t               getObjectDependencyIdx( size_t i, size_t objnum ) { return objectDependencyIdx_[i][objnum]; }
  bool                   getVisibilityConesUpdateFlag( size_t i ) { return visibilityConesUpdateFlag_[i]; }
  uint32_t               getDirectionX( size_t i ) { return directionX_[i]; }
  uint32_t               getDirectionY( size_t i ) { return directionY_[i]; }
  uint32_t               getDirectionZ( size_t i ) { return directionZ_[i]; }
  uint16_t               getAngle( size_t i ) { return angle_[i]; }
  bool                   get3dBoundingBoxUpdateFlag( size_t i ) { return boundingBoxUpdateFlag_[i]; }
  uint32_t               get3dBoundingBoxX( size_t i ) { return boundingBoxX_[i]; }
  uint32_t               get3dBoundingBoxY( size_t i ) { return boundingBoxY_[i]; }
  uint32_t               get3dBoundingBoxZ( size_t i ) { return boundingBoxZ_[i]; }
  uint32_t               get3dBoundingBoxDeltaX( size_t i ) { return boundingBoxDeltaX_[i]; }
  uint32_t               get3dBoundingBoxDeltaY( size_t i ) { return boundingBoxDeltaY_[i]; }
  uint32_t               get3dBoundingBoxDeltaZ( size_t i ) { return boundingBoxDeltaZ_[i]; }
  bool                   getCollisionShapeUpdateFlag( size_t i ) { return collisionShapeUpdateFlag_[i]; }
  uint16_t               getCollisionShapeId( size_t i ) { return collisionShapeId_[i]; }
  bool                   getPointStyleUpdateFlag( size_t i ) { return pointStyleUpdateFlag_[i]; }
  uint8_t                getPointShapeId( size_t i ) { return pointShapeId_[i]; }
  uint16_t               getPointSize( size_t i ) { return pointSize_[i]; }
  bool                   getMaterialIdUpdateFlag( size_t i ) { return materialIdUpdateFlag_[i]; }
  uint16_t               getMaterialId( size_t i ) { return materialId_[i]; }

  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }
  void setResetFlag( bool value ) { resetFlag_ = value; }
  void setNumObjectUpdates( uint32_t value ) { numObjectUpdates_ = value; }
  void setSimpleObjectsFlag( bool value ) { soiSimpleObjectsFlag_ = value; }
  void setObjectLabelPresentFlag( bool value ) { objectLabelPresentFlag_ = value; }
  void setPriorityPresentFlag( bool value ) { priorityPresentFlag_ = value; }
  void setObjectHiddenPresentFlag( bool value ) { objectHiddenPresentFlag_ = value; }
  void setObjectDependencyPresentFlag( bool value ) { objectDependencyPresentFlag_ = value; }
  void setVisibilityConesPresentFlag( bool value ) { visibilityConesPresentFlag_ = value; }
  void set3dBoundingBoxPresentFlag( bool value ) { boundingBoxPresentFlag_ = value; }
  void setCollisionShapePresentFlag( bool value ) { collisionShapePresentFlag_ = value; }
  void setPointStylePresentFlag( bool value ) { pointStylePresentFlag_ = value; }
  void setMaterialIdPresentFlag( bool value ) { materialIdPresentFlag_ = value; }
  void setExtensionPresentFlag( bool value ) { extensionPresentFlag_ = value; }
  void set3dBoundingBoxScaleLog2( uint8_t value ) { boundingBoxScaleLog2_ = value; }
  void set3dBoundingBoxPrecisionMinus8( uint8_t value ) { boundingBoxPrecisionMinus8_ = value; }
  void setLog2MaxObjectIdxUpdated( uint8_t value ) { log2MaxObjectIdxUpdated_ = value; }
  void setLog2MaxObjectDependencyIdx( uint8_t value ) { log2MaxObjectDependencyIdx_ = value; }
  void setObjectIdx( size_t i, uint32_t value ) { objectIdx_[i] = value; }
  void setObjectCancelFlag( size_t i, bool value ) { objectCancelFlag_[i] = value; }
  void setObjectLabelUpdateFlag( size_t i, bool value ) { objectLabelUpdateFlag_[i] = value; }
  void setObjectLabelIdx( size_t i, uint32_t value ) { objectLabelIdx_[i] = value; }
  void setPriorityUpdateFlag( size_t i, bool value ) { priorityUpdateFlag_[i] = value; }
  void setPriorityValue( size_t i, uint8_t value ) { priorityValue_[i] = value; }
  void setObjectHiddenFlag( size_t i, bool value ) { objectHiddenFlag_[i] = value; }
  void setObjectDependencyUpdateFlag( size_t i, bool value ) { objectDependencyUpdateFlag_[i] = value; }
  void setObjectNumDependencies( size_t i, uint8_t value ) { objectNumDependencies_[i] = value; }
  void setObjectDependencyIdx( size_t i, size_t objnum, uint32_t value ) { objectDependencyIdx_[i][objnum] = value; }
  void setVisibilityConesUpdateFlag( size_t i, bool value ) { visibilityConesUpdateFlag_[i] = value; }
  void setDirectionX( size_t i, uint32_t value ) { directionX_[i] = value; }
  void setDirectionY( size_t i, uint32_t value ) { directionY_[i] = value; }
  void setDirectionZ( size_t i, uint32_t value ) { directionZ_[i] = value; }
  void setAngle( size_t i, uint16_t value ) { angle_[i] = value; }
  void set3dBoundingBoxUpdateFlag( size_t i, bool value ) { boundingBoxUpdateFlag_[i] = value; }
  void set3dBoundingBoxX( size_t i, uint32_t value ) { boundingBoxX_[i] = value; }
  void set3dBoundingBoxY( size_t i, uint32_t value ) { boundingBoxY_[i] = value; }
  void set3dBoundingBoxZ( size_t i, uint32_t value ) { boundingBoxZ_[i] = value; }
  void set3dBoundingBoxDeltaX( size_t i, uint32_t value ) { boundingBoxDeltaX_[i] = value; }
  void set3dBoundingBoxDeltaY( size_t i, uint32_t value ) { boundingBoxDeltaY_[i] = value; }
  void set3dBoundingBoxDeltaZ( size_t i, uint32_t value ) { boundingBoxDeltaZ_[i] = value; }
  void setCollisionShapeUpdateFlag( size_t i, bool value ) { collisionShapeUpdateFlag_[i] = value; }
  void setCollisionShapeId( size_t i, uint16_t value ) { collisionShapeId_[i] = value; }
  void setPointStyleUpdateFlag( size_t i, bool value ) { pointStyleUpdateFlag_[i] = value; }
  void setPointShapeId( size_t i, uint8_t value ) { pointShapeId_[i] = value; }
  void setPointSize( size_t i, uint16_t value ) { pointSize_[i] = value; }
  void setMaterialIdUpdateFlag( size_t i, bool value ) { materialIdUpdateFlag_[i] = value; }
  void setMaterialId( size_t i, uint16_t value ) { materialId_[i] = value; }

 private:
  bool                               persistenceFlag_;
  bool                               resetFlag_;
  uint32_t                           numObjectUpdates_;
  bool                               soiSimpleObjectsFlag_;
  bool                               objectLabelPresentFlag_;
  bool                               priorityPresentFlag_;
  bool                               objectHiddenPresentFlag_;
  bool                               objectDependencyPresentFlag_;
  bool                               visibilityConesPresentFlag_;
  bool                               boundingBoxPresentFlag_;
  bool                               collisionShapePresentFlag_;
  bool                               pointStylePresentFlag_;
  bool                               materialIdPresentFlag_;
  bool                               extensionPresentFlag_;
  uint8_t                            boundingBoxScaleLog2_;
  uint8_t                            boundingBoxPrecisionMinus8_;
  uint8_t                            log2MaxObjectIdxUpdated_;
  uint8_t                            log2MaxObjectDependencyIdx_;
  std::vector<uint32_t>              objectIdx_;
  std::vector<bool>                  objectCancelFlag_;
  std::vector<bool>                  objectLabelUpdateFlag_;
  std::vector<uint32_t>              objectLabelIdx_;
  std::vector<bool>                  priorityUpdateFlag_;
  std::vector<uint8_t>               priorityValue_;
  std::vector<bool>                  objectHiddenFlag_;
  std::vector<bool>                  objectDependencyUpdateFlag_;
  std::vector<uint8_t>               objectNumDependencies_;
  std::vector<std::vector<uint32_t>> objectDependencyIdx_;  // 16
  std::vector<bool>                  visibilityConesUpdateFlag_;
  std::vector<uint32_t>              directionX_;
  std::vector<uint32_t>              directionY_;
  std::vector<uint32_t>              directionZ_;
  std::vector<uint16_t>              angle_;
  std::vector<bool>                  boundingBoxUpdateFlag_;
  std::vector<uint32_t>              boundingBoxX_;
  std::vector<uint32_t>              boundingBoxY_;
  std::vector<uint32_t>              boundingBoxZ_;
  std::vector<uint32_t>              boundingBoxDeltaX_;
  std::vector<uint32_t>              boundingBoxDeltaY_;
  std::vector<uint32_t>              boundingBoxDeltaZ_;
  std::vector<bool>                  collisionShapeUpdateFlag_;
  std::vector<uint16_t>              collisionShapeId_;
  std::vector<bool>                  pointStyleUpdateFlag_;
  std::vector<uint8_t>               pointShapeId_;
  std::vector<uint16_t>              pointSize_;
  std::vector<bool>                  materialIdUpdateFlag_;
  std::vector<uint16_t>              materialId_;
};

// F.2.12.2 Object label information SEI message syntax
class SEIObjectLabelInformation : public SEI {
 public:
  SEIObjectLabelInformation() : cancelFlag_( false ) {}
  ~SEIObjectLabelInformation() {}
  SEIObjectLabelInformation& operator=( const SEIObjectLabelInformation& ) = default;

  SeiPayloadType getPayloadType() { return OBJECT_LABEL_INFORMATION; }
  void           allocate() { labelIdx_.resize( numLabelUpdates_, 0 ); }
  bool           getCancelFlag() { return cancelFlag_; }
  bool           getLabelLanguagePresentFlag() { return labelLanguagePresentFlag_; }
  std::string&   getLabelLanguage() { return labelLanguage_; }
  uint32_t       getNumLabelUpdates() { return numLabelUpdates_; }
  uint32_t       getLabelIdx( size_t i ) { return labelIdx_[i]; }
  bool           getLabelCancelFlag() { return labelCancelFlag_; }
  bool           getBitEqualToZero() { return bitEqualToZero_; }
  std::string&   getLabel( size_t i ) { return label_[i]; }
  bool           getPersistenceFlag() { return persistenceFlag_; }

  void setCancelFlag( bool value ) { cancelFlag_ = value; }
  void setLabelLanguagePresentFlag( bool value ) { labelLanguagePresentFlag_ = value; }
  void setLabelLanguage( std::string value ) { labelLanguage_ = value; }
  void setNumLabelUpdates( uint32_t value ) { numLabelUpdates_ = value; }
  void setLabelIdx( size_t i, uint32_t value ) { labelIdx_[i] = value; }
  void setLabelCancelFlag( bool value ) { labelCancelFlag_ = value; }
  void setBitEqualToZero( bool value ) { bitEqualToZero_ = value; }
  void setLabel( size_t i, std::string value ) {
    if ( label_.size() < i ) { label_.resize( i + 1 ); }
    label_[i] = value;
  }
  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }

 private:
  bool                     cancelFlag_;
  bool                     labelLanguagePresentFlag_;
  std::string              labelLanguage_;  // st(v)
  uint32_t                 numLabelUpdates_;
  std::vector<uint32_t>    labelIdx_;
  bool                     labelCancelFlag_;
  bool                     bitEqualToZero_;
  std::vector<std::string> label_;  // st(v)
  bool                     persistenceFlag_;
};

// F.2.12.3 Patch information SEI message syntax
class SEIPatchInformation : public SEI {
 public:
  SEIPatchInformation() {}
  ~SEIPatchInformation() {}
  SEIPatchInformation& operator=( const SEIPatchInformation& ) = default;
  SeiPayloadType       getPayloadType() { return PATCH_INFORMATION; }

  void     allocate() {}  // TODO: allocate data
  bool     getPersistenceFlag() { return persistenceFlag_; }
  bool     getResetFlag() { return resetFlag_; }
  uint32_t getNumTileUpdates() { return numTileUpdates_; }
  uint8_t  getLog2MaxObjectIdxTracked() { return log2MaxObjectIdxTracked_; }
  uint8_t  getLog2MaxPatchIdxUpdated() { return log2MaxPatchIdxUpdated_; }
  uint32_t getTileId( size_t i ) { return tileId_[i]; }
  bool     getTileCancelFlag( size_t i ) { return tileCancelFlag_[i]; }
  uint32_t getNumPatchUpdates( size_t i ) { return numPatchUpdates_[i]; }
  uint32_t getPatchIdx( size_t i, size_t j ) { return patchIdx_[i][j]; }
  bool     getPatchCancelFlag( size_t i, size_t j ) { return patchCancelFlag_[i][j]; }
  uint32_t getPatchNumberOfObjectsMinus1( size_t i, size_t j ) { return patchNumberOfObjectsMinus1_[i][j]; }
  uint32_t getPatchObjectIdx( size_t i, size_t j, size_t k ) { return patchObjectIdx_[i][j][k]; }

  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }
  void setResetFlag( bool value ) { resetFlag_ = value; }
  void setNumTileUpdates( uint32_t value ) { numTileUpdates_ = value; }
  void setLog2MaxObjectIdxTracked( uint8_t value ) { log2MaxObjectIdxTracked_ = value; }
  void setLog2MaxPatchIdxUpdated( uint8_t value ) { log2MaxPatchIdxUpdated_ = value; }
  void setTileId( size_t i, uint32_t value ) { tileId_[i] = value; }
  void setTileCancelFlag( size_t i, bool value ) { tileCancelFlag_[i] = value; }
  void setNumPatchUpdates( size_t i, uint32_t value ) { numPatchUpdates_[i] = value; }
  void setPatchIdx( size_t i, size_t j, uint32_t value ) { patchIdx_[i][j] = value; }
  void setPatchCancelFlag( size_t i, size_t j, bool value ) { patchCancelFlag_[i][j] = value; }
  void setPatchNumberOfObjectsMinus1( size_t i, size_t j, uint32_t value ) {
    patchNumberOfObjectsMinus1_[i][j] = value;
  }
  void setPatchObjectIdx( size_t i, size_t j, size_t index3, uint32_t value ) { patchObjectIdx_[i][j][index3] = value; }

 private:
  bool                                            persistenceFlag_;
  bool                                            resetFlag_;
  uint32_t                                        numTileUpdates_;
  uint8_t                                         log2MaxObjectIdxTracked_;
  uint8_t                                         log2MaxPatchIdxUpdated_;
  std::vector<uint32_t>                           tileId_;
  std::vector<bool>                               tileCancelFlag_;
  std::vector<uint32_t>                           numPatchUpdates_;
  std::vector<std::vector<uint32_t>>              patchIdx_;
  std::vector<std::vector<bool>>                  patchCancelFlag_;
  std::vector<std::vector<uint32_t>>              patchNumberOfObjectsMinus1_;
  std::vector<std::vector<std::vector<uint32_t>>> patchObjectIdx_;
};

// F.2.12.4 Volumetric rectangle information SEI message syntax
class SEIVolumetricRectangleInformation : public SEI {
 public:
  SEIVolumetricRectangleInformation() {}
  ~SEIVolumetricRectangleInformation() {}
  SEIVolumetricRectangleInformation& operator=( const SEIVolumetricRectangleInformation& ) = default;
  void                               allocate( size_t size ) {
    boundingBoxUpdateFlag_.resize( size );
    boundingBoxTop_.resize( size );
    boundingBoxLeft_.resize( size );
    boundingBoxWidth_.resize( size );
    boundingBoxHeight_.resize( size );
    rectangleObjectIdx_.resize( size );
  }
  void allocateRectangleObjectIdx( size_t i, size_t size ) { rectangleObjectIdx_[i].resize( size ); }

  SeiPayloadType getPayloadType() { return VOLUMETRIC_RECTANGLE_INFORMATION; }
  bool           getPersistenceFlag() { return persistenceFlag_; }
  bool           getResetFlag() { return resetFlag_; }
  uint32_t       getNumRectanglesUpdates() { return numRectanglesUpdates_; }
  uint8_t        getLog2MaxObjectIdxTracked() { return log2MaxObjectIdxTracked_; }
  uint8_t        getLog2MaxRectangleIdxUpdated() { return log2MaxRectangleIdxUpdated_; }
  uint32_t       getRectangleIdx( size_t i ) { return rectangleIdx_[i]; }
  bool           getRectangleCancelFlag( size_t i ) { return rectangleCancelFlag_[i]; }
  bool           getBoundingBoxUpdateFlag( size_t i ) { return boundingBoxUpdateFlag_[i]; }
  uint32_t       getBoundingBoxTop( size_t i ) { return boundingBoxTop_[i]; }
  uint32_t       getBoundingBoxLeft( size_t i ) { return boundingBoxLeft_[i]; }
  uint32_t       getBoundingBoxWidth( size_t i ) { return boundingBoxWidth_[i]; }
  uint32_t       getBoundingBoxHeight( size_t i ) { return boundingBoxHeight_[i]; }
  uint32_t       getRectangleNumberOfObjectsMinus1( size_t i ) { return rectangleNumberOfObjectsMinus1_[i]; }
  uint32_t       getRectangleObjectIdx( size_t i, size_t j ) { return rectangleObjectIdx_[i][j]; }

  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }
  void setResetFlag( bool value ) { resetFlag_ = value; }
  void setNumRectanglesUpdates( uint32_t value ) { numRectanglesUpdates_ = value; }
  void setLog2MaxObjectIdxTracked( uint8_t value ) { log2MaxObjectIdxTracked_ = value; }
  void setLog2MaxRectangleIdxUpdated( uint8_t value ) { log2MaxRectangleIdxUpdated_ = value; }
  void setRectangleIdx( size_t i, uint32_t value ) { rectangleIdx_[i] = value; }
  void setRectangleCancelFlag( size_t i, bool value ) { rectangleCancelFlag_[i] = value; }
  void setBoundingBoxUpdateFlag( size_t i, bool value ) { boundingBoxUpdateFlag_[i] = value; }
  void setBoundingBoxTop( size_t i, uint32_t value ) { boundingBoxTop_[i] = value; }
  void setBoundingBoxLeft( size_t i, uint32_t value ) { boundingBoxLeft_[i] = value; }
  void setBoundingBoxWidth( size_t i, uint32_t value ) { boundingBoxWidth_[i] = value; }
  void setBoundingBoxHeight( size_t i, uint32_t value ) { boundingBoxHeight_[i] = value; }
  void setRectangleNumberOfObjectsMinus1( size_t i, uint32_t value ) { rectangleNumberOfObjectsMinus1_[i] = value; }
  void setRectangleObjectIdx( size_t i, size_t j, uint32_t value ) { rectangleObjectIdx_[i][j] = value; }

 private:
  bool                               persistenceFlag_;
  bool                               resetFlag_;
  uint32_t                           numRectanglesUpdates_;
  uint8_t                            log2MaxObjectIdxTracked_;
  uint8_t                            log2MaxRectangleIdxUpdated_;
  std::vector<uint32_t>              rectangleIdx_;
  std::vector<bool>                  rectangleCancelFlag_;
  std::vector<bool>                  boundingBoxUpdateFlag_;
  std::vector<uint32_t>              boundingBoxTop_;
  std::vector<uint32_t>              boundingBoxLeft_;
  std::vector<uint32_t>              boundingBoxWidth_;
  std::vector<uint32_t>              boundingBoxHeight_;
  std::vector<uint32_t>              rectangleNumberOfObjectsMinus1_;
  std::vector<std::vector<uint32_t>> rectangleObjectIdx_;
};

// F.2.13  Buffering period SEI message syntax
class SEIBufferingPeriod : public SEI {
 public:
  SEIBufferingPeriod() :
      irapCabParamsPresentFlag_( false ),
      concatenationFlag_( false ),
      atlasSequenceParameterSetId_( 0 ),
      cabDelayOffset_( 0 ),
      dabDelayOffset_( 0 ),
      atlasCabRemovalDelayDeltaMinus1_( 0 ),
      maxSubLayersMinus1_( 0 ) {}
  ~SEIBufferingPeriod() {
    for ( auto& element : nalInitialCabRemovalDelay_ ) { element.clear(); }
    for ( auto& element : nalInitialCabRemovalOffset_ ) { element.clear(); }
    for ( auto& element : aclInitialCabRemovalDelay_ ) { element.clear(); }
    for ( auto& element : aclInitialCabRemovalOffset_ ) { element.clear(); }
    for ( auto& element : nalInitialAltCabRemovalDelay_ ) { element.clear(); }
    for ( auto& element : nalInitialAltCabRemovalOffset_ ) { element.clear(); }
    for ( auto& element : aclInitialAltCabRemovalDelay_ ) { element.clear(); }
    for ( auto& element : aclInitialAltCabRemovalOffset_ ) { element.clear(); }
    hrdCabCntMinus1_.clear();
    nalInitialCabRemovalDelay_.clear();
    nalInitialCabRemovalOffset_.clear();
    aclInitialCabRemovalDelay_.clear();
    aclInitialCabRemovalOffset_.clear();
    nalInitialAltCabRemovalDelay_.clear();
    nalInitialAltCabRemovalOffset_.clear();
    aclInitialAltCabRemovalDelay_.clear();
    aclInitialAltCabRemovalOffset_.clear();
  }
  SEIBufferingPeriod& operator=( const SEIBufferingPeriod& ) = default;

  SeiPayloadType getPayloadType() { return BUFFERING_PERIOD; }
  void           allocate() {
    hrdCabCntMinus1_.resize( maxSubLayersMinus1_ + 1 );
    nalInitialCabRemovalDelay_.resize( maxSubLayersMinus1_ + 1 );
    nalInitialCabRemovalOffset_.resize( maxSubLayersMinus1_ + 1 );
    aclInitialCabRemovalDelay_.resize( maxSubLayersMinus1_ + 1 );
    aclInitialCabRemovalOffset_.resize( maxSubLayersMinus1_ + 1 );
    nalInitialAltCabRemovalDelay_.resize( maxSubLayersMinus1_ + 1 );
    nalInitialAltCabRemovalOffset_.resize( maxSubLayersMinus1_ + 1 );
    aclInitialAltCabRemovalDelay_.resize( maxSubLayersMinus1_ + 1 );
    aclInitialAltCabRemovalOffset_.resize( maxSubLayersMinus1_ + 1 );
  }
  void allocate( size_t i ) {
    nalInitialCabRemovalDelay_[i].resize( hrdCabCntMinus1_[i] + 1 );
    nalInitialCabRemovalOffset_[i].resize( hrdCabCntMinus1_[i] + 1 );
    aclInitialCabRemovalDelay_[i].resize( hrdCabCntMinus1_[i] + 1 );
    aclInitialCabRemovalOffset_[i].resize( hrdCabCntMinus1_[i] + 1 );
    nalInitialAltCabRemovalDelay_[i].resize( hrdCabCntMinus1_[i] + 1 );
    nalInitialAltCabRemovalOffset_[i].resize( hrdCabCntMinus1_[i] + 1 );
    aclInitialAltCabRemovalDelay_[i].resize( hrdCabCntMinus1_[i] + 1 );
    aclInitialAltCabRemovalOffset_[i].resize( hrdCabCntMinus1_[i] + 1 );
  }

  bool                   getNalHrdParamsPresentFlag() { return nalHrdParamsPresentFlag_; }
  bool                   getAclHrdParamsPresentFlag() { return aclHrdParamsPresentFlag_; }
  bool                   getInitialCabRemovalDelayLengthMinus1() { return initialCabRemovalDelayLengthMinus1_; }
  bool                   getAuCabRemovalDelayLengthMinus1() { return auCabRemovalDelayLengthMinus1_; }
  bool                   getDabOutputDelayLengthMinus1() { return dabOutputDelayLengthMinus1_; }
  bool                   getIrapCabParamsPresentFlag() { return irapCabParamsPresentFlag_; }
  bool                   getConcatenationFlag() { return concatenationFlag_; }
  uint8_t                getAtlasSequenceParameterSetId() { return atlasSequenceParameterSetId_; }
  uint32_t               getCabDelayOffset() { return cabDelayOffset_; }
  uint32_t               getDabDelayOffset() { return dabDelayOffset_; }
  uint32_t               getAtlasCabRemovalDelayDeltaMinus1() { return atlasCabRemovalDelayDeltaMinus1_; }
  uint32_t               getMaxSubLayersMinus1() { return maxSubLayersMinus1_; }
  std::vector<uint32_t>& getHrdCabCntMinus1() { return hrdCabCntMinus1_; }
  std::vector<std::vector<uint32_t>>& getNalInitialCabRemovalDelay() { return nalInitialCabRemovalDelay_; }
  std::vector<std::vector<uint32_t>>& getNalInitialCabRemovalOffse() { return nalInitialCabRemovalOffset_; }
  std::vector<std::vector<uint32_t>>& getNalInitialAltCabRemovalDelay() { return nalInitialAltCabRemovalDelay_; }
  std::vector<std::vector<uint32_t>>& getNalInitialAltCabRemovalOffset() { return nalInitialAltCabRemovalOffset_; }
  std::vector<std::vector<uint32_t>>& getAclInitialCabRemovalDelay() { return aclInitialCabRemovalDelay_; }
  std::vector<std::vector<uint32_t>>& getAclInitialCabRemovalOffset() { return aclInitialCabRemovalOffset_; }
  std::vector<std::vector<uint32_t>>& getAclInitialAltCabRemovalDelay() { return aclInitialAltCabRemovalDelay_; }
  std::vector<std::vector<uint32_t>>& getAclInitialAltCabRemovalOffset() { return aclInitialAltCabRemovalOffset_; }
  uint32_t                            getHrdCabCntMinus1( size_t i ) { return hrdCabCntMinus1_[i]; }
  uint32_t getNalInitialCabRemovalDelay( size_t i, size_t j ) { return nalInitialCabRemovalDelay_[i][j]; }
  uint32_t getNalInitialCabRemovalOffset( size_t i, size_t j ) { return nalInitialCabRemovalOffset_[i][j]; }
  uint32_t getAclInitialCabRemovalDelay( size_t i, size_t j ) { return aclInitialCabRemovalDelay_[i][j]; }
  uint32_t getAclInitialCabRemovalOffset( size_t i, size_t j ) { return aclInitialCabRemovalOffset_[i][j]; }
  uint32_t getNalInitialAltCabRemovalDelay( size_t i, size_t j ) { return nalInitialAltCabRemovalDelay_[i][j]; }
  uint32_t getNalInitialAltCabRemovalOffset( size_t i, size_t j ) { return nalInitialAltCabRemovalOffset_[i][j]; }
  uint32_t getAclInitialAltCabRemovalDelay( size_t i, size_t j ) { return aclInitialAltCabRemovalDelay_[i][j]; }
  uint32_t getAclInitialAltCabRemovalOffset( size_t i, size_t j ) { return aclInitialAltCabRemovalOffset_[i][j]; }
  std::vector<uint32_t>& getNalInitialCabRemovalDelay( size_t i ) { return nalInitialCabRemovalDelay_[i]; }
  std::vector<uint32_t>& getNalInitialCabRemovalOffsey( size_t i ) { return nalInitialCabRemovalOffset_[i]; }
  std::vector<uint32_t>& getAclInitialCabRemovalDelay( size_t i ) { return aclInitialCabRemovalDelay_[i]; }
  std::vector<uint32_t>& getAclInitialCabRemovalOffset( size_t i ) { return aclInitialCabRemovalOffset_[i]; }
  std::vector<uint32_t>& getNalInitialAltCabRemovalDelay( size_t i ) { return nalInitialAltCabRemovalDelay_[i]; }
  std::vector<uint32_t>& getNalInitialAltCabRemovalOffset( size_t i ) { return nalInitialAltCabRemovalOffset_[i]; }
  std::vector<uint32_t>& getAclInitialAltCabRemovalDelay( size_t i ) { return aclInitialAltCabRemovalDelay_[i]; }
  std::vector<uint32_t>& getAclInitialAltCabRemovalOffset( size_t i ) { return aclInitialAltCabRemovalOffset_[i]; }

  void setNalHrdParamsPresentFlag( bool value ) { nalHrdParamsPresentFlag_ = value; }
  void setAclHrdParamsPresentFlag( bool value ) { aclHrdParamsPresentFlag_ = value; }
  void setInitialCabRemovalDelayLengthMinus1( bool value ) { initialCabRemovalDelayLengthMinus1_ = value; }
  void setAuCabRemovalDelayLengthMinus1( bool value ) { auCabRemovalDelayLengthMinus1_ = value; }
  void setDabOutputDelayLengthMinus1( bool value ) { dabOutputDelayLengthMinus1_ = value; }
  void setIrapCabParamsPresentFlag( bool value ) { irapCabParamsPresentFlag_ = value; }
  void setConcatenationFlag( bool value ) { concatenationFlag_ = value; }
  void setAtlasSequenceParameterSetId( uint8_t value ) { atlasSequenceParameterSetId_ = value; }
  void setCabDelayOffset( uint32_t value ) { cabDelayOffset_ = value; }
  void setDabDelayOffset( uint32_t value ) { dabDelayOffset_ = value; }
  void setAtlasCabRemovalDelayDeltaMinus1( uint32_t value ) { atlasCabRemovalDelayDeltaMinus1_ = value; }
  void setMaxSubLayersMinus1( uint32_t value ) { maxSubLayersMinus1_ = value; }
  void setHrdCabCntMinus1( size_t i, uint32_t v ) { hrdCabCntMinus1_[i] = v; }
  void setNalInitialAltCabRemovalDelay( size_t i, size_t j, uint32_t v ) { nalInitialAltCabRemovalDelay_[i][j] = v; }
  void setNalInitialAltCabRemovalOffset( size_t i, size_t j, uint32_t v ) { nalInitialAltCabRemovalOffset_[i][j] = v; }
  void setAclInitialAltCabRemovalDelay( size_t i, size_t j, uint32_t v ) { aclInitialAltCabRemovalDelay_[i][j] = v; }
  void setAclInitialAltCabRemovalOffset( size_t i, size_t j, uint32_t v ) { aclInitialAltCabRemovalOffset_[i][j] = v; }
  void setNalInitialCabRemovalDelay( size_t i, size_t j, uint32_t v ) { nalInitialCabRemovalDelay_[i][j] = v; }
  void setNalInitialCabRemovalOffset( size_t i, size_t j, uint32_t v ) { nalInitialCabRemovalOffset_[i][j] = v; }
  void setAclInitialCabRemovalDelay( size_t i, size_t j, uint32_t v ) { aclInitialCabRemovalDelay_[i][j] = v; }
  void setAclInitialCabRemovalOffset( size_t i, size_t j, uint32_t v ) { aclInitialCabRemovalOffset_[i][j] = v; }

 private:
  bool                               nalHrdParamsPresentFlag_;
  bool                               aclHrdParamsPresentFlag_;
  bool                               initialCabRemovalDelayLengthMinus1_;
  bool                               auCabRemovalDelayLengthMinus1_;
  bool                               dabOutputDelayLengthMinus1_;
  bool                               irapCabParamsPresentFlag_;
  bool                               concatenationFlag_;
  uint8_t                            atlasSequenceParameterSetId_;
  uint32_t                           cabDelayOffset_;
  uint32_t                           dabDelayOffset_;
  uint32_t                           atlasCabRemovalDelayDeltaMinus1_;
  uint32_t                           maxSubLayersMinus1_;
  std::vector<uint32_t>              hrdCabCntMinus1_;
  std::vector<std::vector<uint32_t>> nalInitialCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> nalInitialCabRemovalOffset_;
  std::vector<std::vector<uint32_t>> nalInitialAltCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> nalInitialAltCabRemovalOffset_;
  std::vector<std::vector<uint32_t>> aclInitialCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> aclInitialCabRemovalOffset_;
  std::vector<std::vector<uint32_t>> aclInitialAltCabRemovalDelay_;
  std::vector<std::vector<uint32_t>> aclInitialAltCabRemovalOffset_;
};

// F.2.14  Atlas frame timing SEI message syntax
class SEIAtlasFrameTiming : public SEI {
 public:
  SEIAtlasFrameTiming() {}
  ~SEIAtlasFrameTiming() {
    cabRemovalDelayMinus1_.clear();
    dabOutputDelay_.clear();
  }
  SEIAtlasFrameTiming& operator=( const SEIAtlasFrameTiming& ) = default;

  SeiPayloadType getPayloadType() { return ATLAS_FRAME_TIMING; }

  void allocate( int32_t size ) {
    cabRemovalDelayMinus1_.resize( size, 0 );
    dabOutputDelay_.resize( size, 0 );
  }
  std::vector<uint32_t>& getAftCabRemovalDelayMinus1() { return cabRemovalDelayMinus1_; }
  std::vector<uint32_t>& getAftDabOutputDelay() { return dabOutputDelay_; }
  uint32_t               getAftCabRemovalDelayMinus1( uint32_t i ) { return cabRemovalDelayMinus1_[i]; }
  uint32_t               getAftDabOutputDelay( uint32_t i ) { return dabOutputDelay_[i]; }
  void setAftCabRemovalDelayMinus1( uint32_t i, uint32_t value ) { cabRemovalDelayMinus1_[i] = value; }
  void setAftDabOutputDelay( uint32_t i, uint32_t value ) { dabOutputDelay_[i] = value; }

 private:
  std::vector<uint32_t> cabRemovalDelayMinus1_;
  std::vector<uint32_t> dabOutputDelay_;
};

// F.2.12.5  Atlas object information  SEI message syntax
class SEIAtlasInformation : public SEI {
 public:
  SEIAtlasInformation() :
      persistenceFlag_( false ),
      resetFlag_( false ),
      numAtlasesMinus1_( 0 ),
      numUpdates_( 0 ),
      log2MaxObjectIdxTracked_( 0 ) {}
  ~SEIAtlasInformation() {
    atlasId_.clear();
    objectIdx_.clear();
    for ( auto& element : objectInAtlasPresentFlag_ ) { element.clear(); }
    objectInAtlasPresentFlag_.clear();
  }
  SEIAtlasInformation& operator=( const SEIAtlasInformation& ) = default;

  SeiPayloadType getPayloadType() { return ATLAS_OBJECT_INFORMATION; }

  void allocateAltasId() { atlasId_.resize( numAtlasesMinus1_ + 1, 0 ); }
  void allocateObjectIdx() {
    objectIdx_.resize( numUpdates_, 0 );
    objectInAtlasPresentFlag_.resize( numUpdates_ );
    for ( auto& element : objectInAtlasPresentFlag_ ) { element.resize( numAtlasesMinus1_ + 1, false ); }
  }

  bool     getPersistenceFlag() { return persistenceFlag_; }
  bool     getResetFlag() { return resetFlag_; }
  uint32_t getNumAtlasesMinus1() { return numAtlasesMinus1_; }
  uint32_t getNumUpdates() { return numUpdates_; }
  uint32_t getLog2MaxObjectIdxTracked() { return log2MaxObjectIdxTracked_; }
  uint32_t getAtlasId( size_t i ) { return atlasId_[i]; }
  uint32_t getObjectIdx( size_t i ) { return objectIdx_[i]; }
  bool     getObjectInAtlasPresentFlag( size_t i, size_t j ) { return objectInAtlasPresentFlag_[i][j]; }

  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }
  void setResetFlag( bool value ) { resetFlag_ = value; }
  void setNumAtlasesMinus1( uint32_t value ) { numAtlasesMinus1_ = value; }
  void setNumUpdates( uint32_t value ) { numUpdates_ = value; }
  void setLog2MaxObjectIdxTracked( uint32_t value ) { log2MaxObjectIdxTracked_ = value; }
  void setAtlasId( size_t i, uint32_t value ) { atlasId_[i] = value; }
  void setObjectIdx( size_t i, uint32_t value ) { objectIdx_[i] = value; }
  void setObjectInAtlasPresentFlag( size_t i, size_t j, bool value ) { objectInAtlasPresentFlag_[i][j] = value; }

 private:
  bool                           persistenceFlag_;
  bool                           resetFlag_;
  uint32_t                       numAtlasesMinus1_;
  uint32_t                       numUpdates_;
  uint32_t                       log2MaxObjectIdxTracked_;
  std::vector<uint32_t>          atlasId_;
  std::vector<uint32_t>          objectIdx_;
  std::vector<std::vector<bool>> objectInAtlasPresentFlag_;
};

// F.2.15.1	Viewport camera parameters SEI messages syntax
class SEIViewportCameraParameters : public SEI {
 public:
  SEIViewportCameraParameters() :
      cameraId_( 0 ),
      cancelFlag_( false ),
      persistenceFlag_( false ),
      cameraType_( 0.f ),
      erpHorizontalFov_( 0.f ),
      erpVerticalFov_( 0.f ),
      perspectiveAspectRatio_( 0.f ),
      perspectiveHorizontalFov_( 0.f ),
      orthoAspectRatio_( 0.f ),
      orthoHorizontalSize_( 0.f ),
      clippingNearPlane_( 0.f ),
      clippingFarPlane_( 0.f ) {}
  ~SEIViewportCameraParameters() {}
  SEIViewportCameraParameters& operator=( const SEIViewportCameraParameters& ) = default;

  SeiPayloadType getPayloadType() { return VIEWPORT_CAMERA_PARAMETERS; }

  uint32_t getCameraId() { return cameraId_; }
  bool     getCancelFlag() { return cancelFlag_; }
  bool     getPersistenceFlag() { return persistenceFlag_; }
  uint8_t  getCameraType() { return cameraType_; }
  float    getErpHorizontalFov() { return erpHorizontalFov_; }
  float    getErpVerticalFov() { return erpVerticalFov_; }
  float    getPerspectiveAspectRatio() { return perspectiveAspectRatio_; }
  float    getPerspectiveHorizontalFov() { return perspectiveHorizontalFov_; }
  float    getOrthoAspectRatio() { return orthoAspectRatio_; }
  float    getOrthoHorizontalSize() { return orthoHorizontalSize_; }
  float    getClippingNearPlane() { return clippingNearPlane_; }
  float    getClippingFarPlane() { return clippingFarPlane_; }

  void setCameraId( uint32_t value ) { cameraId_ = value; }
  void setCancelFlag( bool value ) { cancelFlag_ = value; }
  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }
  void setCameraType( uint8_t value ) { cameraType_ = value; }
  void setErpHorizontalFov( float value ) { erpHorizontalFov_ = value; }
  void setErpVerticalFov( float value ) { erpVerticalFov_ = value; }
  void setPerspectiveAspectRatio( float value ) { perspectiveAspectRatio_ = value; }
  void setPerspectiveHorizontalFov( float value ) { perspectiveHorizontalFov_ = value; }
  void setOrthoAspectRatio( float value ) { orthoAspectRatio_ = value; }
  void setOrthoHorizontalSize( float value ) { orthoHorizontalSize_ = value; }
  void setClippingNearPlane( float value ) { clippingNearPlane_ = value; }
  void setClippingFarPlane( float value ) { clippingFarPlane_ = value; }

 private:
  uint32_t cameraId_;
  bool     cancelFlag_;
  bool     persistenceFlag_;
  uint8_t  cameraType_;
  float    erpHorizontalFov_;
  float    erpVerticalFov_;
  float    perspectiveAspectRatio_;
  float    perspectiveHorizontalFov_;
  float    orthoAspectRatio_;
  float    orthoHorizontalSize_;
  float    clippingNearPlane_;
  float    clippingFarPlane_;
};

// F.2.15.2	Viewport position SEI messages syntax
class SEIViewportPosition : public SEI {
 public:
  SEIViewportPosition() :
      viewportId_( 0 ),
      cameraParametersPresentFlag_( false ),
      CameraId_( 0 ),
      cancelFlag_( false ),
      persistenceFlag_( false ),
      position_{0.0f, 0.0f, 0.0f},
      rotationQX_( 0.0f ),
      rotationQY_( 0.0f ),
      rotationQZ_( 0.0f ),
      centerViewFlag_( false ),
      leftViewFlag_( false ) {}
  ~SEIViewportPosition() {}
  SEIViewportPosition& operator=( const SEIViewportPosition& ) = default;

  SeiPayloadType getPayloadType() { return VIEWPORT_POSITION; }

  uint32_t getViewportId() { return viewportId_; }
  bool     getCameraParametersPresentFlag() { return cameraParametersPresentFlag_; }
  uint8_t  getVcpCameraId() { return CameraId_; }
  bool     getCancelFlag() { return cancelFlag_; }
  bool     getPersistenceFlag() { return persistenceFlag_; }
  float    getPosition( size_t i ) { return position_[i]; }
  float    getRotationQX() { return rotationQX_; }
  float    getRotationQY() { return rotationQY_; }
  float    getRotationQZ() { return rotationQZ_; }
  bool     getCenterViewFlag() { return centerViewFlag_; }
  bool     getLeftViewFlag() { return leftViewFlag_; }

  void setViewportId( uint32_t value ) { viewportId_ = value; }
  void setCameraParametersPresentFlag( bool value ) { cameraParametersPresentFlag_ = value; }
  void setVcpCameraId( uint8_t value ) { CameraId_ = value; }
  void setCancelFlag( bool value ) { cancelFlag_ = value; }
  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }
  void setPosition( size_t i, float value ) { position_[i] = value; }
  void setRotationQX( float value ) { rotationQX_ = value; }
  void setRotationQY( float value ) { rotationQY_ = value; }
  void setRotationQZ( float value ) { rotationQZ_ = value; }
  void setCenterViewFlag( bool value ) { centerViewFlag_ = value; }
  void setLeftViewFlag( bool value ) { leftViewFlag_ = value; }

 private:
  uint32_t viewportId_;
  bool     cameraParametersPresentFlag_;
  uint8_t  CameraId_;
  bool     cancelFlag_;
  bool     persistenceFlag_;
  float    position_[3];
  float    rotationQX_;
  float    rotationQY_;
  float    rotationQZ_;
  bool     centerViewFlag_;
  bool     leftViewFlag_;
};

// F.2.16 Decoded Atlas Information Hash SEI message syntax
// F.2.16.1 Decoded high level hash unit syntax
class SEIDecodedAtlasInformationHash : public SEI {
 public:
  SEIDecodedAtlasInformationHash() :
      cancelFlag_( false ),
      persistenceFlag_( false ),
      hashType_( 1 ),
      decodedHighLevelHashPresentFlag_( true ),
      decodedAtlasHashPresentFlag_( false ),
      decodedAtlasB2pHashPresentFlag_( false ),
      decodedAtlasTilesHashPresentFlag_( false ),
      decodedAtlasTilesB2pHashPresentFlag_( false ),
      numTilesMinus1_( 0 ),
      tileIdLenMinus1_( 0 ),
      highLevelCrc_( 0 ),
      highLevelChecksum_( 0 ),
      atlasCrc_( 0 ),
      atlasChecksum_( 0 ),
      atlasB2pCrc_( 0 ),
      atlasB2pChecksum_( 0 ) {
    tileId_.resize( numTilesMinus1_ + 1 );
    highLevelMd5_.resize( 16 );
    atlasMd5_.resize( 16 );
    atlasB2pMd5_.resize( 16 );
    atlasTilesCrc_.resize( numTilesMinus1_ + 1 );
    atlasTilesChecksum_.resize( numTilesMinus1_ + 1 );
    ;
    atlasTilesMd5_.resize( numTilesMinus1_ + 1 );
    atlasTilesB2pCrc_.resize( numTilesMinus1_ + 1 );
    atlasTilesB2pChecksum_.resize( numTilesMinus1_ + 1 );
    atlasTilesB2pMd5_.resize( numTilesMinus1_ + 1 );
    for ( auto& element : atlasTilesMd5_ ) { element.resize( 16 ); }
    for ( auto& element : atlasTilesB2pMd5_ ) { element.resize( 16 ); }
  }

  ~SEIDecodedAtlasInformationHash() {
    highLevelMd5_.clear();
    atlasMd5_.clear();
    atlasB2pMd5_.clear();
    tileId_.clear();
    atlasTilesCrc_.clear();
    atlasTilesChecksum_.clear();
    atlasTilesB2pCrc_.clear();
    atlasTilesB2pChecksum_.clear();
    for ( auto& element : atlasTilesMd5_ ) { element.clear(); }
    for ( auto& element : atlasTilesB2pMd5_ ) { element.clear(); }
    atlasTilesMd5_.clear();
    atlasTilesB2pMd5_.clear();
  }
  SEIDecodedAtlasInformationHash& operator=( const SEIDecodedAtlasInformationHash& ) = default;

  SeiPayloadType getPayloadType() { return DECODED_ATLAS_INFORMATION_HASH; }

  void allocateAtlasTilesHash( size_t numTiles ) {
    assert( numTiles > 0 );
    numTilesMinus1_ = numTiles - 1;
    tileId_.resize( numTiles );
    if ( highLevelMd5_.size() != 16 ) highLevelMd5_.resize( 16 );
    if ( atlasMd5_.size() != 16 ) atlasMd5_.resize( 16 );
    if ( atlasB2pMd5_.size() != 16 ) atlasB2pMd5_.resize( 16 );
    atlasTilesMd5_.resize( numTiles );
    atlasTilesCrc_.resize( numTiles );
    atlasTilesChecksum_.resize( numTiles );
    atlasTilesB2pMd5_.resize( numTiles );
    atlasTilesB2pCrc_.resize( numTiles );
    atlasTilesB2pChecksum_.resize( numTiles );
    for ( auto& element : atlasTilesMd5_ ) {
      if ( element.size() == 0 ) element.resize( 16 );
    }
    for ( auto& element : atlasTilesB2pMd5_ ) {
      if ( element.size() == 0 ) element.resize( 16 );
    }
  }

  bool     getCancelFlag() { return cancelFlag_; }
  bool     getPersistenceFlag() { return persistenceFlag_; }
  uint8_t  getHashType() { return hashType_; }
  bool     getDecodedHighLevelHashPresentFlag() { return decodedHighLevelHashPresentFlag_; }
  bool     getDecodedAtlasHashPresentFlag() { return decodedAtlasHashPresentFlag_; }
  bool     getDecodedAtlasB2pHashPresentFlag() { return decodedAtlasB2pHashPresentFlag_; }
  bool     getDecodedAtlasTilesHashPresentFlag() { return decodedAtlasTilesHashPresentFlag_; }
  bool     getDecodedAtlasTilesB2pHashPresentFlag() { return decodedAtlasTilesB2pHashPresentFlag_; }
  uint16_t getHighLevelCrc() { return highLevelCrc_; }
  uint16_t getAtlasCrc() { return atlasCrc_; }
  uint32_t getHighLevelCheckSum() { return highLevelChecksum_; }
  uint32_t getAtlasCheckSum() { return atlasChecksum_; }
  uint16_t getAtlasB2pCrc() { return atlasB2pCrc_; }
  uint32_t getAtlasB2pCheckSum() { return atlasB2pChecksum_; }
  uint32_t getNumTilesMinus1() { return numTilesMinus1_; }
  uint32_t getTileIdLenMinus1() { return tileIdLenMinus1_; }
  uint32_t getTileId( size_t i ) { return tileId_[i]; }
  uint8_t  getHighLevelMd5( size_t i ) { return highLevelMd5_[i]; }
  uint8_t  getAtlasMd5( size_t i ) { return atlasMd5_[i]; }
  uint8_t  getAtlasB2pMd5( size_t i ) { return atlasB2pMd5_[i]; }
  uint32_t getAtlasTilesCrc( size_t i ) { return atlasTilesCrc_[i]; }
  uint32_t getAtlasTilesCheckSum( size_t i ) { return atlasTilesChecksum_[i]; }
  uint32_t getAtlasTilesB2pCrc( size_t i ) { return atlasTilesB2pCrc_[i]; }
  uint32_t getAtlasTilesB2pCheckSum( size_t i ) { return atlasTilesB2pChecksum_[i]; }
  uint32_t getAtlasTilesMd5( size_t i, size_t j ) { return atlasTilesMd5_[i][j]; }
  uint32_t getAtlasTilesB2pMd5( size_t i, size_t j ) { return atlasTilesB2pMd5_[i][j]; }

  void setCancelFlag( bool value ) { cancelFlag_ = value; }
  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }
  void setHashType( uint8_t value ) { hashType_ = value; }
  void setDecodedHighLevelHashPresentFlag( bool value ) { decodedHighLevelHashPresentFlag_ = value; }
  void setDecodedAtlasHashPresentFlag( bool value ) { decodedAtlasHashPresentFlag_ = value; }
  void setDecodedAtlasB2pHashPresentFlag( bool value ) { decodedAtlasB2pHashPresentFlag_ = value; }
  void setDecodedAtlasTilesHashPresentFlag( bool value ) { decodedAtlasTilesHashPresentFlag_ = value; }
  void setDecodedAtlasTilesB2pHashPresentFlag( bool value ) { decodedAtlasTilesB2pHashPresentFlag_ = value; }
  void setHighLevelCrc( uint16_t value ) { highLevelCrc_ = value; }
  void setAtlasCrc( uint16_t value ) { atlasCrc_ = value; }
  void setHighLevelCheckSum( uint32_t value ) { highLevelChecksum_ = value; }
  void setAtlasCheckSum( uint32_t value ) { atlasChecksum_ = value; }
  void setAtlasB2pCrc( uint16_t value ) { atlasB2pCrc_ = value; }
  void setAtlasB2pCheckSum( uint32_t value ) { atlasB2pChecksum_ = value; }
  void setNumTilesMinus1( uint32_t value ) { numTilesMinus1_ = value; }
  void setTileIdLenMinus1( uint32_t value ) { tileIdLenMinus1_ = value; }
  void setTileId( size_t i, uint32_t value ) { tileId_[i] = value; }
  void setHighLevelMd5( size_t i, uint8_t value ) { highLevelMd5_[i] = value; }
  void setAtlasMd5( size_t i, uint8_t value ) { atlasMd5_[i] = value; }
  void setAtlasB2pMd5( size_t i, uint8_t value ) { atlasB2pMd5_[i] = value; }
  void setAtlasTilesCrc( size_t i, uint32_t value ) { atlasTilesCrc_[i] = value; }
  void setAtlasTilesCheckSum( size_t i, uint32_t value ) { atlasTilesChecksum_[i] = value; }
  void setAtlasTilesB2pCrc( size_t i, uint32_t value ) { atlasTilesB2pCrc_[i] = value; }
  void setAtlasTilesB2pCheckSum( size_t i, uint32_t value ) { atlasTilesB2pChecksum_[i] = value; }
  void setAtlasTilesMd5( size_t i, size_t j, uint32_t value ) { atlasTilesMd5_[i][j] = value; }
  void setAtlasTilesB2pMd5( size_t i, size_t j, uint32_t value ) { atlasTilesB2pMd5_[i][j] = value; }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = cancelFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = persistenceFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = hashType_ & 0xFF;
    byteStrData_.push_back( val );
    val = decodedHighLevelHashPresentFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = decodedAtlasHashPresentFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = decodedAtlasB2pHashPresentFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = decodedAtlasTilesHashPresentFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = decodedAtlasTilesB2pHashPresentFlag_ & 0xFF;
    byteStrData_.push_back( val );
    if ( decodedHighLevelHashPresentFlag_ ) {
      switch ( hashType_ ) {
        case 0:
          for ( int i = 0; i < 16; i++ ) {
            val = highLevelMd5_[i] & 0xFF;
            byteStrData_.push_back( val );
          }
          break;
        case 1:
          val = highLevelCrc_ & 0xFF;
          byteStrData_.push_back( val );
          val = ( highLevelCrc_ >> 8 ) & 0xFF;
          byteStrData_.push_back( val );
          break;
        case 2:
          val = highLevelChecksum_ & 0xFF;
          byteStrData_.push_back( val );
          val = ( highLevelChecksum_ >> 8 ) & 0xFF;
          byteStrData_.push_back( val );
          val = ( highLevelChecksum_ >> 16 ) & 0xFF;
          byteStrData_.push_back( val );
          val = ( highLevelChecksum_ >> 24 ) & 0xFF;
          byteStrData_.push_back( val );
          break;
        default: std::cerr << " Undefined Hash Type " << std::endl;
      }
      if ( decodedAtlasHashPresentFlag_ ) {
        switch ( hashType_ ) {
          case 0:
            for ( int i = 0; i < 16; i++ ) {
              val = atlasMd5_[i] & 0xFF;
              byteStrData_.push_back( val );
            }
            break;
          case 1:
            val = atlasCrc_ & 0xFF;
            byteStrData_.push_back( val );
            val = ( atlasCrc_ >> 8 ) & 0xFF;
            byteStrData_.push_back( val );
            break;
          case 2:;
            val = atlasChecksum_ & 0xFF;
            byteStrData_.push_back( val );
            val = ( atlasChecksum_ >> 8 ) & 0xFF;
            byteStrData_.push_back( val );
            val = ( atlasChecksum_ >> 16 ) & 0xFF;
            byteStrData_.push_back( val );
            val = ( atlasChecksum_ >> 24 ) & 0xFF;
            byteStrData_.push_back( val );
            break;
          default: std::cerr << " Undefined Hash Type " << std::endl;
        }
      }
      if ( decodedAtlasB2pHashPresentFlag_ ) {
        switch ( hashType_ ) {
          case 0:
            for ( int i = 0; i < 16; i++ ) {
              val = atlasB2pMd5_[i] & 0xFF;
              byteStrData_.push_back( val );
            }
            break;
          case 1:
            val = atlasB2pCrc_ & 0xFF;
            byteStrData_.push_back( val );
            val = ( atlasB2pCrc_ >> 8 ) & 0xFF;
            byteStrData_.push_back( val );
            break;
          case 2:;
            val = atlasChecksum_ & 0xFF;
            byteStrData_.push_back( val );
            val = ( atlasB2pChecksum_ >> 8 ) & 0xFF;
            byteStrData_.push_back( val );
            val = ( atlasB2pChecksum_ >> 16 ) & 0xFF;
            byteStrData_.push_back( val );
            val = ( atlasB2pChecksum_ >> 24 ) & 0xFF;
            byteStrData_.push_back( val );
            break;
          default: std::cerr << " Undefined Hash Type " << std::endl;
        }
      }
      if ( decodedAtlasTilesHashPresentFlag_ || decodedAtlasTilesB2pHashPresentFlag_ ) {
        val = numTilesMinus1_ & 0XFF;
        byteStrData_.push_back( val );
        val = ( numTilesMinus1_ >> 8 ) & 0XFF;
        byteStrData_.push_back( val );
        val = ( numTilesMinus1_ >> 16 ) & 0XFF;
        byteStrData_.push_back( val );
        val = ( numTilesMinus1_ >> 24 ) & 0XFF;
        byteStrData_.push_back( val );
        val = tileIdLenMinus1_ & 0XFF;
        byteStrData_.push_back( val );
        val = ( tileIdLenMinus1_ >> 8 ) & 0XFF;
        byteStrData_.push_back( val );
        val = ( tileIdLenMinus1_ >> 16 ) & 0XFF;
        byteStrData_.push_back( val );
        val = ( tileIdLenMinus1_ >> 24 ) & 0XFF;
        byteStrData_.push_back( val );
        for ( int t = 0; t < numTilesMinus1_ + 1; t++ ) {
          val = tileId_[t] & 0XFF;
          byteStrData_.push_back( val );
          val = ( tileId_[t] >> 8 ) & 0XFF;
          byteStrData_.push_back( val );
          val = ( tileId_[t] >> 16 ) & 0XFF;
          byteStrData_.push_back( val );
          val = ( tileId_[t] >> 24 ) & 0XFF;
          byteStrData_.push_back( val );
        }
        for ( int t = 0; t < numTilesMinus1_ + 1; t++ ) {
          uint32_t j = tileId_[t];
          if ( decodedAtlasTilesHashPresentFlag_ ) {
            switch ( hashType_ ) {
              case 0:
                for ( int i = 0; i < 16; i++ ) {
                  val = atlasTilesMd5_[j][i] & 0xFF;
                  byteStrData_.push_back( val );
                }
                break;
              case 1:
                val = atlasTilesCrc_[j] & 0xFF;
                byteStrData_.push_back( val );
                val = ( atlasTilesCrc_[j] >> 8 ) & 0xFF;
                byteStrData_.push_back( val );
                break;
              case 2:;
                val = atlasTilesChecksum_[j] & 0xFF;
                byteStrData_.push_back( val );
                val = ( atlasTilesChecksum_[j] >> 8 ) & 0xFF;
                byteStrData_.push_back( val );
                val = ( atlasTilesChecksum_[j] >> 16 ) & 0xFF;
                byteStrData_.push_back( val );
                val = ( atlasTilesChecksum_[j] >> 24 ) & 0xFF;
                byteStrData_.push_back( val );
                break;
              default: std::cerr << " Undefined Hash Type " << std::endl;
            }
          }
          if ( decodedAtlasTilesB2pHashPresentFlag_ ) {
            switch ( hashType_ ) {
              case 0:
                for ( int i = 0; i < 16; i++ ) {
                  val = atlasTilesB2pMd5_[j][i] & 0xFF;
                  byteStrData_.push_back( val );
                }
                break;
              case 1:
                val = atlasTilesB2pCrc_[j] & 0xFF;
                byteStrData_.push_back( val );
                val = ( atlasTilesB2pCrc_[j] >> 8 ) & 0xFF;
                byteStrData_.push_back( val );
                break;
              case 2:;
                val = atlasTilesB2pChecksum_[j] & 0xFF;
                byteStrData_.push_back( val );
                val = ( atlasTilesB2pChecksum_[j] >> 8 ) & 0xFF;
                byteStrData_.push_back( val );
                val = ( atlasTilesB2pChecksum_[j] >> 16 ) & 0xFF;
                byteStrData_.push_back( val );
                val = ( atlasTilesB2pChecksum_[j] >> 24 ) & 0xFF;
                byteStrData_.push_back( val );
                break;
              default: std::cerr << " Undefined Hash Type " << std::endl;
            }
          }
        }
      }
    }
    return byteStrData_;
  }

 private:
  bool                               cancelFlag_;
  bool                               persistenceFlag_;
  uint8_t                            hashType_;
  bool                               decodedHighLevelHashPresentFlag_;
  bool                               decodedAtlasHashPresentFlag_;
  bool                               decodedAtlasB2pHashPresentFlag_;
  bool                               decodedAtlasTilesHashPresentFlag_;
  bool                               decodedAtlasTilesB2pHashPresentFlag_;
  uint32_t                           numTilesMinus1_;
  uint32_t                           tileIdLenMinus1_;
  std::vector<uint32_t>              tileId_;
  std::vector<uint8_t>               highLevelMd5_;
  uint16_t                           highLevelCrc_;
  uint32_t                           highLevelChecksum_;
  std::vector<uint8_t>               atlasMd5_;
  uint16_t                           atlasCrc_;
  uint32_t                           atlasChecksum_;
  std::vector<uint8_t>               atlasB2pMd5_;
  uint16_t                           atlasB2pCrc_;
  uint32_t                           atlasB2pChecksum_;
  std::vector<uint16_t>              atlasTilesCrc_;
  std::vector<uint32_t>              atlasTilesChecksum_;
  std::vector<std::vector<uint32_t>> atlasTilesMd5_;
  std::vector<uint16_t>              atlasTilesB2pCrc_;
  std::vector<uint32_t>              atlasTilesB2pChecksum_;
  std::vector<std::vector<uint32_t>> atlasTilesB2pMd5_;
};

// H.20.2.18 Occupancy synthesis SEI message syntax
class SEIOccupancySynthesis : public SEI {
 public:
  SEIOccupancySynthesis() : persistenceFlag_( false ), resetFlag_( false ), instancesUpdated_( 0 ) {}
  ~SEIOccupancySynthesis() {
    instanceIndex_.clear();
    instanceCancelFlag_.clear();
    methodType_.clear();
    pbfLog2ThresholdMinus1_.clear();
    pbfPassesCountMinus1_.clear();
    pbfFilterSizeMinus1_.clear();
  }
  SEIOccupancySynthesis& operator=( const SEIOccupancySynthesis& ) = default;

  SeiPayloadType getPayloadType() { return OCCUPANCY_SYNTHESIS; }
  void           allocate() {
    instanceIndex_.resize( instancesUpdated_, 0 );
    instanceCancelFlag_.resize( instancesUpdated_, 0 );
    methodType_.resize( instancesUpdated_, 0 );
    pbfLog2ThresholdMinus1_.resize( instancesUpdated_, 0 );
    pbfPassesCountMinus1_.resize( instancesUpdated_, 0 );
    pbfFilterSizeMinus1_.resize( instancesUpdated_, 0 );
  }

  bool    getPersistenceFlag() { return persistenceFlag_; }
  bool    getResetFlag() { return resetFlag_; }
  uint8_t getInstancesUpdated() { return instancesUpdated_; }
  uint8_t getInstanceIndex( size_t i ) { return instanceIndex_[i]; }
  bool    getInstanceCancelFlag( size_t i ) { return instanceCancelFlag_[i]; }
  uint8_t getMethodType( size_t i ) { return methodType_[i]; }
  uint8_t getPbfLog2ThresholdMinus1( size_t i ) { return pbfLog2ThresholdMinus1_[i]; }
  uint8_t getPbfPassesCountMinus1( size_t i ) { return pbfPassesCountMinus1_[i]; }
  uint8_t getPbfFilterSizeMinus1( size_t i ) { return pbfFilterSizeMinus1_[i]; }

  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }
  void setResetFlag( bool value ) { resetFlag_ = value; }
  void setInstancesUpdated( uint8_t value ) { instancesUpdated_ = value; }
  void setInstanceIndex( size_t i, uint8_t value ) { instanceIndex_[i] = value; }
  void setInstanceCancelFlag( size_t i, bool value ) { instanceCancelFlag_[i] = value; }
  void setMethodType( size_t i, uint8_t value ) { methodType_[i] = value; }
  void setPbfLog2ThresholdMinus1( size_t i, uint8_t value ) { pbfLog2ThresholdMinus1_[i] = value; }
  void setPbfPassesCountMinus1( size_t i, uint8_t value ) { pbfPassesCountMinus1_[i] = value; }
  void setPbfFilterSizeMinus1( size_t i, uint8_t value ) { pbfFilterSizeMinus1_[i] = value; }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = persistenceFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = resetFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = instancesUpdated_ & 0xFF;
    byteStrData_.push_back( val );
    for ( int i = 0; i < instancesUpdated_; i++ ) {
      uint8_t k = instanceIndex_[i];
      val       = instanceIndex_[i] & 0xFF;
      byteStrData_.push_back( val );
      val = instanceCancelFlag_[k] & 0xFF;
      byteStrData_.push_back( val );
      val = methodType_[k] & 0xFF;
      byteStrData_.push_back( val );
      if ( methodType_[k] == 1 ) {
        val = pbfLog2ThresholdMinus1_[k] & 0xFF;
        byteStrData_.push_back( val );
        val = pbfPassesCountMinus1_[k] & 0xFF;
        byteStrData_.push_back( val );
        val = pbfFilterSizeMinus1_[k] & 0xFF;
        byteStrData_.push_back( val );
      }
    }
    return byteStrData_;
  }

 private:
  bool                 persistenceFlag_;
  bool                 resetFlag_;
  uint8_t              instancesUpdated_;
  std::vector<uint8_t> instanceIndex_;
  std::vector<bool>    instanceCancelFlag_;
  std::vector<uint8_t> methodType_;
  std::vector<uint8_t> pbfLog2ThresholdMinus1_;
  std::vector<uint8_t> pbfPassesCountMinus1_;
  std::vector<uint8_t> pbfFilterSizeMinus1_;
};

// H.20.2.19 Geometry smoothing SEI message syntax
class SEIGeometrySmoothing : public SEI {
 public:
  SEIGeometrySmoothing() : persistenceFlag_( false ), resetFlag_( false ), instancesUpdated_( 0 ) {}
  ~SEIGeometrySmoothing() {
    instanceIndex_.clear();
    instanceCancelFlag_.clear();
    methodType_.clear();
    filterEomPointsFlag_.clear();
    gridSizeMinus2_.clear();
    threshold_.clear();
  }
  SEIGeometrySmoothing& operator=( const SEIGeometrySmoothing& ) = default;

  SeiPayloadType getPayloadType() { return GEOMETRY_SMOOTHING; }
  void           allocate() {
    instanceIndex_.resize( instancesUpdated_, 0 );
    instanceCancelFlag_.resize( instancesUpdated_, 0 );
    methodType_.resize( instancesUpdated_, 0 );
    filterEomPointsFlag_.resize( instancesUpdated_, 0 );
    gridSizeMinus2_.resize( instancesUpdated_, 0 );
    threshold_.resize( instancesUpdated_, 0 );
  }

  bool    getPersistenceFlag() { return persistenceFlag_; }
  bool    getResetFlag() { return resetFlag_; }
  uint8_t getInstancesUpdated() { return instancesUpdated_; }
  uint8_t getInstanceIndex( size_t i ) { return instanceIndex_[i]; }
  bool    getInstanceCancelFlag( size_t i ) { return instanceCancelFlag_[i]; }
  uint8_t getMethodType( size_t i ) { return methodType_[i]; }
  uint8_t getFilterEomPointsFlag( size_t i ) { return filterEomPointsFlag_[i]; }
  uint8_t getGridSizeMinus2( size_t i ) { return gridSizeMinus2_[i]; }
  uint8_t getThreshold( size_t i ) { return threshold_[i]; }

  void setPersistenceFlag( bool value ) { persistenceFlag_ = value; }
  void setResetFlag( bool value ) { resetFlag_ = value; }
  void setInstancesUpdated( uint8_t value ) { instancesUpdated_ = value; }
  void setInstanceIndex( size_t i, uint8_t value ) { instanceIndex_[i] = value; }
  void setInstanceCancelFlag( size_t i, bool value ) { instanceCancelFlag_[i] = value; }
  void setMethodType( size_t i, uint8_t value ) { methodType_[i] = value; }
  void setFilterEomPointsFlag( size_t i, uint8_t value ) { filterEomPointsFlag_[i] = value; }
  void setGridSizeMinus2( size_t i, uint8_t value ) { gridSizeMinus2_[i] = value; }
  void setThreshold( size_t i, uint8_t value ) { threshold_[i] = value; }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = persistenceFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = resetFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = instancesUpdated_ & 0xFF;
    byteStrData_.push_back( val );
    for ( int i = 0; i < instancesUpdated_; i++ ) {
      uint8_t k = instanceIndex_[i];
      val       = instanceIndex_[i] & 0xFF;
      byteStrData_.push_back( val );
      val = instanceCancelFlag_[k] & 0xFF;
      byteStrData_.push_back( val );
      if ( instanceCancelFlag_[k] != 1 ) {
        val = methodType_[k] & 0xFF;
        byteStrData_.push_back( val );
        if ( methodType_[k] == 1 ) {
          val = filterEomPointsFlag_[k] & 0xFF;
          byteStrData_.push_back( val );
          val = gridSizeMinus2_[k] & 0xFF;
          byteStrData_.push_back( val );
          val = threshold_[k] & 0xFF;
          byteStrData_.push_back( val );
        }
      }
    }
    return byteStrData_;
  }

 private:
  bool                 persistenceFlag_;
  bool                 resetFlag_;
  uint8_t              instancesUpdated_;
  std::vector<uint8_t> instanceIndex_;
  std::vector<bool>    instanceCancelFlag_;
  std::vector<uint8_t> methodType_;
  std::vector<bool>    filterEomPointsFlag_;
  std::vector<uint8_t> gridSizeMinus2_;
  std::vector<uint8_t> threshold_;
};

// H.20.2.20 Attribute smoothing SEI message syntax
class SEIAttributeSmoothing : public SEI {
 public:
  SEIAttributeSmoothing() : persistenceFlag_( false ), resetFlag_( false ), numAttributesUpdated_( 0 ) {}
  ~SEIAttributeSmoothing() {
    attributeIdx_.clear();
    attributeSmoothingCancelFlag_.clear();
    instancesUpdated_.clear();
    for ( auto& element : instanceIndex_ ) { element.clear(); }
    for ( auto& element : instanceCancelFlag_ ) { element.clear(); }
    for ( auto& element : methodType_ ) { element.clear(); }
    for ( auto& element : gridSizeMinus2_ ) { element.clear(); }
    for ( auto& element : threshold_ ) { element.clear(); }
    for ( auto& element : thresholdVariation_ ) { element.clear(); }
    for ( auto& element : thresholdDifference_ ) { element.clear(); }
    instanceIndex_.clear();
    instanceCancelFlag_.clear();
    methodType_.clear();
    filterEomPointsFlag_.clear();
    gridSizeMinus2_.clear();
    threshold_.clear();
    thresholdVariation_.clear();
    thresholdDifference_.clear();
  }
  SEIAttributeSmoothing& operator=( const SEIAttributeSmoothing& ) = default;

  SeiPayloadType getPayloadType() { return ATTRIBUTE_SMOOTHING; }

  void allocate() {
    attributeIdx_.resize( numAttributesUpdated_, 0 );
    attributeSmoothingCancelFlag_.resize( numAttributesUpdated_, false );
    instancesUpdated_.resize( numAttributesUpdated_, 0 );
  }
  void allocate( size_t size, size_t dimension ) {
    if ( instanceIndex_.size() < size ) {
      instanceIndex_.resize( size );
      instanceCancelFlag_.resize( size );
      methodType_.resize( size );
      filterEomPointsFlag_.resize( size );
      gridSizeMinus2_.resize( size );
      threshold_.resize( size );
      thresholdVariation_.resize( size );
      thresholdDifference_.resize( size );
    }
    instanceIndex_[size - 1].resize( dimension );
    instanceCancelFlag_[size - 1].resize( dimension );
    methodType_[size - 1].resize( dimension );
    filterEomPointsFlag_[size - 1].resize( dimension );
    gridSizeMinus2_[size - 1].resize( dimension );
    threshold_[size - 1].resize( dimension );
    thresholdVariation_[size - 1].resize( dimension );
    thresholdDifference_[size - 1].resize( dimension );
  }
  uint8_t getPersistenceFlag() { return persistenceFlag_; }
  uint8_t getResetFlag() { return resetFlag_; }
  uint8_t getNumAttributesUpdated() { return numAttributesUpdated_; }
  uint8_t getAttributeIdx( size_t i ) { return attributeIdx_[i]; }
  uint8_t getAttributeSmoothingCancelFlag( size_t i ) { return attributeSmoothingCancelFlag_[i]; }
  uint8_t getInstancesUpdated( size_t i ) { return instancesUpdated_[i]; }
  uint8_t getInstanceIndex( size_t i, size_t j ) { return instanceIndex_[i][j]; }
  uint8_t getInstanceCancelFlag( size_t i, size_t j ) { return instanceCancelFlag_[i][j]; }
  uint8_t getMethodType( size_t i, size_t j ) { return methodType_[i][j]; }
  uint8_t getFilterEomPointsFlag( size_t i, size_t j ) { return filterEomPointsFlag_[i][j]; }
  uint8_t getGridSizeMinus2( size_t i, size_t j ) { return gridSizeMinus2_[i][j]; }
  uint8_t getThreshold( size_t i, size_t j ) { return threshold_[i][j]; }
  uint8_t getThresholdVariation( size_t i, size_t j ) { return thresholdVariation_[i][j]; }
  uint8_t getThresholdDifference( size_t i, size_t j ) { return thresholdDifference_[i][j]; }

  void setPersistenceFlag( uint8_t value ) { persistenceFlag_ = value; }
  void setResetFlag( uint8_t value ) { resetFlag_ = value; }
  void setNumAttributesUpdated( uint8_t value ) { numAttributesUpdated_ = value; }
  void setAttributeIdx( size_t i, uint8_t value ) { attributeIdx_[i] = value; }
  void setAttributeSmoothingCancelFlag( size_t i, uint8_t value ) { attributeSmoothingCancelFlag_[i] = value; }
  void setInstancesUpdated( size_t i, uint8_t value ) { instancesUpdated_[i] = value; }
  void setInstanceIndex( size_t i, size_t j, uint8_t value ) { instanceIndex_[i][j] = value; }
  void setInstanceCancelFlag( size_t i, size_t j, uint8_t value ) { instanceCancelFlag_[i][j] = value; }
  void setMethodType( size_t i, size_t j, uint8_t value ) { methodType_[i][j] = value; }
  void setFilterEomPointsFlag( size_t i, size_t j, uint8_t value ) { filterEomPointsFlag_[i][j] = value; }
  void setGridSizeMinus2( size_t i, size_t j, uint8_t value ) { gridSizeMinus2_[i][j] = value; }
  void setThreshold( size_t i, size_t j, uint8_t value ) { threshold_[i][j] = value; }
  void setThresholdVariation( size_t i, size_t j, uint8_t value ) { thresholdVariation_[i][j] = value; }
  void setThresholdDifference( size_t i, size_t j, uint8_t value ) { thresholdDifference_[i][j] = value; }

  std::vector<uint8_t>& getMD5ByteStrData() {
    uint8_t val;
    val = persistenceFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = resetFlag_ & 0xFF;
    byteStrData_.push_back( val );
    val = numAttributesUpdated_ & 0xFF;
    byteStrData_.push_back( val );
    for ( int j = 0; j < numAttributesUpdated_; j++ ) {
      val = attributeIdx_[j] & 0xFF;
      byteStrData_.push_back( val );
      uint8_t k = attributeIdx_[j];
      val       = attributeSmoothingCancelFlag_[k] & 0xFF;
      byteStrData_.push_back( val );
      val = instancesUpdated_[k] & 0xFF;
      byteStrData_.push_back( val );
      for ( int i = 0; i < instancesUpdated_[k]; i++ ) {
        val = instanceIndex_[k][i] & 0xFF;
        byteStrData_.push_back( val );
        uint8_t m = instanceIndex_[k][i];
        val       = instanceCancelFlag_[k][m] & 0xFF;
        byteStrData_.push_back( val );
        if ( instanceCancelFlag_[k][m] != 1 ) {
          val = methodType_[k][m] & 0XFF;
          byteStrData_.push_back( val );
          if ( methodType_[k][m] == 1 ) {
            val = filterEomPointsFlag_[k][m] & 0XFF;
            byteStrData_.push_back( val );
            val = gridSizeMinus2_[k][m] & 0XFF;
            byteStrData_.push_back( val );
            val = threshold_[k][m] & 0XFF;
            byteStrData_.push_back( val );
            val = thresholdVariation_[k][m] & 0XFF;
            byteStrData_.push_back( val );
            val = thresholdDifference_[k][m] & 0XFF;
            byteStrData_.push_back( val );
          }
        }
      }
    }
    return byteStrData_;
  }

 private:
  bool                              persistenceFlag_;
  bool                              resetFlag_;
  uint8_t                           numAttributesUpdated_;
  std::vector<uint8_t>              attributeIdx_;
  std::vector<bool>                 attributeSmoothingCancelFlag_;
  std::vector<uint8_t>              instancesUpdated_;
  std::vector<std::vector<uint8_t>> instanceIndex_;
  std::vector<std::vector<uint8_t>> instanceCancelFlag_;
  std::vector<std::vector<uint8_t>> methodType_;
  std::vector<std::vector<uint8_t>> filterEomPointsFlag_;
  std::vector<std::vector<uint8_t>> gridSizeMinus2_;
  std::vector<std::vector<uint8_t>> threshold_;
  std::vector<std::vector<uint8_t>> thresholdVariation_;
  std::vector<std::vector<uint8_t>> thresholdDifference_;
};

// F.2.17 Time code SEI message syntax
class SEITimeCode : public SEI {
 public:
  SEITimeCode() :
      numUnitsInTick_( 0 ),
      timeScale_( 0 ),
      countingType_( 0 ),
      fullTimestampFlag_( false ),
      discontinuityFlag_( false ),
      cntDroppedFlag_( false ),
      secondFlag_( false ),
      minutesFlag_( false ),
      hoursFlag_( 0 ),
      nFrames_( 0 ),
      secondsValue_( 0 ),
      minutesValue_( 0 ),
      hoursValue_( 0 ),
      timeOffsetLength_( 0 ),
      timeOffsetValue_( 0 ) {}
  ~SEITimeCode() {}
  SEITimeCode& operator=( const SEITimeCode& ) = default;

  SeiPayloadType getPayloadType() { return TIME_CODE; }

  uint32_t getNumUnitsInTick() { return numUnitsInTick_; }
  uint32_t getTimeScale() { return timeScale_; }
  uint16_t getCountingType() { return countingType_; }
  bool     getFullTimestampFlag() { return fullTimestampFlag_; }
  bool     getDiscontinuityFlag() { return discontinuityFlag_; }
  bool     getCntDroppedFlag() { return cntDroppedFlag_; }
  bool     getSecondFlag() { return secondFlag_; }
  bool     getMinutesFlag() { return minutesFlag_; }
  bool     getHoursFlag() { return hoursFlag_; }
  uint16_t getNFrames() { return nFrames_; }
  uint16_t getSecondsValue() { return secondsValue_; }
  uint16_t getMinutesValue() { return minutesValue_; }
  uint16_t getHoursValue() { return hoursValue_; }
  uint16_t getTimeOffsetLength() { return timeOffsetLength_; }
  int16_t  getTimeOffsetValue() { return timeOffsetValue_; }

  void setNumUnitsInTick( uint32_t value ) { numUnitsInTick_ = value; }
  void setTimeScale( uint32_t value ) { timeScale_ = value; }
  void setCountingType( uint16_t value ) { countingType_ = value; }
  void setFullTimestampFlag( bool value ) { fullTimestampFlag_ = value; }
  void setDiscontinuityFlag( bool value ) { discontinuityFlag_ = value; }
  void setCntDroppedFlag( bool value ) { cntDroppedFlag_ = value; }
  void setSecondFlag( bool value ) { secondFlag_ = value; }
  void setMinutesFlag( bool value ) { minutesFlag_ = value; }
  void setHoursFlag( bool value ) { hoursFlag_ = value; }
  void setNFrames( uint16_t value ) { nFrames_ = value; }
  void setSecondsValue( uint16_t value ) { secondsValue_ = value; }
  void setMinutesValue( uint16_t value ) { minutesValue_ = value; }
  void setHoursValue( uint16_t value ) { hoursValue_ = value; }
  void setTimeOffsetLength( uint16_t value ) { timeOffsetLength_ = value; }
  void setTimeOffsetValue( int16_t value ) { timeOffsetValue_ = value; }

 private:
  uint32_t numUnitsInTick_;
  uint32_t timeScale_;
  uint16_t countingType_;
  bool     fullTimestampFlag_;
  bool     discontinuityFlag_;
  bool     cntDroppedFlag_;
  bool     secondFlag_;
  bool     minutesFlag_;
  bool     hoursFlag_;
  uint16_t nFrames_;
  uint16_t secondsValue_;
  uint16_t minutesValue_;
  uint16_t hoursValue_;
  uint16_t timeOffsetLength_;
  int16_t  timeOffsetValue_;
};

class PCCSEI {
 public:
  PCCSEI() {
    seiPrefix_.clear();
    seiSuffix_.clear();
  }
  ~PCCSEI() {
    seiPrefix_.clear();
    seiSuffix_.clear();
  }

  SEI& addSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    std::shared_ptr<SEI> sharedPtr;
    switch ( payloadType ) {
      case BUFFERING_PERIOD: sharedPtr = std::make_shared<SEIBufferingPeriod>(); break;
      case ATLAS_FRAME_TIMING: sharedPtr = std::make_shared<SEIAtlasFrameTiming>(); break;
      case FILLER_PAYLOAD: break;
      case USER_DATAREGISTERED_ITUTT35: sharedPtr = std::make_shared<SEIUserDataRegisteredItuTT35>(); break;
      case USER_DATA_UNREGISTERED: sharedPtr = std::make_shared<SEIUserDataUnregistered>(); break;
      case RECOVERY_POINT: sharedPtr = std::make_shared<SEIRecoveryPoint>(); break;
      case NO_RECONSTRUCTION: sharedPtr = std::make_shared<SEINoDisplay>(); break;
      case TIME_CODE: sharedPtr = std::make_shared<SEITimeCode>(); break;
      case SEI_MANIFEST: sharedPtr = std::make_shared<SEIManifest>(); break;
      case SEI_PREFIX_INDICATION: sharedPtr = std::make_shared<SEIPrefixIndication>(); break;
      case ACTIVE_SUB_BITSTREAMS: sharedPtr = std::make_shared<SEIActiveSubBitstreams>(); break;
      case COMPONENT_CODEC_MAPPING: sharedPtr = std::make_shared<SEIComponentCodecMapping>(); break;
      case SCENE_OBJECT_INFORMATION: sharedPtr = std::make_shared<SEISceneObjectInformation>(); break;
      case OBJECT_LABEL_INFORMATION: sharedPtr = std::make_shared<SEIObjectLabelInformation>(); break;
      case PATCH_INFORMATION: sharedPtr = std::make_shared<SEIPatchInformation>(); break;
      case VOLUMETRIC_RECTANGLE_INFORMATION: sharedPtr = std::make_shared<SEIVolumetricRectangleInformation>(); break;
      case ATLAS_OBJECT_INFORMATION: sharedPtr = std::make_shared<SEIAtlasInformation>(); break;
      case VIEWPORT_CAMERA_PARAMETERS: sharedPtr = std::make_shared<SEIViewportCameraParameters>(); break;
      case VIEWPORT_POSITION: sharedPtr = std::make_shared<SEIViewportPosition>(); break;
      case DECODED_ATLAS_INFORMATION_HASH: sharedPtr = std::make_shared<SEIDecodedAtlasInformationHash>(); break;
      case ATTRIBUTE_TRANSFORMATION_PARAMS: sharedPtr = std::make_shared<SEIAttributeTransformationParams>(); break;
      case OCCUPANCY_SYNTHESIS: sharedPtr = std::make_shared<SEIOccupancySynthesis>(); break;
      case GEOMETRY_SMOOTHING: sharedPtr = std::make_shared<SEIGeometrySmoothing>(); break;
      case ATTRIBUTE_SMOOTHING: sharedPtr = std::make_shared<SEIAttributeSmoothing>(); break;
      case RESERVED_SEI_MESSAGE: sharedPtr = std::make_shared<SEIReservedSeiMessage>(); break;
      default:
        fprintf( stderr, "SEI payload type not supported \n" );
        exit( -1 );
        break;
    }
    if ( nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ) {
      seiPrefix_.push_back( sharedPtr );
      return *( seiPrefix_.back().get() );
    } else if ( nalUnitType == NAL_SUFFIX_ESEI || nalUnitType == NAL_SUFFIX_NSEI ) {
      seiSuffix_.push_back( sharedPtr );
      return *( seiSuffix_.back().get() );
    } else {
      fprintf( stderr, "Nal unit type of SEI not correct\n" );
      exit( -1 );
    }
    return *( sharedPtr );
  }
  bool seiIsPresent( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    if ( nalUnitType != NAL_PREFIX_ESEI && nalUnitType != NAL_SUFFIX_ESEI && nalUnitType != NAL_PREFIX_NSEI &&
         nalUnitType != NAL_SUFFIX_NSEI ) {
      return false;
    }
    for ( auto& sei : nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ? seiPrefix_ : seiSuffix_ ) {
      if ( sei->getPayloadType() == payloadType ) { return true; }
    }
    return false;
  }
  SEI* getSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    auto& seis = ( nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ) ? seiPrefix_ : seiSuffix_;
    for ( auto& sei : seis ) {
      if ( sei->getPayloadType() == payloadType ) { return sei.get(); }
    }
    assert( 0 );
    return (SEI*)nullptr;
  }

  SEI* getLastSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    auto& seis = ( nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ) ? seiPrefix_ : seiSuffix_;
    for ( auto sei = seis.rbegin(); sei != seis.rend(); ++sei ) {
      if ( sei->get()->getPayloadType() == payloadType ) { return sei->get(); }
    }
    assert( 0 );
    return (SEI*)nullptr;
  }
  SEI& addSeiPrefix( SeiPayloadType payloadType, bool essensial ) {
    return addSei( essensial ? NAL_PREFIX_ESEI : NAL_PREFIX_NSEI, payloadType );
  }
  SEI& addSeiSuffix( SeiPayloadType payloadType, bool essensial ) {
    return addSei( essensial ? NAL_SUFFIX_ESEI : NAL_SUFFIX_NSEI, payloadType );
  }
  void addSeiToSeiSuffix( SeiPayloadType payloadType, bool essensial, SEIDecodedAtlasInformationHash& seiContext ) {
    seiSuffix_.push_back( std::make_shared<SEIDecodedAtlasInformationHash>( seiContext ) );
  }

  std::vector<std::shared_ptr<SEI>>& getSeiPrefix() { return seiPrefix_; }
  std::vector<std::shared_ptr<SEI>>& getSeiSuffix() { return seiSuffix_; }
  SEI&                               getSeiPrefix( size_t index ) { return *( seiPrefix_[index] ); }
  SEI&                               getSeiSuffix( size_t index ) { return *( seiSuffix_[index] ); }

 private:
  std::vector<std::shared_ptr<SEI>> seiPrefix_;
  std::vector<std::shared_ptr<SEI>> seiSuffix_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_SEI_H
