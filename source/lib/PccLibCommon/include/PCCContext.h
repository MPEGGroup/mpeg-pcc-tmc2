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
#include "PCCMetadata.h"
#include "PCCVideoBitstream.h"
#include <map>

namespace pcc {

class PCCGroupOfFrames;
class PCCFrameContext;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoTexture;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoOccupancyMap;

class PCCPatch;
typedef std::map<size_t, PCCPatch>                  unionPatch;     // [TrackIndex, PatchUnion]
typedef std::pair<size_t, size_t>                   GlobalPatch;    // [FrameIndex, PatchIndex]
typedef std::map<size_t, std::vector<GlobalPatch> > GlobalPatches;  // [TrackIndex, <GlobalPatch>]
typedef std::pair<size_t, size_t>                   SubContext;     // [start, end)

// 7.3.28  Reference list structure syntax
class RefListStruct {
 public:
  RefListStruct() : numRefEntries_( 0 ) {
    absDeltaPfocSt_.clear();
    pfocLsbLt_.clear();
    stRefPatchFrameFlag_.clear();
    strpfEntrySignFlag_.clear();
  }
  ~RefListStruct() {
    absDeltaPfocSt_.clear();
    pfocLsbLt_.clear();
    stRefPatchFrameFlag_.clear();
    strpfEntrySignFlag_.clear();
  }
  RefListStruct& operator=( const RefListStruct& ) = default;

  uint8_t&              getNumRefEntries() { return numRefEntries_; }
  std::vector<uint8_t>& getAbsDeltaPfocSt() { return absDeltaPfocSt_; }
  std::vector<uint8_t>& getPfocLsbLt() { return pfocLsbLt_; }
  std::vector<bool>&    getStRefPatchFrameFlag() { return stRefPatchFrameFlag_; }
  std::vector<bool>&    getStrpfEntrySignFlag() { return strpfEntrySignFlag_; }

  void allocate() {
    absDeltaPfocSt_.resize( numRefEntries_, 0 );
    pfocLsbLt_.resize( numRefEntries_, 0 );
    stRefPatchFrameFlag_.resize( numRefEntries_, false );
    strpfEntrySignFlag_.resize( numRefEntries_, false );
  }

 private:
  uint8_t              numRefEntries_;
  std::vector<uint8_t> absDeltaPfocSt_;
  std::vector<uint8_t> pfocLsbLt_;
  std::vector<bool>    stRefPatchFrameFlag_;
  std::vector<bool>    strpfEntrySignFlag_;
};

// 7.3.27  Patch frame header syntax
class PatchFrameHeader {
 public:
  PatchFrameHeader() {}
  PatchFrameHeader& operator=( const PatchFrameHeader& ) = default;

 private:
};

// 7.3.26  Patch frame layer unit syntax
class PatchFrameLayerUnit {
 public:
  PatchFrameLayerUnit() {}
  PatchFrameLayerUnit& operator=( const PatchFrameLayerUnit& ) = default;

 private:
};

// 7.3.25  Patch frame parameter set syntax
class PatchFrameParameterSet {
 public:
  PatchFrameParameterSet() :
      patchFrameParameterSetId_( 0 ),
      patchSequenceParameterSetId_( 0 ),
      geometryPatchFrameParameterSetId_( 0 ),
      additionalLtPfocLsbLen_( 0 ),
      localOverrideGeometryPatchEnableFlag_( false ),
      patchOrientationPresentFlag_( false ) {
    localOverrideAttributePatchEnableFlag_.clear();
    attributePatchFrameParameterSetId_.clear();
  }
  ~PatchFrameParameterSet() {
    localOverrideAttributePatchEnableFlag_.clear();
    attributePatchFrameParameterSetId_.clear();
  }
  PatchFrameParameterSet& operator=( const PatchFrameParameterSet& ) = default;

  uint8_t getPatchFrameParameterSetId() { return patchFrameParameterSetId_; }
  uint8_t getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  uint8_t getGeometryPatchFrameParameterSetId() { return geometryPatchFrameParameterSetId_; }
  std::vector<uint8_t> getAttributePatchFrameParameterSetId() {
    return attributePatchFrameParameterSetId_;
  }
  uint8_t           getAdditionalLtPfocLsbLen() { return additionalLtPfocLsbLen_; }
  std::vector<bool> getLocalOverrideAttributePatchEnableFlag() {
    return localOverrideAttributePatchEnableFlag_;
  }
  bool getPatchOrientationPresentFlag() { return patchOrientationPresentFlag_; }
  bool getLocalOverrideGeometryPatchEnableFlag() { return localOverrideGeometryPatchEnableFlag_; }

  void setPatchFrameParameterSetId( uint8_t setIdx ) { patchFrameParameterSetId_ = setIdx; }
  void setPatchSequenceParameterSetId( uint8_t setIdx ) { patchSequenceParameterSetId_ = setIdx; }
  void setGeometryPatchFrameParameterSetId( uint8_t setIdx ) {
    geometryPatchFrameParameterSetId_ = setIdx;
  }
  void setAttributePatchFrameParameterSetId( std::vector<uint8_t> setIdxArray ) {
    attributePatchFrameParameterSetId_ = setIdxArray;
  }
  void setAdditionalLtPfocLsbLen( uint8_t len ) { additionalLtPfocLsbLen_ = len; }
  void setLocalOverrideAttributePatchEnableFlag( std::vector<bool> flagArray ) {
    localOverrideAttributePatchEnableFlag_ = flagArray;
  }
  void setLocalOverrideAttributePatchEnableFlag( size_t index, bool flag ) {
    localOverrideAttributePatchEnableFlag_[index] = flag;
  }
  void setPatchOrientationPresentFlag( bool flag ) { patchOrientationPresentFlag_ = flag; }
  void setLocalOverrideGeometryPatchEnableFlag( bool flag ) {
    localOverrideGeometryPatchEnableFlag_ = flag;
  }

 private:
  uint8_t              patchFrameParameterSetId_;
  uint8_t              patchSequenceParameterSetId_;
  uint8_t              geometryPatchFrameParameterSetId_;   
  std::vector<uint8_t> attributePatchFrameParameterSetId_;  // sps_attribute_count size
  uint8_t              additionalLtPfocLsbLen_;
  bool                 localOverrideGeometryPatchEnableFlag_;
  bool                 patchOrientationPresentFlag_;
  std::vector<bool>    localOverrideAttributePatchEnableFlag_;
};

// 7.3.24 Attribute patch params syntax
class AttributePatchParams {
 public:
  AttributePatchParams() :
      attributePatchScaleParamsPresentFlag_( false ),
      attributePatchOffsetParamsPresentFlag_( false ) {
    attributePatchScale_.clear();
    attributePatchOffset_.clear();
  }
  ~AttributePatchParams() {
    attributePatchScale_.clear();
    attributePatchOffset_.clear();
  }
  AttributePatchParams& operator=( const AttributePatchParams& ) = default;
  bool getAttributePatchScaleParamsPresentFlag() { return attributePatchScaleParamsPresentFlag_; }
  bool getAttributePatchOffsetParamsPresentFlag() { return attributePatchOffsetParamsPresentFlag_; }
  std::vector<uint32_t> getAttributePatchScale() { return attributePatchScale_; }
  std::vector<int32_t>  getAttributePatchOffset() { return attributePatchOffset_; }
  uint32_t getAttributePatchScale( size_t index ) { return attributePatchScale_[index]; }
  int32_t  getAttributePatchOffset( size_t index ) { return attributePatchOffset_[index]; }

  void setAttributePatchScaleParamsPresentFlag( bool flag ) {
    attributePatchScaleParamsPresentFlag_ = flag;
  }
  void setAttributePatchOffsetParamsPresentFlag( bool flag ) {
    attributePatchOffsetParamsPresentFlag_ = flag;
  }
  void setAttributePatchScale( std::vector<uint32_t> scaleArray ) {
    attributePatchScale_ = scaleArray;
  }
  void setAttributePatchOffset( std::vector<int32_t> offsetArray ) {
    attributePatchOffset_ = offsetArray;
  }
  void setAttributePatchScale( size_t index, uint32_t scale ) {
    attributePatchScale_[index] = scale;
  }
  void setAttributePatchOffset( size_t index, int32_t offset ) {
    attributePatchOffset_[index] = offset;
  }

 private:
  bool                  attributePatchScaleParamsPresentFlag_;   
  std::vector<uint32_t> attributePatchScale_;                    
  bool                  attributePatchOffsetParamsPresentFlag_;  
  std::vector<int32_t>  attributePatchOffset_;                   
};

// 7.3.23 Attribute Patch Parameter Set syntax
class AttributePatchParameterSet {
 public:
  AttributePatchParameterSet() :

      attributePatchParameterSetId_( 0 ),
      attributeFrameParameterSetId_( 0 ),
      // dimension_( 3 ),
      attributePatchParamsPresentFlag_( false ) {}
  AttributePatchParameterSet& operator=( const AttributePatchParameterSet& ) = default;

  uint8_t getAttributePatchParameterSetId() { return attributePatchParameterSetId_; }
  uint8_t getAttributeFrameParameterSetId() { return attributeFrameParameterSetId_; }
  // uint8_t getDimension() { return dimension_; }
  bool    getAttributePatchParamsPresentFlag() { return attributePatchParamsPresentFlag_; }
  AttributePatchParams& getAttributePatchParams() { return attributePatchParams_; }

  void setAttributePatchParameterSetId( uint8_t setId ) { attributePatchParameterSetId_ = setId; }
  void setAttributeFrameParameterSetId( uint8_t setId ) { attributeFrameParameterSetId_ = setId; }
  // void setDimension( uint8_t dimension ) { dimension_ = dimension; }
  void setAttributePatchParamsPresentFlag( bool flag ) { attributePatchParamsPresentFlag_ = flag; }
  void setAttributePatchParams( AttributePatchParams params ) { attributePatchParams_ = params; }

 private:
  uint8_t              attributePatchParameterSetId_;  
  uint8_t              attributeFrameParameterSetId_;  
  // uint8_t              dimension_;                  //JR: must be check with JK 
  bool                 attributePatchParamsPresentFlag_;
  AttributePatchParams attributePatchParams_;
};

// 7.3.22 Geometry patch params syntax
class GeometryPatchParams {
 public:
  GeometryPatchParams() :
      geometryPatchScaleParamsPresentFlag_( false ),
      geometryPatchOffsetParamsPresentFlag_( false ),
      geometryPatchRotationParamsPresentFlag_( false ),
      geometryPatchPointSizeInfoPresentFlag_( false ),
      geometryPatchPointShapeInfoPresentFlag_( false ),
      geometryPatchPointSizeInfo_( 0 ),
      geometryPatchPointShapeInfo_( 0 ) {
    for ( size_t d = 0; d < 3; d++ ) {
      geometryPatchScaleOnAxis_[d]    = 0;  // u(32)
      geometryPatchOffsetOnAxis_[d]   = 0;  // i(32)
      geometryPatchRotationOnAxis_[d] = 0;  // i(32)
    }
  }
  GeometryPatchParams& operator=( const GeometryPatchParams& ) = default;

  bool getGeometryPatchScaleParamsPresentFlag() { return geometryPatchScaleParamsPresentFlag_; }
  bool getGeometryPatchOffsetParamsPresentFlag() { return geometryPatchOffsetParamsPresentFlag_; }
  bool getGeometryPatchRotationParamsPresentFlag() {
    return geometryPatchRotationParamsPresentFlag_;
  }
  bool getGeometryPatchPointSizeInfoPresentFlag() { return geometryPatchPointSizeInfoPresentFlag_; }
  bool getGeometryPatchPointShapeInfoPresentFlag() {
    return geometryPatchPointShapeInfoPresentFlag_;
  }
  uint32_t getGeometryPatchScaleOnAxis( size_t index ) { return geometryPatchScaleOnAxis_[index]; }
  int32_t getGeometryPatchOffsetOnAxis( size_t index ) { return geometryPatchOffsetOnAxis_[index]; }
  int32_t getGeometryPatchRotationOnAxis( size_t index ) {
    return geometryPatchRotationOnAxis_[index];
  }
  uint16_t getGeometryPatchPointSizeInfo() { return geometryPatchPointSizeInfo_; }
  uint32_t getGeometryPatchPointShapeInfo() { return geometryPatchPointShapeInfo_; }

  void setGeometryPatchScaleParamsPresentFlag( bool flag ) {
    geometryPatchScaleParamsPresentFlag_ = flag;
  }
  void setGeometryPatchOffsetParamsPresentFlag( bool flag ) {
    geometryPatchOffsetParamsPresentFlag_ = flag;
  }
  void setGeometryPatchRotationParamsPresentFlag( bool flag ) {
    geometryPatchRotationParamsPresentFlag_ = flag;
  }
  void setGeometryPatchPointSizeInfoPresentFlag( bool flag ) {
    geometryPatchPointSizeInfoPresentFlag_ = flag;
  }
  void setGeometryPatchPointShapeInfoPresentFlag( bool flag ) {
    geometryPatchPointShapeInfoPresentFlag_ = flag;
  }
  void setGeometryPatchScaleOnAxis( size_t index, uint32_t scale ) {
    geometryPatchScaleOnAxis_[index] = scale;
  }
  void setGeometryPatchOffsetOnAxis( size_t index, int32_t offset ) {
    geometryPatchOffsetOnAxis_[index] = offset;
  }
  void setGeometryPatchRotationOnAxis( size_t index, int32_t rotation ) {
    geometryPatchRotationOnAxis_[index] = rotation;
  }
  void setGeometryPatchPointSizeInfo( uint16_t pointSize ) {
    geometryPatchPointSizeInfo_ = pointSize;
  }
  void setGeometryPatchPointShapeInfo( uint32_t pointShape ) {
    geometryPatchPointShapeInfo_ = pointShape;
  }

 private:
  bool     geometryPatchScaleParamsPresentFlag_;
  bool     geometryPatchOffsetParamsPresentFlag_;
  bool     geometryPatchRotationParamsPresentFlag_;
  bool     geometryPatchPointSizeInfoPresentFlag_;
  bool     geometryPatchPointShapeInfoPresentFlag_;
  uint32_t geometryPatchScaleOnAxis_[3];     
  int32_t  geometryPatchOffsetOnAxis_[3];    
  int32_t  geometryPatchRotationOnAxis_[3];  
  uint16_t geometryPatchPointSizeInfo_;      
  uint32_t geometryPatchPointShapeInfo_;     
};

// 7.3.21 Geometry patch parameter set syntax
class GeometryPatchParameterSet {
 public:
  GeometryPatchParameterSet() :
      geometryPatchParameterSetId_( 0 ),
      geometryFrameParameterSetId_( 0 ),
      geometryPatchParamsPresentFlag_( false ) {}
  GeometryPatchParameterSet& operator=( const GeometryPatchParameterSet& ) = default;
  uint8_t getGeometryPatchParameterSetId() { return geometryPatchParameterSetId_; }
  uint8_t getGeometryFrameParameterSetId() { return geometryFrameParameterSetId_; }
  bool    getGeometryPatchParamsPresentFlag() { return geometryPatchParamsPresentFlag_; }
  GeometryPatchParams& getGeometryPatchParams() { return geometryPatchParams_; }
  void setGeometryPatchParameterSetId( uint8_t setId ) { geometryPatchParameterSetId_ = setId; }
  void setGeometryFrameParameterSetId( uint8_t setId ) { geometryFrameParameterSetId_ = setId; }
  void setGeometryPatchParamsPresentFlag( bool flag ) { geometryPatchParamsPresentFlag_ = flag; }
  void setGeometryPatchParams( GeometryPatchParams params ) { geometryPatchParams_ = params; }

 private:
  uint8_t             geometryPatchParameterSetId_;  
  uint8_t             geometryFrameParameterSetId_;  
  bool                geometryPatchParamsPresentFlag_;
  GeometryPatchParams geometryPatchParams_;
};

// 7.3.20 Attribute frame paramse syntax
class AttributeFrameParams {
 public:
  AttributeFrameParams() :
      attributeSmoothingParamsPresentFlag_( false ),
      attributeScaleParamsPresentFlag_( false ),
      attributeOffsetParamsPresentFlag_( false ),
      attributeSmoothingRadius_( 0 ),
      attributeSmoothingNeighbourCount_( 0 ),
      attributeSmoothingRadius2BoundaryDetection_( 0 ),
      attributeSmoothingThreshold_( 0 ),
      attributeSmoothingThresholdLocalEntropy_( 0 ),
      attributeDimension_( 0 ) {
    attributeScale_.clear();
    attributeOffset_.clear();
  }
  AttributeFrameParams& operator=( const AttributeFrameParams& ) = default;
  bool    getAttributeSmoothingParamsPresentFlag() { return attributeSmoothingParamsPresentFlag_; }
  bool    getAttributeScaleParamsPresentFlag() { return attributeScaleParamsPresentFlag_; }
  bool    getAttributeOffsetParamsPresentFlag() { return attributeOffsetParamsPresentFlag_; }
  uint8_t getAttributeSmoothingRadius() { return attributeSmoothingRadius_; }
  uint8_t getAttributeSmoothingNeighbourCount() { return attributeSmoothingNeighbourCount_; }
  uint8_t getAttributeSmoothingRadius2BoundaryDetection() {
    return attributeSmoothingRadius2BoundaryDetection_;
  }
  uint8_t getAttributeSmoothingThreshold() { return attributeSmoothingThreshold_; }
  uint32_t getAttributeSmoothingThresholdLocalEntropy() {
    return attributeSmoothingThresholdLocalEntropy_;
  }  
  std::vector<uint32_t> getAttributeScale() { return attributeScale_; }
  std::vector<int32_t>  getAttributeOffset() { return attributeOffset_; }
  uint32_t              getAttributeScale( size_t index ) { return attributeScale_[index]; }
  int32_t               getAttributeOffset( size_t index ) { return attributeOffset_[index]; }

  void setAttributeSmoothingParamsPresentFlag( bool flag ) {
    attributeSmoothingParamsPresentFlag_ = flag;
  }
  void setAttributeScaleParamsPresentFlag( bool flag ) { attributeScaleParamsPresentFlag_ = flag; }
  void setAttributeOffsetParamsPresentFlag( bool flag ) {
    attributeOffsetParamsPresentFlag_ = flag;
  }
  void setAttributeSmoothingRadius( uint8_t radius ) { attributeSmoothingRadius_ = radius; }
  void setAttributeSmoothingNeighbourCount( uint8_t count ) {
    attributeSmoothingNeighbourCount_ = count;
  }
  void setAttributeSmoothingRadius2BoundaryDetection( uint8_t boundaryDetection ) {
    attributeSmoothingRadius2BoundaryDetection_ = boundaryDetection;
  }
  void setAttributeSmoothingThreshold( uint8_t threshold ) {
    attributeSmoothingThreshold_ = threshold;
  }
  void setAttributeSmoothingThresholdLocalEntropy( uint32_t thresholdLocalEntropy ) {
    attributeSmoothingThresholdLocalEntropy_ = thresholdLocalEntropy;
  }
  void setAttributeScale( std::vector<uint32_t> scaleArray ) { attributeScale_ = scaleArray; }
  void setAttributeOffset( std::vector<int32_t> offsetArray ) { attributeOffset_ = offsetArray; }
  void setAttributeScale( size_t index, uint32_t scale ) { attributeScale_[index] = scale; }
  void setAttributeOffset( size_t index, int32_t offset ) { attributeOffset_[index] = offset; }
  void setDimension( size_t dimension ) { attributeDimension_ = dimension; }

 private:
  bool                  attributeSmoothingParamsPresentFlag_;
  bool                  attributeScaleParamsPresentFlag_;
  bool                  attributeOffsetParamsPresentFlag_;
  uint8_t               attributeSmoothingRadius_;
  uint8_t               attributeSmoothingNeighbourCount_;
  uint8_t               attributeSmoothingRadius2BoundaryDetection_;
  uint8_t               attributeSmoothingThreshold_;
  uint32_t              attributeSmoothingThresholdLocalEntropy_;  // u3
  std::vector<uint32_t> attributeScale_;
  std::vector<int32_t>  attributeOffset_;
  size_t                attributeDimension_;
};

// 7.3.19 Attribute frame parameter set syntax
class AttributeFrameParameterSet {
 public:
  AttributeFrameParameterSet() :
      attributeFrameParameterSetId_( 0 ),
      patchSequencParameterSetId_( 0 ),
      attributeDimensionMinus1_( 3 ),
      overrideAttributeParamsFlag_( false ),
      overrideAttributePatchParamsFlag_( false ),
      attributePatchScaleParamsEnabledFlag_( false ),
      attributePatchOffsetParamsEnabledFlag_( false ) {
    attributeFrameParams_.setDimension(
        attributeDimensionMinus1_ + 1 );  // jkei : is it correct?? attributeDimensionMinus1_+1?? => JR add 1
  }
  AttributeFrameParameterSet& operator=( const AttributeFrameParameterSet& ) = default;
  uint8_t getAttributeFrameParameterSetId() { return attributeFrameParameterSetId_; }
  uint8_t getPatchSequencParameterSetId() { return patchSequencParameterSetId_; }
  uint8_t getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
  bool    getOverrideAttributeParamsFlag() { return overrideAttributeParamsFlag_; }
  bool    getOverrideAttributePatchParamsFlag() { return overrideAttributePatchParamsFlag_; }
  bool getAttributePatchScaleParamsEnabledFlag() { return attributePatchScaleParamsEnabledFlag_; }
  bool getAttributePatchOffsetParamsEnabledFlag() { return attributePatchOffsetParamsEnabledFlag_; }
  AttributeFrameParams& getAttributeFrameParams() { return attributeFrameParams_; }

  void setAttributeFrameParameterSetId( uint8_t setId ) { attributeFrameParameterSetId_ = setId; }
  void setPatchSequencParameterSetId( uint8_t setId ) { patchSequencParameterSetId_ = setId; }
  void setAttributeDimensionMinus1( uint8_t dimensionMinus1 ) {
    attributeDimensionMinus1_ = dimensionMinus1;
  }
  void setOverrideAttributeParamsFlag( bool flag ) { overrideAttributeParamsFlag_ = flag; }
  void setOverrideAttributePatchParamsFlag( bool flag ) {
    overrideAttributePatchParamsFlag_ = flag;
  }
  void setAttributePatchScaleParamsEnabledFlag( bool flag ) {
    attributePatchScaleParamsEnabledFlag_ = flag;
  }
  void setAttributePatchOffsetParamsEnabledFlag( bool flag ) {
    attributePatchOffsetParamsEnabledFlag_ = flag;
  }
  void setAttributeFrameParams( AttributeFrameParams params ) { attributeFrameParams_ = params; }

 private:
  uint8_t attributeFrameParameterSetId_;   
  uint8_t patchSequencParameterSetId_;     
  uint8_t attributeDimensionMinus1_;       
  bool    overrideAttributeParamsFlag_;
  bool    overrideAttributePatchParamsFlag_;
  bool    attributePatchScaleParamsEnabledFlag_;
  bool    attributePatchOffsetParamsEnabledFlag_;
  AttributeFrameParams attributeFrameParams_;
};

// 7.3.18 Geometry frame params syntax
class GeometryFrameParams {
 public:
  GeometryFrameParams() :
      geometrySmoothingParamsPresentFlag_( false ),
      geometryScaleParamsPresentFlag_( false ),
      geometryOffsetParamsPresentFlag_( false ),
      geometryRotationParamsPresentFlag_( false ),
      geometryPointSizeInfoPresentFlag_( false ),
      geometryPointShapeInfoPresentFlag_( false ),
      geometrySmoothingEnabledFlag_( false ),
      geometrySmoothingGridSize_( 0 ),
      geometrySmoothingThreshold_( 0 ),
      geometryPointSizeInfo_( 0 ),
      geometryPointShapeInfo_( 0 ) {
    for ( size_t d = 0; d < 3; d++ ) {
      geometryScaleOnAxis_[d]    = 0;
      geometryOffsetOnAxis_[d]   = 0;
      geometryRotationOnAxis_[d] = 0;
    }
  }
  GeometryFrameParams& operator=( const GeometryFrameParams& ) = default;

  bool     getGeometrySmoothingParamsPresentFlag() { return geometrySmoothingParamsPresentFlag_; }
  bool     getGeometryScaleParamsPresentFlag() { return geometryScaleParamsPresentFlag_; }
  bool     getGeometryOffsetParamsPresentFlag() { return geometryOffsetParamsPresentFlag_; }
  bool     getGeometryRotationParamsPresentFlag() { return geometryRotationParamsPresentFlag_; }
  bool     getGeometryPointSizeInfoPresentFlag() { return geometryPointSizeInfoPresentFlag_; }
  bool     getGeometryPointShapeInfoPresentFlag() { return geometryPointShapeInfoPresentFlag_; }
  bool     getGeometrySmoothingEnabledFlag() { return geometrySmoothingEnabledFlag_; }
  uint8_t  getGeometrySmoothingGridSize() { return geometrySmoothingGridSize_; }
  uint8_t  getGeometrySmoothingThreshold() { return geometrySmoothingThreshold_; }
  uint32_t getGeometryScaleOnAxis( size_t index ) { return geometryScaleOnAxis_[index]; }
  int32_t  getGeometryOffsetOnAxis( size_t index ) { return geometryOffsetOnAxis_[index]; }
  int32_t  getGeometryRotationOnAxis( size_t index ) { return geometryRotationOnAxis_[index]; }
  uint16_t getGeometryPointSizeInfo() { return geometryPointSizeInfo_; }
  uint32_t getGeometryPointShapeInfo() { return geometryPointShapeInfo_; }

  void setGeometrySmoothingParamsPresentFlag( bool flag ) {
    geometrySmoothingParamsPresentFlag_ = flag;
  }
  void setGeometryScaleParamsPresentFlag( bool flag ) { geometryScaleParamsPresentFlag_ = flag; }
  void setGeometryOffsetParamsPresentFlag( bool flag ) { geometryOffsetParamsPresentFlag_ = flag; }
  void setGeometryRotationParamsPresentFlag( bool flag ) {
    geometryRotationParamsPresentFlag_ = flag;
  }
  void setGeometryPointSizeInfoPresentFlag( bool flag ) {
    geometryPointSizeInfoPresentFlag_ = flag;
  }
  void setGeometryPointShapeInfoPresentFlag( bool flag ) {
    geometryPointShapeInfoPresentFlag_ = flag;
  }
  void setGeometrySmoothingEnabledFlag( bool flag ) { geometrySmoothingEnabledFlag_ = flag; }
  void setGeometrySmoothingGridSize( uint8_t gridSize ) { geometrySmoothingGridSize_ = gridSize; }
  void setGeometrySmoothingThreshold( uint8_t threshold ) {
    geometrySmoothingThreshold_ = threshold;
  }
  void setGeometryScaleOnAxis( size_t index, uint32_t scale ) {
    geometryScaleOnAxis_[index] = scale;
  }
  void setGeometryOffsetOnAxis( size_t index, int32_t offset ) {
    geometryOffsetOnAxis_[index] = offset;
  }
  void setGeometryRotationOnAxis( size_t index, int32_t rotation ) {
    geometryRotationOnAxis_[index] = rotation;
  }
  void setGeometryPointSizeInfo( uint16_t pointSize ) { geometryPointSizeInfo_ = pointSize; }
  void setGeometryPointShapeInfo( uint32_t pointShape ) { geometryPointShapeInfo_ = pointShape; }

 private:
  bool     geometrySmoothingParamsPresentFlag_;
  bool     geometryScaleParamsPresentFlag_;
  bool     geometryOffsetParamsPresentFlag_;
  bool     geometryRotationParamsPresentFlag_;
  bool     geometryPointSizeInfoPresentFlag_;
  bool     geometryPointShapeInfoPresentFlag_;
  bool     geometrySmoothingEnabledFlag_;
  uint8_t  geometrySmoothingGridSize_;
  uint8_t  geometrySmoothingThreshold_;
  uint32_t geometryScaleOnAxis_[3];     
  int32_t  geometryOffsetOnAxis_[3];    
  int32_t  geometryRotationOnAxis_[3];  
  uint16_t geometryPointSizeInfo_;      
  uint32_t geometryPointShapeInfo_;     
};

// 7.3.17 Geometry frame parameter set syntax
class GeometryFrameParameterSet {
 public:
  GeometryFrameParameterSet() :
      geometryFrameParameterSetId_( 0 ),
      patchSequenceParameterSetId_( 0 ),
      geometryParamsEnabledFlag_( false ),
      overrideGeometryParamsFlag_( false ),
      geometryPatchParamsEnabledFlag_( false ),
      geometryPatchScaleParamsEnabledFlag_( false ),
      geometryPatchOffsetParamsEnabledFlag_( false ),
      geometryPatchRotationParamsEnabledFlag_( false ),
      geometryPatchPointSizeInfoEnabledFlag_( false ),
      geometryPatchPointShapeInfoEnabledFlag_( false ) {}
  GeometryFrameParameterSet& operator=( const GeometryFrameParameterSet& ) = default;
  uint8_t getGeometryFrameParameterSetId() { return geometryFrameParameterSetId_; }
  uint8_t getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  bool    getGeometryParamsEnabledFlag() { return geometryParamsEnabledFlag_; }
  bool    getOverrideGeometryParamsFlag() { return overrideGeometryParamsFlag_; }
  bool    getGeometryPatchParamsEnabledFlag() { return geometryPatchParamsEnabledFlag_; }
  bool    getOverrideGeometryPatchParamsFlag() { return overrideGeometryPatchParamsFlag_; }
  bool    getGeometryPatchScaleParamsEnabledFlag() { return geometryPatchScaleParamsEnabledFlag_; }
  bool getGeometryPatchOffsetParamsEnabledFlag() { return geometryPatchOffsetParamsEnabledFlag_; }
  bool getGeometryPatchRotationParamsEnabledFlag() {
    return geometryPatchRotationParamsEnabledFlag_;
  }
  bool getGeometryPatchPointSizeInfoEnabledFlag() { return geometryPatchPointSizeInfoEnabledFlag_; }
  bool getGeometryPatchPointShapeInfoEnabledFlag() {
    return geometryPatchPointShapeInfoEnabledFlag_;
  }
  GeometryFrameParams& getGeometryFrameParams() { return geometryFrameParams_; }

  void setGeometryFrameParameterSetId( uint8_t setId ) { geometryFrameParameterSetId_ = setId; }
  void setPatchSequenceParameterSetId( uint8_t setId ) { patchSequenceParameterSetId_ = setId; }
  void setGeometryParamsEnabledFlag( bool flag ) { geometryParamsEnabledFlag_ = flag; }
  void setOverrideGeometryParamsFlag( bool flag ) { overrideGeometryParamsFlag_ = flag; }
  void setGeometryPatchParamsEnabledFlag( bool flag ) { geometryPatchParamsEnabledFlag_ = flag; }
  void setOverrideGeometryPatchParamsFlag( bool flag ) { overrideGeometryPatchParamsFlag_ = flag; }
  void setGeometryPatchScaleParamsEnabledFlag( bool flag ) {
    geometryPatchScaleParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchOffsetParamsEnabledFlag( bool flag ) {
    geometryPatchOffsetParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchRotationParamsEnabledFlag( bool flag ) {
    geometryPatchRotationParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchPointSizeInfoEnabledFlag( bool flag ) {
    geometryPatchPointSizeInfoEnabledFlag_ = flag;
  }
  void setGeometryPatchPointShapeInfoEnabledFlag( bool flag ) {
    geometryPatchPointShapeInfoEnabledFlag_ = flag;
  }
  void setGeometryFrameParams( GeometryFrameParams geometryFrameParams ) {
    geometryFrameParams_ = geometryFrameParams;
  }

 private:
  uint8_t             geometryFrameParameterSetId_;
  uint8_t             patchSequenceParameterSetId_;
  bool                geometryParamsEnabledFlag_;
  bool                overrideGeometryParamsFlag_;
  bool                geometryPatchParamsEnabledFlag_;
  bool                overrideGeometryPatchParamsFlag_;
  bool                geometryPatchScaleParamsEnabledFlag_;
  bool                geometryPatchOffsetParamsEnabledFlag_;
  bool                geometryPatchRotationParamsEnabledFlag_;
  bool                geometryPatchPointSizeInfoEnabledFlag_;
  bool                geometryPatchPointShapeInfoEnabledFlag_;
  GeometryFrameParams geometryFrameParams_;
};

// 7.3.16  Patch sequence parameter set syntax
class PatchSequenceParameterSet {
 public:
  PatchSequenceParameterSet() :
      patchSequenceParameterSetId_( 0 ),
      log2MaxPatchFrameOrderCntLsb_( 0 ),
      maxDecPatchFrameBuffering_( 0 ),
      numRefPatchFrameListsInSps_( 0 ),
      longTermRefPatchFramesFlag_( false ) {
    refListStruct_.clear();
  }
  ~PatchSequenceParameterSet() { refListStruct_.clear(); }
  PatchSequenceParameterSet& operator=( const PatchSequenceParameterSet& ) = default;

  uint8_t        getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  uint8_t        getLog2MaxPatchFrameOrderCntLsbMinus4() { return log2MaxPatchFrameOrderCntLsb_; }
  uint8_t        getMaxDecPatchFrameBufferingMinus1() { return maxDecPatchFrameBuffering_; }
  uint8_t        getNumRefPatchFrameListsInSps() { return numRefPatchFrameListsInSps_; }
  bool           getLongTermRefPatchFramesFlag() { return longTermRefPatchFramesFlag_; }
  RefListStruct& getRefListStruct( uint8_t index ) { return refListStruct_[index]; }
  std::vector<RefListStruct>& getRefListStruct() { return refListStruct_; }

  void setPatchSequenceParameterSetId( uint8_t setIdx ) { patchSequenceParameterSetId_ = setIdx; }
  void setLog2MaxPatchFrameOrderCntLsbMinus4( uint8_t frameOrderCnt ) {
    log2MaxPatchFrameOrderCntLsb_ = frameOrderCnt;
  }
  void setMaxDecPatchFrameBufferingMinus1( uint8_t frameBuffering ) {
    maxDecPatchFrameBuffering_ = frameBuffering;
  }
  void setNumRefPatchFrameListsInSps( uint8_t frameListInSPS ) {
    numRefPatchFrameListsInSps_ = frameListInSPS;
  }
  void setLongTermRefPatchFramesFlag( bool flag ) { longTermRefPatchFramesFlag_ = flag; }
  void setRefListStruct( uint8_t index, RefListStruct refStruct ) {
    refListStruct_[index] = refStruct;
  }
  void setRefListStruct( std::vector<RefListStruct> refLit ) { refListStruct_ = refLit; }

 private:
  uint8_t                    patchSequenceParameterSetId_;
  uint8_t                    log2MaxPatchFrameOrderCntLsb_;
  uint8_t                    maxDecPatchFrameBuffering_;
  uint8_t                    numRefPatchFrameListsInSps_;
  bool                       longTermRefPatchFramesFlag_;
  std::vector<RefListStruct> refListStruct_;
};

// 7.3.15  Patch sequence unit payload syntax
class PatchSequenceUnitPayload {
 public:
  PatchSequenceUnitPayload() : unitType_( PSD_SPS ), frameIndex_( 0 ) {}
  PatchSequenceUnitPayload& operator=( const PatchSequenceUnitPayload& ) = default;

  PSDUnitType& getUnitType() { return unitType_; }
  uint8_t&     getFrameIndex() { return frameIndex_; }
  void         setUnitType( PSDUnitType unitType ) { unitType_ = unitType; }
  void         setFrameIndex( uint8_t idx ) { frameIndex_ = idx; }

  PatchSequenceParameterSet&  getPatchSequenceParameterSet() { return patchSequenceParameterSet_; }
  GeometryPatchParameterSet&  getGeometryPatchParameterSet() { return geometryPatchParameterSet_; }
  AttributePatchParameterSet& getAttributePatchParameterSet() {
    return attributePatchParameterSet_;
  }
  PatchFrameParameterSet&     getPatchFrameParameterSet() { return patchFrameParameterSet_; }
  AttributeFrameParameterSet& getAttributeFrameParameterSet() {
    return attributeFrameParameterSet_;
  }
  GeometryFrameParameterSet& getGeometryFrameParameterSet() { return geometryFrameParameterSet_; }
  PatchFrameLayerUnit&       getPatchFrameLayerUnit() { return patchFrameLayerUnit_; }

  void setPatchSequenceParameterSet( PatchSequenceParameterSet param ) {
    patchSequenceParameterSet_ = param;
  }
  void setGeometryPatchParameterSet( GeometryPatchParameterSet param ) {
    geometryPatchParameterSet_ = param;
  }
  void setAttributePatchParameterSet( AttributePatchParameterSet param ) {
    attributePatchParameterSet_ = param;
  }
  void setPatchFrameParameterSet( PatchFrameParameterSet param ) {
    patchFrameParameterSet_ = param;
  }
  void setAttributeFrameParameterSet( AttributeFrameParameterSet param ) {
    attributeFrameParameterSet_ = param;
  }
  void setGeometryFrameParameterSet( GeometryFrameParameterSet param ) {
    geometryFrameParameterSet_ = param;
  }
  void setPatchFrameLayerUnit( PatchFrameLayerUnit param ) { patchFrameLayerUnit_ = param; }

 private:
  PSDUnitType                unitType_;
  uint8_t                    frameIndex_;
  PatchSequenceParameterSet  patchSequenceParameterSet_;
  GeometryPatchParameterSet  geometryPatchParameterSet_;
  AttributePatchParameterSet attributePatchParameterSet_;
  PatchFrameParameterSet     patchFrameParameterSet_;
  AttributeFrameParameterSet attributeFrameParameterSet_;
  GeometryFrameParameterSet  geometryFrameParameterSet_;
  PatchFrameLayerUnit        patchFrameLayerUnit_;
};

// 7.3.14  Patch sequence data unit syntax
class PatchSequenceDataUnit {
 public:
  PatchSequenceDataUnit() : frameCount_( 0 ) { patchSequenceUnitPayload_.clear(); }
  ~PatchSequenceDataUnit() { patchSequenceUnitPayload_.clear(); }

  PatchSequenceDataUnit& operator=( const PatchSequenceDataUnit& ) = default;

  uint8_t getPatchSequenceDataUnitSize() { return patchSequenceUnitPayload_.size(); }
  uint8_t getFrameCount( std::vector<PatchSequenceUnitPayload> patchSequenceUnitPayload ) {
    uint8_t frameCount = 0;
    for ( int payloadIndex = 0; payloadIndex < patchSequenceUnitPayload.size(); payloadIndex++ ) {
      frameCount += ( patchSequenceUnitPayload[payloadIndex].getUnitType() == PSD_PFLU );
    }
    return frameCount;
  }
  uint8_t&                               getFrameCount() { return frameCount_; }  // PSD_PFLU only
  std::vector<PatchSequenceUnitPayload>& getPatchSequenceUnitPayload() {
    return patchSequenceUnitPayload_;
  }
  void setFrameCount( uint8_t frameCount ) { frameCount_ = frameCount; }
  void setPatchSequenceUnitPayload( std::vector<PatchSequenceUnitPayload> unitPayload ) {
    patchSequenceUnitPayload_ = unitPayload;
  }
 private:
  uint8_t frameCount_;

  std::vector<PatchSequenceUnitPayload> patchSequenceUnitPayload_;
};

// 7.3.13 Attribute Sequence Params Syntax
class AttributeSequenceParams {
 public:
  AttributeSequenceParams() :
      attributeSmoothingParamsPresentFlag_( false ),
      attributeScaleParamsPresentFlag_( false ),
      attributeOffsetParamsPresentFlag_( false ),
      attributeSmoothingRadius_( 0 ),
      attributeSmoothingNeighbourCount_( 0 ),
      attributeSmoothingRadius2BoundaryDetection_( 0 ),
      attributeSmoothingThreshold_( 0 ),
      attributeSmoothingThresholdLocalEntropy_( 0 ) {
    attributeScale_.clear();
    attributeOffset_.clear();
  }
  AttributeSequenceParams& operator=( const AttributeSequenceParams& ) = default;

  void init( bool     smoothingParamsPresentFlag,
             bool     scaleParamsPresentFlag,
             bool     offsetParamsPresentFlag,
             uint8_t  smoothingRadius,
             uint8_t  smoothingNeighbourCount,
             uint8_t  smoothingRadius2BoundaryDetection,
             uint8_t  smoothingThreshold,
             uint32_t smoothingThresholdLocalEntropy ) {
    attributeSmoothingParamsPresentFlag_        = smoothingParamsPresentFlag;
    attributeScaleParamsPresentFlag_            = scaleParamsPresentFlag;
    attributeOffsetParamsPresentFlag_           = offsetParamsPresentFlag;
    attributeSmoothingRadius_                   = smoothingRadius;
    attributeSmoothingNeighbourCount_           = smoothingNeighbourCount;
    attributeSmoothingRadius2BoundaryDetection_ = smoothingRadius2BoundaryDetection;
    attributeSmoothingThreshold_                = smoothingThreshold;
    attributeSmoothingThresholdLocalEntropy_    = smoothingThresholdLocalEntropy;
  }
  bool    getAttributeSmoothingParamsPresentFlag() { return attributeSmoothingParamsPresentFlag_; }
  bool    getAttributeScaleParamsPresentFlag() { return attributeScaleParamsPresentFlag_; }
  bool    getAttributeOffsetParamsPresentFlag() { return attributeOffsetParamsPresentFlag_; }
  uint8_t getAttributeSmoothingRadius() { return attributeSmoothingRadius_; }
  uint8_t getAttributeSmoothingNeighbourCount() { return attributeSmoothingNeighbourCount_; }
  uint8_t getAttributeSmoothingRadius2BoundaryDetection() {
    return attributeSmoothingRadius2BoundaryDetection_;
  }
  uint8_t getAttributeSmoothingThreshold() { return attributeSmoothingThreshold_; }
  uint32_t getAttributeSmoothingThresholdLocalEntropy() {
    return attributeSmoothingThresholdLocalEntropy_;
  }
  std::vector<uint32_t>& getAttributeScaleParams() { return attributeScale_; }
  std::vector<int32_t>&  getAttributeOffsetParams() { return attributeOffset_; }

  void setAttributeSmoothingParamsPresentFlag( bool flag ) {
    attributeSmoothingParamsPresentFlag_ = flag;
  }
  void setAttributeScaleParamsPresentFlag( bool flag ) { attributeScaleParamsPresentFlag_ = flag; }
  void setAttributeOffsetParamsPresentFlag( bool flag ) {
    attributeOffsetParamsPresentFlag_ = flag;
  }
  void setAttributeSmoothingRadius( uint8_t flag ) { attributeSmoothingRadius_ = flag; }
  void setAttributeSmoothingNeighbourCount( uint8_t flag ) {
    attributeSmoothingNeighbourCount_ = flag;
  }
  void setAttributeSmoothingRadius2BoundaryDetection( uint8_t flag ) {
    attributeSmoothingRadius2BoundaryDetection_ = flag;
  }
  void setAttributeSmoothingThreshold( uint8_t threshold ) {
    attributeSmoothingThreshold_ = threshold;
  }
  void setAttributeSmoothingThresholdLocalEntropy( uint32_t localEntropy ) {
    attributeSmoothingThresholdLocalEntropy_ = localEntropy;
  }
  void setAttributeScaleParams( std::vector<uint32_t> params ) { attributeScale_ = params; }
  void setAttributeScaleParam( size_t index, uint32_t scale ) { attributeScale_[index] = scale; }
  void setAttributeOffsetParams( std::vector<int32_t> params ) { attributeOffset_ = params; }
  void setAttributeOffsetParam( size_t index, uint32_t offset ) {
    attributeOffset_[index] = offset;
  }

 private:
  bool                  attributeSmoothingParamsPresentFlag_;
  bool                  attributeScaleParamsPresentFlag_;
  bool                  attributeOffsetParamsPresentFlag_;
  uint8_t               attributeSmoothingRadius_;
  uint8_t               attributeSmoothingNeighbourCount_;
  uint8_t               attributeSmoothingRadius2BoundaryDetection_;
  uint8_t               attributeSmoothingThreshold_;
  uint32_t              attributeSmoothingThresholdLocalEntropy_;  
  std::vector<uint32_t> attributeScale_;
  std::vector<int32_t>  attributeOffset_;
};

// 7.3.12 Attribute Parameter Set Syntax
class AttributeParameterSet {
 public:
  AttributeParameterSet() :
      attributeTypeId_( 0 ),
      attributeDimensionMinus1_( 2 ),
      attributeCodecId_( 0 ),
      pcmAttributeCodecId_( 0 ),
      attributeParamsEnabledFlag_( false ),
      attributePatchParamsEnabledFlag_( false ),
      attributePatchScaleParamsEnabledFlag_( false ),
      attributePatchOffsetParamsEnabledFlag_( false ) {}

  AttributeParameterSet& operator=( const AttributeParameterSet& ) = default;

  uint8_t getAttributeTypeId() { return attributeTypeId_; }
  uint8_t getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
  uint8_t getAttributeCodecId() { return attributeCodecId_; }
  uint8_t getPcmAttributeCodecId() { return pcmAttributeCodecId_; }
  bool    getAttributeParamsEnabledFlag() { return attributeParamsEnabledFlag_; }
  bool    getAttributePatchParamsEnabledFlag() { return attributePatchParamsEnabledFlag_; }
  bool getAttributePatchScaleParamsEnabledFlag() { return attributePatchScaleParamsEnabledFlag_; }
  bool getAttributePatchOffsetParamsEnabledFlag() { return attributePatchOffsetParamsEnabledFlag_; }
  AttributeSequenceParams& getAttributeSequenceParams() { return attributeSequenceParams_; }

  void setAttributeTypeId( uint8_t typeId ) { attributeTypeId_ = typeId; }
  void setAttributeDimensionMinus1( uint8_t dimension ) { attributeDimensionMinus1_ = dimension; }
  void setAttributeCodecId( uint8_t codecIdx ) { attributeCodecId_ = codecIdx; }
  void setPcmAttributeCodecId( uint8_t codecIdx ) { pcmAttributeCodecId_ = codecIdx; }
  void setAttributeParamsEnabledFlag( bool flag ) { attributeParamsEnabledFlag_ = flag; }
  void setAttributePatchParamsEnabledFlag( bool flag ) { attributePatchParamsEnabledFlag_ = flag; }
  void setAttributePatchScaleParamsEnabledFlag( bool flag ) {
    attributePatchScaleParamsEnabledFlag_ = flag;
  }
  void setAttributePatchOffsetParamsEnabledFlag( bool flag ) {
    attributePatchOffsetParamsEnabledFlag_ = flag;
  }
  void setAttributeSequenceParams( AttributeSequenceParams params ) {
    attributeSequenceParams_ = params;
  }

 private:
  uint8_t                 attributeTypeId_;
  uint8_t                 attributeDimensionMinus1_;
  uint8_t                 attributeCodecId_;
  uint8_t                 pcmAttributeCodecId_;
  bool                    attributeParamsEnabledFlag_;
  bool                    attributePatchParamsEnabledFlag_;
  bool                    attributePatchScaleParamsEnabledFlag_;
  bool                    attributePatchOffsetParamsEnabledFlag_;
  AttributeSequenceParams attributeSequenceParams_;
};

// 7.3.11 Geometry Sequence Params Syntax
class GeometrySequenceParams {
 public:
  GeometrySequenceParams() :
      geometrySmoothingParamsPresentFlag_( false ),
      geometryScaleParamsPresentFlag_( false ),
      geometryOffsetParamsPresentFlag_( false ),
      geometryRotationParamsPresentFlag_( false ),
      geometryPointSizeInfoPresentFlag_( false ),
      geometryPointShapeInfoPresentFlag_( false ),
      geometrySmoothingEnableFlag_( false ),
      geometrySmoothingGridSize_( 0 ),
      geometrySmoothingThreshold_( 0 ),
      geometryPointSizeInfo_( 0 ),
      geometryPointShapeInfo_( 0 ) {
    for ( size_t d = 0; d < 3; d++ ) {
      geometryScaleOnAxis_[d]    = 0;
      geometryOffsetOnAxis_[d]   = 0;
      geometryRotationOnAxis_[d] = 0;
    }
  }
  GeometrySequenceParams& operator=( const GeometrySequenceParams& ) = default;
  void                    init( bool    smoothingParamsPresentFlag,
                                bool    scaleParamsPresentFlag,
                                bool    offsetParamsPresentFlag,
                                bool    rotationParamsPresentFlag,
                                bool    pointSizeInfoPresentFlag,
                                bool    pointShapeInfoPresentFlag,
                                bool    smoothingEnableFlag,
                                uint8_t smoothingGridSize,
                                uint8_t smoothingThreshold ) {
    geometrySmoothingParamsPresentFlag_ = smoothingParamsPresentFlag;
    geometryScaleParamsPresentFlag_     = scaleParamsPresentFlag;
    geometryOffsetParamsPresentFlag_    = offsetParamsPresentFlag;
    geometryRotationParamsPresentFlag_  = rotationParamsPresentFlag;
    geometryPointSizeInfoPresentFlag_   = pointSizeInfoPresentFlag;
    geometryPointShapeInfoPresentFlag_  = pointShapeInfoPresentFlag;
    geometrySmoothingEnableFlag_        = smoothingEnableFlag;
    geometrySmoothingGridSize_          = smoothingGridSize;
    geometrySmoothingThreshold_         = smoothingThreshold;
  }
  bool     getGeometrySmoothingParamsPresentFlag() { return geometrySmoothingParamsPresentFlag_; }
  bool     getGeometryScaleParamsPresentFlag() { return geometryScaleParamsPresentFlag_; }
  bool     getGeometryOffsetParamsPresentFlag() { return geometryOffsetParamsPresentFlag_; }
  bool     getGeometryRotationParamsPresentFlag() { return geometryRotationParamsPresentFlag_; }
  bool     getGeometryPointSizeInfoPresentFlag() { return geometryPointSizeInfoPresentFlag_; }
  bool     getGeometryPointShapeInfoPresentFlag() { return geometryPointShapeInfoPresentFlag_; }
  bool     getGeometrySmoothingEnabledFlag() { return geometrySmoothingEnableFlag_; }
  uint8_t  getGeometrySmoothingGridSize() { return geometrySmoothingGridSize_; }
  uint8_t  getGeometrySmoothingThreshold() { return geometrySmoothingThreshold_; }
  uint32_t getGeometryScaleOnAxis( int index ) { return geometryScaleOnAxis_[index]; }
  int32_t  getGeometryOffsetOnAxis( int index ) { return geometryOffsetOnAxis_[index]; }
  int32_t  getGeometryRotationOnAxis( int index ) { return geometryRotationOnAxis_[index]; }
  uint16_t getGeometryPointSizeInfo() { return geometryPointSizeInfo_; }
  uint32_t getGeometryPointShapeInfo() { return geometryPointShapeInfo_; }

  void setGeometrySmoothingParamsPresentFlag( bool flag ) {
    geometrySmoothingParamsPresentFlag_ = flag;
  }
  void setGeometryScaleParamsPresentFlag( bool flag ) { geometryScaleParamsPresentFlag_ = flag; }
  void setGeometryOffsetParamsPresentFlag( bool flag ) { geometryOffsetParamsPresentFlag_ = flag; }
  void setGeometryRotationParamsPresentFlag( bool flag ) {
    geometryRotationParamsPresentFlag_ = flag;
  }
  void setGeometryPointSizeInfoPresentFlag( bool flag ) {
    geometryPointSizeInfoPresentFlag_ = flag;
  }
  void setGeometryPointShapeInfoPresentFlag( bool flag ) {
    geometryPointShapeInfoPresentFlag_ = flag;
  }
  void setGeometrySmoothingEnabledFlag( bool flag ) { geometrySmoothingEnableFlag_ = flag; }
  void setGeometrySmoothingGridSize( uint8_t gridSize ) { geometrySmoothingGridSize_ = gridSize; }
  void setGeometrySmoothingThreshold( uint8_t threshold ) {
    geometrySmoothingThreshold_ = threshold;
  }
  void setGeometryScaleOnAxis( int index, uint32_t scale ) { geometryScaleOnAxis_[index] = scale; }
  void setGeometryOffsetOnAxis( int index, int32_t offset ) {
    geometryOffsetOnAxis_[index] = offset;
  }
  void setGeometryRotationOnAxis( int index, int32_t rotation ) {
    geometryRotationOnAxis_[index] = rotation;
  }
  void setGeometryPointSizeInfo( uint16_t pointSize ) { geometryPointSizeInfo_ = pointSize; }
  void setGeometryPointShapeInfo( uint32_t pointShape ) { geometryPointShapeInfo_ = pointShape; }

 private:
  bool     geometrySmoothingParamsPresentFlag_;
  bool     geometryScaleParamsPresentFlag_;
  bool     geometryOffsetParamsPresentFlag_;
  bool     geometryRotationParamsPresentFlag_;
  bool     geometryPointSizeInfoPresentFlag_;
  bool     geometryPointShapeInfoPresentFlag_;
  bool     geometrySmoothingEnableFlag_;
  uint8_t  geometrySmoothingGridSize_;
  uint8_t  geometrySmoothingThreshold_;
  uint32_t geometryScaleOnAxis_[3];     
  int32_t  geometryOffsetOnAxis_[3];    
  int32_t  geometryRotationOnAxis_[3];  
  uint16_t geometryPointSizeInfo_;      
  uint32_t geometryPointShapeInfo_;     
};

// 7.3.10 Geometry Parameter Set Syntax
class GeometryParameterSet {
 public:
  GeometryParameterSet() :
      geometryCodecId_( 0 ),
      geometryNominal2dBitdepthMinus1_( 0 ),
      geometry3dCoordinatesBitdepthMinus1_( 0 ),
      pcmGeometryCodecId_( 0 ),
      geometryParamsEnabledFlag_( false ),
      geometryPatchParamsEnabledFlag_( false ),
      geometryPatchScaleParamsEnabledFlag_( false ),
      geometryPatchOffsetParamsEnabledFlag_( false ),
      geometryPatchRotationParamsEnabledFlag_( false ),
      geometryPatchPointSizeInfoEnabledFlag_( false ),
      geometryPatchPointShapeInfoEnabledFlag_( false ) {}
  GeometryParameterSet& operator=( const GeometryParameterSet& ) = default;

  void init( uint8_t codecId,
             uint8_t nominal2dBitdepthMinus1,
             uint8_t coordinatesBitdepthMinus1,
             uint8_t pcmGeometryCodecId,
             bool    paramsEnabledFlag,
             bool    patchParamsEnabledFlag,
             bool    patchScaleParamsEnabledFlag,
             bool    patchOffsetParamsEnabledFlag,
             bool    patchRotationParamsEnabledFlag,
             bool    patchPointSizeInfoEnabledFlag,
             bool    patchPointShapeInfoEnabledFlag ) {
    geometryCodecId_                        = codecId;
    geometryNominal2dBitdepthMinus1_        = nominal2dBitdepthMinus1;
    geometry3dCoordinatesBitdepthMinus1_    = coordinatesBitdepthMinus1;
    pcmGeometryCodecId_                     = pcmGeometryCodecId;
    geometryParamsEnabledFlag_              = paramsEnabledFlag;
    geometryPatchParamsEnabledFlag_         = patchParamsEnabledFlag;
    geometryPatchScaleParamsEnabledFlag_    = patchParamsEnabledFlag;
    geometryPatchOffsetParamsEnabledFlag_   = patchOffsetParamsEnabledFlag;
    geometryPatchRotationParamsEnabledFlag_ = patchRotationParamsEnabledFlag;
    geometryPatchPointSizeInfoEnabledFlag_  = patchPointSizeInfoEnabledFlag;
    geometryPatchPointShapeInfoEnabledFlag_ = patchPointShapeInfoEnabledFlag;
  }
  uint8_t getGeometryCodecId() { return geometryCodecId_; }
  uint8_t getGeometryNominal2dBitdepthMinus1() { return geometryNominal2dBitdepthMinus1_; }
  uint8_t getGeometry3dCoordinatesBitdepthMinus1() { return geometry3dCoordinatesBitdepthMinus1_; }
  uint8_t getPcmGeometryCodecId() { return pcmGeometryCodecId_; }
  bool    getGeometryParamsEnabledFlag() { return geometryParamsEnabledFlag_; }
  bool    getGeometryPatchParamsEnabledFlag() { return geometryPatchParamsEnabledFlag_; }
  bool    getGeometryPatchScaleParamsEnabledFlag() { return geometryPatchScaleParamsEnabledFlag_; }
  bool getGeometryPatchOffsetParamsEnabledFlag() { return geometryPatchOffsetParamsEnabledFlag_; }
  bool getGeometryPatchRotationParamsEnabledFlag() {
    return geometryPatchRotationParamsEnabledFlag_;
  }
  bool getGeometryPatchPointSizeInfoEnabledFlag() { return geometryPatchPointSizeInfoEnabledFlag_; }
  bool getGeometryPatchPointShapeInfoEnabledFlag() {
    return geometryPatchPointShapeInfoEnabledFlag_;
  }
  GeometrySequenceParams& getGeometrySequenceParams() { return geometrySequenceParams_; }

  void setGeometryCodecId( uint8_t codecId ) { geometryCodecId_ = codecId; }
  void setGeometryNominal2dBitdepthMinus1( uint8_t bitdepth ) {
    geometryNominal2dBitdepthMinus1_ = bitdepth;
  }
  void setGeometry3dCoordinatesBitdepthMinus1( uint8_t bitdepth ) {
    geometry3dCoordinatesBitdepthMinus1_ = bitdepth;
  }
  void setPcmGeometryCodecId( uint8_t codecId ) { pcmGeometryCodecId_ = codecId; }
  void setGeometryParamsEnabledFlag( bool flag ) { geometryParamsEnabledFlag_ = flag; }
  void setGeometryPatchParamsEnabledFlag( bool flag ) { geometryPatchParamsEnabledFlag_ = flag; }
  void setGeometryPatchScaleParamsEnabledFlag( bool flag ) {
    geometryPatchScaleParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchOffsetParamsEnabledFlag( bool flag ) {
    geometryPatchOffsetParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchRotationParamsEnabledFlag( bool flag ) {
    geometryPatchRotationParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchPointSizeInfoEnabledFlag( bool flag ) {
    geometryPatchPointSizeInfoEnabledFlag_ = flag;
  }
  void setGeometryPatchPointShapeInfoEnabledFlag( bool flag ) {
    geometryPatchPointShapeInfoEnabledFlag_ = flag;
  }
  void setGeometrySequenceParams( GeometrySequenceParams params ) {
    geometrySequenceParams_ = params;
  }

 private:
  uint8_t                geometryCodecId_;
  uint8_t                geometryNominal2dBitdepthMinus1_;     
  uint8_t                geometry3dCoordinatesBitdepthMinus1_; 
  uint8_t                pcmGeometryCodecId_;
  bool                   geometryParamsEnabledFlag_;
  bool                   geometryPatchParamsEnabledFlag_;
  bool                   geometryPatchScaleParamsEnabledFlag_;
  bool                   geometryPatchOffsetParamsEnabledFlag_;
  bool                   geometryPatchRotationParamsEnabledFlag_;
  bool                   geometryPatchPointSizeInfoEnabledFlag_;
  bool                   geometryPatchPointShapeInfoEnabledFlag_;
  GeometrySequenceParams geometrySequenceParams_;
};

// 7.3.9 Occupancy Parameter Set Syntax
class OccupancyParameterSet {
 public:
  OccupancyParameterSet() : occupancyCodecId_( 0 ), occupancyPackingBlockSize_( 0 ) {}
  OccupancyParameterSet& operator=( const OccupancyParameterSet& ) = default;

  void init( uint8_t codecId, uint8_t packingBlockSize ) {
    occupancyCodecId_          = codecId;
    occupancyPackingBlockSize_ = packingBlockSize;
  }

  uint8_t getOccupancyCodecId() { return occupancyCodecId_; }
  uint8_t getOccupancyPackingBlockSize() { return occupancyPackingBlockSize_; }
  void    setOccupancyCodecId( uint8_t codecIdx ) { occupancyCodecId_ = codecIdx; }
  void setOccupancyPackingBlockSize( uint8_t blocksize ) { occupancyPackingBlockSize_ = blocksize; }

 private:
  uint8_t occupancyCodecId_;
  uint8_t occupancyPackingBlockSize_;
};

// 7.3.8 Profile, Tiere and Level Syntax
class ProfileTierLevel {
 public:
  ProfileTierLevel() : tierFlag_( false ), profileIdc_( 0 ), levelIdc_( 0 ) {}
  ProfileTierLevel& operator=( const ProfileTierLevel& ) = default;

  uint8_t getProfileIdc() { return profileIdc_; }
  bool    getTierFlag() { return tierFlag_; }
  uint8_t getLevelIdc() { return levelIdc_; }

  void setTierFlag( bool tier ) { tierFlag_ = tier; }
  void setProfileIdc( uint8_t profileIdx ) { profileIdc_ = profileIdx; }
  void setLevelIdc( uint8_t levelIdx ) { levelIdc_ = levelIdx; }

 private:
  bool    tierFlag_;
  uint8_t profileIdc_;
  uint8_t levelIdc_;
};

// 7.3.6  Sequence parameter set syntax
class SequenceParameterSet {
 public:
  SequenceParameterSet() :
      sequenceParameterSetId_( 0 ),
      frameWidth_( 0 ),
      frameHeight_( 0 ),
      avgFrameRate_( 0 ),
      layerCountMinus1_( 0 ),
      attributeCount_( 0 ),
      avgFrameRatePresentFlag_( false ),
      enhancedOccupancyMapForDepthFlag_( false ),
      multipleLayerStreamsPresentFlag_( false ),
      pcmPatchEnabledFlag_( false ),
      pcmSeparateVideoPresentFlag_( false ),
      patchSequenceOrientationEnabledFlag_( false ),
      patchInterPredictionEnabledFlag_( false ),
      pixelDeinterleavingFlag_( false ),
      pointLocalReconstructionEnabledFlag_( false ),
      removeDuplicatePointEnabledFlag_( false ) {
    layerAbsoluteCodingEnabledFlag_.clear();
    layerPredictorIndexDiff_.clear();
  }

  ~SequenceParameterSet() {
    layerAbsoluteCodingEnabledFlag_.clear();
    layerPredictorIndexDiff_.clear();
  }

  SequenceParameterSet& operator=( const SequenceParameterSet& ) = default;
  void                  init( uint32_t sequenceParameterSetId,
                              uint16_t frameWidth,
                              uint16_t frameHeight,
                              uint16_t avgFrameRate,
                              uint32_t layerCountMinus1,
                              uint16_t attributeCount,
                              bool     avgFrameRatePresentFlag,
                              bool     enhancedOccupancyMapForDepthFlag,
                              bool     multipleLayerStreamsPresentFlag,
                              bool     pcmPatchEnabledFlag,
                              bool     pcmSeparateVideoPresentFlag,
                              bool     patchSequenceOrientationEnabledFlag,
                              bool     patchInterPredictionEnabledFlag,
                              bool     pixelDeinterleavingFlag,
                              bool     pointLocalReconstructionEnabledFlag,
                              bool     removeDuplicatePointEnabledFlag ) {
    sequenceParameterSetId_              = sequenceParameterSetId;
    frameWidth_                          = frameWidth;
    frameHeight_                         = frameHeight;
    avgFrameRate_                        = avgFrameRate;
    layerCountMinus1_                    = layerCountMinus1;
    attributeCount_                      = attributeCount;
    avgFrameRatePresentFlag_             = avgFrameRatePresentFlag;
    enhancedOccupancyMapForDepthFlag_    = enhancedOccupancyMapForDepthFlag;
    multipleLayerStreamsPresentFlag_     = multipleLayerStreamsPresentFlag;
    pcmPatchEnabledFlag_                 = pcmPatchEnabledFlag;
    pcmSeparateVideoPresentFlag_         = pcmSeparateVideoPresentFlag;
    patchSequenceOrientationEnabledFlag_ = patchSequenceOrientationEnabledFlag;
    patchInterPredictionEnabledFlag_     = patchInterPredictionEnabledFlag;
    pixelDeinterleavingFlag_             = pixelDeinterleavingFlag;
    pointLocalReconstructionEnabledFlag_ = pointLocalReconstructionEnabledFlag;
    removeDuplicatePointEnabledFlag_     = removeDuplicatePointEnabledFlag;
    allocateAttributeParameterSets();
  }

  uint32_t getSequenceParameterSetId() { return sequenceParameterSetId_; }
  uint16_t getFrameWidth() { return frameWidth_; }
  uint16_t getFrameHeight() { return frameHeight_; }
  uint16_t getAvgFrameRate() { return avgFrameRate_; }
  uint32_t getLayerCountMinus1() { return layerCountMinus1_; }
  uint16_t getAttributeCount() { return attributeCount_; }
  bool     getAvgFrameRatePresentFlag() { return avgFrameRatePresentFlag_; }
  bool     getEnhancedOccupancyMapForDepthFlag() { return enhancedOccupancyMapForDepthFlag_; }
  bool     getMultipleLayerStreamsPresentFlag() { return multipleLayerStreamsPresentFlag_; }
  bool     getPcmPatchEnabledFlag() { return pcmPatchEnabledFlag_; }
  bool     getPcmSeparateVideoPresentFlag() { return pcmSeparateVideoPresentFlag_; }
  bool     getPatchSequenceOrientationEnabledFlag() { return patchSequenceOrientationEnabledFlag_; }
  bool     getPatchInterPredictionEnabledFlag() { return patchInterPredictionEnabledFlag_; }
  bool     getPixelDeinterleavingFlag() { return pixelDeinterleavingFlag_; }
  bool     getPointLocalReconstructionEnabledFlag() { return pointLocalReconstructionEnabledFlag_; }
  bool     getRemoveDuplicatePointEnabledFlag() { return removeDuplicatePointEnabledFlag_; }
  std::vector<size_t>& getLayerPredictorIndexDiff() { return layerPredictorIndexDiff_; }
  std::vector<bool>& getLayerAbsoluteCodingEnabledFlag() { return layerAbsoluteCodingEnabledFlag_; }
  ProfileTierLevel&  getProfileTierLevel() { return profileTierLevel_; }
  GeometryParameterSet&               getGeometryParameterSet() { return geometryParameterSet_; }
  OccupancyParameterSet&              getOccupancyParameterSet() { return occupancyParameterSet_; }
  std::vector<AttributeParameterSet>& getAttributeParameterSets() {
    return attributeParameterSets_;
  }
  AttributeParameterSet& getAttributeParameterSet( size_t attributeIndex ) {
    return attributeParameterSets_[attributeIndex];
  }

  void setSequenceParameterSetId( uint32_t idx ) { sequenceParameterSetId_ = idx; }
  void setFrameWidth( uint16_t width ) { frameWidth_ = width; }
  void setFrameHeight( uint16_t height ) { frameHeight_ = height; }
  void setAvgFrameRate( uint16_t frameRate ) { avgFrameRate_ = frameRate; }
  void setLayerCountMinus1( uint32_t countMinus1 ) { layerCountMinus1_ = countMinus1; }
  void setAttributeCount( size_t count ) { attributeCount_ = count; }
  void setAvgFrameRatePresentFlag( bool flag ) { avgFrameRatePresentFlag_ = flag; }
  void setEnhancedOccupancyMapForDepthFlag( bool flag ) {
    enhancedOccupancyMapForDepthFlag_ = flag;
  }
  void setMultipleLayerStreamsPresentFlag( bool flag ) { multipleLayerStreamsPresentFlag_ = flag; }
  void setPcmPatchEnabledFlag( bool flag ) { pcmPatchEnabledFlag_ = flag; }
  void setPcmSeparateVideoPresentFlag( bool flag ) { pcmSeparateVideoPresentFlag_ = flag; }
  void setPatchSequenceOrientationEnabledFlag( bool flag ) {
    patchSequenceOrientationEnabledFlag_ = flag;
  }
  void setPatchInterPredictionEnabledFlag( bool flag ) { patchInterPredictionEnabledFlag_ = flag; }
  void setPixelDeinterleavingFlag( bool flag ) { pixelDeinterleavingFlag_ = flag; }
  void setPointLocalReconstructionEnabledFlag( bool flag ) {
    pointLocalReconstructionEnabledFlag_ = flag;
  }
  void setRemoveDuplicatePointEnabledFlag( bool flag ) { removeDuplicatePointEnabledFlag_ = flag; }
  void setLayerPredictorIndexDiff( std::vector<size_t> predictorIdxDiffList ) {
    layerPredictorIndexDiff_ = predictorIdxDiffList;
  }
  void setLayerAbsoluteCodingEnabledFlag( std::vector<bool> flagList ) {
    layerAbsoluteCodingEnabledFlag_ = flagList;
  }
  void setProfileTierLevel( ProfileTierLevel level ) { profileTierLevel_ = level; }
  void setGeometryParameterSet( GeometryParameterSet params ) { geometryParameterSet_ = params; }
  void setOccupancyParameterSet( OccupancyParameterSet params ) { occupancyParameterSet_ = params; }
  void setAttributeParameterSets( std::vector<AttributeParameterSet> paramsList ) {
    attributeParameterSets_ = paramsList;
  }

  void allocateAttributeParameterSets() { attributeParameterSets_.resize( attributeCount_ ); }

 private:
  uint32_t              sequenceParameterSetId_;
  uint16_t              frameWidth_;
  uint16_t              frameHeight_;
  uint16_t              avgFrameRate_;
  uint32_t              layerCountMinus1_; 
  uint16_t              attributeCount_;
  bool                  avgFrameRatePresentFlag_;
  bool                  enhancedOccupancyMapForDepthFlag_;
  bool                  multipleLayerStreamsPresentFlag_;
  bool                  pcmPatchEnabledFlag_;
  bool                  pcmSeparateVideoPresentFlag_;
  bool                  patchSequenceOrientationEnabledFlag_;
  bool                  patchInterPredictionEnabledFlag_;
  bool                  pixelDeinterleavingFlag_; 
  bool                  pointLocalReconstructionEnabledFlag_;
  bool                  removeDuplicatePointEnabledFlag_;
  std::vector<bool>     layerAbsoluteCodingEnabledFlag_;  // layerCount_ size
  std::vector<size_t>   layerPredictorIndexDiff_;         // layerCount_ size
  ProfileTierLevel      profileTierLevel_;
  GeometryParameterSet  geometryParameterSet_;
  OccupancyParameterSet occupancyParameterSet_;
  std::vector<AttributeParameterSet> attributeParameterSets_;  // attributeCount_ size
};

// 7.3.4  PCM separate video data syntax
class VPCCParameterSet {
 public:
  VPCCParameterSet() :
      pcmVideoFlag_( false ),
      sequenceParamterSetId_( 0 ),
      attributeIndex_( 0 ),
      layerIndex_( 0 ) {}
  VPCCParameterSet& operator=( const VPCCParameterSet& ) = default;

  uint8_t getUnitType() { return unitType_; }
  bool    getPCMVideoFlag() { return pcmVideoFlag_; }
  uint8_t getSequenceParameterSetId() { return sequenceParamterSetId_; }
  uint8_t getAttributeIndex() { return attributeIndex_; }
  uint8_t getLayerIndex() { return layerIndex_; }

  void setUnitType( uint8_t type ) { unitType_ = type; }
  void setPCMVideoFlag( bool flag ) { pcmVideoFlag_ = flag; }
  void setSequenceParameterSetId( uint8_t setId ) { sequenceParamterSetId_ = setId; }
  void setAttributeIndex( uint8_t attributeIdx ) { attributeIndex_ = attributeIdx; }
  void setLayerIndex( uint8_t layerIdx ) { layerIndex_ = layerIdx; }

 private:
  uint8_t unitType_;
  bool    pcmVideoFlag_;
  uint8_t sequenceParamterSetId_;
  uint8_t attributeIndex_;
  uint8_t layerIndex_;
};

class PCCContext {
 public:
  PCCContext();
  ~PCCContext();

  std::vector<PCCFrameContext>::iterator begin() { return frames_.begin(); }
  std::vector<PCCFrameContext>::iterator end() { return frames_.end(); }

  void resize( size_t size );

  const size_t                  size() { return frames_.size(); }
  std::vector<PCCFrameContext>& getFrames() { return frames_; }
  PCCFrameContext&              operator[]( int i ) { return frames_[i]; }

  PCCVideoGeometry&     getVideoGeometry() { return videoGeometry_; }
  PCCVideoGeometry&     getVideoGeometryD1() { return videoGeometryD1_; }
  PCCVideoTexture&      getVideoTexture() { return videoTexture_; }
  PCCVideoOccupancyMap& getVideoOccupancyMap() { return videoOccupancyMap_; }
  PCCVideoGeometry&     getVideoMPsGeometry() { return videoMPsGeometry_; }
  PCCVideoTexture&      getVideoMPsTexture() { return videoMPsTexture_; }

  // deprecated, must be removed:
  bool&                    getLosslessGeo444() { return losslessGeo444_; }
  bool&                    getLosslessGeo() { return losslessGeo_; }
  bool&                    getLosslessTexture() { return losslessTexture_; }
  uint8_t&                 getOccupancyPrecision() { return occupancyPrecision_; }
  bool&                    getGridSmoothing() { return gridSmoothing_; }
  bool&                    getNoAttributes() { return noAttributes_; }
  bool&                    getAbsoluteD1() { return absoluteD1_; }
  bool&                    getBinArithCoding() { return binArithCoding_; }
  float&                   getModelScale() { return modelScale_; }
  PCCVector3<float>&       getModelOrigin() { return modelOrigin_; }
  bool&                    getImproveEDD() { return improveEDD_; }
  bool&                    getDeltaCoding() { return deltaCoding_; }
  bool&                    getSixDirectionMode() { return sixDirectionMode_; }
  bool&                    getUseAdditionalPointsPatch() { return useAdditionalPointsPatch_; }
  uint8_t&                 getMinLevel() { return minLevel_; }
  bool&                    getGlobalPatchAllocation() { return globalPatchAllocation_; }
  bool&                    getUse3dmc() { return use3dmc_; }
  size_t&                  getMPGeoWidth() { return MPGeoWidth_; }
  size_t&                  getMPGeoHeight() { return MPGeoHeight_; }
  size_t&                  getMPAttWidth() { return MPAttWidth_; }
  size_t&                  getMPAttHeight() { return MPAttHeight_; }
  PCCMetadata&             getGOFLevelMetadata() { return gofLevelMetadata_; }
  std::vector<SubContext>& getSubContexts() { return subContexts_; }
  std::vector<unionPatch>& getUnionPatch() { return unionPatch_; }
  //~ deprecated, must be removed

  PCCVideoBitstream& createVideoBitstream( PCCVideoType type ) {
    videoBitstream_.push_back( PCCVideoBitstream( type ) );
    return videoBitstream_.back();
  }
  size_t             getVideoBitstreamCount() { return videoBitstream_.size(); }
  PCCVideoBitstream& getVideoBitstream( size_t index ) { return videoBitstream_[index]; }
  PCCVideoBitstream& getVideoBitstream( PCCVideoType type ) {
    for ( auto& value : videoBitstream_ ) {
      if ( value.type() == type ) { return value; }
    }
    printf( "ERROR: can't get video bitstream of type %s \n", toString( type ).c_str() );
    fflush( stdout );
    exit( -1 );
  }

  void                  allocOneLayerData( const size_t occupancyResolution );
  void                  printVideoBitstream();
  void                  printBlockToPatch( const size_t occupancyResolution );
  VPCCParameterSet&     getVPCC() { return vpccParameterSet_; }
  SequenceParameterSet& getSps() {
    for ( auto& sps : sequenceParameterSets_ ) {
      if ( vpccParameterSet_.getSequenceParameterSetId() == sps.getSequenceParameterSetId() ) {
        return sps;
      }
    }
    fprintf( stderr, "Error: can't find sps[ %lc ] \n",
             vpccParameterSet_.getSequenceParameterSetId() );
    exit( -1 );
  }

  PatchSequenceDataUnit& getPatchSequenceDataUnit() { return patchSequenceDataUnit_; }

 private:
  std::vector<PCCFrameContext> frames_;
  PCCVideoGeometry             videoGeometry_;
  PCCVideoGeometry             videoGeometryD1_;
  PCCVideoTexture              videoTexture_;
  PCCVideoOccupancyMap         videoOccupancyMap_;
  PCCVideoGeometry             videoMPsGeometry_;
  PCCVideoTexture              videoMPsTexture_;

  // deprecated, must be removed:
  bool losslessGeo444_;
  bool losslessGeo_;
  bool losslessTexture_;
  bool gridSmoothing_;
  bool absoluteD1_;
  bool binArithCoding_;
  bool improveEDD_;
  bool deltaCoding_;
  bool sixDirectionMode_;
  bool useAdditionalPointsPatch_;
  bool globalPatchAllocation_;
  bool use3dmc_;
  bool noAttributes_;

  uint8_t                        minLevel_;
  uint8_t                        occupancyPrecision_;
  size_t                         MPGeoWidth_;
  size_t                         MPGeoHeight_;
  size_t                         MPAttWidth_;
  size_t                         MPAttHeight_;
  float                          modelScale_;
  PCCVector3<float>              modelOrigin_;
  PCCMetadata                    gofLevelMetadata_;
  std::vector<PCCVideoBitstream> videoBitstream_;
  std::vector<SubContext>        subContexts_;
  std::vector<unionPatch>        unionPatch_;
  //~ deprecated, must be removed

  VPCCParameterSet                  vpccParameterSet_;
  PatchSequenceDataUnit             patchSequenceDataUnit_;
  std::vector<SequenceParameterSet> sequenceParameterSets_;
};
};  // namespace pcc

#endif /* PCCContext_h */
