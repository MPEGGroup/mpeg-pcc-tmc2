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
#ifndef PCC_BITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H
#define PCC_BITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H

#include "PCCBitstreamCommon.h"
#include "PCCVUIParameters.h"
#include "PCCRefListStruct.h"
#include "PCCPointLocalReconstructionInformation.h"

namespace pcc {
    
// 7.3.6.1 Atlas sequence parameter set RBSP
class AtlasSequenceParameterSetRbsp {
 public:
  AtlasSequenceParameterSetRbsp() :
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
  ~AtlasSequenceParameterSetRbsp() {
    refListStruct_.clear();
    pointLocalReconstructionInformation_.clear();
  }

  AtlasSequenceParameterSetRbsp& operator=( const AtlasSequenceParameterSetRbsp& ) = default;

  void allocateRefListStruct() { refListStruct_.resize( numRefAtlasFrameListsInAsps_ ); }
  void allocatePointLocalReconstructionInformation() {
    pointLocalReconstructionInformation_.resize( mapCountMinus1_ + 1 );
  }
  uint8_t        getAltasSequenceParameterSetId() { return altasSequenceParameterSetId_; }
  uint16_t       getFrameWidth() { return frameWidth_; }
  uint16_t       getFrameHeight() { return frameHeight_; }
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
  void setFrameWidth( uint16_t value ) { frameWidth_ = value; }
  void setFrameHeight( uint16_t value ) { frameHeight_ = value; }
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

};  // namespace pcc

#endif //~PCC_BITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H