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
#include "PCCVpccExtension.h"

namespace pcc {

// 7.3.6.1 Atlas sequence parameter set RBSP
class AtlasSequenceParameterSetRbsp {
 public:
  AtlasSequenceParameterSetRbsp() :
      atlasSequenceParameterSetId_( 0 ),
      frameWidth_( 0 ),
      frameHeight_( 0 ),
      log2MaxAtlasFrameOrderCntLsbMinus4_( 4 ),
      maxDecAtlasFrameBufferingMinus1_( 0 ),
      longTermRefAtlasFramesFlag_( false ),
      numRefAtlasFrameListsInAsps_( 0 ),
      useEightOrientationsFlag_( false ),
      extendedProjectionEnabledFlag_( false ),
      maxNumberProjectionsMinus1_( 5 ),
      normalAxisLimitsQuantizationEnabledFlag_( false ),
      normalAxisMaxDeltaValueEnabledFlag_( false ),
      patchPrecedenceOrderFlag_( false ),
      log2PatchPackingBlockSize_( 0 ),
      patchSizeQuantizerPresentFlag_( false ),
      mapCountMinus1_( 0 ),
      pixelDeinterleavingFlag_( false ),
      pixeDeinterleavingMapFlag_( false ),
      eomPatchEnabledFlag_( false ),
      eomFixBitCountMinus1_( 0 ),
      rawPatchEnabledFlag_( false ),
      auxiliaryVideoEnabledFlag_( false ),
      pointLocalReconstructionEnabledFlag_( false ),
      vuiParametersPresentFlag_( false ),
      extensionFlag_( false ),
      vpccExtensionFlag_( false ),
      mivExtensionFlag_( false ),
      extension6Bits_( 0 ) {}
  ~AtlasSequenceParameterSetRbsp() {
    refListStruct_.clear();
    pixeDeinterleavingMapFlag_.clear();
    pointLocalReconstructionInformation_.clear();
  }

  AtlasSequenceParameterSetRbsp& operator=( const AtlasSequenceParameterSetRbsp& ) = default;

  void allocateRefListStruct() { refListStruct_.resize( numRefAtlasFrameListsInAsps_ ); }
  void allocatePixeDeinterleavingMapFlag() { pixeDeinterleavingMapFlag_.resize( mapCountMinus1_ ); }
  void allocatePointLocalReconstructionInformation() {
    pointLocalReconstructionInformation_.resize( mapCountMinus1_ + 1 );
  }

  uint8_t            getAtlasSequenceParameterSetId() { return atlasSequenceParameterSetId_; }
  uint16_t           getFrameWidth() { return frameWidth_; }
  uint16_t           getFrameHeight() { return frameHeight_; }
  uint8_t            getLog2MaxAtlasFrameOrderCntLsbMinus4() { return log2MaxAtlasFrameOrderCntLsbMinus4_; }
  uint8_t            getMaxDecAtlasFrameBufferingMinus1() { return maxDecAtlasFrameBufferingMinus1_; }
  bool               getLongTermRefAtlasFramesFlag() { return longTermRefAtlasFramesFlag_; }
  uint8_t            getNumRefAtlasFrameListsInAsps() { return numRefAtlasFrameListsInAsps_; }
  bool               getUseEightOrientationsFlag() { return useEightOrientationsFlag_; }
  bool               getExtendedProjectionEnabledFlag() { return extendedProjectionEnabledFlag_; }
  uint8_t            getMaxNumberProjectionsMinus1() { return maxNumberProjectionsMinus1_; }
  bool               getNormalAxisLimitsQuantizationEnabledFlag() { return normalAxisLimitsQuantizationEnabledFlag_; }
  bool               getNormalAxisMaxDeltaValueEnabledFlag() { return normalAxisMaxDeltaValueEnabledFlag_; }
  bool               getPatchPrecedenceOrderFlag() { return patchPrecedenceOrderFlag_; }
  uint8_t            getLog2PatchPackingBlockSize() { return log2PatchPackingBlockSize_; }
  bool               getPatchSizeQuantizerPresentFlag() { return patchSizeQuantizerPresentFlag_; }
  uint8_t            getMapCountMinus1() { return mapCountMinus1_; }
  bool               getPixelDeinterleavingFlag() { return pixelDeinterleavingFlag_; }
  bool               getxeDeinterleavingMapFlag( size_t index ) { return pixeDeinterleavingMapFlag_[index]; }
  bool               getEomPatchEnabledFlag() { return eomPatchEnabledFlag_; }
  uint8_t            getEomFixBitCountMinus1() { return eomFixBitCountMinus1_; }
  bool               getRawPatchEnabledFlag() { return rawPatchEnabledFlag_; }
  bool               getAuxiliaryVideoEnabledFlag() { return auxiliaryVideoEnabledFlag_; }
  bool               getPointLocalReconstructionEnabledFlag() { return pointLocalReconstructionEnabledFlag_; }
  bool               getVuiParametersPresentFlag() { return vuiParametersPresentFlag_; }
  bool               getExtensionFlag() { return extensionFlag_; }
  bool               getVpccExtensionFlag() { return vpccExtensionFlag_; }
  bool               getMivExtensionFlag() { return mivExtensionFlag_; }
  uint8_t            getExtension6Bits() { return extension6Bits_; }
  VUIParameters&     getVuiParameters() { return vuiParameters_; }
  AspsVpccExtension& getAspsVpccExtension() { return aspsVpccExtension_; }
  void               setAtlasSequenceParameterSetId( uint8_t value ) { atlasSequenceParameterSetId_ = value; }
  void               setFrameWidth( uint16_t value ) { frameWidth_ = value; }
  void               setFrameHeight( uint16_t value ) { frameHeight_ = value; }
  void setLog2MaxAtlasFrameOrderCntLsbMinus4( uint8_t value ) { log2MaxAtlasFrameOrderCntLsbMinus4_ = value; }
  void setMaxDecAtlasFrameBufferingMinus1( uint8_t value ) { maxDecAtlasFrameBufferingMinus1_ = value; }
  void setLongTermRefAtlasFramesFlag( bool value ) { longTermRefAtlasFramesFlag_ = value; }
  void setNumRefAtlasFrameListsInAsps( uint8_t value ) { numRefAtlasFrameListsInAsps_ = value; }
  void setUseEightOrientationsFlag( bool value ) { useEightOrientationsFlag_ = value; }
  void setExtendedProjectionEnabledFlag( bool value ) { extendedProjectionEnabledFlag_ = value; }
  void setMaxNumberProjectionsMinus1( uint8_t value ) { maxNumberProjectionsMinus1_ = value; }
  void setNormalAxisLimitsQuantizationEnabledFlag( bool value ) { normalAxisLimitsQuantizationEnabledFlag_ = value; }
  void setNormalAxisMaxDeltaValueEnabledFlag( bool value ) { normalAxisMaxDeltaValueEnabledFlag_ = value; }
  void setPatchPrecedenceOrderFlag( bool value ) { patchPrecedenceOrderFlag_ = value; }
  void setLog2PatchPackingBlockSize( uint8_t value ) { log2PatchPackingBlockSize_ = value; }
  void setPatchSizeQuantizerPresentFlag( bool value ) { patchSizeQuantizerPresentFlag_ = value; }
  void setMapCountMinus1( uint8_t value ) { mapCountMinus1_ = value; }
  void setxelDeinterleavingFlag( bool value ) { pixelDeinterleavingFlag_ = value; }
  void setxeDeinterleavingMapFlag( size_t index, bool value ) { pixeDeinterleavingMapFlag_[index] = value; }
  void setEomPatchEnabledFlag( bool value ) { eomPatchEnabledFlag_ = value; }
  void setEomFixBitCountMinus1( uint8_t value ) { eomFixBitCountMinus1_ = value; }
  void setRawPatchEnabledFlag( bool value ) { rawPatchEnabledFlag_ = value; }
  void setAuxiliaryVideoEnabledFlag( bool value ) { auxiliaryVideoEnabledFlag_ = value; }
  void setPointLocalReconstructionEnabledFlag( bool value ) { pointLocalReconstructionEnabledFlag_ = value; }
  void setVuiParametersPresentFlag( bool value ) { vuiParametersPresentFlag_ = value; }
  void setExtensionFlag( bool value ) { extensionFlag_ = value; }
  void setVpccExtensionFlag( bool value ) { vpccExtensionFlag_ = value; }
  void setMivExtensionFlag( bool value ) { mivExtensionFlag_ = value; }
  void setExtension6Bits( uint8_t value ) { extension6Bits_ = value; }

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
  uint8_t                                          atlasSequenceParameterSetId_;
  uint16_t                                         frameWidth_;
  uint16_t                                         frameHeight_;
  uint8_t                                          log2MaxAtlasFrameOrderCntLsbMinus4_;
  uint8_t                                          maxDecAtlasFrameBufferingMinus1_;
  bool                                             longTermRefAtlasFramesFlag_;
  uint8_t                                          numRefAtlasFrameListsInAsps_;
  std::vector<RefListStruct>                       refListStruct_;
  bool                                             useEightOrientationsFlag_;
  bool                                             extendedProjectionEnabledFlag_;
  uint8_t                                          maxNumberProjectionsMinus1_;
  bool                                             normalAxisLimitsQuantizationEnabledFlag_;
  bool                                             normalAxisMaxDeltaValueEnabledFlag_;
  bool                                             patchPrecedenceOrderFlag_;
  uint8_t                                          log2PatchPackingBlockSize_;
  bool                                             patchSizeQuantizerPresentFlag_;
  uint8_t                                          mapCountMinus1_;
  bool                                             pixelDeinterleavingFlag_;
  std::vector<bool>                                pixeDeinterleavingMapFlag_;
  bool                                             eomPatchEnabledFlag_;
  uint8_t                                          eomFixBitCountMinus1_;
  bool                                             rawPatchEnabledFlag_;
  bool                                             auxiliaryVideoEnabledFlag_;
  bool                                             pointLocalReconstructionEnabledFlag_;
  std::vector<PointLocalReconstructionInformation> pointLocalReconstructionInformation_;
  bool                                             vuiParametersPresentFlag_;
  bool                                             extensionFlag_;
  bool                                             vpccExtensionFlag_;
  bool                                             mivExtensionFlag_;
  uint8_t                                          extension6Bits_;
  VUIParameters                                    vuiParameters_;
  AspsVpccExtension                                aspsVpccExtension_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_ATLASSEQUENCEPARAMETERSETRBSP_H