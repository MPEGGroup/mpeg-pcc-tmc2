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
#ifndef PCC_BITSTREAM_ATLASFRAMEPARAMETERSETRBSP_H
#define PCC_BITSTREAM_ATLASFRAMEPARAMETERSETRBSP_H

#include "PCCBitstreamCommon.h"
#include "PCCAtlasFrameTileInformation.h"

namespace pcc {

// 7.3.6.3  Atlas frame parameter set RBSP syntax
class AtlasFrameParameterSetRbsp {
 public:
  AtlasFrameParameterSetRbsp() :
      afpsAtlasFrameParameterSetId_( 0 ),
      afpsAtlasSequenceParameterSetId_( 0 ),
      afpsOutputFlagPresentFlag_( 0 ),
      afpsNumRefIdxDefaultActiveMinus1_( 0 ),
      afpsAdditionalLtAfocLsbLen_( 0 ),
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
    afpsLodModeEnableFlag_                = refAfps.getLodModeEnableFlag();
    afpsOverrideEomForDepthFlag_          = refAfps.getAfpsOverrideEomForDepthFlag();
    afpsEomNumberOfPatchBitCountMinus1_   = refAfps.getAfpsEomNumberOfPatchBitCountMinus1();
    afpsEomMaxBitCountMinus1_             = refAfps.getAfpsEomMaxBitCountMinus1();
    afpsRaw3dPosBitCountExplicitModeFlag_ = refAfps.getAfpsRaw3dPosBitCountExplicitModeFlag();
    afpsExtensionPresentFlag_             = refAfps.getAfpsExtensionPresentFlag();
    afpsExtensionDataFlag_                = refAfps.getAfpsExtensionDataFlag();
    atlasFrameTileInformation_            = refAfps.getAtlasFrameTileInformation();
  }

  uint8_t                    getAtlasFrameParameterSetId() { return afpsAtlasFrameParameterSetId_; }
  uint8_t                    getAtlasSequenceParameterSetId() { return afpsAtlasSequenceParameterSetId_; }
  bool                       getAfpsOutputFlagPresentFlag() { return afpsOutputFlagPresentFlag_; }
  uint8_t                    getAfpsNumRefIdxDefaultActiveMinus1() { return afpsNumRefIdxDefaultActiveMinus1_; }
  uint8_t                    getAfpsAdditionalLtAfocLsbLen() { return afpsAdditionalLtAfocLsbLen_; }
  bool                       getAfpsOverrideEomForDepthFlag() { return afpsOverrideEomForDepthFlag_; }
  uint8_t                    getAfpsEomNumberOfPatchBitCountMinus1() { return afpsEomNumberOfPatchBitCountMinus1_; }
  uint8_t                    getAfpsEomMaxBitCountMinus1() { return afpsEomMaxBitCountMinus1_; }
  bool                       getAfpsRaw3dPosBitCountExplicitModeFlag() { return afpsRaw3dPosBitCountExplicitModeFlag_; }
  uint8_t                    getAfpsExtensionPresentFlag() { return afpsExtensionPresentFlag_; }
  bool                       getAfpsExtensionDataFlag() { return afpsExtensionDataFlag_; }
  AtlasFrameTileInformation& getAtlasFrameTileInformation() { return atlasFrameTileInformation_; }
  bool                       getLodModeEnableFlag() { return afpsLodModeEnableFlag_; }

  void setAtlasFrameParameterSetId( uint8_t value ) { afpsAtlasFrameParameterSetId_ = value; }
  void setAtlasSequenceParameterSetId( uint8_t value ) { afpsAtlasSequenceParameterSetId_ = value; }
  void setAfpsOutputFlagPresentFlag( bool value ) { afpsOutputFlagPresentFlag_ = value; }
  void setAfpsNumRefIdxDefaultActiveMinus1( uint8_t value ) { afpsNumRefIdxDefaultActiveMinus1_ = value; }
  void setAfpsAdditionalLtAfocLsbLen( uint8_t value ) { afpsAdditionalLtAfocLsbLen_ = value; }
  void setAfpsOverrideEomForDepthFlag( bool value ) { afpsOverrideEomForDepthFlag_ = value; }
  void setAfpsEomNumberOfPatchBitCountMinus1( uint8_t value ) { afpsEomNumberOfPatchBitCountMinus1_ = value; }
  void setAfpsEomMaxBitCountMinus1( uint8_t value ) { afpsEomMaxBitCountMinus1_ = value; }
  void setAfpsRaw3dPosBitCountExplicitModeFlag( bool value ) { afpsRaw3dPosBitCountExplicitModeFlag_ = value; }
  void setAfpsExtensionPresentFlag( uint8_t value ) { afpsExtensionPresentFlag_ = value; }
  void setAfpsExtensionDataFlag( bool value ) { afpsExtensionDataFlag_ = value; }
  void setAtlasFrameTileInformation( AtlasFrameTileInformation value ) { atlasFrameTileInformation_ = value; }
  void setLodModeEnableFlag( bool value ) { afpsLodModeEnableFlag_ = value; }

 private:
  uint8_t                   afpsAtlasFrameParameterSetId_;
  uint8_t                   afpsAtlasSequenceParameterSetId_;
  AtlasFrameTileInformation atlasFrameTileInformation_;
  bool                      afpsOutputFlagPresentFlag_;
  uint8_t                   afpsNumRefIdxDefaultActiveMinus1_;
  uint8_t                   afpsAdditionalLtAfocLsbLen_;
  bool                      afpsLodModeEnableFlag_;
  bool                      afpsOverrideEomForDepthFlag_;
  uint8_t                   afpsEomNumberOfPatchBitCountMinus1_;
  uint8_t                   afpsEomMaxBitCountMinus1_;
  bool                      afpsRaw3dPosBitCountExplicitModeFlag_;
  uint8_t                   afpsExtensionPresentFlag_;
  bool                      afpsExtensionDataFlag_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_ATLASFRAMEPARAMETERSETRBSP_H
