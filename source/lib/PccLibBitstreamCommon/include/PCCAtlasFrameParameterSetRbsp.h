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

// 7.3.6.2 Atlas frame parameter set Rbsp syntax
// 7.3.6.2.1 General atlas frame parameter set Rbsp syntax
class AtlasFrameParameterSetRbsp {
 public:
  AtlasFrameParameterSetRbsp() :
      atlasFrameParameterSetId_( 0 ),
      atlasSequenceParameterSetId_( 0 ),
      outputFlagPresentFlag_( 0 ),
      numRefIdxDefaultActiveMinus1_( 0 ),
      additionalLtAfocLsbLen_( 0 ),
      lodModeEnableFlag_( false ),
      raw3dPosBitCountExplicitModeFlag_( 0 ),
      extensionFlag_( 0 ),
      vpccExtensionFlag_( 0 ),
      mivExtensionFlag_( 0 ),
      extension6Bits_( 0 ) {}

  ~AtlasFrameParameterSetRbsp() {}
  AtlasFrameParameterSetRbsp& operator=( const AtlasFrameParameterSetRbsp& ) = default;
  void                        copyFrom( AtlasFrameParameterSetRbsp& refAfps ) {
    atlasSequenceParameterSetId_      = refAfps.getAtlasSequenceParameterSetId();
    numRefIdxDefaultActiveMinus1_     = refAfps.getNumRefIdxDefaultActiveMinus1();
    additionalLtAfocLsbLen_           = refAfps.getAdditionalLtAfocLsbLen();
    lodModeEnableFlag_                = refAfps.getLodModeEnableFlag();
    raw3dPosBitCountExplicitModeFlag_ = refAfps.getRaw3dPosBitCountExplicitModeFlag();
    extensionFlag_                    = refAfps.getExtensionFlag();
    vpccExtensionFlag_                = refAfps.getVpccExtensionFlag();
    mivExtensionFlag_                 = refAfps.getMivExtensionFlag();
    extension6Bits_                   = refAfps.getExtension6Bits();
    atlasFrameTileInformation_        = refAfps.getAtlasFrameTileInformation();
  }

  uint8_t                    getAtlasFrameParameterSetId() { return atlasFrameParameterSetId_; }
  uint8_t                    getAtlasSequenceParameterSetId() { return atlasSequenceParameterSetId_; }
  bool                       getOutputFlagPresentFlag() { return outputFlagPresentFlag_; }
  uint8_t                    getNumRefIdxDefaultActiveMinus1() { return numRefIdxDefaultActiveMinus1_; }
  uint8_t                    getAdditionalLtAfocLsbLen() { return additionalLtAfocLsbLen_; }
  bool                       getRaw3dPosBitCountExplicitModeFlag() { return raw3dPosBitCountExplicitModeFlag_; }
  bool                       getExtensionFlag() { return extensionFlag_; }
  bool                       getVpccExtensionFlag() { return vpccExtensionFlag_; }
  bool                       getMivExtensionFlag() { return mivExtensionFlag_; }
  uint8_t                    getExtension6Bits() { return extension6Bits_; }
  AtlasFrameTileInformation& getAtlasFrameTileInformation() { return atlasFrameTileInformation_; }
  bool                       getLodModeEnableFlag() { return lodModeEnableFlag_; }
  AfpsVpccExtension&         getAfpsVpccExtension() { return afpsVpccExtension_; }

  void setAtlasFrameParameterSetId( uint8_t value ) { atlasFrameParameterSetId_ = value; }
  void setAtlasSequenceParameterSetId( uint8_t value ) { atlasSequenceParameterSetId_ = value; }
  void setOutputFlagPresentFlag( bool value ) { outputFlagPresentFlag_ = value; }
  void setNumRefIdxDefaultActiveMinus1( uint8_t value ) { numRefIdxDefaultActiveMinus1_ = value; }
  void setAdditionalLtAfocLsbLen( uint8_t value ) { additionalLtAfocLsbLen_ = value; }
  void setRaw3dPosBitCountExplicitModeFlag( bool value ) { raw3dPosBitCountExplicitModeFlag_ = value; }
  void setExtensionFlag( bool value ) { extensionFlag_ = value; }
  void setVpccExtensionFlag( bool value ) { vpccExtensionFlag_ = value; }
  void setMivExtensionFlag( bool value ) { mivExtensionFlag_ = value; }
  void setExtension6Bits( uint8_t value ) { extension6Bits_ = value; }
  void setLodModeEnableFlag( bool value ) { lodModeEnableFlag_ = value; }

 private:
  uint8_t                   atlasFrameParameterSetId_;
  uint8_t                   atlasSequenceParameterSetId_;
  AtlasFrameTileInformation atlasFrameTileInformation_;
  bool                      outputFlagPresentFlag_;
  uint8_t                   numRefIdxDefaultActiveMinus1_;
  uint8_t                   additionalLtAfocLsbLen_;
  bool                      lodModeEnableFlag_;
  bool                      raw3dPosBitCountExplicitModeFlag_;
  bool                      extensionFlag_;
  bool                      vpccExtensionFlag_;
  bool                      mivExtensionFlag_;
  uint8_t                   extension6Bits_;
  AfpsVpccExtension         afpsVpccExtension_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_ATLASFRAMEPARAMETERSETRBSP_H
