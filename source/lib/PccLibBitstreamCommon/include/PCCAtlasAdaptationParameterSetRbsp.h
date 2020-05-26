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
#ifndef PCC_BITSTREAM_ATLASADAPTATIONPARAMETERSETRBSP_H
#define PCC_BITSTREAM_ATLASADAPTATIONPARAMETERSETRBSP_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 7.3.6.3.1	General atlas adaptation parameter set RBSP syntax
class AtlasAdaptationParameterSetRbsp {
 public:
  AtlasAdaptationParameterSetRbsp() :
      atlasAdaptationParameterSetId_( 0 ),
      log2MaxAfocPresentFlag_( 0 ),
      log2MaxAtlasFrameOrderCntLsbMinus4_( 0 ),
      extensionFlag_( false ),
      vpccExtensionFlag_( false ),
      mivExtensionFlag_( false ),
      extension6Bits_( 0 ){};
  ~AtlasAdaptationParameterSetRbsp(){};
  AtlasAdaptationParameterSetRbsp& operator=( const AtlasAdaptationParameterSetRbsp& ) = default;

  uint8_t getAtlasAdaptationParameterSetId() { return atlasAdaptationParameterSetId_; }
  bool    getLog2MaxAfocPresentFlag() { return log2MaxAfocPresentFlag_; }
  uint8_t getLog2MaxAtlasFrameOrderCntLsbMinus4() { return log2MaxAtlasFrameOrderCntLsbMinus4_; }
  bool    getExtensionFlag() { return extensionFlag_; }
  bool    getVpccExtensionFlag() { return vpccExtensionFlag_; }
  bool    getMivExtensionFlag() { return mivExtensionFlag_; }
  bool    getExtension6Bits() { return extension6Bits_; }

  void setAtlasAdaptationParameterSetId( uint8_t value ) { atlasAdaptationParameterSetId_ = value; }
  void setLog2MaxAfocPresentFlag( bool value ) { log2MaxAfocPresentFlag_ = value; }
  void setLog2MaxAtlasFrameOrderCntLsbMinus4( uint8_t value ) { log2MaxAtlasFrameOrderCntLsbMinus4_ = value; }
  void setExtensionFlag( bool value ) { extensionFlag_ = value; }
  void setVpccExtensionFlag( bool value ) { vpccExtensionFlag_ = value; }
  void setMivExtensionFlag( bool value ) { mivExtensionFlag_ = value; }
  void setExtension6Bits( bool value ) { extension6Bits_ = value; }
  AapsVpccExtension& getAapsVpccExtension() { return aapsVpccExtension_; }

 private:
  uint8_t           atlasAdaptationParameterSetId_;
  bool              log2MaxAfocPresentFlag_;
  uint8_t           log2MaxAtlasFrameOrderCntLsbMinus4_;
  bool              extensionFlag_;
  bool              vpccExtensionFlag_;
  bool              mivExtensionFlag_;
  uint8_t           extension6Bits_;
  AapsVpccExtension aapsVpccExtension_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_ATLASADAPTATIONPARAMETERSETRBSP_H
