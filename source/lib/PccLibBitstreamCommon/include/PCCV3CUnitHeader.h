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
#ifndef PCC_BITSTREAM_V3CUNITHEADER_H
#define PCC_BITSTREAM_V3CUNITHEADER_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 8.3.2.2  V3C unit header syntax
class V3CUnitHeader {
 public:
  V3CUnitHeader() :
      // unitType_( 0 ),
      sequenceParamterSetId_( 0 ),
      atlasId_( 0 ),
      attributeIndex_( 0 ),
      attributeDimensionIndex_( 0 ),
      mapIndex_( 0 ),
      auxiliaryVideoFlag_( false ) {}

  ~V3CUnitHeader() {}
  V3CUnitHeader& operator=( const V3CUnitHeader& ) = default;
  uint8_t        getV3CParameterSetId() { return sequenceParamterSetId_; }
  uint8_t        getAtlasId() { return atlasId_; }
  uint8_t        getAttributeIndex() { return attributeIndex_; }
  uint8_t        getAttributeDimensionIndex() { return attributeDimensionIndex_; }
  uint8_t        getMapIndex() { return mapIndex_; }
  bool           getAuxiliaryVideoFlag() { return auxiliaryVideoFlag_; }
  void           setV3CParameterSetId( uint8_t value ) { sequenceParamterSetId_ = value; }
  void           setAtlasId( uint8_t value ) { atlasId_ = value; }
  void           setAttributeIndex( uint8_t value ) { attributeIndex_ = value; }
  void           setAttributeDimensionIndex( uint8_t value ) { attributeDimensionIndex_ = value; }
  void           setMapIndex( uint8_t value ) { mapIndex_ = value; }
  void           setAuxiliaryVideoFlag( bool value ) { auxiliaryVideoFlag_ = value; }

 private:
  uint8_t sequenceParamterSetId_;
  uint8_t atlasId_;
  uint8_t attributeIndex_;
  uint8_t attributeDimensionIndex_;
  uint8_t mapIndex_;
  bool    auxiliaryVideoFlag_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_V3CUNITHEADER_H