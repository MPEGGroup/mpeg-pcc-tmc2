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
#ifndef PCC_BITSTREAM_V3CUNIT_H
#define PCC_BITSTREAM_V3CUNIT_H

#include "PCCBitstreamCommon.h"
#include "PCCBitstream.h"

namespace pcc {

// 7.3.2.1	General V3C unit syntax
class V3CUnit {
 public:
  V3CUnit() {}
  ~V3CUnit() {}
  void   setV3CUnitSize( size_t value ) { v3cUnitSize_ = value; }
  void   setV3CUnitSize() { v3cUnitSize_ = V3CUnitDataBitstream_.size(); }
  size_t getV3CUnitSize() { return v3cUnitSize_; }

  void setV3CUnitDataBitstream( PCCBitstream&& bitstream, V3CUnitType unitType ) {
    V3CUnitDataBitstream_ = std::move( bitstream );
    v3cUnitSize_          = bitstream.size();
    v3cUnitType_          = unitType;
  }
  PCCBitstream& getV3CUnitDataBitstream() {
    V3CUnitDataBitstream_.beginning();
    return V3CUnitDataBitstream_;
  }
  void        allocate() { V3CUnitDataBitstream_.initialize( v3cUnitSize_ ); }
  void        getV3CUnitHeader() {}
  void        getV3CUnitPayload() {}
  V3CUnitType getV3CUnitType() { return v3cUnitType_; }

  void setV3CUnitType( size_t value ) { v3cUnitType_ = (V3CUnitType)value; }
  void setV3CUnitType( V3CUnitType value ) { v3cUnitType_ = value; }

 private:
  V3CUnitType  v3cUnitType_;
  size_t       v3cUnitSize_;
  PCCBitstream V3CUnitDataBitstream_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_V3CUNIT_H