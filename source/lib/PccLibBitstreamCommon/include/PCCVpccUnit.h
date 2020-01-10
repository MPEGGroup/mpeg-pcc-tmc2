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
#ifndef PCC_BITSTREAM_VPCCUNIT_H
#define PCC_BITSTREAM_VPCCUNIT_H

#include "PCCBitstreamCommon.h"
#include "PCCBitstream.h"

namespace pcc {

// 7.3.2 V-PCC unit syntax
class VpccUnit {
 public:
  VpccUnit() {}
  ~VpccUnit() {}

  void setVpccUnitSize( size_t value ) { vpccUnitSize_ = value; }
  void setVpccUnitSize() { vpccUnitSize_ = vpccUnitDataBitstream_.size(); }
  size_t getVpccUnitSize() { return vpccUnitSize_; }

  void setVpccUnitDataBitstream( PCCBitstream&& bitstream, VPCCUnitType unitType ) {
    vpccUnitDataBitstream_ = std::move( bitstream );
    vpccUnitSize_          = bitstream.size();
    vpccUnitType_          = unitType;
  }
  PCCBitstream& getVpccUnitDataBitstream() {
    vpccUnitDataBitstream_.beginning();
    return vpccUnitDataBitstream_;
  }
  void allocate() { vpccUnitDataBitstream_.initialize( vpccUnitSize_ ); }
  void         getVpccUnitHeader() {}
  void         getVpccUnitPayload() {}
  VPCCUnitType getVpccUnitType() { return vpccUnitType_; }

  void setVpccUnitType( size_t value ) { vpccUnitType_ = (VPCCUnitType)value; }
  void setVpccUnitType( VPCCUnitType value ) { vpccUnitType_ = value; }

 private:
  VPCCUnitType vpccUnitType_;
  size_t       vpccUnitSize_;
  PCCBitstream vpccUnitDataBitstream_;
};

};  // namespace pcc

#endif //~PCC_BITSTREAM_VPCCUNIT_H