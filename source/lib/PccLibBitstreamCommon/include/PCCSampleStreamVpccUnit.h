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
#ifndef PCC_BITSTREAM_SAMPLESTREAMVPCCUNIT_H
#define PCC_BITSTREAM_SAMPLESTREAMVPCCUNIT_H

#include "PCCBitstreamCommon.h"
#include "PCCVpccUnit.h"

namespace pcc {

// B.2 Sample stream V-PCC unit syntax and semantics
// 6.1 V-PCC bistreams format
class SampleStreamVpccUnit {
 public:
  SampleStreamVpccUnit() : ssvhUnitSizePrecisionBytesMinus1_( 0 ) {}
  ~SampleStreamVpccUnit() { vpccUnits_.clear(); }
  VpccUnit& addVpccUnit() {
    vpccUnits_.resize( vpccUnits_.size() + 1 );
    return vpccUnits_.back();
  }
  void                   popFront() { vpccUnits_.erase( vpccUnits_.begin(), vpccUnits_.begin() + 1 ); }
  VpccUnit&              front() { return *( vpccUnits_.begin() ); }
  std::vector<VpccUnit>& getVpccUnit() { return vpccUnits_; }
  size_t                 getVpccUnitCount() { return vpccUnits_.size(); }
  VpccUnit&              getLastVpccUnit() { return vpccUnits_.back(); }
  uint32_t               getSsvhUnitSizePrecisionBytesMinus1() { return ssvhUnitSizePrecisionBytesMinus1_; }
  void setSsvhUnitSizePrecisionBytesMinus1( uint32_t value ) { ssvhUnitSizePrecisionBytesMinus1_ = value; }

 private:
  std::vector<VpccUnit> vpccUnits_;
  uint32_t              ssvhUnitSizePrecisionBytesMinus1_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_SAMPLESTREAMVPCCUNIT_H