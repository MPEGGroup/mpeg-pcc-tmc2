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
#ifndef PCC_BITSTREAM_NALUNIT_H
#define PCC_BITSTREAM_NALUNIT_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 7.3.5 NAL unit syntax
class NalUnit {
 public:
  NalUnit( NalUnitType nalUnitType = NAL_TRAIL, uint8_t layerId = 0, uint8_t temporalyIdPlus1 = 0 ) :
      nalUnitType_( nalUnitType ),
      layerId_( layerId ),
      temporalyIdPlus1_( temporalyIdPlus1 ),
      nalUnitSize_( 0 ) {}
  ~NalUnit() { nalUnitData_.clear(); }
  void                  allocate() { nalUnitData_.resize( nalUnitSize_ - 2, 0 ); }
  NalUnitType           getNalUnitType() { return nalUnitType_; }
  uint8_t               getLayerId() { return layerId_; }
  uint8_t               getTemporalyIdPlus1() { return temporalyIdPlus1_; }
  size_t                getNalUnitSize() { return nalUnitSize_; }
  uint8_t               getNalUnitData( size_t index ) { return nalUnitData_[index]; }
  std::vector<uint8_t>& getNalUnitData() { return nalUnitData_; }

  void setNalUnitType( NalUnitType value ) { nalUnitType_ = value; }
  void setLayerId( uint8_t value ) { layerId_ = value; }
  void setTemporalyIdPlus1( uint8_t value ) { temporalyIdPlus1_ = value; }
  void setNalUnitSize( size_t value ) { nalUnitSize_ = value; }
  void setNalUnitData( size_t index, uint8_t data ) { nalUnitData_[index] = data; }

  void initialize( NalUnitType nalUnitType, uint8_t layerId, uint8_t temporalyIdPlus1 ) {
    nalUnitType_      = nalUnitType;
    layerId_          = layerId;
    temporalyIdPlus1_ = temporalyIdPlus1;
  }
  void initialize( NalUnitType nalUnitType, uint8_t layerId, uint8_t temporalyIdPlus1, size_t size, uint8_t* data ) {
    nalUnitType_      = nalUnitType;
    layerId_          = layerId;
    temporalyIdPlus1_ = temporalyIdPlus1;
    nalUnitSize_      = size;
    if ( size > 0 ) {
      nalUnitData_.resize( size, 0 );
      memcpy( nalUnitData_.data(), data, size * sizeof( uint8_t ) );
    }
  }

 private:
  NalUnitType          nalUnitType_;
  uint8_t              layerId_;
  uint8_t              temporalyIdPlus1_;
  size_t               nalUnitSize_;
  std::vector<uint8_t> nalUnitData_;
};

};  // namespace pcc

#endif //~PCC_BITSTREAM_NALUNIT_H