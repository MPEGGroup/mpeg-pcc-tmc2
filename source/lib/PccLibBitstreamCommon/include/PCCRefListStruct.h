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
#ifndef PCC_BITSTREAM_REFLISTSTRUCT_H
#define PCC_BITSTREAM_REFLISTSTRUCT_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 8.3.6.12  Reference list structure syntax
class RefListStruct {
 public:
  RefListStruct() : numRefEntries_( 0 ) {
    absDeltaAfocSt_.clear();
    afocLsbLt_.clear();
    stRefAtlasFrameFlag_.clear();
    strpfEntrySignFlag_.clear();
  }
  ~RefListStruct() {
    absDeltaAfocSt_.clear();
    afocLsbLt_.clear();
    stRefAtlasFrameFlag_.clear();
    strpfEntrySignFlag_.clear();
  }
  RefListStruct& operator=( const RefListStruct& ) = default;

  void allocate() {
    absDeltaAfocSt_.resize( numRefEntries_, 0 );
    afocLsbLt_.resize( numRefEntries_, 0 );
    stRefAtlasFrameFlag_.resize( numRefEntries_, false );
    strpfEntrySignFlag_.resize( numRefEntries_, false );
  }

  void addAbsDeltaAfocSt( uint8_t value ) { absDeltaAfocSt_.push_back( value ); }
  void addStrafEntrySignFlag( bool value ) { strpfEntrySignFlag_.push_back( value ); }
  void addAfocLsbLt( uint8_t value ) { afocLsbLt_.push_back( value ); }
  void addStRefAtalsFrameFlag( bool value ) { stRefAtlasFrameFlag_.push_back( value ); }

  uint8_t getNumRefEntries() { return numRefEntries_; }
  uint8_t getAbsDeltaAfocSt( uint16_t index ) { return absDeltaAfocSt_[index]; }
  bool    getStrafEntrySignFlag( uint16_t index ) { return strpfEntrySignFlag_[index]; }
  uint8_t getAfocLsbLt( uint16_t index ) { return afocLsbLt_[index]; }
  bool    getStRefAtalsFrameFlag( uint16_t index ) { return stRefAtlasFrameFlag_[index]; }

  void setNumRefEntries( uint8_t value ) { numRefEntries_ = value; }
  void setAbsDeltaAfocSt( uint16_t index, uint8_t value ) { absDeltaAfocSt_[index] = value; }
  void setStrafEntrySignFlag( uint16_t index, bool value ) { strpfEntrySignFlag_[index] = value; }
  void setAfocLsbLt( uint16_t index, uint8_t value ) { afocLsbLt_[index] = value; }
  void setStRefAtalsFrameFlag( uint16_t index, bool value ) { stRefAtlasFrameFlag_[index] = value; }

 private:
  uint8_t              numRefEntries_;
  std::vector<uint8_t> absDeltaAfocSt_;
  std::vector<uint8_t> afocLsbLt_;
  std::vector<bool>    stRefAtlasFrameFlag_;
  std::vector<bool>    strpfEntrySignFlag_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_REFLISTSTRUCT_H