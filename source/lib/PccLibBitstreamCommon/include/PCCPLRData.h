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
#ifndef PCC_BITSTREAM_PLRD_H
#define PCC_BITSTREAM_PLRD_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 8.3.7.9 Point local reconstruction data syntax
class PLRData {
 public:
  PLRData() :
      blockToPatchMapHeight_( 0 ),
      blockToPatchMapWidth_( 0 ),
      levelFlag_( 0 ),
      presentFlag_( false ),
      modeMinus1_( 0 ) {
    blockPresentFlag_.clear();
    blockModeMinus1_.clear();
  };
  ~PLRData() {
    blockPresentFlag_.clear();
    blockModeMinus1_.clear();
  };

  PLRData& operator=( const PLRData& ) = default;

  void allocate( size_t blockToPatchMapWidth, size_t blockToPatchMapHeight ) {
    blockToPatchMapWidth_  = blockToPatchMapWidth;
    blockToPatchMapHeight_ = blockToPatchMapHeight;
    blockPresentFlag_.resize( blockToPatchMapWidth_ * blockToPatchMapHeight_, false );
    blockModeMinus1_.resize( blockToPatchMapWidth_ * blockToPatchMapHeight_, 0 );
  }
  size_t  getBlockToPatchMapHeight() { return blockToPatchMapHeight_; }
  size_t  getBlockToPatchMapWidth() { return blockToPatchMapWidth_; }
  bool    getLevelFlag() { return levelFlag_; }
  bool    getPresentFlag() { return presentFlag_; }
  uint8_t getModeMinus1() { return modeMinus1_; }
  bool    getBlockPresentFlag( size_t index ) { return blockPresentFlag_[index]; }
  uint8_t getBlockModeMinus1( size_t index ) { return blockModeMinus1_[index]; }

  void setLevelFlag( bool value ) { levelFlag_ = value; }
  void setPresentFlag( bool value ) { presentFlag_ = value; }
  void setModeMinus1( uint8_t value ) { modeMinus1_ = value; }
  void setBlockPresentFlag( size_t index, bool value ) { blockPresentFlag_[index] = value; }
  void setBlockModeMinus1( size_t index, uint8_t value ) { blockModeMinus1_[index] = value; }

 private:
  size_t               blockToPatchMapHeight_;
  size_t               blockToPatchMapWidth_;
  bool                 levelFlag_;
  bool                 presentFlag_;
  uint8_t              modeMinus1_;
  std::vector<bool>    blockPresentFlag_;
  std::vector<uint8_t> blockModeMinus1_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_PLRD_H