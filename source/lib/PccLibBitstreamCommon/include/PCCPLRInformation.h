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
#ifndef PCC_BITSTREAM_POINTLOCALRECONSTRUCTIONINFORMATION_H
#define PCC_BITSTREAM_POINTLOCALRECONSTRUCTIONINFORMATION_H

#include "PCCBitstreamCommon.h"

namespace pcc {

typedef struct PointLocalReconstructionMode {
  bool    interpolate_;
  bool    filling_;
  uint8_t minD1_;
  uint8_t neighbor_;
} PointLocalReconstructionMode;

// 8.3.6.1.2 Point local reconstruction information syntax
class PLRInformation {
 public:
  PLRInformation() : numberOfModesMinus1_( 0 ), blockThresholdPerPatchMinus1_( 0 ) {
    minimumDepth_.clear();
    neighbourMinus1_.clear();
    interpolateFlag_.clear();
    fillingFlag_.clear();
  };
  ~PLRInformation() {
    minimumDepth_.clear();
    neighbourMinus1_.clear();
    interpolateFlag_.clear();
    fillingFlag_.clear();
  };

  PLRInformation& operator=( const PLRInformation& ) = default;

  void allocate() {
    minimumDepth_.resize( numberOfModesMinus1_ + 1, 0 );
    neighbourMinus1_.resize( numberOfModesMinus1_ + 1, 0 );
    interpolateFlag_.resize( numberOfModesMinus1_ + 1, false );
    fillingFlag_.resize( numberOfModesMinus1_ + 1, false );
  }
  bool    getMapEnabledFlag() { return mapEnabledFlag_; }
  uint8_t getNumberOfModesMinus1() { return numberOfModesMinus1_; }
  uint8_t getBlockThresholdPerPatchMinus1() { return blockThresholdPerPatchMinus1_; }
  uint8_t getMinimumDepth( size_t index ) { return minimumDepth_[index]; }
  uint8_t getNeighbourMinus1( size_t index ) { return neighbourMinus1_[index]; }
  bool    getInterpolateFlag( size_t index ) { return interpolateFlag_[index]; }
  bool    getFillingFlag( size_t index ) { return fillingFlag_[index]; }
  void    setMapEnabledFlag( bool value ) { mapEnabledFlag_ = value; }
  void    setNumberOfModesMinus1( uint8_t value ) { numberOfModesMinus1_ = value; }
  void    setBlockThresholdPerPatchMinus1( uint8_t value ) { blockThresholdPerPatchMinus1_ = value; }
  void    setMinimumDepth( size_t index, uint8_t value ) { minimumDepth_[index] = value; }
  void    setNeighbourMinus1( size_t index, uint8_t value ) { neighbourMinus1_[index] = value; }
  void    setInterpolateFlag( size_t index, bool value ) { interpolateFlag_[index] = value; }
  void    setFillingFlag( size_t index, bool value ) { fillingFlag_[index] = value; }

 private:
  bool                 mapEnabledFlag_;
  uint8_t              numberOfModesMinus1_;
  std::vector<bool>    interpolateFlag_;
  std::vector<bool>    fillingFlag_;
  std::vector<uint8_t> minimumDepth_;
  std::vector<uint8_t> neighbourMinus1_;
  uint8_t              blockThresholdPerPatchMinus1_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_POINTLOCALRECONSTRUCTIONINFORMATION_H