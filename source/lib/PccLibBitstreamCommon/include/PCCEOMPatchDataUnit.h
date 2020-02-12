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
#ifndef PCC_BITSTREAM_EOMPATCHDATAUNIT_H
#define PCC_BITSTREAM_EOMPATCHDATAUNIT_H

#include "PCCBitstreamCommon.h"

namespace pcc {
    
// 7.3.7.8  EOM patch data unit syntax
class EOMPatchDataUnit {
 public:
  EOMPatchDataUnit() :
      epdu2dPosX_( 0 ),
      epdu2dPosY_( 0 ),
      epdu2dSizeXMinus1_( 0 ),
      epdu2dSizeYMinus1_( 0 ),
      epduAssociatedPatcheCountMinus1_( 0 ),
      epduPatchIndex_( 0 ),
      epduFrameIndex_( 0 ) {
    epduAssociatedPatches_.clear();
    epduEomPointsPerPatch_.clear();
  };
  ~EOMPatchDataUnit(){};
  EOMPatchDataUnit& operator=( const EOMPatchDataUnit& ) = default;

  size_t               getEpdu2dPosX() { return epdu2dPosX_; }
  size_t               getEpdu2dPosY() { return epdu2dPosY_; }
  int64_t              getEpdu2dSizeXMinus1() { return epdu2dSizeXMinus1_; }
  int64_t              getEpdu2dSizeYMinus1() { return epdu2dSizeYMinus1_; }
  int64_t              getEpduAssociatedPatchesCountMinus1() { return epduAssociatedPatcheCountMinus1_; }
  std::vector<size_t>& getEpduAssociatedPatches() { return epduAssociatedPatches_; }
  size_t               getEpduAssociatedPatches( size_t index ) { return epduAssociatedPatches_[index]; }
  std::vector<size_t>  getEpduEomPointsPerPatch() { return epduEomPointsPerPatch_; }
  size_t               getEpduEomPointsPerPatch( size_t index ) { return epduEomPointsPerPatch_[index]; }
  size_t               getPatchIndex() { return epduPatchIndex_; }
  size_t               getFrameIndex() { return epduFrameIndex_; }
  void                 setPatchIndex( size_t value ) { epduPatchIndex_ = value; }
  void                 setFrameIndex( size_t value ) { epduFrameIndex_ = value; }

  void setEpduAssociatedPatchesCountMinus1( uint32_t value ) {
    epduAssociatedPatcheCountMinus1_ = value;
    epduAssociatedPatches_.resize( epduAssociatedPatcheCountMinus1_ + 1 );
    epduEomPointsPerPatch_.resize( epduAssociatedPatcheCountMinus1_ + 1 );
  }
  void setEpduAssociatedPatches( size_t value, size_t index ) { epduAssociatedPatches_[index] = value; }
  void setEpduEomPointsPerPatch( size_t value, size_t index ) { epduEomPointsPerPatch_[index] = value; }

  void setEpdu2dPosX( size_t value ) { epdu2dPosX_ = value; }
  void setEpdu2dPosY( size_t value ) { epdu2dPosY_ = value; }
  void setEpdu2dSizeXMinus1( uint64_t value ) { epdu2dSizeXMinus1_ = value; }
  void setEpdu2dSizeYMinus1( uint64_t value ) { epdu2dSizeYMinus1_ = value; }

 private:
  size_t              epdu2dPosX_;
  size_t              epdu2dPosY_;
  uint64_t            epdu2dSizeXMinus1_;
  uint64_t            epdu2dSizeYMinus1_;
  size_t              epduAssociatedPatcheCountMinus1_;
  size_t              epduPatchIndex_;
  size_t              epduFrameIndex_;
  std::vector<size_t> epduAssociatedPatches_;
  std::vector<size_t> epduEomPointsPerPatch_;
};

};  // namespace pcc

#endif //~PCC_BITSTREAM_EOMPATCHDATAUNIT_H
