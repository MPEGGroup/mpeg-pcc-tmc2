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

// 8.3.7.8  EOM patch data unit syntax
class EOMPatchDataUnit {
 public:
  EOMPatchDataUnit() :
      pos2dX_( 0 ),
      pos2dY_( 0 ),
      size2dXMinus1_( 0 ),
      size2dYMinus1_( 0 ),
      patchCountMinus1_( 0 ),
      patchIndex_( 0 ),
      frameIndex_( 0 ),
      tileIndex_( 0 ) {
    associatedPatchesIdx_.clear();
    points_.clear();
  };
  ~EOMPatchDataUnit(){};
  EOMPatchDataUnit& operator=( const EOMPatchDataUnit& ) = default;

  bool   getPatchInAuxiliaryVideoFlag() { return patchInAuxiliaryVideoFlag_; }
  size_t get2dPosX() { return pos2dX_; }
  size_t get2dPosY() { return pos2dY_; }
  size_t get2dSizeXMinus1() { return size2dXMinus1_; }
  size_t get2dSizeYMinus1() { return size2dYMinus1_; }
  size_t getPatchCountMinus1() { return patchCountMinus1_; }
  size_t getAssociatedPatchesIdx( size_t index ) { return associatedPatchesIdx_[index]; }
  size_t getPoints( size_t index ) { return points_[index]; }
  size_t getPatchIndex() { return patchIndex_; }
  size_t getFrameIndex() { return frameIndex_; }
  size_t getTileIndex() { return tileIndex_; }

  void setPatchInAuxiliaryVideoFlag( bool value ) { patchInAuxiliaryVideoFlag_ = value; }
  void set2dPosX( size_t value ) { pos2dX_ = value; }
  void set2dPosY( size_t value ) { pos2dY_ = value; }
  void set2dSizeXMinus1( size_t value ) { size2dXMinus1_ = value; }
  void set2dSizeYMinus1( size_t value ) { size2dYMinus1_ = value; }
  void setPatchIndex( size_t value ) { patchIndex_ = value; }
  void setFrameIndex( size_t value ) { frameIndex_ = value; }
  void setTileOrder( size_t value ) { tileIndex_ = value; }
  void setAssociatedPatchesIdx( size_t index, size_t value ) { associatedPatchesIdx_[index] = value; }
  void setPoints( size_t index, size_t value ) { points_[index] = value; }
  void setPatchCountMinus1( uint32_t value ) {
    patchCountMinus1_ = value;
    associatedPatchesIdx_.resize( patchCountMinus1_ + 1 );
    points_.resize( patchCountMinus1_ + 1 );
  }

 private:
  bool                patchInAuxiliaryVideoFlag_;
  size_t              pos2dX_;
  size_t              pos2dY_;
  size_t              size2dXMinus1_;
  size_t              size2dYMinus1_;
  size_t              patchCountMinus1_;
  std::vector<size_t> associatedPatchesIdx_;
  std::vector<size_t> points_;
  size_t              patchIndex_;
  size_t              frameIndex_;
  size_t              tileIndex_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_EOMPATCHDATAUNIT_H
