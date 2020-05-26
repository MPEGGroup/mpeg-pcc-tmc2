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
#ifndef PCC_BITSTREAM_INTERPATCHDATAUNIT_H
#define PCC_BITSTREAM_INTERPATCHDATAUNIT_H

#include "PCCBitstreamCommon.h"
#include "PCCPointLocalReconstructionData.h"

namespace pcc {

// 7.3.7.6  Inter patch data unit syntax
class InterPatchDataUnit {
 public:
  InterPatchDataUnit() :
      refIndex_( 0 ),
      refPatchIndex_( 0 ),
      pos2dX_( 0 ),
      pos2dY_( 0 ),
      delta2dSizeX_( 0 ),
      delta2dSizeY_( 0 ),
      pos3dX_( 0 ),
      pos3dY_( 0 ),
      pos3dMinZ_( 0 ),
      pos3dDeltaMaxZ_( 0 ),
      patchIndex_( 0 ),
      frameIndex_( 0 ){};
  ~InterPatchDataUnit(){};
  InterPatchDataUnit& operator=( const InterPatchDataUnit& ) = default;

  size_t                        getPatchIndex() { return patchIndex_; }
  size_t                        getFrameIndex() { return frameIndex_; }
  int64_t                       getRefIndex() { return refIndex_; }
  int64_t                       getRefPatchIndex() { return refPatchIndex_; }
  int64_t                       get2dPosX() { return pos2dX_; }
  int64_t                       get2dPosY() { return pos2dY_; }
  int64_t                       get2dDeltaSizeX() { return delta2dSizeX_; }
  int64_t                       get2dDeltaSizeY() { return delta2dSizeY_; }
  int64_t                       get3dPosX() { return pos3dX_; }
  int64_t                       get3dPosY() { return pos3dY_; }
  int64_t                       get3dPosMinZ() { return pos3dMinZ_; }
  int64_t                       get3dPosDeltaMaxZ() { return pos3dDeltaMaxZ_; }
  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }

  void setPatchIndex( size_t value ) { patchIndex_ = value; }
  void setFrameIndex( size_t value ) { frameIndex_ = value; }
  void setRefIndex( int64_t value ) { refIndex_ = value; }
  void setRefPatchIndex( int64_t value ) { refPatchIndex_ = value; }
  void set2dPosX( int64_t value ) { pos2dX_ = value; }
  void set2dPosY( int64_t value ) { pos2dY_ = value; }
  void set2dDeltaSizeX( int64_t value ) { delta2dSizeX_ = value; }
  void set2dDeltaSizeY( int64_t value ) { delta2dSizeY_ = value; }
  void set3dPosX( int64_t value ) { pos3dX_ = value; }
  void set3dPosY( int64_t value ) { pos3dY_ = value; }
  void set3dPosMinZ( int64_t value ) { pos3dMinZ_ = value; }
  void set3dPosDeltaMaxZ( int64_t value ) { pos3dDeltaMaxZ_ = value; }
  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }

 private:
  int64_t                      refIndex_;
  int64_t                      refPatchIndex_;
  int64_t                      pos2dX_;
  int64_t                      pos2dY_;
  int64_t                      delta2dSizeX_;
  int64_t                      delta2dSizeY_;
  int64_t                      pos3dX_;
  int64_t                      pos3dY_;
  int64_t                      pos3dMinZ_;
  int64_t                      pos3dDeltaMaxZ_;
  size_t                       patchIndex_;
  size_t                       frameIndex_;
  PointLocalReconstructionData pointLocalReconstructionData_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_INTERPATCHDATAUNIT_H