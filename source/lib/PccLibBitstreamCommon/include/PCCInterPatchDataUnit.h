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
#include "PCCPLRData.h"

namespace pcc {

// 8.3.7.6  Inter patch data unit syntax
class InterPatchDataUnit {
 public:
  InterPatchDataUnit() :
      refIndex_( 0 ),
      refPatchIndex_( 0 ),
      pos2dX_( 0 ),
      pos2dY_( 0 ),
      delta2dSizeX_( 0 ),
      delta2dSizeY_( 0 ),
      pos3dOffsetU_( 0 ),
      pos3dOffsetV_( 0 ),
      pos3dOffsetD_( 0 ),
      pos3dRangeD_( 0 ),
      patchIndex_( 0 ),
      frameIndex_( 0 ),
      tileOrder_( 0 ){};
  ~InterPatchDataUnit(){};
  InterPatchDataUnit& operator=( const InterPatchDataUnit& ) = default;

  size_t   getPatchIndex() { return patchIndex_; }
  size_t   getFrameIndex() { return frameIndex_; }
  size_t   getTileOrder() { return tileOrder_; }
  int64_t  getRefIndex() { return refIndex_; }
  int64_t  getRefPatchIndex() { return refPatchIndex_; }
  int64_t  get2dPosX() { return pos2dX_; }
  int64_t  get2dPosY() { return pos2dY_; }
  int64_t  get2dDeltaSizeX() { return delta2dSizeX_; }
  int64_t  get2dDeltaSizeY() { return delta2dSizeY_; }
  int64_t  get3dOffsetU() { return pos3dOffsetU_; }
  int64_t  get3dOffsetV() { return pos3dOffsetV_; }
  int64_t  get3dOffsetD() { return pos3dOffsetD_; }
  int64_t  get3dRangeD() { return pos3dRangeD_; }
  PLRData& getPLRData() { return pointLocalReconstructionData_; }

  void setPatchIndex( size_t value ) { patchIndex_ = value; }
  void setFrameIndex( size_t value ) { frameIndex_ = value; }
  void setTileOrder( size_t value ) { tileOrder_ = value; }
  void setRefIndex( int64_t value ) { refIndex_ = value; }
  void setRefPatchIndex( int64_t value ) { refPatchIndex_ = value; }
  void set2dPosX( int64_t value ) { pos2dX_ = value; }
  void set2dPosY( int64_t value ) { pos2dY_ = value; }
  void set2dDeltaSizeX( int64_t value ) { delta2dSizeX_ = value; }
  void set2dDeltaSizeY( int64_t value ) { delta2dSizeY_ = value; }
  void set3dOffsetU( int64_t value ) { pos3dOffsetU_ = value; }
  void set3dOffsetV( int64_t value ) { pos3dOffsetV_ = value; }
  void set3dOffsetD( int64_t value ) { pos3dOffsetD_ = value; }
  void set3dRangeD( int64_t value ) { pos3dRangeD_ = value; }
  void setPLRData( PLRData value ) { pointLocalReconstructionData_ = value; }

 private:
  int64_t refIndex_;
  int64_t refPatchIndex_;
  int64_t pos2dX_;
  int64_t pos2dY_;
  int64_t delta2dSizeX_;
  int64_t delta2dSizeY_;
  int64_t pos3dOffsetU_;
  int64_t pos3dOffsetV_;
  int64_t pos3dOffsetD_;
  int64_t pos3dRangeD_;
  size_t  patchIndex_;
  size_t  frameIndex_;
  size_t  tileOrder_;
  PLRData pointLocalReconstructionData_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_INTERPATCHDATAUNIT_H
