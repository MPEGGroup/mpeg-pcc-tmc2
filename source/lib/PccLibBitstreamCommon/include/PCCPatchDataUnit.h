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
#ifndef PCC_BITSTREAM_PATCHDATAUNIT_H
#define PCC_BITSTREAM_PATCHDATAUNIT_H

#include "PCCBitstreamCommon.h"
#include "PCCPointLocalReconstructionData.h"

namespace pcc {
    
// 7.3.7.3  Patch data unit syntax
class PatchDataUnit {
 public:
  PatchDataUnit() :
      pdu2dPosX_( 0 ),
      pdu2dPosY_( 0 ),
      pdu2dSizeXMinus1_( 0 ),
      pdu2dSizeYMinus1_( 0 ),
      pdu3dPosX_( 0 ),
      pdu3dPosY_( 0 ),
      pdu3dPosMinZ_( 0 ),
      pdu3dPosDeltaMaxZ_( 0 ),
      pduProjectionId_( 0 ),
      pduOrientationIndex_( 0 ),
      pduLodEnableFlag_( false ),
      pduLodScaleXminus1_( 0 ),
      pduLodScaleY_( 0 ),
      pduPatchIndex_( 0 ),
      pduFrameIndex_( 0 ) {}
  ~PatchDataUnit() {}

  PatchDataUnit&                operator=( const PatchDataUnit& ) = default;
  size_t                        getPdu2dPosX() { return pdu2dPosX_; }
  size_t                        getPdu2dPosY() { return pdu2dPosY_; }
  int64_t                       getPdu2dSizeXMinus1() { return pdu2dSizeXMinus1_; }
  int64_t                       getPdu2dSizeYMinus1() { return pdu2dSizeYMinus1_; }
  size_t                        getPdu3dPosX() { return pdu3dPosX_; }
  size_t                        getPdu3dPosY() { return pdu3dPosY_; }
  size_t                        getPdu3dPosMinZ() { return pdu3dPosMinZ_; }
  size_t                        getPdu3dPosDeltaMaxZ() { return pdu3dPosDeltaMaxZ_; }
  size_t                        getPduProjectionId() { return pduProjectionId_; }
  size_t                        getPduOrientationIndex() { return pduOrientationIndex_; }
  bool                          getLodEnableFlag() { return pduLodEnableFlag_; }
  uint8_t                       getLodScaleXminus1() { return pduLodScaleXminus1_; }
  uint8_t                       getLodScaleY() { return pduLodScaleY_; }
  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }

  size_t getPatchIndex() { return pduPatchIndex_; }
  size_t getFrameIndex() { return pduFrameIndex_; }
  void   setPatchIndex( size_t value ) { pduPatchIndex_ = value; }
  void   setFrameIndex( size_t value ) { pduFrameIndex_ = value; }
  void   setPdu2dPosX( size_t value ) { pdu2dPosX_ = value; }
  void   setPdu2dPosY( size_t value ) { pdu2dPosY_ = value; }
  void   setPdu2dSizeXMinus1( uint64_t value ) { pdu2dSizeXMinus1_ = value; }
  void   setPdu2dSizeYMinus1( uint64_t value ) { pdu2dSizeYMinus1_ = value; }
  void   setPdu3dPosX( size_t value ) { pdu3dPosX_ = value; }
  void   setPdu3dPosY( size_t value ) { pdu3dPosY_ = value; }
  void   setPdu3dPosMinZ( size_t value ) { pdu3dPosMinZ_ = value; }
  void   setPdu3dPosDeltaMaxZ( size_t value ) { pdu3dPosDeltaMaxZ_ = value; }
  void   setPduProjectionId( size_t value ) { pduProjectionId_ = value; }
  void   setPduOrientationIndex( size_t value ) { pduOrientationIndex_ = value; }

  void setLodEnableFlag( bool value ) { pduLodEnableFlag_ = value; }
  void setLodScaleXminus1( uint8_t value ) { pduLodScaleXminus1_ = value; }
  void setLodScaleY( uint8_t value ) { pduLodScaleY_ = value; }
  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }

 private:
  size_t                       pdu2dPosX_;
  size_t                       pdu2dPosY_;
  uint64_t                     pdu2dSizeXMinus1_;
  uint64_t                     pdu2dSizeYMinus1_;
  size_t                       pdu3dPosX_;
  size_t                       pdu3dPosY_;
  size_t                       pdu3dPosMinZ_;
  size_t                       pdu3dPosDeltaMaxZ_;
  size_t                       pduProjectionId_;
  size_t                       pduOrientationIndex_;
  bool                         pduLodEnableFlag_;
  uint8_t                      pduLodScaleXminus1_;
  uint8_t                      pduLodScaleY_;
  PointLocalReconstructionData pointLocalReconstructionData_;
  size_t                       pduPatchIndex_;
  size_t                       pduFrameIndex_;
};

};  // namespace pcc

#endif //~PCC_BITSTREAM_PATCHDATAUNIT_H
