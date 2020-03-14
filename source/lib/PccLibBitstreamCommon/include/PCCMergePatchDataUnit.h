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
#ifndef PCC_BITSTREAM_MERGEPATCHDATAUNIT_H
#define PCC_BITSTREAM_MERGEPATCHDATAUNIT_H

#include "PCCBitstreamCommon.h"
#include "PCCPointLocalReconstructionData.h"

namespace pcc {

// 7.3.7.5  Merge patch data unit syntax
class MergePatchDataUnit {
 public:
  MergePatchDataUnit() :
      mpduOverride2dParamsFlag_( 0 ),
      mpduOverride3dParamsFlag_( 0 ),
      mpduRefIndex_( 0 ),
      mpdu2dPosX_( 0 ),
      mpdu2dPosY_( 0 ),
      mpdu2dDeltaSizeX_( 0 ),
      mpdu2dDeltaSizeY_( 0 ),
      mpdu3dPosX_( 0 ),
      mpdu3dPosY_( 0 ),
      mpdu3dPosMinZ_( 0 ),
      mpdu3dPosDeltaMaxZ_( 0 ),
      mpduOverridePlrFlag_( 0 ),
      mpduPatchIndex_( 0 ),
      mpduFrameIndex_( 0 ){};
  ~MergePatchDataUnit(){};
  MergePatchDataUnit& operator=( const MergePatchDataUnit& ) = default;

  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }
  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }
  size_t getPatchIndex() { return mpduPatchIndex_; }
  size_t getFrameIndex() { return mpduFrameIndex_; }
  void   setPatchIndex( size_t value ) { mpduPatchIndex_ = value; }
  void   setFrameIndex( size_t value ) { mpduFrameIndex_ = value; }

  bool    getMpduOverride2dParamsFlag() { return mpduOverride2dParamsFlag_; }
  bool    getMpduOverride3dParamsFlag() { return mpduOverride3dParamsFlag_; }
  int64_t getMpduRefIndex() { return mpduRefIndex_; }
  int64_t getMpdu2dPosX() { return mpdu2dPosX_; }
  int64_t getMpdu2dPosY() { return mpdu2dPosY_; }
  int64_t getMpdu2dDeltaSizeX() { return mpdu2dDeltaSizeX_; }
  int64_t getMpdu2dDeltaSizeY() { return mpdu2dDeltaSizeY_; }
  int64_t getMpdu3dPosX() { return mpdu3dPosX_; }
  int64_t getMpdu3dPosY() { return mpdu3dPosY_; }
  int64_t getMpdu3dPosMinZ() { return mpdu3dPosMinZ_; }
  int64_t getMpdu3dPosDeltaMaxZ() { return mpdu3dPosDeltaMaxZ_; }
  int64_t getMpduOverridePlrFlag() { return mpduOverridePlrFlag_; }

  void setMpduOverride2dParamsFlag( bool value ) { mpduOverride2dParamsFlag_ = value; }
  void setMpduOverride3dParamsFlag( bool value ) { mpduOverride3dParamsFlag_ = value; }
  void setMpduRefIndex( int64_t value ) { mpduRefIndex_ = value; }
  void setMpdu2dPosX( int64_t value ) { mpdu2dPosX_ = value; }
  void setMpdu2dPosY( int64_t value ) { mpdu2dPosY_ = value; }
  void setMpdu2dDeltaSizeX( int64_t value ) { mpdu2dDeltaSizeX_ = value; }
  void setMpdu2dDeltaSizeY( int64_t value ) { mpdu2dDeltaSizeY_ = value; }
  void setMpdu3dPosX( int64_t value ) { mpdu3dPosX_ = value; }
  void setMpdu3dPosY( int64_t value ) { mpdu3dPosY_ = value; }
  void setMpdu3dPosMinZ( int64_t value ) { mpdu3dPosMinZ_ = value; }
  void setMpdu3dPosDeltaMaxZ( int64_t value ) { mpdu3dPosDeltaMaxZ_ = value; }
  void setMpduOverridePlrFlag( int64_t value ) { mpduOverridePlrFlag_ = value; }

 private:
  bool                         mpduOverride2dParamsFlag_;
  bool                         mpduOverride3dParamsFlag_;
  int64_t                      mpduRefIndex_;
  int64_t                      mpdu2dPosX_;
  int64_t                      mpdu2dPosY_;
  int64_t                      mpdu2dDeltaSizeX_;
  int64_t                      mpdu2dDeltaSizeY_;
  int64_t                      mpdu3dPosX_;
  int64_t                      mpdu3dPosY_;
  int64_t                      mpdu3dPosMinZ_;
  int64_t                      mpdu3dPosDeltaMaxZ_;
  int64_t                      mpduOverridePlrFlag_;
  size_t                       mpduPatchIndex_;
  size_t                       mpduFrameIndex_;
  PointLocalReconstructionData pointLocalReconstructionData_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_MERGEPATCHDATAUNIT_H