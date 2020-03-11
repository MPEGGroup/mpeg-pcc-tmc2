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
      ipduRefIndex_( 0 ),
      ipduRefpatchIndex_( 0 ),
      ipdu2dPosX_( 0 ),
      ipdu2dPosY_( 0 ),
      ipdu2dDeltaSizeX_( 0 ),
      ipdu2dDeltaSizeY_( 0 ),
      ipdu3dPosX_( 0 ),
      ipdu3dPosY_( 0 ),
      ipdu3dPosMinZ_( 0 ),
      ipdu3dPosDeltaMaxZ_( 0 ),
      ipduPatchIndex_( 0 ),
      ipduFrameIndex_( 0 ){};
  ~InterPatchDataUnit(){};
  InterPatchDataUnit& operator=( const InterPatchDataUnit& ) = default;

  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }
  int64_t                       getIpduRefIndex() { return ipduRefIndex_; }
  int64_t                       getIpduRefPatchIndex() { return ipduRefpatchIndex_; }
  int64_t                       getIpdu2dPosX() { return ipdu2dPosX_; }
  int64_t                       getIpdu2dPosY() { return ipdu2dPosY_; }
  int64_t                       getIpdu2dDeltaSizeX() { return ipdu2dDeltaSizeX_; }
  int64_t                       getIpdu2dDeltaSizeY() { return ipdu2dDeltaSizeY_; }
  int64_t                       getIpdu3dPosX() { return ipdu3dPosX_; }
  int64_t                       getIpdu3dPosY() { return ipdu3dPosY_; }
  int64_t                       getIpdu3dPosMinZ() { return ipdu3dPosMinZ_; }
  int64_t                       getIpdu3dPosDeltaMaxZ() { return ipdu3dPosDeltaMaxZ_; }

  void setIpduRefIndex( int64_t value ) { ipduRefIndex_ = value; }
  void setIpduRefPatchIndex( int64_t value ) { ipduRefpatchIndex_ = value; }
  void setIpdu2dPosX( int64_t value ) { ipdu2dPosX_ = value; }
  void setIpdu2dPosY( int64_t value ) { ipdu2dPosY_ = value; }
  void setIpdu2dDeltaSizeX( int64_t value ) { ipdu2dDeltaSizeX_ = value; }
  void setIpdu2dDeltaSizeY( int64_t value ) { ipdu2dDeltaSizeY_ = value; }
  void setIpdu3dPosX( int64_t value ) { ipdu3dPosX_ = value; }
  void setIpdu3dPosY( int64_t value ) { ipdu3dPosY_ = value; }
  void setIpdu3dPosMinZ( int64_t value ) { ipdu3dPosMinZ_ = value; }
  void setIpdu3dPosDeltaMaxZ( int64_t value ) { ipdu3dPosDeltaMaxZ_ = value; }

  size_t getPatchIndex() { return ipduPatchIndex_; }
  size_t getFrameIndex() { return ipduFrameIndex_; }
  void   setPatchIndex( size_t value ) { ipduPatchIndex_ = value; }
  void   setFrameIndex( size_t value ) { ipduFrameIndex_ = value; }

  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }

 private:
  int64_t                      ipduRefIndex_;
  int64_t                      ipduRefpatchIndex_;
  int64_t                      ipdu2dPosX_;
  int64_t                      ipdu2dPosY_;
  int64_t                      ipdu2dDeltaSizeX_;
  int64_t                      ipdu2dDeltaSizeY_;
  int64_t                      ipdu3dPosX_;
  int64_t                      ipdu3dPosY_;
  int64_t                      ipdu3dPosMinZ_;
  int64_t                      ipdu3dPosDeltaMaxZ_;
  size_t                       ipduPatchIndex_;
  size_t                       ipduFrameIndex_;
  PointLocalReconstructionData pointLocalReconstructionData_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_INTERPATCHDATAUNIT_H