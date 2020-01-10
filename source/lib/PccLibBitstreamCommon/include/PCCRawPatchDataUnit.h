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
#ifndef PCC_BITSTREAM_RAWPATCHDATAUNIT_H
#define PCC_BITSTREAM_RAWPATCHDATAUNIT_H

#include "PCCBitstreamCommon.h"

namespace pcc {
    
// 7.3.7.7  raw patch data unit syntax
class RawPatchDataUnit {
 public:
  RawPatchDataUnit() :
      rpduPatchInRawVideoFlag_( false ),
      rpdu2dPosX_( 0 ),
      rpdu2dPosY_( 0 ),
      rpdu2dDeltaSizeX_( 0 ),
      rpdu2dDeltaSizeY_( 0 ),
      rpdu3dPosX_( 0 ),
      rpdu3dPosY_( 0 ),
      rpdu3dPosZ_( 0 ),
      rpduRawPoints_( 0 ),
      rpduPatchIndex_( 0 ),
      rpduFrameIndex_( 0 ) {}
  ~RawPatchDataUnit() {}
  RawPatchDataUnit& operator=( const RawPatchDataUnit& ) = default;

  bool     getRpduPatchInRawVideoFlag() { return rpduPatchInRawVideoFlag_; }
  size_t   getRpdu2dPosX() { return rpdu2dPosX_; }
  size_t   getRpdu2dPosY() { return rpdu2dPosY_; }
  int64_t  getRpdu2dDeltaSizeX() { return rpdu2dDeltaSizeX_; }
  int64_t  getRpdu2dDeltaSizeY() { return rpdu2dDeltaSizeY_; }
  size_t   getRpdu3dPosX() { return rpdu3dPosX_; }
  size_t   getRpdu3dPosY() { return rpdu3dPosY_; }
  size_t   getRpdu3dPosZ() { return rpdu3dPosZ_; }
  uint32_t getRpduRawPoints() { return rpduRawPoints_; }
  size_t   getPatchIndex() { return rpduPatchIndex_; }
  size_t   getFrameIndex() { return rpduFrameIndex_; }
  void     setPatchIndex( size_t value ) { rpduPatchIndex_ = value; }
  void     setFrameIndex( size_t value ) { rpduFrameIndex_ = value; }
  void     setRpduPatchInRawVideoFlag( bool value ) { rpduPatchInRawVideoFlag_ = value; }
  void     setRpdu2dPosX( size_t value ) { rpdu2dPosX_ = value; }
  void     setRpdu2dPosY( size_t value ) { rpdu2dPosY_ = value; }
  void     setRpdu2dDeltaSizeX( int64_t value ) { rpdu2dDeltaSizeX_ = value; }
  void     setRpdu2dDeltaSizeY( int64_t value ) { rpdu2dDeltaSizeY_ = value; }
  void     setRpdu3dPosX( size_t value ) { rpdu3dPosX_ = value; }
  void     setRpdu3dPosY( size_t value ) { rpdu3dPosY_ = value; }
  void     setRpdu3dPosZ( size_t value ) { rpdu3dPosZ_ = value; }
  void     setRpduRawPoints( uint32_t value ) { rpduRawPoints_ = value; }

 private:
  bool     rpduPatchInRawVideoFlag_;
  size_t   rpdu2dPosX_;
  size_t   rpdu2dPosY_;
  int64_t  rpdu2dDeltaSizeX_;
  int64_t  rpdu2dDeltaSizeY_;
  size_t   rpdu3dPosX_;
  size_t   rpdu3dPosY_;
  size_t   rpdu3dPosZ_;
  uint32_t rpduRawPoints_;
  size_t   rpduPatchIndex_;
  size_t   rpduFrameIndex_;
};

};  // namespace pcc

#endif //~PCC_BITSTREAM_RAWPATCHDATAUNIT_H