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
#ifndef PCC_BITSTREAM_ATLASTILEGROUPDATAUNIT_H
#define PCC_BITSTREAM_ATLASTILEGROUPDATAUNIT_H

#include "PCCBitstreamCommon.h"
#include "PCCPatchInformationData.h"

namespace pcc {
    
// 7.3.7.1  General atlas tile group data unit syntax
// atgduPatchMode_?)
class AtlasTileGroupDataUnit {
 public:
  AtlasTileGroupDataUnit() {}
  ~AtlasTileGroupDataUnit() { patchInformationData_.clear(); }
  AtlasTileGroupDataUnit& operator=( const AtlasTileGroupDataUnit& ) = default;

  void init() { patchInformationData_.clear(); }
  void allocate( size_t size ) { patchInformationData_.resize( size ); }

  void addPatchInformationData( PatchInformationData& value ) { patchInformationData_.push_back( value ); }
  PatchInformationData& addPatchInformationData( uint8_t patchMode ) {
    PatchInformationData pid;
    pid.setPatchMode( patchMode );
    patchInformationData_.push_back( pid );
    return patchInformationData_.back();
  }

  size_t  getFrameIndex() { return frameIndex_; }
  uint8_t getPatchMode( size_t index ) { return patchInformationData_[index].getPatchMode(); }
  uint8_t getPatchCount() { return patchInformationData_.size(); }
  size_t  getAtgduPatchMode() { return atgduPatchMode_; }

  PatchInformationData&              getPatchInformationData( size_t index ) { return patchInformationData_[index]; }
  std::vector<PatchInformationData>& getPatchInformationData() { return patchInformationData_; }
  size_t                             getMatchedPatchCount() {
    size_t matchedPatchCount = 0;
    for ( auto& v : patchInformationData_ ) {
      if ( v.getPatchMode() == PATCH_MODE_P_INTER ) { matchedPatchCount++; }
    }
    return matchedPatchCount;
  }

  void setFrameIndex( size_t value ) { frameIndex_ = value; }
  void setPatchCount( size_t value ) { patchCount_ = value; }
  void setAtgduPatchMode( size_t value ) { atgduPatchMode_ = value; }
  void setPatchInformationData( size_t index, PatchInformationData& value ) { patchInformationData_[index] = value; }

 private:
  size_t                            frameIndex_;
  size_t                            patchCount_;
  size_t                            atgduPatchMode_;
  std::vector<PatchInformationData> patchInformationData_;
};

};  // namespace pcc

#endif //~PCC_BITSTREAM_ATLASTILEGROUPDATAUNIT_H