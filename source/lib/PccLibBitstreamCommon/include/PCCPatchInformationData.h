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
#ifndef PCC_BITSTREAM_PATCHINFORMATIONDATA_H
#define PCC_BITSTREAM_PATCHINFORMATIONDATA_H

#include "PCCBitstreamCommon.h"
#include "PCCPatchDataUnit.h"
#include "PCCInterPatchDataUnit.h"
#include "PCCMergePatchDataUnit.h"
#include "PCCMergePatchDataUnit.h"
#include "PCCSkipPatchDataUnit.h"
#include "PCCRawPatchDataUnit.h"
#include "PCCEOMPatchDataUnit.h"

namespace pcc {

// 8.3.7.2  Patch information data syntax (pid)
class PatchInformationData {
 public:
  PatchInformationData(){};
  ~PatchInformationData(){};
  PatchInformationData& operator=( const PatchInformationData& ) = default;

  size_t              getTileOrder() { return tileOrder_; }
  size_t              getPatchIndex() { return patchIndex_; }
  uint8_t             getPatchMode() { return patchMode_; }
  PatchDataUnit&      getPatchDataUnit() { return patchDataUnit_; }
  InterPatchDataUnit& getInterPatchDataUnit() { return interPatchDataUnit_; }
  MergePatchDataUnit& getMergePatchDataUnit() { return mergePatchDataUnit_; }
  SkipPatchDataUnit&  getSkipPatchDataUnit() { return skipPatchDataUnit_; }
  RawPatchDataUnit&   getRawPatchDataUnit() { return rawPatchyDataUnit_; }
  EOMPatchDataUnit&   getEomPatchDataUnit() { return eomPatchDataUnit_; }

  void setTileOrder( size_t value ) { tileOrder_ = value; }
  void setPatchIndex( size_t value ) { patchIndex_ = value; }
  void setPatchMode( uint8_t value ) { patchMode_ = value; }
  void setPatchDataUnit( PatchDataUnit& value ) { patchDataUnit_ = value; }
  void setInterPatchDataUnit( InterPatchDataUnit& value ) { interPatchDataUnit_ = value; }
  void setMergePatchDataUnit( MergePatchDataUnit& value ) { mergePatchDataUnit_ = value; }
  void setSkipPatchDataUnit( SkipPatchDataUnit& value ) { skipPatchDataUnit_ = value; }
  void setRawPatchDataUnit( RawPatchDataUnit& value ) { rawPatchyDataUnit_ = value; }
  void setEomPatchDataUnit( EOMPatchDataUnit& value ) { eomPatchDataUnit_ = value; }

 private:
  size_t             tileOrder_;
  size_t             patchIndex_;
  uint8_t            patchMode_;
  PatchDataUnit      patchDataUnit_;
  InterPatchDataUnit interPatchDataUnit_;
  MergePatchDataUnit mergePatchDataUnit_;
  SkipPatchDataUnit  skipPatchDataUnit_;
  RawPatchDataUnit   rawPatchyDataUnit_;
  EOMPatchDataUnit   eomPatchDataUnit_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_PATCHINFORMATIONDATA_H
