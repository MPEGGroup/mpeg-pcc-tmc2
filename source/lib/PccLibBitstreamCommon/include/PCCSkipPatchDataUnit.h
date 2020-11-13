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
#ifndef PCC_BITSTREAM_SKIPPATCHDATAUNIT_H
#define PCC_BITSTREAM_SKIPPATCHDATAUNIT_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 8.3.7.4  Skip patch data unit syntax
class SkipPatchDataUnit {
 public:
  SkipPatchDataUnit() : patchIndex_( 0 ), frameIndex_( 0 ), tileOrder_( 0 ){};
  ~SkipPatchDataUnit(){};
  SkipPatchDataUnit& operator=( const SkipPatchDataUnit& ) = default;
  size_t             getPatchIndex() { return patchIndex_; }
  size_t             getFrameIndex() { return frameIndex_; }
  size_t             getTileOrder() { return tileOrder_; }
  void               setPatchIndex( size_t value ) { patchIndex_ = value; }
  void               setFrameIndex( size_t value ) { frameIndex_ = value; }
  void               setTileOrder( size_t value ) { tileOrder_ = value; }

 private:
  size_t patchIndex_;
  size_t frameIndex_;
  size_t tileOrder_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_SKIPPATCHDATAUNIT_H
