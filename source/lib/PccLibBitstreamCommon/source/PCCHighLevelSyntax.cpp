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
#include "PCCBitstreamCommon.h"
#include "PCCHighLevelSyntax.h"

using namespace pcc;

PCCHighLevelSyntax::PCCHighLevelSyntax() : gofSize_( 0 ) {}

PCCHighLevelSyntax::~PCCHighLevelSyntax() {
  vpccParameterSets_.clear();
  atlasHLS_.clear();
}

size_t PCCAtlasHighLevelSyntax::getNumRefIdxActive( AtlasTileHeader& ath ) {
  size_t afpsId          = ath.getAtlasFrameParameterSetId();
  auto&  afps            = getAtlasFrameParameterSet( afpsId );
  size_t numRefIdxActive = 0;
  if ( ath.getType() == P_TILE || ath.getType() == SKIP_TILE ) {
    if ( ath.getNumRefIdxActiveOverrideFlag() ) {
      numRefIdxActive = ath.getNumRefIdxActiveMinus1() + 1;
    } else {
      auto& asps    = getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
      auto& refList = ath.getRefAtlasFrameListSpsFlag() ? asps.getRefListStruct( ath.getRefAtlasFrameListIdx() )
                                                        : ath.getRefListStruct();
      numRefIdxActive =
          static_cast<size_t>( ( std::min )( static_cast<int>( refList.getNumRefEntries() ),
                                             static_cast<int>( afps.getNumRefIdxDefaultActiveMinus1() ) + 1 ) );
    }
  }
  return numRefIdxActive;
}

void PCCAtlasHighLevelSyntax::printVideoBitstream() {
  size_t index = 0;
  printf( "VideoBitstream list: \n" );
  for ( auto& value : videoBitstream_ ) {
    printf( "  * %zu / %zu: ", index, videoBitstream_.size() );
    value.trace();
    index++;
  }
  fflush( stdout );
}

PCCAtlasHighLevelSyntax::PCCAtlasHighLevelSyntax() {
  activeAFPS_ = 0;
  activeASPS_ = 0;
}

PCCAtlasHighLevelSyntax::~PCCAtlasHighLevelSyntax() { videoBitstream_.clear(); }
