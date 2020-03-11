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

void PCCAtlasHighLevelSyntax::constructRefList( size_t aspsIdx, size_t afpsIdx ) {
  auto& asps = atlasSequenceParameterSet_[aspsIdx];
  // construction of reference frame list from ASPS refList (decoder)
  setNumOfRefAtlasFrameList( asps.getNumRefAtlasFrameListsInAsps() );
  for ( size_t list = 0; list < getNumOfRefAtlasFrameList(); list++ ) {
    auto& refList = asps.getRefListStruct( list );
    setMaxNumRefAtlasFrame( refList.getNumRefEntries() );
    setSizeOfRefAtlasFrameList( list, maxNumRefAtlasFrame_ );
    for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
      int  absDiff = refList.getAbsDeltaAfocSt( i );
      bool sign    = refList.getStrpfEntrySignFlag( i );
      setRefAtlasFrame( list, i, static_cast<int>(sign) == 0 ? ( -absDiff ) : absDiff );
    }
  }
}
size_t PCCAtlasHighLevelSyntax::getNumRefIdxActive( AtlasTileGroupHeader& atgh ) {
  size_t afpsId          = atgh.getAtghAtlasFrameParameterSetId();
  auto&  afps            = getAtlasFrameParameterSet( afpsId );
  size_t numRefIdxActive = 0;
  if ( atgh.getAtghType() == P_TILE_GRP || atgh.getAtghType() == SKIP_TILE_GRP ) {
    if ( atgh.getAtghNumRefIdxActiveOverrideFlag() ) {
      numRefIdxActive = atgh.getAtghNumRefIdxActiveMinus1() + 1;
    } else {
      auto& refList   = atgh.getRefListStruct();
      numRefIdxActive = static_cast< size_t>(( std::min )( static_cast<int>(refList.getNumRefEntries()),
                                                static_cast<int>(afps.getAfpsNumRefIdxDefaultActiveMinus1()) + 1 ));
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
