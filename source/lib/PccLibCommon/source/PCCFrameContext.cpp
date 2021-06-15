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
#include "PCCCommon.h"
#include "PCCPatch.h"
#include "PCCFrameContext.h"

using namespace pcc;

PCCFrameContext::PCCFrameContext() :
    frameIndex_( 0 ),
    numMatchedPatches_( 0 ),
    width_( 0 ),
    height_( 0 ),
    numberOfRawPointsPatches_( 0 ),
    totalNumberOfRawPoints_( 0 ),
    totalNumberOfEOMPoints_( 0 ),
    totalNumberOfRegularPoints_( 0 ),
    globalPatchCount_( 0 ),
    geometry3dCoordinatesBitdepth_( 10 ),
    pointLocalReconstructionNumber_( 0 ),
    useRawPointsSeparateVideo_( false ),
    rawPatchEnabledFlag_( false ),
    geometry2dBitdepth_( 8 ) {
  log2PatchQuantizerSizeX_ = 4;
  log2PatchQuantizerSizeY_ = 4;
  bestRefListIndexInAsps_  = 0;
  leftTopXInFrame_         = 0;
  leftTopYInFrame_         = 0;
  refAFOCList_.resize( 0 );
  tileIndex_    = 0;
  referredTile_ = false;
}

PCCFrameContext::~PCCFrameContext() {
  pointToPixel_.clear();
  blockToPatch_.clear();
  occupancyMap_.clear();
  fullOccupancyMap_.clear();
  patches_.clear();
  srcPointCloudByPatch_.clear();
  srcPointCloudByBlock_.clear();
  recPointCloudByBlock_.clear();
  for ( auto& block : pointToPixelByBlock_ ) { block.clear(); }
  pointToPixelByBlock_.clear();
}

void PCCFrameContext::setRefAfocList( PCCContext& context, AtlasTileHeader& ath, size_t afpsIndex ) {
  // decoder, per tile
  auto&         afps   = context.getAtlasFrameParameterSet( afpsIndex );
  size_t        aspsId = afps.getAtlasSequenceParameterSetId();
  auto&         asps   = context.getAtlasSequenceParameterSet( aspsId );
  RefListStruct refList;
  if ( ath.getRefAtlasFrameListSpsFlag() == 0 ) {
    refList = ath.getRefListStruct();
  } else {
    size_t refListIdx = ath.getRefAtlasFrameListIdx();
    refList           = asps.getRefListStruct( refListIdx );
  }
  refAFOCList_.clear();
  size_t listSize = refList.getNumRefEntries();
  for ( size_t idx = 0; idx < listSize; idx++ ) {
    int deltaAfocSt = 0;
    if ( refList.getStRefAtalsFrameFlag( idx ) )
      deltaAfocSt = ( 2 * refList.getStrafEntrySignFlag( idx ) - 1 ) * refList.getAbsDeltaAfocSt( idx );  // Eq.26
    int refPOC = idx == 0 ? ( int( frameIndex_ ) - deltaAfocSt ) : ( refAFOCList_[idx - 1] - deltaAfocSt );
    if ( refPOC >= 0 ) refAFOCList_.push_back( refPOC );
  }
}

void PCCFrameContext::setRefAfocList( PCCContext& context, size_t refListIdx ) {
  size_t maxRefNum = context.getSizeOfRefAtlasFrameList( refListIdx );
  refAFOCList_.clear();
  for ( size_t j = 0; j < maxRefNum; j++ ) {
    int refPOC = int( frameIndex_ ) - int( context.getRefAtlasFrame( refListIdx, j ) );
    if ( refPOC >= 0 ) { refAFOCList_.push_back( refPOC ); }
  }
}

void PCCFrameContext::constructAtghRefListStruct( PCCContext& context, AtlasTileHeader& ath ) {
  size_t afpsId = ath.getAtlasFrameParameterSetId();
  auto&  afps   = context.getAtlasFrameParameterSet( afpsId );
  ath.setRefAtlasFrameListSpsFlag( true );
  ath.setRefAtlasFrameListIdx( getBestRefListIndexInAsps() );
  if ( !ath.getRefAtlasFrameListSpsFlag() ) {
    RefListStruct refList;
    refList.setNumRefEntries( getRefAfocListSize() );
    refList.allocate();
    for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
      int afocDiff = -1;
      if ( i == 0 ) {
        afocDiff = frameIndex_ - getRefAfoc( i );
      } else {
        afocDiff = getRefAfoc( i - 1 ) - getRefAfoc( i );
      }
      refList.setAbsDeltaAfocSt( i, std::abs( afocDiff ) );
      refList.setStrafEntrySignFlag( i, afocDiff < 0 ? false : true );
      refList.setStRefAtalsFrameFlag( i, true );
    }
    ath.setRefListStruct( refList );
  }
  if ( numRefIdxActive_ > 0 ) {
    bool bNumRefIdxActiveOverrideFlag = false;
    if ( getRefAfocListSize() >= ( afps.getNumRefIdxDefaultActiveMinus1() ) ) {
      bNumRefIdxActiveOverrideFlag = ( numRefIdxActive_ != afps.getNumRefIdxDefaultActiveMinus1() + 1 );
    } else {
      bNumRefIdxActiveOverrideFlag = numRefIdxActive_ != context.getMaxNumRefAtlasFrame( getBestRefListIndexInAsps() );
    }
    ath.setNumRefIdxActiveOverrideFlag( bNumRefIdxActiveOverrideFlag );
    if ( ath.getNumRefIdxActiveOverrideFlag() ) ath.setNumRefIdxActiveMinus1( numRefIdxActive_ - 1 );
  }
}

void PCCFrameContext::allocOneLayerData() {
  for ( auto& patch : patches_ ) { patch.allocOneLayerData(); }
}

void PCCFrameContext::printBlockToPatch( const size_t resolution ) {
  printVector( blockToPatch_, width_ / resolution, height_ / resolution,
               stringFormat( "blockToPatch[%d]", frameIndex_ ), true );
}
void PCCFrameContext::printPatch() {
  size_t index = 0;
  printf( "Patch %4zu:", patches_.size() );
  for ( auto& patch : patches_ ) {
    printf( "  Patch[%4zu]: ", index++ );
    patch.print();
  }
  fflush( stdout );
}

void PCCFrameContext::printPatchDecoder() {
  size_t index = 0;
  printf( "Patch %4zu:", patches_.size() );
  for ( auto& patch : patches_ ) {
    printf( "  Patch[%4zu]: ", index++ );
    patch.printDecoder();
  }
  fflush( stdout );
}
