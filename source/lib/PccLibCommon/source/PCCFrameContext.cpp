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
    index_( 0 ),
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
#if REFERENCELIST_BUGFIX
  bestRefListIndexInAsps_ = 0;
#else
  numOfAvailableRefAtlasFrameList_ = 1;
  activeRefAtlasFrameIndex_        = 0;
#endif
  leftTopXInFrame_ = 0;
  leftTopYInFrame_ = 0;
  refAFOCList_.resize( 0 );
  tileIndex_ = 0;
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
#if REFERENCELIST_BUGFIX
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
  for ( size_t idx = 0; idx < listSize; idx++ ) {  // jkei: numActive
    // RefAtlasFrmAfocList[ j ] = afocBase − DeltaAfocSt[ RlsIdx ][ j ]
    int deltaAfocSt = 0;
    if ( refList.getStRefAtalsFrameFlag( idx ) )
      deltaAfocSt = ( 2 * refList.getStrafEntrySignFlag( idx ) - 1 ) * refList.getAbsDeltaAfocSt( idx );  // Eq.26
    int refPOC = idx == 0 ? ( int( index_ ) - deltaAfocSt ) : ( refAFOCList_[idx - 1] - deltaAfocSt );
    if ( refPOC >= 0 ) refAFOCList_.push_back( refPOC );
  }
}

void PCCFrameContext::setRefAfocList( PCCContext& context, size_t refListIdx ) {
  int    refPOC    = 0;
  size_t maxRefNum = context.getSizeOfRefAtlasFrameList( refListIdx );
  refAFOCList_.clear();
  for ( size_t j = 0; j < maxRefNum; j++ ) {
    refPOC = int( index_ ) - int( context.getRefAtlasFrame( refListIdx, j ) );
    if ( refPOC >= 0 ) { refAFOCList_.push_back( refPOC ); }
  }
  // if ( refAFOCList_.empty() ) { refAFOCList_.push_back( 255 ); } //jkei: size is used for signalling
}

void PCCFrameContext::constructAtghRefListStruct( PCCContext& context, AtlasTileHeader& ath ) {
  size_t afpsId = ath.getAtlasFrameParameterSetId();
  auto&  afps   = context.getAtlasFrameParameterSet( afpsId );
  ath.setRefAtlasFrameListSpsFlag( true );                     // using ASPS refList
  ath.setRefAtlasFrameListIdx( getBestRefListIndexInAsps() );  // ath.atgh_ref_atlas_frame_list_idx

  // jkei: do we need this?
  // ath.setRefListStruct( asps.getRefListStruct( ath.getRefAtlasFrameListIdx() ) );  // copied to ath's refList,
  // not signalled
  if ( !ath.getRefAtlasFrameListSpsFlag() ) {
    RefListStruct refList;
    refList.setNumRefEntries( getRefAfocListSize() );
    refList.allocate();
    for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
      int afocDiff = -1;
      if ( i == 0 )
        afocDiff = index_ - getRefAfoc( i );  // index_+1,index_+2,index_+3,index_+4
      else
        afocDiff =
            getRefAfoc( i - 1 ) - getRefAfoc( i );  // RefAtlasFrmAfocList[ j ] = afocBase − DeltaAfocSt[ RlsIdx ][ j ]
      refList.setAbsDeltaAfocSt( i, std::abs( afocDiff ) );
      refList.setStrafEntrySignFlag( i, afocDiff < 0 ? false : !false );
      refList.setStRefAtalsFrameFlag( i, true );
    }
    ath.setRefListStruct( refList );
  }
  // jkei: FDIS 7.3.6.11
  if ( numRefIdxActive_ > 0 ) {
    bool bNumRefIdxActiveOverrideFlag = false;  //# is same as context
    if ( getRefAfocListSize() >= ( afps.getNumRefIdxDefaultActiveMinus1() ) )
      bNumRefIdxActiveOverrideFlag = ( numRefIdxActive_ != afps.getNumRefIdxDefaultActiveMinus1() + 1 );
    else
      bNumRefIdxActiveOverrideFlag = numRefIdxActive_ != context.getMaxNumRefAtlasFrame( getBestRefListIndexInAsps() );
    ath.setNumRefIdxActiveOverrideFlag( bNumRefIdxActiveOverrideFlag );
    if ( ath.getNumRefIdxActiveOverrideFlag() ) ath.setNumRefIdxActiveMinus1( numRefIdxActive_ - 1 );
  }
}
#else
// void PCCFrameContext::setRefAFOCList( std::vector<std::vector<size_t>>& list ) {
//  size_t listSize = ( std::min )( refAFOCList_.size(), list.size() );
//  for ( size_t i = 0; i < listSize; i++ ) {
//    size_t refSize = ( std::min )( refAFOCList_[i].size(), list[i].size() );
//    for ( size_t j = 0; j < refSize; j++ ) { refAFOCList_[i][j] = list[i][j]; }
//  }
//}
// void PCCFrameContext::setRefAFOCList( PCCContext& context ) {
//  numOfAvailableRefAtlasFrameList_ = context.getNumOfRefAtlasFrameList();
//  refAFOCList_.resize( numOfAvailableRefAtlasFrameList_ );
//  int refPOC = 0;
//  for ( size_t i = 0; i < numOfAvailableRefAtlasFrameList_; i++ ) {
//    size_t maxRefNum = context.getSizeOfRefAtlasFrameList( i );
//    for ( size_t j = 0; j < maxRefNum; j++ ) {
//      refPOC = int( index_ ) + int( context.getRefAtlasFrame( i, j ) );
//      if ( refPOC >= 0 ) { refAFOCList_[i].push_back( refPOC ); }
//    }
//    if ( refAFOCList_[i].empty() ) { refAFOCList_[i].push_back( 255 ); }
//  }
//}
//
// void PCCFrameContext::constructAtghRefListStruct( PCCContext& context, AtlasTileHeader& ath ) {
//  size_t afpsId = ath.getAtlasFrameParameterSetId();
//  auto&  afps   = context.getAtlasFrameParameterSet( afpsId );
//  size_t aspsId = afps.getAtlasSequenceParameterSetId();
//  auto&  asps   = context.getAtlasSequenceParameterSet( aspsId );
//  ath.setRefAtlasFrameListSpsFlag( true );                                         // using ASPS refList
//  ath.setRefAtlasFrameListIdx( getActiveRefAtlasFrameIndex() );                    //
//  ath.atgh_ref_atlas_frame_list_idx ath.setRefListStruct( asps.getRefListStruct( ath.getRefAtlasFrameListIdx() ) ); //
//  copied to ath's refList,
//                                                                                   // not signalled
//  if ( index_ <= afps.getNumRefIdxDefaultActiveMinus1() ) {                        // 3
//    ath.setNumRefIdxActiveOverrideFlag( true );
//    ath.setNumRefIdxActiveMinus1( index_ == 0 ? 0 : ( index_ - 1 ) );
//  } else {
//    ath.setNumRefIdxActiveOverrideFlag( false );
//  }
//}
#endif
void PCCFrameContext::allocOneLayerData() {
  for ( auto& patch : patches_ ) { patch.allocOneLayerData(); }
}
void PCCFrameContext::printBlockToPatch( const size_t resolution ) {
  printVector( blockToPatch_, width_ / resolution, height_ / resolution, stringFormat( "blockToPatch[%d]", index_ ),
               true );
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
