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
    geometry2dNorminalBitdepth_( 8 ) {
  log2PatchQuantizerSizeX_         = 4;
  log2PatchQuantizerSizeY_         = 4;
  numOfAvailableRefAtlasFrameList_ = 1;
  activeRefAtlasFrameIndex_        = 0;
  refAFOCList_.resize( 0 );
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

void PCCFrameContext::setRefAFOCList( std::vector<std::vector<size_t>>& list ) {
  size_t listSize = ( std::min )( refAFOCList_.size(), list.size() );
  for ( size_t i = 0; i < listSize; i++ ) {
    size_t refSize = ( std::min )( refAFOCList_[i].size(), list[i].size() );
    for ( size_t j = 0; j < refSize; j++ ) { refAFOCList_[i][j] = list[i][j]; }
  }
}
void PCCFrameContext::setRefAFOCList( PCCContext& context ) {
  numOfAvailableRefAtlasFrameList_ = context.getNumOfRefAtlasFrameList();
  refAFOCList_.resize( numOfAvailableRefAtlasFrameList_ );
  int refPOC = 0;
  for ( size_t i = 0; i < numOfAvailableRefAtlasFrameList_; i++ ) {
    size_t maxRefNum = context.getSizeOfRefAtlasFrameList( i );
    for ( size_t j = 0; j < maxRefNum; j++ ) {
      refPOC = int( index_ ) + int( context.getRefAtlasFrame( i, j ) );
      if ( refPOC >= 0 ) { refAFOCList_[i].push_back( refPOC ); }
    }
    if ( refAFOCList_[i].empty() ) { refAFOCList_[i].push_back( 255 ); }
  }
}

void PCCFrameContext::constructAtghRefListStruct( PCCContext& context, AtlasTileGroupHeader& atgh ) {
  size_t afpsId = atgh.getAtghAtlasFrameParameterSetId();
  auto&  afps   = context.getAtlasFrameParameterSet( afpsId );
  size_t aspsId = afps.getAtlasSequenceParameterSetId();
  auto&  asps   = context.getAtlasSequenceParameterSet( aspsId );
  atgh.setAtghRefAtlasFrameListSpsFlag( true );                       // using ASPS refList
  atgh.setAtghRefAtlasFrameListIdx( getActiveRefAtlasFrameIndex() );  // atgh.atgh_ref_atlas_frame_list_idx
  atgh.setRefListStruct( asps.getRefListStruct( atgh.getAtghRefAtlasFrameListIdx() ) );  // copied to atgh's refList,
                                                                                         // not signalled

  if ( index_ <= afps.getAfpsNumRefIdxDefaultActiveMinus1() ) {  // 3
    atgh.setAtghNumRefIdxActiveOverrideFlag( true );
    atgh.setAtghNumRefIdxActiveMinus1( index_ == 0 ? 0 : ( index_ - 1 ) );
  } else {
    atgh.setAtghNumRefIdxActiveOverrideFlag( false );
  }
}

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
