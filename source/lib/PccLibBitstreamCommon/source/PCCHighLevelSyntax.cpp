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



void PCCHighLevelSyntax::aspsCommonByteString( std::vector<uint8_t>& stringByte) {
  auto& asps = getAtlasSequenceParameterSet();
  uint8_t val  = asps.getFrameWidth() && 0xFF;
  stringByte.push_back( val );
  val = (asps.getFrameWidth() >> 8 ) && 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameWidth() >> 16 ) && 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameWidth() >> 24 ) && 0xFF;
  stringByte.push_back( val );
  val = asps.getFrameHeight() && 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameHeight() >> 8 ) && 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameHeight() >> 16 ) && 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameHeight() >> 24 ) && 0xFF;
  stringByte.push_back( val );
  val = asps.getGeometry3dBitdepthMinus1() && 0xFF;
  stringByte.push_back( val );
  val = asps.getGeometry2dBitdepthMinus1() && 0xFF;
  stringByte.push_back( val );
  val = asps.getMapCountMinus1() && 0xFF;
  stringByte.push_back( val );
  val = asps.getMaxNumberProjectionsMinus1() && 0xFF;
  stringByte.push_back( val );
  val = asps.getPatchPrecedenceOrderFlag() && 0xFF;
  stringByte.push_back( val ); 
}

void PCCHighLevelSyntax::aspsApplicationByteString( std::vector<uint8_t>& stringByte ) {
  auto&   asps = getAtlasSequenceParameterSet();
  uint8_t val;
  if ( asps.getPixelDeinterleavingFlag() ) {
    for ( int j = 0; j <= asps.getMapCountMinus1(); j++ ) {
      val = asps.getPixeDeinterleavingMapFlag( j ) && 0xFF;  // asps_map_pixel_deinterleaving_flag[ j ]
      stringByte.push_back( val );
    }
  }

  val = asps.getRawPatchEnabledFlag() && 0xFF;
  stringByte.push_back( val );
  val = asps.getEomPatchEnabledFlag() && 0xFF;
  stringByte.push_back( val );
  if ( asps.getEomPatchEnabledFlag() && asps.getMapCountMinus1() == 0 ) {
      val = asps.getEomFixBitCountMinus1() && 0xFF;
      stringByte.push_back( val );
   }
  if( asps.getAuxiliaryVideoEnabledFlag() && ( asps.getRawPatchEnabledFlag() || asps.getEomPatchEnabledFlag() ) ) {
      auto& afps = getAtlasFrameParameterSet();
      auto&  afti            = afps.getAtlasFrameTileInformation();
      size_t auxVideoWidthNF = ( afti.getAuxiliaryVideoTileRowWidthMinus1() + 1 ) * 64;
      uint8_t val             = auxVideoWidthNF && 0xFF; //val = AuxVideoWidthNF && 0xFF
      stringByte.push_back( val );
      val =( auxVideoWidthNF >> 8 ) && 0xFF;  //val = ( AuxVideoWidthNF >> 8 ) && 0xFF;
      stringByte.push_back( val );
      size_t  auxVideoHeightNF = 0;
      for ( int i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ )
          auxVideoHeightNF += ( afti.getAuxiliaryVideoTileRowHeight( i ) * 64 ); 
      val = auxVideoHeightNF && 0xFF;  //val = AuxVideoHeightNF && 0xFF
      stringByte.push_back( val );
      val = ( auxVideoHeightNF >> 8 ) && 0xFF; //( AuxVideoHeightNF >> 8 ) && 0xFF
      stringByte.push_back( val );
  }
  val = asps.getPLREnabledFlag() && 0xFF;
  stringByte.push_back( val );
  if ( asps.getPLREnabledFlag() ) {
    for ( int i = 0; i < asps.getMapCountMinus1() + 1; i++ ) {
      val = asps.getPLRInformation( i ).getMapEnabledFlag() && 0xFF; //plri_map_present_flag in the spec?
      stringByte.push_back( val );
      if ( asps.getPLRInformation( i ).getMapEnabledFlag() ) {
          val = asps.getPLRInformation( i ).getNumberOfModesMinus1() && 0xFF; 
          stringByte.push_back( val );
          for ( int j = 0; j < asps.getPLRInformation( i ).getNumberOfModesMinus1() + 1; j++ ) {
              val = asps.getPLRInformation( i ).getInterpolateFlag( j ) && 0xFF;
              stringByte.push_back( val );
              val = asps.getPLRInformation( i ).getFillingFlag( j ) && 0xFF;
              stringByte.push_back( val );
              val = asps.getPLRInformation( i ).getMinimumDepth( j ) && 0xFF;
              stringByte.push_back( val );
              val = asps.getPLRInformation( i ).getNeighbourMinus1( j ) && 0xFF;
              stringByte.push_back( val );
          }
          val = asps.getPLRInformation( i ).getBlockThresholdPerPatchMinus1() && 0xFF;
          stringByte.push_back( val );
      }
    }
  }
  auto& ext = asps.getAspsVpccExtension();
  val = ext.getRemoveDuplicatePointEnableFlag() && 0xFF;
  stringByte.push_back( val );
  if ( asps.getPixelDeinterleavingFlag() || asps.getPLREnabledFlag() ) {
  val = ext.getSurfaceThicknessMinus1() && 0xFF;
  stringByte.push_back( val );
  val = ( ext.getSurfaceThicknessMinus1() >> 8 ) && 0xFF;
  stringByte.push_back( val );
  }
}



void PCCHighLevelSyntax::afpsCommonByteString( std::vector<uint8_t>& stringByte ) {

  size_t  prevAuxTileOffset = 0;
  auto&   afps              = getAtlasFrameParameterSet();
  auto&   afti = afps.getAtlasFrameTileInformation();
  uint8_t val  = afti.getNumTilesInAtlasFrameMinus1() && 0xFF;
  stringByte.push_back( val );

  size_t auxVideoWidthNF = ( afti.getAuxiliaryVideoTileRowWidthMinus1() + 1 ) * 64;

  getTileOffsetAndSize();

  for ( int i = 1; i < afti.getNumTilesInAtlasFrameMinus1() + 1; i++ ) {
    size_t   topLeftColumn     = afti.getTopLeftPartitionIdx( i ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t   topLeftRow        = afti.getTopLeftPartitionIdx( i ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t   bottomRightColumn = topLeftColumn + afti.getBottomRightPartitionColumnOffset( i );
    size_t   bottomRightRow    = topLeftRow + afti.getBottomRightPartitionRowOffset( i );
    uint32_t tileOffsetX       = afti.getPartitionPosX( topLeftColumn );
    uint32_t tileOffsetY       = afti.getPartitionPosY( topLeftColumn );
    size_t   tileWidth         = 0;
    size_t   tileHeight        = 0;

    for ( int j = topLeftColumn; j <= bottomRightColumn; j++ ) {
      tileWidth += ( afti.getPartitionColumnWidthMinus1( j ) + 1 );
    }

    for ( int j = topLeftRow; j <= bottomRightRow; j++ ) { 
        tileHeight += ( afti.getPartitionRowHeightMinus1( j ) + 1 );
    }

    size_t auxTileHeight = afti.getAuxiliaryVideoTileRowHeight( i );
    size_t auxTileOffset = prevAuxTileOffset + auxTileHeight;
    prevAuxTileOffset    = auxTileOffset;

    val = tileOffsetX && 0xFF;
    stringByte.push_back( val );  //val = TileOffsetX[i] && 0xFF;
    val = ( tileOffsetX >> 8 ) && 0xFF;
    stringByte.push_back( val );  //val = ( TileOffsetX[i] >> 8 ) && 0xFF;
    val = tileOffsetY && 0xFF;
    stringByte.push_back( val ); //val = TileOffsetY[i] && 0xFF;
    val = ( tileOffsetY >> 8 ) && 0xFF;
    stringByte.push_back( val ); // val = ( TileOffsetY[i] >> 8 ) && 0xFF;
    val = tileWidth && 0xFF;
    stringByte.push_back( val ); //val = TileWidth[i] && 0xFF;
    val = ( tileWidth >> 8 ) && 0xFF;
    stringByte.push_back( val );  //val = ( TileWidth[i] >> 8 ) && 0xFF;
    val = tileHeight && 0xFF;
    stringByte.push_back( val ); //val = TileHeight[i] && 0xFF;
    val = ( tileHeight >> 8 ) && 0xFF;
    stringByte.push_back( val );  //val = ( TileHeight[i] >> 8 ) && 0xFF;
    val = auxTileOffset && 0xFF;
    stringByte.push_back( val );   //val = AuxTileOffset[i] && 0xFF;
    val = ( auxTileOffset >> 8 ) && 0xFF;
    stringByte.push_back( val ); //val = ( AuxTileOffset[i] >> 8 ) && 0xFF;
    val = auxVideoWidthNF && 0xFF;
    stringByte.push_back( val ); //val = AuxTileWidth[i] && 0xFF;
    val = ( auxVideoWidthNF >> 8 ) && 0xFF;
    stringByte.push_back( val ); //val = (AuxTileWidth[i] >> 8 ) && 0xFF
    val = auxTileHeight && 0xFF;
    stringByte.push_back( val ); //val = AuxTileHeight[i] && 0xFF;
    val = ( auxTileHeight >> 8 ) && 0xFF;
    stringByte.push_back( val );  //val = AuxTileHeight[i] && 0xFF;
    val = afps.getAtlasFrameTileInformation().getTileId( i ) && 0xFF; //
    stringByte.push_back( val );
    val = ( afps.getAtlasFrameTileInformation().getTileId( i ) >> 8 ) && 0xFF;
    stringByte.push_back( val );
  }
}

void PCCHighLevelSyntax::afpsApplicationByteString( std::vector<uint8_t>& stringByte ) {
}

void PCCHighLevelSyntax::getTileOffsetAndSize() {
    auto& asps = getAtlasSequenceParameterSet();
    auto& afps = getAtlasFrameParameterSet();
    auto& afti = afps.getAtlasFrameTileInformation();
    afti.initializePartitionPosX( asps.getFrameWidth() );
    afti.initializePartitionPosY( asps.getFrameHeight() );
    afti.initializeTileOffsetAndSize();
}
