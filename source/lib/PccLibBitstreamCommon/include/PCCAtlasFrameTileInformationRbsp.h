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
#ifndef PCC_BITSTREAM_ATLASFRAMETILEINFORMATIONRBSP_H
#define PCC_BITSTREAM_ATLASFRAMETILEINFORMATIONRBSP_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 8.3.6.2.2 Atlas frame tile information Rsyntax
class AtlasFrameTileInformationRbsp {
 public:
  AtlasFrameTileInformationRbsp() :
      singleTileInAtlasFrameFlag_( 0 ),
      uniformPartitionSpacingFlag_( 1 ),
      numPartitionColumnsMinus1_( 0 ),
      numPartitionRowsMinus1_( 0 ),
      singlePartitionPerTileFlag_( 0 ),
      numTilesInAtlasFrameMinus1_( 0 ),
      signalledTileIdFlag_( 0 ),
      signalledTileIdLengthMinus1_( 0 ) {
    partitionColumnWidthMinus1_.resize( 1, 0 );
    partitionRowHeightMinus1_.resize( 1, 0 );
    topLeftPartitionIdx_.resize( 1, 0 );
    bottomRightPartitionColumnOffset_.resize( 1, 0 );
    bottomRightPartitionRowOffset_.resize( 1, 0 );
    tileId_.resize( 1, 0 );
    auxiliaryVideoTileRowHeight_.clear();
  };
  ~AtlasFrameTileInformationRbsp() {
    partitionColumnWidthMinus1_.clear();
    partitionRowHeightMinus1_.clear();
    topLeftPartitionIdx_.clear();
    bottomRightPartitionColumnOffset_.clear();
    bottomRightPartitionRowOffset_.clear();
    tileId_.clear();
    auxiliaryVideoTileRowHeight_.clear();
  };

  AtlasFrameTileInformationRbsp& operator=( const AtlasFrameTileInformationRbsp& ) = default;
  bool                       operator==( const AtlasFrameTileInformationRbsp& other ) const {
    if ( singleTileInAtlasFrameFlag_ != other.singleTileInAtlasFrameFlag_ ) {
      return false;
    } else if ( !singleTileInAtlasFrameFlag_ ) {
      if ( uniformPartitionSpacingFlag_ != other.uniformPartitionSpacingFlag_ ) { return false; }
      if ( uniformPartitionSpacingFlag_ ) {
        if ( partitionColumnWidthMinus1_[0] != other.partitionColumnWidthMinus1_[0] ) { return false; }
        if ( partitionRowHeightMinus1_[0] != other.partitionRowHeightMinus1_[0] ) { return false; }
      } else {
        if ( numPartitionColumnsMinus1_ != other.numPartitionColumnsMinus1_ ) { return false; }
        if ( numPartitionRowsMinus1_ != other.numPartitionRowsMinus1_ ) { return false; }

        for ( size_t i = 0; i <= numPartitionColumnsMinus1_; i++ ) {
          if ( partitionColumnWidthMinus1_[i] != other.partitionColumnWidthMinus1_[i] ) { return false; }
        }
        for ( size_t i = 0; i <= numPartitionRowsMinus1_; i++ ) {
          if ( partitionRowHeightMinus1_[i] != other.partitionRowHeightMinus1_[i] ) { return false; }
        }
      }
      if ( singlePartitionPerTileFlag_ != other.singlePartitionPerTileFlag_ ) { return false; }
      if ( numTilesInAtlasFrameMinus1_ != other.numTilesInAtlasFrameMinus1_ ) { return false; }
      if ( !singleTileInAtlasFrameFlag_ ) {
        for ( uint32_t i = 0; i < numTilesInAtlasFrameMinus1_ + 1; i++ ) {
          if ( topLeftPartitionIdx_[i] != other.topLeftPartitionIdx_[i] ) { return false; }
          if ( bottomRightPartitionColumnOffset_[i] != other.bottomRightPartitionColumnOffset_[i] ) { return false; }
          if ( bottomRightPartitionRowOffset_[i] != other.bottomRightPartitionRowOffset_[i] ) { return false; }
        }
      }
      if ( auxiliaryVideoTileRowHeight_.size() != other.auxiliaryVideoTileRowHeight_.size() ) { return false; }
      if ( auxiliaryVideoTileRowHeight_.size() != 0 ) {
        if ( auxiliaryVideoTileRowWidthMinus1_ != other.auxiliaryVideoTileRowWidthMinus1_ ) { return false; }
        for ( size_t ti = 0; ti < ( numTilesInAtlasFrameMinus1_ + 1 ); ti++ ) {
          if ( auxiliaryVideoTileRowHeight_[ti] != other.auxiliaryVideoTileRowHeight_[ti] ) { return false; }
        }
      }
    }
    return true;
  }
  bool     getSingleTileInAtlasFrameFlag() { return singleTileInAtlasFrameFlag_; }
  bool     getUniformPartitionSpacingFlag() { return uniformPartitionSpacingFlag_; }
  uint32_t getNumPartitionColumnsMinus1() { return numPartitionColumnsMinus1_; }
  uint32_t getNumPartitionRowsMinus1() { return numPartitionRowsMinus1_; }
  uint32_t getSinglePartitionPerTileFlag() { return singlePartitionPerTileFlag_; }
  uint32_t getNumTilesInAtlasFrameMinus1() { return numTilesInAtlasFrameMinus1_; }
  bool     getSignalledTileIdFlag() { return signalledTileIdFlag_; }
  uint32_t getSignalledTileIdLengthMinus1() { return signalledTileIdLengthMinus1_; }
  uint32_t getPartitionColumnWidthMinus1( size_t index ) { return partitionColumnWidthMinus1_[index]; }
  uint32_t getPartitionRowHeightMinus1( size_t index ) { return partitionRowHeightMinus1_[index]; }
  uint32_t getTopLeftPartitionIdx( size_t index ) { return topLeftPartitionIdx_[index]; }
  uint32_t getBottomRightPartitionColumnOffset( size_t index ) { return bottomRightPartitionColumnOffset_[index]; }
  uint32_t getBottomRightPartitionRowOffset( size_t index ) { return bottomRightPartitionRowOffset_[index]; }
  uint32_t getTileId( size_t index ) { return tileId_[index]; }
  uint32_t getAuxiliaryVideoTileRowWidthMinus1() { return auxiliaryVideoTileRowWidthMinus1_; }
  uint32_t getAuxiliaryVideoTileRowHeight( size_t index ) {
    return auxiliaryVideoTileRowHeight_.size() == 0 ? 0 : auxiliaryVideoTileRowHeight_[index];
  }

  void setSingleTileInAtlasFrameFlag( bool value ) { singleTileInAtlasFrameFlag_ = value; }
  void setUniformPartitionSpacingFlag( bool value ) { uniformPartitionSpacingFlag_ = value; }
  void setNumPartitionColumnsMinus1( uint32_t value ) {
    numPartitionColumnsMinus1_ = value;
    partitionColumnWidthMinus1_.resize( std::max( (uint32_t)1, value ), 0 );
  }
  void setNumPartitionRowsMinus1( uint32_t value ) {
    numPartitionRowsMinus1_ = value;
    partitionRowHeightMinus1_.resize( std::max( (uint32_t)1, value ), 0 );
  }
  void setSinglePartitionPerTileFlag( uint32_t value ) { singlePartitionPerTileFlag_ = value; }
  void setNumTilesInAtlasFrameMinus1( uint32_t value ) { numTilesInAtlasFrameMinus1_ = value; }
  void setSignalledTileIdFlag( bool value ) { signalledTileIdFlag_ = value; }
  void setSignalledTileIdLengthMinus1( uint32_t value ) { signalledTileIdLengthMinus1_ = value; }
  void setPartitionColumnWidthMinus1( size_t index, uint32_t value ) {
    if ( index == ( partitionColumnWidthMinus1_.size() ) ) {
      partitionColumnWidthMinus1_.resize( partitionColumnWidthMinus1_.size() + 1 );
    } else if ( index > partitionColumnWidthMinus1_.size() ) {
      assert( 0 );
    }
    partitionColumnWidthMinus1_[index] = value;
  }
  void setPartitionRowHeightMinus1( size_t index, uint32_t value ) {
    if ( index == ( partitionRowHeightMinus1_.size() ) )
      partitionRowHeightMinus1_.resize( partitionRowHeightMinus1_.size() + 1 );
    else if ( index > partitionRowHeightMinus1_.size() )
      assert( 0 );
    partitionRowHeightMinus1_[index] = value;
  }
  void setTopLeftPartitionIdx( size_t index, uint32_t value ) {
    if ( index == ( topLeftPartitionIdx_.size() ) )
      topLeftPartitionIdx_.resize( topLeftPartitionIdx_.size() + 1 );
    else if ( index > topLeftPartitionIdx_.size() )
      assert( 0 );
    topLeftPartitionIdx_[index] = value;
  }
  void setBottomRightPartitionColumnOffset( size_t index, uint32_t value ) {
    if ( index == ( bottomRightPartitionColumnOffset_.size() ) )
      bottomRightPartitionColumnOffset_.resize( bottomRightPartitionColumnOffset_.size() + 1 );
    else if ( index > bottomRightPartitionColumnOffset_.size() )
      assert( 0 );
    bottomRightPartitionColumnOffset_[index] = value;
  }
  void setBottomRightPartitionRowOffset( size_t index, uint32_t value ) {
    if ( index == ( bottomRightPartitionRowOffset_.size() ) )
      bottomRightPartitionRowOffset_.resize( bottomRightPartitionRowOffset_.size() + 1 );
    else if ( index > bottomRightPartitionRowOffset_.size() )
      assert( 0 );
    bottomRightPartitionRowOffset_[index] = value;
  }
  void setTileId( size_t index, uint32_t value ) {
    if ( index == ( tileId_.size() ) )
      tileId_.resize( tileId_.size() + 1 );
    else if ( index > tileId_.size() )
      assert( 0 );
    tileId_[index] = value;
  }
  void setAuxiliaryVideoTileRowWidthMinus1( uint32_t value ) { auxiliaryVideoTileRowWidthMinus1_ = value; }
  void setAuxiliaryVideoTileRowHeight( size_t index, uint32_t value ) {
    if ( index == ( auxiliaryVideoTileRowHeight_.size() ) )
      auxiliaryVideoTileRowHeight_.resize( auxiliaryVideoTileRowHeight_.size() + 1 );
    else if ( index > auxiliaryVideoTileRowHeight_.size() )
      assert( 0 );
    auxiliaryVideoTileRowHeight_[index] = value;
  }
  std::vector<size_t>& getPartitionWidth() { return colWidth_; }
  std::vector<size_t>& getPartitionHeight() { return rowHeight_; }
  std::vector<size_t>& getPartitionPosX() { return partitionPosX_; }
  std::vector<size_t>& getPartitionPosY() { return partitionPosY_; }
  size_t               getPartitionPosX( size_t index ) { return partitionPosX_[index]; }
  size_t               getPartitionPosY( size_t index ) { return partitionPosY_[index]; }
  size_t               getPartitionWidth( size_t index ) { return colWidth_[index]; }
  size_t               getPartitionHeight( size_t index ) { return rowHeight_[index]; }

 private:
  bool                  singleTileInAtlasFrameFlag_;
  bool                  uniformPartitionSpacingFlag_;
  uint32_t              numPartitionColumnsMinus1_;
  uint32_t              numPartitionRowsMinus1_;
  uint32_t              singlePartitionPerTileFlag_;
  uint32_t              numTilesInAtlasFrameMinus1_;
  bool                  signalledTileIdFlag_;
  uint32_t              signalledTileIdLengthMinus1_;
  std::vector<uint32_t> partitionColumnWidthMinus1_;
  std::vector<uint32_t> partitionRowHeightMinus1_;
  std::vector<uint32_t> topLeftPartitionIdx_;
  std::vector<uint32_t> bottomRightPartitionColumnOffset_;
  std::vector<uint32_t> bottomRightPartitionRowOffset_;
  std::vector<uint32_t> tileId_;
  uint32_t              auxiliaryVideoTileRowWidthMinus1_;
  std::vector<uint32_t> auxiliaryVideoTileRowHeight_;
  std::vector<size_t>   colWidth_;
  std::vector<size_t>   rowHeight_;
  std::vector<size_t>   partitionPosX_;
  std::vector<size_t>   partitionPosY_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_ATLASFRAMETILEINFORMATIONRBSP_H
