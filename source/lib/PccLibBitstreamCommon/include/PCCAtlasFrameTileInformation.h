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
#ifndef PCC_BITSTREAM_ATLASFRAMETILEINFORMATION_H
#define PCC_BITSTREAM_ATLASFRAMETILEINFORMATION_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 7.3.6.4  Atlas frame tile information syntax
class AtlasFrameTileInformation {
 public:
  AtlasFrameTileInformation() :
      singleTileInAtlasFrameFlag_( 0 ),
      uniformPartitionSpacingFlag_( 0 ),
      numPartitionColumnsMinus1_( 0 ),
      numPartitionRowsMinus1_( 0 ),
      singlePartitionPerTileFlag_( 0 ),
      numTilesInAtlasFrameMinus1_( 0 ),
      signalledTileIdFlag_( 0 ),
      signalledTileIdLengthMinus1_( 0 ) {
    partitionColsWidthMinus1_.resize( 1, 0 );
    partitionRowsHeightMinus1_.resize( 1, 0 );
    topLeftPartitionIdx_.resize( 1, 0 );
    bottomRightPartitionColumnOffset_.resize( 1, 0 );
    bottomRightPartitionRowOffset_.resize( 1, 0 );
    tileId_.resize( 1, 0 );
  };
  ~AtlasFrameTileInformation() {
    partitionColsWidthMinus1_.clear();
    partitionRowsHeightMinus1_.clear();
    topLeftPartitionIdx_.clear();
    bottomRightPartitionColumnOffset_.clear();
    bottomRightPartitionRowOffset_.clear();
    tileId_.clear();
    auxiliaryVideoTileRowHeight_.clear();
  };

  AtlasFrameTileInformation& operator=( const AtlasFrameTileInformation& ) = default;

  bool     getSingleTileInAtlasFrameFlag() { return singleTileInAtlasFrameFlag_; }
  bool     getUniformPartitionSpacingFlag() { return uniformPartitionSpacingFlag_; }
  uint32_t getNumPartitionColumnsMinus1() { return numPartitionColumnsMinus1_; }
  uint32_t getNumPartitionRowsMinus1() { return numPartitionRowsMinus1_; }
  uint32_t getSinglePartitionPerTileFlag() { return singlePartitionPerTileFlag_; }
  uint32_t getNumTilesInAtlasFrameMinus1() { return numTilesInAtlasFrameMinus1_; }
  bool     getSignalledTileIdFlag() { return signalledTileIdFlag_; }
  uint32_t getSignalledTileIdLengthMinus1() { return signalledTileIdLengthMinus1_; }
  uint32_t getPartitionColsWidthMinus1( size_t index ) { return partitionColsWidthMinus1_[index]; }
  uint32_t getPartitionRowsHeightMinus1( size_t index ) { return partitionRowsHeightMinus1_[index]; }
  uint32_t getTopLeftPartitionIdx( size_t index ) { return topLeftPartitionIdx_[index]; }
  uint32_t getBottomRightPartitionColumnOffset( size_t index ) { return bottomRightPartitionColumnOffset_[index]; }
  uint32_t getBottomRightPartitionRowOffset( size_t index ) { return bottomRightPartitionRowOffset_[index]; }
  uint32_t getTileId( size_t index ) { return tileId_[index]; }
  uint32_t getAuxiliaryVideoTileRowWidthMinus1() { return auxiliaryVideoTileRowWidthMinus1_; }
  uint32_t getAuxiliaryVideoTileRowHeight( size_t index ) { return auxiliaryVideoTileRowHeight_[index]; }

  void setSingleTileInAtlasFrameFlag( bool value ) { singleTileInAtlasFrameFlag_ = value; }
  void setUniformPartitionSpacingFlag( bool value ) { uniformPartitionSpacingFlag_ = value; }
  void setNumPartitionColumnsMinus1( uint32_t value ) { numPartitionColumnsMinus1_ = value; }
  void setNumPartitionRowsMinus1( uint32_t value ) { numPartitionRowsMinus1_ = value; }
  void setSinglePartitionPerTileFlag( uint32_t value ) { singlePartitionPerTileFlag_ = value; }
  void setNumTilesInAtlasFrameMinus1( uint32_t value ) { numTilesInAtlasFrameMinus1_ = value; }
  void setSignalledTileIdFlag( bool value ) { signalledTileIdFlag_ = value; }
  void setSignalledTileIdLengthMinus1( uint32_t value ) { signalledTileIdLengthMinus1_ = value; }

  void setPartitionColsWidthMinus1( size_t index, uint32_t value ) {
    if ( index == ( partitionColsWidthMinus1_.size() ) )
      partitionColsWidthMinus1_.resize( partitionColsWidthMinus1_.size() + 1 );
    else if ( index > partitionColsWidthMinus1_.size() )
      assert( 0 );
    partitionColsWidthMinus1_[index] = value;
  }
  void setPartitionRowsHeightMinus1( size_t index, uint32_t value ) {
    if ( index == ( partitionRowsHeightMinus1_.size() ) )
      partitionRowsHeightMinus1_.resize( partitionRowsHeightMinus1_.size() + 1 );
    else if ( index > partitionRowsHeightMinus1_.size() )
      assert( 0 );
    partitionRowsHeightMinus1_[index] = value;
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
  void setAuxiliaryVideoTileRowHeight( size_t index, uint32_t value ) { auxiliaryVideoTileRowHeight_[index] = value; }

 private:
  bool                  singleTileInAtlasFrameFlag_;
  bool                  uniformPartitionSpacingFlag_;
  uint32_t              numPartitionColumnsMinus1_;
  uint32_t              numPartitionRowsMinus1_;
  uint32_t              singlePartitionPerTileFlag_;
  uint32_t              numTilesInAtlasFrameMinus1_;
  bool                  signalledTileIdFlag_;
  uint32_t              signalledTileIdLengthMinus1_;
  std::vector<uint32_t> partitionColsWidthMinus1_;
  std::vector<uint32_t> partitionRowsHeightMinus1_;
  std::vector<uint32_t> topLeftPartitionIdx_;
  std::vector<uint32_t> bottomRightPartitionColumnOffset_;
  std::vector<uint32_t> bottomRightPartitionRowOffset_;
  std::vector<uint32_t> tileId_;
  uint32_t              auxiliaryVideoTileRowWidthMinus1_;
  std::vector<uint32_t> auxiliaryVideoTileRowHeight_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_ATLASFRAMETILEINFORMATION_H