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
      uniformTileSpacingFlag_( 0 ),
      numTileColumnsMinus1_( 0 ),
      numTileRowsMinus1_( 0 ),
      singleTilePerTileGroupFlag_( 0 ),
      numTileGroupsInAtlasFrameMinus1_( 0 ),
      signalledTileGroupIdFlag_( 0 ),
      signalledTileGroupIdLengthMinus1_( 0 ) {
    tileColumnWidthMinus1_.resize( 1, 0 );
    tileRowHeightMinus1_.resize( 1, 0 );
    topLeftTileIdx_.resize( 1, 0 );
    bottomRightTileIdxDelta_.resize( 1, 0 );
    tileGroupId_.resize( 1, 0 );
  };
  ~AtlasFrameTileInformation() {
    tileColumnWidthMinus1_.clear();
    tileRowHeightMinus1_.clear();
    topLeftTileIdx_.clear();
    bottomRightTileIdxDelta_.clear();
    tileGroupId_.clear();
  };

  AtlasFrameTileInformation& operator=( const AtlasFrameTileInformation& ) = default;

  bool     getSingleTileInAtlasFrameFlag() { return singleTileInAtlasFrameFlag_; }
  bool     getUniformTileSpacingFlag() { return uniformTileSpacingFlag_; }
  uint32_t getNumTileColumnsMinus1() { return numTileColumnsMinus1_; }
  uint32_t getNumTileRowsMinus1() { return numTileRowsMinus1_; }
  uint32_t getSingleTilePerTileGroupFlag() { return singleTilePerTileGroupFlag_; }
  uint32_t getNumTileGroupsInAtlasFrameMinus1() { return numTileGroupsInAtlasFrameMinus1_; }
  bool     getSignalledTileGroupIdFlag() { return signalledTileGroupIdFlag_; }
  uint32_t getSignalledTileGroupIdLengthMinus1() { return signalledTileGroupIdLengthMinus1_; }
  uint32_t getTileColumnWidthMinus1( size_t index ) { return tileColumnWidthMinus1_[index]; }
  uint32_t getTileRowHeightMinus1( size_t index ) { return tileRowHeightMinus1_[index]; }
  uint32_t getTopLeftTileIdx( size_t index ) { return topLeftTileIdx_[index]; }
  uint32_t getBottomRightTileIdxDelta( size_t index ) { return bottomRightTileIdxDelta_[index]; }
  uint32_t getTileGroupId( size_t index ) { return tileGroupId_[index]; }

  void setSingleTileInAtlasFrameFlag( bool value ) { singleTileInAtlasFrameFlag_ = value; }
  void setUniformTileSpacingFlag( bool value ) { uniformTileSpacingFlag_ = value; }
  void setNumTileColumnsMinus1( uint32_t value ) { numTileColumnsMinus1_ = value; }
  void setNumTileRowsMinus1( uint32_t value ) { numTileRowsMinus1_ = value; }
  void setSingleTilePerTileGroupFlag( uint32_t value ) { singleTilePerTileGroupFlag_ = value; }
  void setNumTileGroupsInAtlasFrameMinus1( uint32_t value ) { numTileGroupsInAtlasFrameMinus1_ = value; }
  void setSignalledTileGroupIdFlag( bool value ) { signalledTileGroupIdFlag_ = value; }
  void setSignalledTileGroupIdLengthMinus1( uint32_t value ) { signalledTileGroupIdLengthMinus1_ = value; }

  void setTileColumnWidthMinus1( size_t index, uint32_t value ) {
    if ( index == ( tileColumnWidthMinus1_.size() ) )
      tileColumnWidthMinus1_.resize( tileColumnWidthMinus1_.size() + 1 );
    else if ( index > tileColumnWidthMinus1_.size() )
      assert( 0 );
    tileColumnWidthMinus1_[index] = value;
  }
  void setTileRowHeightMinus1( size_t index, uint32_t value ) {
    if ( index == ( tileRowHeightMinus1_.size() ) )
      tileRowHeightMinus1_.resize( tileRowHeightMinus1_.size() + 1 );
    else if ( index > tileRowHeightMinus1_.size() )
      assert( 0 );
    tileRowHeightMinus1_[index] = value;
  }
  void setTopLeftTileIdx( size_t index, uint32_t value ) {
    if ( index == ( topLeftTileIdx_.size() ) )
      topLeftTileIdx_.resize( topLeftTileIdx_.size() + 1 );
    else if ( index > topLeftTileIdx_.size() )
      assert( 0 );
    topLeftTileIdx_[index] = value;
  }
  void setBottomRightTileIdxDelta( size_t index, uint32_t value ) {
    if ( index == ( bottomRightTileIdxDelta_.size() ) )
      bottomRightTileIdxDelta_.resize( bottomRightTileIdxDelta_.size() + 1 );
    else if ( index > bottomRightTileIdxDelta_.size() )
      assert( 0 );
    bottomRightTileIdxDelta_[index] = value;
  }
  void setTileGroupId( size_t index, uint32_t value ) {
    if ( index == ( tileGroupId_.size() ) )
      tileGroupId_.resize( tileGroupId_.size() + 1 );
    else if ( index > tileGroupId_.size() )
      assert( 0 );
    tileGroupId_[index] = value;
  }

 private:
  bool                  singleTileInAtlasFrameFlag_;
  bool                  uniformTileSpacingFlag_;
  uint32_t              numTileColumnsMinus1_;
  uint32_t              numTileRowsMinus1_;
  uint32_t              singleTilePerTileGroupFlag_;
  uint32_t              numTileGroupsInAtlasFrameMinus1_;
  bool                  signalledTileGroupIdFlag_;
  uint32_t              signalledTileGroupIdLengthMinus1_;
  std::vector<uint32_t> tileColumnWidthMinus1_;
  std::vector<uint32_t> tileRowHeightMinus1_;
  std::vector<uint32_t> topLeftTileIdx_;
  std::vector<uint32_t> bottomRightTileIdxDelta_;
  std::vector<uint32_t> tileGroupId_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_ATLASFRAMETILEINFORMATION_H