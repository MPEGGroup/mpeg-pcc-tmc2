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
#ifndef PCC_BITSTREAM_ATLASTILEHEADER_H
#define PCC_BITSTREAM_ATLASTILEHEADER_H

#include "PCCBitstreamCommon.h"
#include "PCCRefListStruct.h"

namespace pcc {

// 8.3.6.11  Atlas tile header syntax
class AtlasTileHeader {
 public:
  AtlasTileHeader() :
      noOutputOfPriorAtlasFramesFlag_( false ),
      frameIndex_( 0 ),
      atlasFrameParameterSetId_( 0 ),
      atlasAdaptationParameterSetId_( 0 ),
      id_( 0 ),
      type_( PCCTileType( 0 ) ),
      atlasOutputFlag_( false ),
      atlasFrmOrderCntLsb_( 0 ),
      refAtlasFrameListSpsFlag_( 0 ),
      refAtlasFrameListIdx_( 0 ),
      posMinDQuantizer_( 0 ),
      posDeltaMaxDQuantizer_( 0 ),
      patchSizeXinfoQuantizer_( 0 ),
      patchSizeYinfoQuantizer_( 0 ),
      raw3dOffsetAxisBitCountMinus1_( 0 ),
      numRefIdxActiveOverrideFlag_( 0 ),
      numRefIdxActiveMinus1_( 0 ),
      tileNaluTypeInfo_( 0 ) {
    additionalAfocLsbPresentFlag_.resize( 1, 0 );
    additionalAfocLsbVal_.resize( 1, 0 );
  };
  ~AtlasTileHeader() {
    additionalAfocLsbPresentFlag_.clear();
    additionalAfocLsbVal_.clear();
  };
  AtlasTileHeader&      operator=( const AtlasTileHeader& ) = default;
  RefListStruct&        getRefListStruct() { return refListStruct_; }  // only 1 refList
  void                  setRefListStruct( RefListStruct value ) { refListStruct_ = value; }
  uint8_t               getFrameIndex() { return frameIndex_; }
  void                  setFrameIndex( uint8_t value ) { frameIndex_ = value; }
  bool                  getNoOutputOfPriorAtlasFramesFlag() { return noOutputOfPriorAtlasFramesFlag_; }
  uint8_t               getAtlasFrameParameterSetId() { return atlasFrameParameterSetId_; }
  uint8_t               getAtlasAdaptationParameterSetId() { return atlasAdaptationParameterSetId_; }
  uint32_t              getId() { return id_; }
  PCCTileType           getType() { return type_; }
  bool                  getAtlasOutputFlag() { return atlasOutputFlag_; }
  size_t                getAtlasFrmOrderCntLsb() { return atlasFrmOrderCntLsb_; }
  bool                  getRefAtlasFrameListSpsFlag() { return refAtlasFrameListSpsFlag_; }
  uint8_t               getRefAtlasFrameListIdx() { return refAtlasFrameListIdx_; }
  uint8_t               getPosMinDQuantizer() { return posMinDQuantizer_; }
  uint8_t               getPosDeltaMaxDQuantizer() { return posDeltaMaxDQuantizer_; }
  uint8_t               getPatchSizeXinfoQuantizer() { return patchSizeXinfoQuantizer_; }
  uint8_t               getPatchSizeYinfoQuantizer() { return patchSizeYinfoQuantizer_; }
  uint8_t               getRaw3dOffsetAxisBitCountMinus1() { return raw3dOffsetAxisBitCountMinus1_; }
  bool                  getNumRefIdxActiveOverrideFlag() { return numRefIdxActiveOverrideFlag_; }
  uint8_t               getNumRefIdxActiveMinus1() { return numRefIdxActiveMinus1_; }
  std::vector<bool>&    getAdditionalAfocLsbPresentFlag() { return additionalAfocLsbPresentFlag_; }
  std::vector<uint8_t>& getAdditionalAfocLsbVal() { return additionalAfocLsbVal_; }
  bool                  getAdditionalAfocLsbPresentFlag( size_t idx ) { return additionalAfocLsbPresentFlag_[idx]; }
  uint8_t               getAdditionalAfocLsbVal( size_t idx ) { return additionalAfocLsbVal_[idx]; }
  uint8_t               getTileNaluTypeInfo() { return tileNaluTypeInfo_; }

  void setNoOutputOfPriorAtlasFramesFlag( bool value ) { noOutputOfPriorAtlasFramesFlag_ = value; }
  void setAtlasFrameParameterSetId( uint8_t value ) { atlasFrameParameterSetId_ = value; }
  void setAtlasAdaptationParameterSetId( uint8_t value ) { atlasAdaptationParameterSetId_ = value; }
  void setId( uint32_t value ) { id_ = value; }
  void setType( PCCTileType value ) { type_ = value; }
  void setAtlasOutputFlag( bool value ) { atlasOutputFlag_ = value; }
  void setAtlasFrmOrderCntLsb( size_t value ) { atlasFrmOrderCntLsb_ = value; }
  void setRefAtlasFrameListSpsFlag( bool value ) { refAtlasFrameListSpsFlag_ = value; }
  void setRefAtlasFrameListIdx( uint8_t value ) { refAtlasFrameListIdx_ = value; }
  void setPosMinDQuantizer( uint8_t value ) { posMinDQuantizer_ = value; }
  void setPosDeltaMaxDQuantizer( uint8_t value ) { posDeltaMaxDQuantizer_ = value; }
  void setPatchSizeXinfoQuantizer( uint8_t value ) { patchSizeXinfoQuantizer_ = value; }
  void setPatchSizeYinfoQuantizer( uint8_t value ) { patchSizeYinfoQuantizer_ = value; }
  void setRaw3dOffsetAxisBitCountMinus1( uint8_t value ) { raw3dOffsetAxisBitCountMinus1_ = value; }
  void setNumRefIdxActiveOverrideFlag( bool value ) { numRefIdxActiveOverrideFlag_ = value; }
  void setNumRefIdxActiveMinus1( uint8_t value ) { numRefIdxActiveMinus1_ = value; }
  void setAdditionalAfocLsbPresentFlag( std::vector<bool>& value ) { additionalAfocLsbPresentFlag_ = value; }
  void setAdditionalAfocLsbVal( std::vector<uint8_t>& value ) { additionalAfocLsbVal_ = value; }
  void setAdditionalAfocLsbPresentFlag( size_t idx, bool value ) { additionalAfocLsbPresentFlag_[idx] = value; }
  void setAdditionalAfocLsbVal( size_t idx, uint8_t value ) { additionalAfocLsbVal_[idx] = value; }
  void setTileNaluTypeInfo( uint8_t value ) { tileNaluTypeInfo_ = value; }

 private:
  bool                 noOutputOfPriorAtlasFramesFlag_;
  uint8_t              frameIndex_;
  uint8_t              atlasFrameParameterSetId_;
  uint8_t              atlasAdaptationParameterSetId_;
  uint32_t             id_;
  PCCTileType          type_;
  bool                 atlasOutputFlag_;
  size_t               atlasFrmOrderCntLsb_;
  bool                 refAtlasFrameListSpsFlag_;
  uint8_t              refAtlasFrameListIdx_;
  std::vector<bool>    additionalAfocLsbPresentFlag_;
  std::vector<uint8_t> additionalAfocLsbVal_;
  uint8_t              posMinDQuantizer_;
  uint8_t              posDeltaMaxDQuantizer_;
  uint8_t              patchSizeXinfoQuantizer_;
  uint8_t              patchSizeYinfoQuantizer_;
  uint8_t              raw3dOffsetAxisBitCountMinus1_;
  bool                 numRefIdxActiveOverrideFlag_;
  uint8_t              numRefIdxActiveMinus1_;
  RefListStruct        refListStruct_;
  uint8_t              tileNaluTypeInfo_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_ATLASTILEHEADER_H
