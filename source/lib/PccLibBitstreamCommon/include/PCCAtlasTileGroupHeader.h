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
#ifndef PCC_BITSTREAM_ATLASTILEGROUPHEADER_H
#define PCC_BITSTREAM_ATLASTILEGROUPHEADER_H

#include "PCCBitstreamCommon.h"
#include "PCCRefListStruct.h"

namespace pcc {
    
// 7.3.6.11  Atlas tile group header syntax
class AtlasTileGroupHeader {
 public:
  AtlasTileGroupHeader() :
      atghFrameIndex_( 0 ),
      atghAtlasFrameParameterSetId_( 0 ),
      atghAddress_( 0 ),
      atghType_( PCCTILEGROUP( 0 ) ),
      atghAtlasOutputFlag_( false ),
      atghAtlasFrmOrderCntLsb_( 0 ),
      atghRefAtlasFrameListSpsFlag_( 0 ),
      atghRefAtlasFrameListIdx_( 0 ),
      atghPosMinZQuantizer_( 0 ),
      atghPosDeltaMaxZQuantizer_( 0 ),
      atghPatchSizeXinfoQuantizer_( 0 ),
      atghPatchSizeYinfoQuantizer_( 0 ),
      atghRaw3dPosAxisBitCountMinus1_( 0 ),
      atghNumRefIdxActiveOverrideFlag_( 0 ),
      atghNumRefIdxActiveMinus1_( 0 ) {
    atghAdditionalAfocLsbPresentFlag_.resize( 1, 0 );
    atghAdditionalAfocLsbVal_.resize( 1, 0 );
  };
  ~AtlasTileGroupHeader() {
    atghAdditionalAfocLsbPresentFlag_.clear();
    atghAdditionalAfocLsbVal_.clear();
  };
  AtlasTileGroupHeader& operator=( const AtlasTileGroupHeader& ) = default;
  RefListStruct&        getRefListStruct() { return refListStruct_; }  // only 1 refList
  void                  setRefListStruct( RefListStruct value ) { refListStruct_ = value; }
  uint8_t               getFrameIndex() { return atghFrameIndex_; }
  void                  setFrameIndex( uint8_t value ) { atghFrameIndex_ = value; }
  uint8_t               getAtghAtlasFrameParameterSetId() { return atghAtlasFrameParameterSetId_; }
  uint32_t              getAtghAddress() { return atghAddress_; }
  PCCTILEGROUP          getAtghType() { return atghType_; }
  bool                  getAtghAtlasOutputFlag() { return atghAtlasOutputFlag_; }
  uint8_t               getAtghAtlasFrmOrderCntLsb() { return atghAtlasFrmOrderCntLsb_; }
  bool                  getAtghRefAtlasFrameListSpsFlag() { return atghRefAtlasFrameListSpsFlag_; }
  uint8_t               getAtghRefAtlasFrameListIdx() { return atghRefAtlasFrameListIdx_; }
  uint8_t               getAtghPosMinZQuantizer() { return atghPosMinZQuantizer_; }
  uint8_t               getAtghPosDeltaMaxZQuantizer() { return atghPosDeltaMaxZQuantizer_; }
  uint8_t               getAtghPatchSizeXinfoQuantizer() { return atghPatchSizeXinfoQuantizer_; }
  uint8_t               getAtghPatchSizeYinfoQuantizer() { return atghPatchSizeYinfoQuantizer_; }
  uint8_t               getAtghRaw3dPosAxisBitCountMinus1() { return atghRaw3dPosAxisBitCountMinus1_; }
  bool                  getAtghNumRefIdxActiveOverrideFlag() { return atghNumRefIdxActiveOverrideFlag_; }
  uint8_t               getAtghNumRefIdxActiveMinus1() { return atghNumRefIdxActiveMinus1_; }
  std::vector<bool>&    getAtghAdditionalAfocLsbPresentFlag() { return atghAdditionalAfocLsbPresentFlag_; }
  std::vector<uint8_t>& getAtghAdditionalAfocLsbVal() { return atghAdditionalAfocLsbVal_; }
  bool    getAtghAdditionalAfocLsbPresentFlag( size_t idx ) { return atghAdditionalAfocLsbPresentFlag_[idx]; }
  uint8_t getAtghAdditionalAfocLsbVal( size_t idx ) { return atghAdditionalAfocLsbVal_[idx]; }

  void setAtghAtlasFrameParameterSetId( uint8_t value ) { atghAtlasFrameParameterSetId_ = value; }
  void setAtghAddress( uint32_t value ) { atghAddress_ = value; }
  void setAtghType( PCCTILEGROUP value ) { atghType_ = value; }
  void setAtghAtlasOutputFlag( bool value ) { atghAtlasOutputFlag_ = value; }
  void setAtghAtlasFrmOrderCntLsb( uint8_t value ) { atghAtlasFrmOrderCntLsb_ = value; }
  void setAtghRefAtlasFrameListSpsFlag( bool value ) { atghRefAtlasFrameListSpsFlag_ = value; }
  void setAtghRefAtlasFrameListIdx( uint8_t value ) { atghRefAtlasFrameListIdx_ = value; }
  void setAtghPosMinZQuantizer( uint8_t value ) { atghPosMinZQuantizer_ = value; }
  void setAtghPosDeltaMaxZQuantizer( uint8_t value ) { atghPosDeltaMaxZQuantizer_ = value; }
  void setAtghPatchSizeXinfoQuantizer( uint8_t value ) { atghPatchSizeXinfoQuantizer_ = value; }
  void setAtghPatchSizeYinfoQuantizer( uint8_t value ) { atghPatchSizeYinfoQuantizer_ = value; }
  void setAtghRaw3dPosAxisBitCountMinus1( uint8_t value ) { atghRaw3dPosAxisBitCountMinus1_ = value; }
  void setAtghNumRefIdxActiveOverrideFlag( bool value ) { atghNumRefIdxActiveOverrideFlag_ = value; }
  void setAtghNumRefIdxActiveMinus1( uint8_t value ) { atghNumRefIdxActiveMinus1_ = value; }
  void setAtghAdditionalAfocLsbPresentFlag( std::vector<bool>& value ) { atghAdditionalAfocLsbPresentFlag_ = value; }
  void setAtghAdditionalAfocLsbVal( std::vector<uint8_t>& value ) { atghAdditionalAfocLsbVal_ = value; }
  void setAtghAdditionalAfocLsbPresentFlag( size_t idx, bool value ) { atghAdditionalAfocLsbPresentFlag_[idx] = value; }
  void setAtghAdditionalAfocLsbVal( size_t idx, uint8_t value ) { atghAdditionalAfocLsbVal_[idx] = value; }

 private:
  uint8_t              atghFrameIndex_;
  uint8_t              atghAtlasFrameParameterSetId_;
  uint32_t             atghAddress_;
  PCCTILEGROUP         atghType_;
  bool                 atghAtlasOutputFlag_;
  uint8_t              atghAtlasFrmOrderCntLsb_;
  bool                 atghRefAtlasFrameListSpsFlag_;
  uint8_t              atghRefAtlasFrameListIdx_;
  uint8_t              atghPosMinZQuantizer_;
  uint8_t              atghPosDeltaMaxZQuantizer_;
  uint8_t              atghPatchSizeXinfoQuantizer_;
  uint8_t              atghPatchSizeYinfoQuantizer_;
  uint8_t              atghRaw3dPosAxisBitCountMinus1_;
  bool                 atghNumRefIdxActiveOverrideFlag_;
  uint8_t              atghNumRefIdxActiveMinus1_;
  std::vector<bool>    atghAdditionalAfocLsbPresentFlag_;
  std::vector<uint8_t> atghAdditionalAfocLsbVal_;
  RefListStruct        refListStruct_;
};

};  // namespace pcc

#endif //~PCC_BITSTREAM_ATLASTILEGROUPHEADER_H
