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
#ifndef PCC_BITSTREAM_ATLASTILELAYERRBSP_H
#define PCC_BITSTREAM_ATLASTILELAYERRBSP_H

#include "PCCBitstreamCommon.h"
#include "PCCAtlasTileHeader.h"
#include "PCCAtlasTileDataUnit.h"

namespace pcc {

// 8.3.6.10  Atlas tile layer RBSP syntax
class AtlasTileLayerRbsp {
 public:
  AtlasTileLayerRbsp() :
      atlasFrmOrderCntVal_( 0 ),
      atlasFrmOrderCntMsb_( 0 ),
      tileOrder_( 0 ),
      encFrameIndex_( -1 ),
      encTileIndex_( -1 ) {}
  ~AtlasTileLayerRbsp() {}

  AtlasTileLayerRbsp& operator=( const AtlasTileLayerRbsp& ) = default;

  size_t             getTileOrder() { return tileOrder_; }
  AtlasTileHeader&   getHeader() { return header_; }
  AtlasTileDataUnit& getDataUnit() { return dataUnit_; }
  size_t             getAtlasFrmOrderCntVal() { return atlasFrmOrderCntVal_; }
  size_t             getAtlasFrmOrderCntMsb() { return atlasFrmOrderCntMsb_; }
  size_t             getEncFrameIndex() { return encFrameIndex_; }
  size_t             getEncTileIndex() { return encTileIndex_; }
  void               setTileOrder( size_t value ) { tileOrder_ = value; }
  void               setAtlasFrmOrderCntVal( size_t value ) { atlasFrmOrderCntVal_ = value; }
  void               setAtlasFrmOrderCntMsb( size_t value ) { atlasFrmOrderCntMsb_ = value; }
  void               setHeader( AtlasTileHeader value ) { header_ = value; }
  void               setDataUnit( AtlasTileDataUnit value ) { dataUnit_ = value; }
  void               setEncFrameIndex( size_t value ) { encFrameIndex_ = value; }
  void               setEncTileIndex( size_t value ) { encTileIndex_ = value; }
  PCCSEI&            getSEI() { return sei_; }

 private:
  AtlasTileHeader   header_;
  AtlasTileDataUnit dataUnit_;
  size_t            atlasFrmOrderCntVal_;
  size_t            atlasFrmOrderCntMsb_;
  size_t            tileOrder_;
  size_t            encFrameIndex_;
  size_t            encTileIndex_;
  PCCSEI            sei_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_ATLASTILELAYERRBSP_H
