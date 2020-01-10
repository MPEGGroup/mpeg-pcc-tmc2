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
#ifndef PCC_BITSTREAM_VUIPARAMETERS_H
#define PCC_BITSTREAM_VUIPARAMETERS_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// F.2  VUI syntax
// F.2.3  Sub-layer HRD parameters syntax
class HrdSubLayerParameters {
 public:
  HrdSubLayerParameters() {}
  ~HrdSubLayerParameters() {
    hrdBitRateValueMinus1_.clear();
    hrdCabSizeValueMinus1_.clear();
    hrdCbrFlag_.clear();
  }
  HrdSubLayerParameters& operator=( const HrdSubLayerParameters& ) = default;
  void                   allocate( size_t size ) {
    hrdBitRateValueMinus1_.resize( size, 0 );
    hrdCabSizeValueMinus1_.resize( size, 0 );
    hrdCbrFlag_.resize( size, false );
  }
  size_t   size() { return hrdBitRateValueMinus1_.size(); }
  bool     getHrdCbrFlag( size_t index ) { return hrdCbrFlag_[index]; }
  uint32_t getHrdBitRateValueMinus1( size_t index ) { return hrdBitRateValueMinus1_[index]; }
  uint32_t getHrdCabSizeValueMinus1( size_t index ) { return hrdCabSizeValueMinus1_[index]; }
  void     setHrdCbrFlag( size_t index, bool value ) { hrdCbrFlag_[index] = value; }
  void     setHrdBitRateValueMinus1( size_t index, uint32_t value ) { hrdBitRateValueMinus1_[index] = value; }
  void     setHrdCabSizeValueMinus1( size_t index, uint32_t value ) { hrdCabSizeValueMinus1_[index] = value; }

 private:
  std::vector<uint32_t> hrdBitRateValueMinus1_;
  std::vector<uint32_t> hrdCabSizeValueMinus1_;
  std::vector<bool>     hrdCbrFlag_;
};

// F.2.2  HRD parameters syntax
class HrdParameters {
 public:
  HrdParameters() :
      hrdNalParametersPresentFlag_( 0 ),
      hrdAclParametersPresentFlag_( 0 ),
      hrdBitRateScale_( 0 ),
      hrdCabSizeScale_( 0 ),
      hrdInitialCabRemovalDelayLengthMinus1_( 0 ),
      hrdAuCabRemovalDelayLengthMinus1_( 0 ),
      hrdDabOutputDelayLengthMinus1_( 0 ) {
    for ( size_t i = 0; i <= maxNumSubLayersMinus1_; i++ ) {
      hrdFixedAtlasRateGeneralFlag_[i]   = 0;
      hrdFixedAtlasRateWithinCasFlag_[i] = 0;
      hrdLowDelayFlag[i]                 = 0;
    }
  }
  ~HrdParameters() {}
  HrdParameters& operator=( const HrdParameters& ) = default;

  uint8_t getMaxNumSubLayersMinus1() { return maxNumSubLayersMinus1_; }
  bool    getHrdNalParametersPresentFlag() { return hrdNalParametersPresentFlag_; }
  bool    getHrdAclParametersPresentFlag() { return hrdAclParametersPresentFlag_; }
  uint8_t getHrdBitRateScale() { return hrdBitRateScale_; }
  uint8_t getHrdCabSizeScale() { return hrdCabSizeScale_; }
  uint8_t getHrdInitialCabRemovalDelayLengthMinus1() { return hrdInitialCabRemovalDelayLengthMinus1_; }
  uint8_t getHrdAuCabRemovalDelayLengthMinus1() { return hrdAuCabRemovalDelayLengthMinus1_; }
  uint8_t getHrdDabOutputDelayLengthMinus1() { return hrdDabOutputDelayLengthMinus1_; }
  bool    getHrdFixedAtlasRateGeneralFlag( size_t index ) { return hrdFixedAtlasRateGeneralFlag_[index]; }
  bool    getHrdFixedAtlasRateWithinCasFlag( size_t index ) { return hrdFixedAtlasRateWithinCasFlag_[index]; }
  bool    getHrdLowDelayFlag( size_t index ) { return hrdLowDelayFlag[index]; }
  bool    getHrdElementalDurationInTcMinus1( size_t index ) { return hrdFixedAtlasRateWithinCasFlag_[index]; }
  bool    getHrdCabCntMinus1( size_t index ) { return hrdLowDelayFlag[index]; }

  HrdSubLayerParameters& getHrdSubLayerParameters( size_t i, size_t j ) { return hrdSubLayerParameters_[i][j]; }

  void setHrdNalParametersPresentFlag( bool value ) { hrdNalParametersPresentFlag_ = value; }
  void setHrdAclParametersPresentFlag( bool value ) { hrdAclParametersPresentFlag_ = value; }
  void setHrdBitRateScale( uint8_t value ) { hrdBitRateScale_ = value; }
  void setHrdCabSizeScale( uint8_t value ) { hrdCabSizeScale_ = value; }
  void setHrdInitialCabRemovalDelayLengthMinus1( uint8_t value ) { hrdInitialCabRemovalDelayLengthMinus1_ = value; }
  void setHrdAuCabRemovalDelayLengthMinus1( uint8_t value ) { hrdAuCabRemovalDelayLengthMinus1_ = value; }
  void setHrdDabOutputDelayLengthMinus1( uint8_t value ) { hrdDabOutputDelayLengthMinus1_ = value; }
  void setHrdFixedAtlasRateGeneralFlag( size_t index, bool value ) { hrdFixedAtlasRateGeneralFlag_[index] = value; }
  void setHrdFixedAtlasRateWithinCasFlag( size_t index, bool value ) { hrdFixedAtlasRateWithinCasFlag_[index] = value; }
  void setHrdLowDelayFlag( size_t index, bool value ) { hrdLowDelayFlag[index] = value; }
  void setHrdElementalDurationInTcMinus1( size_t index, uint32_t value ) {
    hrdFixedAtlasRateWithinCasFlag_[index] = value;
  }
  void setHrdCabCntMinus1( size_t index, uint32_t value ) { hrdLowDelayFlag[index] = value; }

 private:
  static const uint8_t  maxNumSubLayersMinus1_ = 0;
  bool                  hrdNalParametersPresentFlag_;
  bool                  hrdAclParametersPresentFlag_;
  uint8_t               hrdBitRateScale_;
  uint8_t               hrdCabSizeScale_;
  uint8_t               hrdInitialCabRemovalDelayLengthMinus1_;
  uint8_t               hrdAuCabRemovalDelayLengthMinus1_;
  uint8_t               hrdDabOutputDelayLengthMinus1_;
  bool                  hrdFixedAtlasRateGeneralFlag_[maxNumSubLayersMinus1_ + 1];
  bool                  hrdFixedAtlasRateWithinCasFlag_[maxNumSubLayersMinus1_ + 1];
  bool                  hrdLowDelayFlag[maxNumSubLayersMinus1_ + 1];
  uint32_t              hrdElementalDurationInTcMinus1_;
  uint32_t              hrdCabCntMinus1_;
  HrdSubLayerParameters hrdSubLayerParameters_[2][maxNumSubLayersMinus1_ + 1];
};

// F.2.1  VUI parameters syntax
class VUIParameters {
 public:
  VUIParameters() :
      vuiTimingInfoPresentFlag_( false ),
      vuiPocProportionalToTimingFlag_( false ),
      vuiHrdParametersPresentFlag_( false ),
      vuiNumUnitsInTick_( 1001 ),
      vuiTimeScale_( 60000 ),
      vuiNumTicksPocDiffOneMinus1_( 0 ) {}
  ~VUIParameters() {}
  VUIParameters& operator=( const VUIParameters& ) = default;

  bool           getVuiTimingInfoPresentFlag() { return vuiTimingInfoPresentFlag_; }
  bool           getVuiPocProportionalToTimingFlag() { return vuiPocProportionalToTimingFlag_; }
  bool           getVuiHrdParametersPresentFlag() { return vuiHrdParametersPresentFlag_; }
  uint32_t       getVuiNumUnitsInTick() { return vuiNumUnitsInTick_; }
  uint32_t       getVuiTimeScale() { return vuiTimeScale_; }
  uint32_t       getVuiNumTicksPocDiffOneMinus1() { return vuiNumTicksPocDiffOneMinus1_; }
  HrdParameters& getHrdParameters() { return hrdParameters_; }

  void setVuiTimingInfoPresentFlag( bool value ) { vuiTimingInfoPresentFlag_ = value; }
  void setVuiPocProportionalToTimingFlag( bool value ) { vuiPocProportionalToTimingFlag_ = value; }
  void setVuiHrdParametersPresentFlag( bool value ) { vuiHrdParametersPresentFlag_ = value; }
  void setVuiNumUnitsInTick( uint32_t value ) { vuiNumUnitsInTick_ = value; }
  void setVuiTimeScale( uint32_t value ) { vuiTimeScale_ = value; }
  void setVuiNumTicksPocDiffOneMinus1( bool value ) { vuiNumTicksPocDiffOneMinus1_ = value; }

 private:
  bool          vuiTimingInfoPresentFlag_;
  bool          vuiPocProportionalToTimingFlag_;
  bool          vuiHrdParametersPresentFlag_;
  uint32_t      vuiNumUnitsInTick_;
  uint32_t      vuiTimeScale_;
  uint32_t      vuiNumTicksPocDiffOneMinus1_;
  HrdParameters hrdParameters_;
};

};  // namespace pcc

#endif //~PCC_BITSTREAM_VUIPARAMETERS_H