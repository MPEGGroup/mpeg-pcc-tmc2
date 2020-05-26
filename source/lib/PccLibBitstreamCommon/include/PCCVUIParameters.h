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

// G.2  VUI syntax

// G.2.4  G.2.4	Coordinate system parameters syntax
class CoordinateSystemParameters {
 public:
  CoordinateSystemParameters() :
      forwardAxis_( 0 ),
      deltaLeftAxis_( 0 ),
      forwardSign_( 0 ),
      leftSign_( 0 ),
      upSign_( 0 ) {}
  ~CoordinateSystemParameters() {}
  CoordinateSystemParameters& operator=( const CoordinateSystemParameters& ) = default;

  uint8_t getForwardAxis() { return forwardAxis_; }
  uint8_t getDeltaLeftAxis() { return deltaLeftAxis_; }
  uint8_t getForwardSign() { return forwardSign_; }
  uint8_t getLeftSign() { return leftSign_; }
  uint8_t getUpSign() { return upSign_; }

  void setForwardAxis( uint8_t value ) { forwardAxis_ = value; }
  void setDeltaLeftAxis( uint8_t value ) { deltaLeftAxis_ = value; }
  void setForwardSign( uint8_t value ) { forwardSign_ = value; }
  void setLeftSign( uint8_t value ) { leftSign_ = value; }
  void setUpSign( uint8_t value ) { upSign_ = value; }

 private:
  uint8_t forwardAxis_;
  uint8_t deltaLeftAxis_;
  uint8_t forwardSign_;
  uint8_t leftSign_;
  uint8_t upSign_;
};

// G.2.3  Sub-layer HRD parameters syntax
class HrdSubLayerParameters {
 public:
  HrdSubLayerParameters() {}
  ~HrdSubLayerParameters() {
    bitRateValueMinus1_.clear();
    cabSizeValueMinus1_.clear();
    cbrFlag_.clear();
  }
  HrdSubLayerParameters& operator=( const HrdSubLayerParameters& ) = default;
  void                   allocate( size_t size ) {
    bitRateValueMinus1_.resize( size, 0 );
    cabSizeValueMinus1_.resize( size, 0 );
    cbrFlag_.resize( size, false );
  }
  size_t   size() { return bitRateValueMinus1_.size(); }
  bool     getCbrFlag( size_t i ) { return cbrFlag_[i]; }
  uint32_t getBitRateValueMinus1( size_t i ) { return bitRateValueMinus1_[i]; }
  uint32_t getCabSizeValueMinus1( size_t i ) { return cabSizeValueMinus1_[i]; }
  void     setCbrFlag( size_t i, bool value ) { cbrFlag_[i] = value; }
  void     setBitRateValueMinus1( size_t i, uint32_t value ) { bitRateValueMinus1_[i] = value; }
  void     setCabSizeValueMinus1( size_t i, uint32_t value ) { cabSizeValueMinus1_[i] = value; }

 private:
  std::vector<uint32_t> bitRateValueMinus1_;
  std::vector<uint32_t> cabSizeValueMinus1_;
  std::vector<bool>     cbrFlag_;
};

// F.2.2  HRD parameters syntax
class HrdParameters {
 public:
  HrdParameters() :
      nalParametersPresentFlag_( 0 ),
      aclParametersPresentFlag_( 0 ),
      bitRateScale_( 0 ),
      cabSizeScale_( 0 ),
      initialCabRemovalDelayLengthMinus1_( 0 ),
      auCabRemovalDelayLengthMinus1_( 0 ),
      dabOutputDelayLengthMinus1_( 0 ),
      elementalDurationInTcMinus1_( 0 ),
      cabCntMinus1_( 0 ) {
    for ( size_t i = 0; i <= maxNumSubLayersMinus1_; i++ ) {
      fixedAtlasRateGeneralFlag_[i]   = 0;
      fixedAtlasRateWithinCasFlag_[i] = 0;
      lowDelayFlag[i]                 = 0;
    }
    elementalDurationInTcMinus1_ = 0;
    cabCntMinus1_                = 0;
  }
  ~HrdParameters() {}
  HrdParameters& operator=( const HrdParameters& ) = default;

  uint8_t getMaxNumSubLayersMinus1() { return maxNumSubLayersMinus1_; }
  bool    getNalParametersPresentFlag() { return nalParametersPresentFlag_; }
  bool    getAclParametersPresentFlag() { return aclParametersPresentFlag_; }
  uint8_t getBitRateScale() { return bitRateScale_; }
  uint8_t getCabSizeScale() { return cabSizeScale_; }
  uint8_t getInitialCabRemovalDelayLengthMinus1() { return initialCabRemovalDelayLengthMinus1_; }
  uint8_t getAuCabRemovalDelayLengthMinus1() { return auCabRemovalDelayLengthMinus1_; }
  uint8_t getDabOutputDelayLengthMinus1() { return dabOutputDelayLengthMinus1_; }
  bool    getFixedAtlasRateGeneralFlag( size_t i ) { return fixedAtlasRateGeneralFlag_[i]; }
  bool    getFixedAtlasRateWithinCasFlag( size_t i ) { return fixedAtlasRateWithinCasFlag_[i]; }
  bool    getLowDelayFlag( size_t i ) { return lowDelayFlag[i]; }
  bool    getElementalDurationInTcMinus1( size_t i ) { return fixedAtlasRateWithinCasFlag_[i]; }
  bool    getCabCntMinus1( size_t i ) { return lowDelayFlag[i]; }

  HrdSubLayerParameters& getHdrSubLayerParameters( size_t i, size_t j ) { return subLayerParameters_[i][j]; }

  void setNalParametersPresentFlag( bool value ) { nalParametersPresentFlag_ = value; }
  void setAclParametersPresentFlag( bool value ) { aclParametersPresentFlag_ = value; }
  void setBitRateScale( uint8_t value ) { bitRateScale_ = value; }
  void setCabSizeScale( uint8_t value ) { cabSizeScale_ = value; }
  void setInitialCabRemovalDelayLengthMinus1( uint8_t value ) { initialCabRemovalDelayLengthMinus1_ = value; }
  void setAuCabRemovalDelayLengthMinus1( uint8_t value ) { auCabRemovalDelayLengthMinus1_ = value; }
  void setDabOutputDelayLengthMinus1( uint8_t value ) { dabOutputDelayLengthMinus1_ = value; }
  void setFixedAtlasRateGeneralFlag( size_t i, bool value ) { fixedAtlasRateGeneralFlag_[i] = value; }
  void setFixedAtlasRateWithinCasFlag( size_t i, bool value ) { fixedAtlasRateWithinCasFlag_[i] = value; }
  void setLowDelayFlag( size_t i, bool value ) { lowDelayFlag[i] = value; }
  void setElementalDurationInTcMinus1( size_t i, uint32_t value ) { fixedAtlasRateWithinCasFlag_[i] = value; }
  void setCabCntMinus1( size_t i, uint32_t value ) { lowDelayFlag[i] = value; }

 private:
  static const uint8_t  maxNumSubLayersMinus1_ = 0;
  bool                  nalParametersPresentFlag_;
  bool                  aclParametersPresentFlag_;
  uint8_t               bitRateScale_;
  uint8_t               cabSizeScale_;
  uint8_t               initialCabRemovalDelayLengthMinus1_;
  uint8_t               auCabRemovalDelayLengthMinus1_;
  uint8_t               dabOutputDelayLengthMinus1_;
  bool                  fixedAtlasRateGeneralFlag_[maxNumSubLayersMinus1_ + 1];
  bool                  fixedAtlasRateWithinCasFlag_[maxNumSubLayersMinus1_ + 1];
  bool                  lowDelayFlag[maxNumSubLayersMinus1_ + 1];
  uint32_t              elementalDurationInTcMinus1_;
  uint32_t              cabCntMinus1_;
  HrdSubLayerParameters subLayerParameters_[2][maxNumSubLayersMinus1_ + 1];
};

// F.2.1  VUI parameters syntax
class VUIParameters {
 public:
  VUIParameters() :
      timingInfoPresentFlag_( false ),
      numUnitsInTick_( 1001 ),
      timeScale_( 60000 ),
      pocProportionalToTimingFlag_( false ),
      numTicksPocDiffOneMinus1_( 0 ),
      hrdParametersPresentFlag_( false ),
      bitstreamRestrictionPresentFlag_( false ),
      tilesRestrictedFlag_( false ),
      consistentTilesForVideoComponentsFlag_( false ),
      maxNumTilesPerAtlas_( 0 ),
      coordinateSystemParametersPresentFlag_( false ),
      unitInMetresFlag_( false ),
      displayBoxInfoPresentFlag_( false ),
      displayBoxOrigin_{0, 0, 0},
      displayBoxSize_{0, 0, 0},
      anchorPointPresentFlag_( false ),
      anchorPoint_{0.f, 0.f, 0.f} {}

  ~VUIParameters() {}
  VUIParameters& operator=( const VUIParameters& ) = default;

  HrdParameters&              getHrdParameters() { return hrdParameters_; }
  CoordinateSystemParameters& getCoordinateSystemParameters() { return coordinateSystemParameters_; }

  bool     getTimingInfoPresentFlag() { return timingInfoPresentFlag_; }
  uint32_t getNumUnitsInTick() { return numUnitsInTick_; }
  uint32_t getTimeScale() { return timeScale_; }
  bool     getPocProportionalToTimingFlag() { return pocProportionalToTimingFlag_; }
  uint32_t getNumTicksPocDiffOneMinus1() { return numTicksPocDiffOneMinus1_; }
  bool     getHrdParametersPresentFlag() { return hrdParametersPresentFlag_; }
  bool     getBitstreamRestrictionPresentFlag() { return bitstreamRestrictionPresentFlag_; }
  bool     getTilesRestrictedFlag() { return tilesRestrictedFlag_; }
  bool     getConsistentTilesForVideoComponentsFlag() { return consistentTilesForVideoComponentsFlag_; }
  uint32_t getMaxNumTilesPerAtlas() { return maxNumTilesPerAtlas_; }
  bool     getCoordinateSystemParametersPresentFlag() { return coordinateSystemParametersPresentFlag_; }
  bool     getUnitInMetresFlag() { return unitInMetresFlag_; }
  bool     getDisplayBoxInfoPresentFlag() { return displayBoxInfoPresentFlag_; }
  uint32_t getDisplayBoxOrigin( size_t i ) { return displayBoxOrigin_[i]; }
  uint32_t getDisplayBoxSize( size_t i ) { return displayBoxSize_[i]; }
  bool     getAnchorPointPresentFlag() { return anchorPointPresentFlag_; }
  float    getAnchorPoint( size_t i ) { return anchorPoint_[i]; }

  void setTimingInfoPresentFlag( bool value ) { timingInfoPresentFlag_ = value; }
  void setNumUnitsInTick( uint32_t value ) { numUnitsInTick_ = value; }
  void setTimeScale( uint32_t value ) { timeScale_ = value; }
  void setPocProportionalToTimingFlag( bool value ) { pocProportionalToTimingFlag_ = value; }
  void setNumTicksPocDiffOneMinus1( uint32_t value ) { numTicksPocDiffOneMinus1_ = value; }
  void setHrdParametersPresentFlag( bool value ) { hrdParametersPresentFlag_ = value; }
  void setBitstreamRestrictionPresentFlag( bool value ) { bitstreamRestrictionPresentFlag_ = value; }
  void setTilesRestrictedFlag( bool value ) { tilesRestrictedFlag_ = value; }
  void setConsistentTilesForVideoComponentsFlag( bool value ) { consistentTilesForVideoComponentsFlag_ = value; }
  void setMaxNumTilesPerAtlas( uint32_t value ) { maxNumTilesPerAtlas_ = value; }
  void setCoordinateSystemParametersPresentFlag( bool value ) { coordinateSystemParametersPresentFlag_ = value; }
  void setUnitInMetresFlag( bool value ) { unitInMetresFlag_ = value; }
  void setDisplayBoxInfoPresentFlag( bool value ) { displayBoxInfoPresentFlag_ = value; }
  void setDisplayBoxOrigin( size_t i, uint32_t value ) { displayBoxOrigin_[i] = value; }
  void setDisplayBoxSize( size_t i, uint32_t value ) { displayBoxSize_[i] = value; }
  void setAnchorPointPresentFlag( bool value ) { anchorPointPresentFlag_ = value; }
  void setAnchorPoint( size_t i, float value ) { anchorPoint_[i] = value; }

 private:
  bool     timingInfoPresentFlag_;
  uint32_t numUnitsInTick_;
  uint32_t timeScale_;
  bool     pocProportionalToTimingFlag_;
  uint32_t numTicksPocDiffOneMinus1_;
  bool     hrdParametersPresentFlag_;
  bool     bitstreamRestrictionPresentFlag_;
  bool     tilesRestrictedFlag_;
  bool     consistentTilesForVideoComponentsFlag_;
  uint32_t maxNumTilesPerAtlas_;
  bool     coordinateSystemParametersPresentFlag_;
  bool     unitInMetresFlag_;
  bool     displayBoxInfoPresentFlag_;
  uint32_t displayBoxOrigin_[3];
  uint32_t displayBoxSize_[3];
  bool     anchorPointPresentFlag_;
  float    anchorPoint_[3];

  HrdParameters              hrdParameters_;
  CoordinateSystemParameters coordinateSystemParameters_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_VUIPARAMETERS_H
