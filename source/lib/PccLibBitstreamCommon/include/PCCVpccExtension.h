/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2025, ISO/IEC
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
#ifndef PCC_BITSTREAM_VPCCEXTENSION_H
#define PCC_BITSTREAM_VPCCEXTENSION_H

#include "PCCBitstreamCommon.h"

namespace pcc {
// H.8.3.6.1.3	ASPS V-PCC extension syntax
class AspsVpccExtension {
 public:
  AspsVpccExtension() : removeDuplicatePointEnableFlag_( true ), surfaceThicknessMinus1_( 3 ) {}
  ~AspsVpccExtension() {}
  AspsVpccExtension &operator=( const AspsVpccExtension & ) = default;

  bool    getRemoveDuplicatePointEnableFlag() { return removeDuplicatePointEnableFlag_; }
  uint8_t getSurfaceThicknessMinus1() { return surfaceThicknessMinus1_; }
  void    setRemoveDuplicatePointEnableFlag( bool value ) { removeDuplicatePointEnableFlag_ = value; }
  void    setSurfaceThicknessMinus1( uint8_t value ) { surfaceThicknessMinus1_ = value; }

 private:
  bool    removeDuplicatePointEnableFlag_;
  uint8_t surfaceThicknessMinus1_;
};

// H.8.3.6.3.3	Atlas camera parameters syntax
class AtlasCameraParameters {
 public:
  AtlasCameraParameters() :
      cameraModel_( 0 ), scaleEnabledFlag_( false ), offsetEnabledFlag_( false ), rotationEnabledFlag_( false ) {}
  ~AtlasCameraParameters() {}
  AtlasCameraParameters &operator=( const AtlasCameraParameters & ) = default;

  bool operator==( const AtlasCameraParameters &other ) const {
    bool retVal = true;
    if ( cameraModel_ != other.cameraModel_ || scaleEnabledFlag_ != other.scaleEnabledFlag_ ||
         offsetEnabledFlag_ != other.offsetEnabledFlag_ || rotationEnabledFlag_ != other.rotationEnabledFlag_ ) {
      retVal = false;
    }

    for ( auto i = 0; i < 3; i++ ) {
      if ( scaleOnAxis_[i] != other.scaleOnAxis_[i] || offsetOnAxis_[i] != other.offsetOnAxis_[i] ||
           rotation_[i] != other.rotation_[i] ) {
        retVal = false;
      }
    }
    return retVal;
  }

  bool operator!=( const AtlasCameraParameters &other ) const { return !( *this == other ); }

  uint8_t  getCameraModel() { return cameraModel_; }
  bool     getScaleEnabledFlag() { return scaleEnabledFlag_; }
  bool     getOffsetEnabledFlag() { return offsetEnabledFlag_; }
  bool     getRotationEnabledFlag() { return rotationEnabledFlag_; }
  uint32_t getScaleOnAxis( size_t index ) { return scaleOnAxis_[index]; }
  int32_t  getOffsetOnAxis( size_t index ) { return offsetOnAxis_[index]; }
  int32_t  getRotation( size_t index ) { return rotation_[index]; }

  void setCameraModel( uint8_t value ) { cameraModel_ = value; }
  void setScaleEnabledFlag( bool value ) { scaleEnabledFlag_ = value; }
  void setOffsetEnabledFlag( bool value ) { offsetEnabledFlag_ = value; }
  void setRotationEnabledFlag( bool value ) { rotationEnabledFlag_ = value; }
  void setScaleOnAxis( size_t index, uint32_t value ) { scaleOnAxis_[index] = value; }
  void setOffsetOnAxis( size_t index, int32_t value ) { offsetOnAxis_[index] = value; }
  void setRotation( size_t index, int32_t value ) { rotation_[index] = value; }

 private:
  uint8_t  cameraModel_;
  bool     scaleEnabledFlag_;
  bool     offsetEnabledFlag_;
  bool     rotationEnabledFlag_;
  uint32_t scaleOnAxis_[3];
  int32_t  offsetOnAxis_[3];
  int32_t  rotation_[3];
};

// H.8.3.6.3.2	AAPS V-PCC extension syntax
class AapsVpccExtension {
 public:
  AapsVpccExtension() {}
  ~AapsVpccExtension() {}
  AapsVpccExtension     &operator=( const AapsVpccExtension & ) = default;
  bool                   getCameraParametersPresentFlag() { return cameraParametersPresentFlag_; }
  AtlasCameraParameters &getAtlasCameraParameters() { return atlasCameraParameters_; }
  void                   setCameraParametersPresentFlag( bool value ) { cameraParametersPresentFlag_ = value; }

 private:
  bool                  cameraParametersPresentFlag_;
  AtlasCameraParameters atlasCameraParameters_;
};

// H.8.3.6.3.5	GS Dequantization syntax
class GsDequantization {
 public:
  GsDequantization() :
      dequantizationType_( 0 ),
      shMultipleRegionsEnableFlag_( false ),
      scaleMultipleRegionsEnableFlag_( false ),
      opacityMultipleRegionsEnableFlag_( false ),
      shSameMinMaxFlag_( false ),
      shSameNumRegionMius1Flag_( false ),
      shSameQuantRegionTransitionPointFlag_( false ),
      shSameRegionStepSizeFlag_( false ),
      rotationPresentFlag_( false ),
      rotQuaternionRepresentationTypeFlag_( false ),
      rotSameMinMaxFlag_( false ),
      scalePresentFlag_( false ),
      scaleSameMinMaxFlag_( false ),
      opacityPresentFlag_( false ),
      shPresentFlag_( false ),
      shNumLvl_( 0 ),
      minOpacity_( 0.0f ),
      maxOpacity_( 0.0f ) {
    for ( int i = 0; i < 3; ++i ) {
      minPosition_[i] = 0.0f;
      maxPosition_[i] = 0.0f;
      minScale_[i]    = 0.0f;
      maxScale_[i]    = 0.0f;
    }
    for ( int i = 0; i < 4; ++i ) {
      minRotation_[i] = 0.0f;
      maxRotation_[i] = 0.0f;
    }
  }
  ~GsDequantization() {}
  GsDequantization &operator=( const GsDequantization & ) = default;

  bool operator==( const GsDequantization &other ) const {
    bool retVal = true;
    for ( int i = 0; i < 3; i++ ) {
      if ( other.minPosition_[i] != minPosition_[i] ) retVal = false;
      if ( other.maxPosition_[i] != maxPosition_[i] ) retVal = false;
    }
    if ( other.rotationPresentFlag_ != rotationPresentFlag_ ) {
      retVal = false;
    } else {
      if ( rotationPresentFlag_ ) {
        if ( other.rotQuaternionRepresentationTypeFlag_ != rotQuaternionRepresentationTypeFlag_ ) retVal = false;
        for ( int i = 0; i < 4; i++ ) {
          if ( other.minRotation_[i] != minRotation_[i] ) retVal = false;
          if ( other.maxRotation_[i] != maxRotation_[i] ) retVal = false;
        }
      }
    }
    if ( other.scalePresentFlag_ != scalePresentFlag_ ) {
      retVal = false;
    } else {
      if ( scalePresentFlag_ ) {
        for ( int i = 0; i < 3; i++ ) {
          if ( other.minScale_[i] != minScale_[i] ) retVal = false;
          if ( other.maxScale_[i] != maxScale_[i] ) retVal = false;
          if ( other.scaleSameNumQuantRegionMinus1_ != scaleSameNumQuantRegionMinus1_ ) retVal = false;
          if ( other.scaleSameQuantRegionTransitionPoint_ != scaleSameQuantRegionTransitionPoint_ ) retVal = false;
          if ( other.scaleSameRegionStepSize_ != scaleSameRegionStepSize_ ) retVal = false;
          if ( other.scaleNumQuantRegionMinus1_[i] != scaleNumQuantRegionMinus1_[i] ) retVal = false;
          for ( int m = 0; m < scaleNumQuantRegionMinus1_[i]; m++ ) {
            if ( other.scaleQuantRegionTransitionPoint_[i][m] != scaleQuantRegionTransitionPoint_[i][m] )
              retVal = false;
            if ( other.scaleQuantRegionTransitionPointDelta_[i][m] != scaleQuantRegionTransitionPointDelta_[i][m] )
              retVal = false;
          }
          for ( int m = 0; m < scaleNumQuantRegionMinus1_[i] + 1; m++ ) {
            if ( other.scaleRegionStepSize_[i][m] != scaleRegionStepSize_[i][m] ) retVal = false;
          }
        }
      }
    }
    if ( other.opacityPresentFlag_ != opacityPresentFlag_ ) {
      retVal = false;
    } else {
      if ( opacityPresentFlag_ ) {
        if ( other.minOpacity_ != minOpacity_ ) retVal = false;
        if ( other.maxOpacity_ != maxOpacity_ ) retVal = false;

        if ( other.opacityNumQuantRegionsMinus1_ != opacityNumQuantRegionsMinus1_ ) retVal = false;
        if ( opacityNumQuantRegionsMinus1_ > 0 ) {
          for ( int m = 0; m < opacityNumQuantRegionsMinus1_; m++ ) {
            if ( other.opacityQuantRegionTransitionPoint_[m] != opacityQuantRegionTransitionPoint_[m] ) retVal = false;
            if ( other.opacityQuantRegionTransitionPointDelta_[m] != opacityQuantRegionTransitionPointDelta_[m] )
              retVal = false;
          }
          for ( int m = 0; m < opacityNumQuantRegionsMinus1_ + 1; m++ ) {
            if ( other.opacityRegionStepSize_[m] != opacityRegionStepSize_[m] ) retVal = false;
          }
        }
      }
    }
    if ( other.shPresentFlag_ != shPresentFlag_ ) {
      retVal = false;
    } else {
      if ( shPresentFlag_ ) {
        if ( ( other.shNumLvl_ != shNumLvl_ ) || ( other.shNumChan_ != shNumChan_ ) ) {
          retVal = false;
        } else {
          for ( int l = 0; l < shNumLvl_; l++ ) {
            for ( int c = 0; c < shNumChan_[l]; c++ ) {
              if ( other.minSH_[l][c] != minSH_[l][c] ) retVal = false;
              if ( other.maxSH_[l][c] != maxSH_[l][c] ) retVal = false;
              if ( other.numQuantRegionsMinus1_[l][c] != numQuantRegionsMinus1_[l][c] ) retVal = false;
              if ( numQuantRegionsMinus1_[l][c] > 0 ) {
                for ( int m = 0; m < numQuantRegionsMinus1_[l][c]; m++ ) {
                  if ( other.quantRegionTransitionPoint_[l][c][m] != quantRegionTransitionPoint_[l][c][m] )
                    retVal = false;
                  if ( other.quantRegionTransitionPointDelta_[l][c][m] != quantRegionTransitionPointDelta_[l][c][m] )
                    retVal = false;
                }
                for ( int m = 0; m < numQuantRegionsMinus1_[l][c] + 1; m++ ) {
                  if ( other.regionStepSize_[l][c][m] != regionStepSize_[l][c][m] ) retVal = false;
                }
              }
            }
          }
        }
      }
    }

    return retVal;
  }

  bool operator!=( const GsDequantization &other ) const { return !( *this == other ); }

  uint8_t  getDequantizationType() { return dequantizationType_; }
  float    getMinPosition( int i ) { return minPosition_[i]; }
  float    getMaxPosition( int i ) { return maxPosition_[i]; }
  bool     getRotationPresentFlag() { return rotationPresentFlag_; }
  bool     getRotQuaternionRepresentationTypeFlag() { return rotQuaternionRepresentationTypeFlag_; }
  float    getMinRotation( int i ) { return minRotation_[i]; }
  float    getMaxRotation( int i ) { return maxRotation_[i]; }
  bool     getScalePresentFlag() { return scalePresentFlag_; }
  float    getMinScale( int i ) { return minScale_[i]; }
  float    getMaxScale( int i ) { return maxScale_[i]; }
  bool     getScaleSameNumQuantRegionMinus1_() { return scaleSameNumQuantRegionMinus1_; }
  bool     getScaleSameQuantRegionTransitionPoint_() { return scaleSameQuantRegionTransitionPoint_; }
  bool     getScaleSameRegionStepSize_() { return scaleSameRegionStepSize_; }
  uint32_t getScaleNumQuantRegionsMinus1( int i ) { return scaleNumQuantRegionMinus1_[i]; }
  uint16_t getScaleQuantRegionTransitionPoint( int i, int j ) { return scaleQuantRegionTransitionPoint_[i][j]; }
  uint16_t getScaleQuantRegionTransitionPointDelta( int i, int j ) {
    return scaleQuantRegionTransitionPointDelta_[i][j];
  }
  float    getScaleRegionStepSize( int i, int j ) { return scaleRegionStepSize_[i][j]; }
  bool     getOpacityPresentFlag() { return opacityPresentFlag_; }
  float    getMinOpacity() { return minOpacity_; }
  float    getMaxOpacity() { return maxOpacity_; }
  uint32_t getOpacityNumQuantRegionsMinus1() { return opacityNumQuantRegionsMinus1_; }
  uint16_t getOpacityQuantRegionTransitionPoint( int m ) { return opacityQuantRegionTransitionPoint_[m]; }
  uint16_t getOpacityQuantRegionTransitionPointDelta( int m ) { return opacityQuantRegionTransitionPointDelta_[m]; }
  float    getOpacityRegionStepSize( int m ) { return opacityRegionStepSize_[m]; }
  bool     getShPresentFlag() { return shPresentFlag_; }
  bool     getScaleSameMinMaxFlag() { return scaleSameMinMaxFlag_; }
  bool     getRotSameMinMaxFlag() { return rotSameMinMaxFlag_; }
  bool     getShMultipleRegionsEnableFlag() const { return shMultipleRegionsEnableFlag_; }
  bool     getOpacityMultipleRegionsEnableFlag() const { return opacityMultipleRegionsEnableFlag_; }
  bool     getScaleMultipleRegionsEnableFlag() const { return scaleMultipleRegionsEnableFlag_; }
  bool     getShSameMinMaxFlag() const { return shSameMinMaxFlag_; }
  bool     getShSameNumRegionMius1Flag() const { return shSameNumRegionMius1Flag_; }
  bool     getShSameQuantRegionTransitionPointFlag() const { return shSameQuantRegionTransitionPointFlag_; }
  bool     getShSameRegionStepSizeFlag() const { return shSameRegionStepSizeFlag_; }
  uint8_t  getShNumLvl() { return shNumLvl_; }
  uint8_t  getShNumChan( int i ) { return shNumChan_[i]; }
  float    getMinSh( int i, int j ) { return minSH_[i][j]; }
  float    getMaxSh( int i, int j ) { return maxSH_[i][j]; }
  uint32_t getNumQuantRegionsMinus1( int i, int j ) { return numQuantRegionsMinus1_[i][j]; }
  uint16_t getQuantRegionTransitionPoint( int i, int j, int m ) { return quantRegionTransitionPoint_[i][j][m]; }
  uint16_t getQuantRegionTransitionPointDelta( int i, int j, int m ) {
    return quantRegionTransitionPointDelta_[i][j][m];
  }
  float getRegionStepSize( int i, int j, int m ) { return regionStepSize_[i][j][m]; }

  void setDequantizationType( uint8_t value ) { dequantizationType_ = value; }
  void setMinPosition( int i, float value ) { minPosition_[i] = value; }
  void setMaxPosition( int i, float value ) { maxPosition_[i] = value; }
  void setRotationPresentFlag( bool value ) { rotationPresentFlag_ = value; }
  void setRotQuaternionRepresentationTypeFlag( bool value ) { rotQuaternionRepresentationTypeFlag_ = value; }
  void setMinRotation( int i, float value ) { minRotation_[i] = value; }
  void setMaxRotation( int i, float value ) { maxRotation_[i] = value; }
  void setScalePresentFlag( bool value ) { scalePresentFlag_ = value; }
  void setMinScale( int i, float value ) { minScale_[i] = value; }
  void setMaxScale( int i, float value ) { maxScale_[i] = value; }
  void setScaleSameNumQuantRegionMinus1_( bool value ) { scaleSameNumQuantRegionMinus1_ = value; }
  void setScaleSameQuantRegionTransitionPoint_( bool value ) { scaleSameQuantRegionTransitionPoint_ = value; }
  void setScaleSameRegionStepSize_( bool value ) { scaleSameRegionStepSize_ = value; }
  void setScaleNumQuantRegionsMinus1( int i, uint32_t value ) {
    scaleNumQuantRegionMinus1_[i] = value;
    scaleQuantRegionTransitionPoint_[i].resize( value, 0 );
    scaleQuantRegionTransitionPointDelta_[i].resize( value, 0 );
    scaleRegionStepSize_[i].resize( value + 1, 0 );
  }
  void setScaleQuantRegionTransitionPoint( int i, int j, uint16_t value ) {
    scaleQuantRegionTransitionPoint_[i][j] = value;
  }

  void setScaleQuantRegionTransitionPointDelta( int i, int j, uint16_t value ) {
    scaleQuantRegionTransitionPointDelta_[i][j] = value;
  }

  void setScaleRegionStepSize( int i, int j, float value ) { scaleRegionStepSize_[i][j] = value; }
  void setOpacityPresentFlag( bool value ) { opacityPresentFlag_ = value; }
  void setMinOpacity( float value ) { minOpacity_ = value; }
  void setMaxOpacity( float value ) { maxOpacity_ = value; }
  void setOpacityNumQuantRegionsMinus1( uint32_t value ) {  // m76122
    opacityNumQuantRegionsMinus1_ = value;
    if ( value > 0 ) {
      opacityQuantRegionTransitionPoint_.resize( value, 0 );
      opacityQuantRegionTransitionPointDelta_.resize( value, 0 );
      opacityRegionStepSize_.resize( value + 1, 0.0f );
    }
  }
  void setOpacityQuantRegionTransitionPoint( int m, int value ) { opacityQuantRegionTransitionPoint_[m] = value; }
  void setOpacityQuantRegionTransitionPointDelta( int m, int value ) {
    opacityQuantRegionTransitionPointDelta_[m] = value;
  }
  void setOpacityRegionStepSize( int m, float value ) { opacityRegionStepSize_[m] = value; }
  void setShPresentFlag( bool value ) { shPresentFlag_ = value; }
  void setScaleSameMinMaxFlag( bool value ) { scaleSameMinMaxFlag_ = value; }
  void setRotSameMinMaxFlag( bool value ) { rotSameMinMaxFlag_ = value; }
  void setShMultipleRegionsEnableFlag( bool value ) { shMultipleRegionsEnableFlag_ = value; }
  void setOpacityMultipleRegionsEnableFlag( bool value ) { opacityMultipleRegionsEnableFlag_ = value; }
  void setScaleMultipleRegionsEnableFlag( bool value ) { scaleMultipleRegionsEnableFlag_ = value; }
  void setShSameMinMaxFlag( bool value ) { shSameMinMaxFlag_ = value; }
  void setShSameNumRegionMius1Flag( bool value ) { shSameNumRegionMius1Flag_ = value; }
  void setShSameQuantRegionTransitionPointFlag( bool value ) { shSameQuantRegionTransitionPointFlag_ = value; }
  void setShSameRegionStepSizeFlag( bool value ) { shSameRegionStepSizeFlag_ = value; }
  void setShNumLvl( int value ) {
    shNumLvl_ = value;
    shNumChan_.resize( value, 0 );
    minSH_.resize( value );
    maxSH_.resize( value );
    numQuantRegionsMinus1_.resize( value );
    quantRegionTransitionPoint_.resize( value );
    quantRegionTransitionPointDelta_.resize( value );
    regionStepSize_.resize( value );
  }
  void setShNumChan( int i, int value ) {
    shNumChan_[i] = value;
    minSH_[i].resize( value, 0 );
    maxSH_[i].resize( value, 0 );
    numQuantRegionsMinus1_[i].resize( value, 0 );
    quantRegionTransitionPoint_[i].resize( value );
    quantRegionTransitionPointDelta_[i].resize( value );
    regionStepSize_[i].resize( value );
  }
  void setMinSh( int i, int j, float value ) { minSH_[i][j] = value; }
  void setMaxSh( int i, int j, float value ) { maxSH_[i][j] = value; }
  void setNumQuantRegionsMinus1( int i, int j, uint32_t value ) {
    numQuantRegionsMinus1_[i][j] = value;
    if ( value > 0 ) {
      quantRegionTransitionPoint_[i][j].resize( value, 0 );
      quantRegionTransitionPointDelta_[i][j].resize( value, 0 );
      regionStepSize_[i][j].resize( value + 1, 0 );
    }
  }
  void setQuantRegionTransitionPoint( int i, int j, int m, uint16_t value ) {
    quantRegionTransitionPoint_[i][j][m] = value;
  }

  void setQuantRegionTransitionPointDelta( int i, int j, int m, uint16_t value ) {
    quantRegionTransitionPointDelta_[i][j][m] = value;
  }
  void setRegionStepSize( int i, int j, int m, float value ) { regionStepSize_[i][j][m] = value; }

 private:
  uint8_t                                         dequantizationType_;
  float                                           minPosition_[3];
  float                                           maxPosition_[3];
  bool                                            rotationPresentFlag_;
  float                                           minRotation_[4];
  float                                           maxRotation_[4];
  bool                                            scalePresentFlag_;
  float                                           minScale_[3];
  float                                           maxScale_[3];
  bool                                            scaleSameNumQuantRegionMinus1_;
  bool                                            scaleSameQuantRegionTransitionPoint_;
  bool                                            scaleSameRegionStepSize_;
  uint32_t                                        scaleNumQuantRegionMinus1_[3];
  std::vector<uint16_t>                           scaleQuantRegionTransitionPoint_[3];
  std::vector<uint16_t>                           scaleQuantRegionTransitionPointDelta_[3];
  std::vector<float>                              scaleRegionStepSize_[3];
  bool                                            opacityPresentFlag_;
  float                                           minOpacity_;
  float                                           maxOpacity_;
  uint32_t                                        opacityNumQuantRegionsMinus1_;
  std::vector<int>                                opacityQuantRegionTransitionPoint_;
  std::vector<int>                                opacityQuantRegionTransitionPointDelta_;
  std::vector<float>                              opacityRegionStepSize_;
  bool                                            shPresentFlag_;
  bool                                            scaleSameMinMaxFlag_;
  bool                                            rotSameMinMaxFlag_;
  bool                                            rotQuaternionRepresentationTypeFlag_;
  bool                                            shMultipleRegionsEnableFlag_;
  bool                                            opacityMultipleRegionsEnableFlag_;
  bool                                            scaleMultipleRegionsEnableFlag_;
  bool                                            shSameMinMaxFlag_;
  bool                                            shSameNumRegionMius1Flag_;
  bool                                            shSameQuantRegionTransitionPointFlag_;
  bool                                            shSameRegionStepSizeFlag_;
  uint32_t                                        shNumLvl_;
  std::vector<uint32_t>                           shNumChan_;
  std::vector<std::vector<float>>                 minSH_;
  std::vector<std::vector<float>>                 maxSH_;
  std::vector<std::vector<uint32_t>>              numQuantRegionsMinus1_;
  std::vector<std::vector<std::vector<uint16_t>>> quantRegionTransitionPoint_;
  std::vector<std::vector<std::vector<uint16_t>>> quantRegionTransitionPointDelta_;
  std::vector<std::vector<std::vector<float>>>    regionStepSize_;
};

// H.8.3.6.3.6	GS Spherical Harmonics Property syntax
class GsSphericalHarmonicsProperty {
 public:
  static constexpr uint32_t NUM_SH_COMPONENTS = 48;
  GsSphericalHarmonicsProperty() : shMappingMethod_( 0 ), totalCodedComponentsMinus1_( 0 ) { shMappingTable_.clear(); }
  ~GsSphericalHarmonicsProperty() { shMappingTable_.clear(); }
  GsSphericalHarmonicsProperty &operator=( const GsSphericalHarmonicsProperty & ) = default;

  bool operator==( const GsSphericalHarmonicsProperty &other ) const {
    bool retVal = true;
    if ( other.shMappingMethod_ != shMappingMethod_ ) retVal = false;
    if ( shMappingMethod_ == 2 ) {
      if ( other.totalCodedComponentsMinus1_ != totalCodedComponentsMinus1_ ) { retVal = false; }
      if ( other.shMappingTable_ != shMappingTable_ ) retVal = false;
    }
    return retVal;
  }

  bool operator!=( const GsSphericalHarmonicsProperty &other ) const { return !( *this == other ); }

  uint32_t getShMappingMethod() const { return shMappingMethod_; }
  void     setShMappingMethod( uint32_t value ) { shMappingMethod_ = value; }

  uint32_t getTotalCodedComponentsMinus1() const { return totalCodedComponentsMinus1_; }
  void     setTotalCodedComponentsMinus1( uint32_t value ) { totalCodedComponentsMinus1_ = value; }

  std::vector<uint32_t> &getShMappingTable() { return shMappingTable_; }
  uint32_t               getShMappingTable( size_t index ) const { return shMappingTable_[index]; }
  void                   setShMappingTable( size_t index, uint32_t value ) { shMappingTable_[index] = value; }
  void                   resizeShMappingTable( size_t size ) { shMappingTable_.resize( size ); }

 private:
  uint32_t              shMappingMethod_;
  uint32_t              totalCodedComponentsMinus1_;
  std::vector<uint32_t> shMappingTable_;
};

// H.8.3.6.3.7	GS spherical harmonics transform syntax
class Gs2DTransform {
 public:
  Gs2DTransform() : trRowNumGroups_( 0 ), dimensionReductionEnableFlag_( false ) {
    trColType_.resize( 16, 0 );
    rowTrSizeMinus1_.resize( 0 );
    trRowGroupIdx_.resize( 0 );
    trRowType_.resize( 0 );
    kltFlag_.resize( 0 );
    kltCoeffs_.resize( 0 );
    kltMean_.resize( 0 );
    kltStdDev_.resize( 0 );
    numOfRetainComponentsMinus1_.resize( 0 );
  }
  ~Gs2DTransform() {}
  Gs2DTransform &operator=( const Gs2DTransform & ) = default;

  bool operator==( const Gs2DTransform &other ) const {
    bool retVal = true;
    if ( trColType_ != other.trColType_ || trRowNumGroups_ != other.trRowNumGroups_ ||
         rowTrSizeMinus1_ != other.rowTrSizeMinus1_ || trRowGroupIdx_ != other.trRowGroupIdx_ ||
         kltFlag_ != other.kltFlag_ || kltCoeffs_ != other.kltCoeffs_ || kltMean_ != other.kltMean_ ||
         kltStdDev_ != other.kltStdDev_ || trRowType_ != other.trRowType_ ||
         numOfRetainComponentsMinus1_ != other.numOfRetainComponentsMinus1_ ||
         dimensionReductionEnableFlag_ != other.dimensionReductionEnableFlag_ ) {
      retVal = false;
    }
    return retVal;
  }

  bool operator!=( const Gs2DTransform &other ) const { return !( *this == other ); }

  uint8_t getTrColType( size_t i ) { return trColType_[i]; }
  void    setTrColType( size_t i, uint8_t value ) { trColType_[i] = value; }

  uint8_t getTrRowNumGroups() { return trRowNumGroups_; }
  void    setTrRowNumGroups( uint8_t value ) {
    trRowNumGroups_ = value;
    rowTrSizeMinus1_.resize( value );
    trRowGroupIdx_.resize( value );
    trRowType_.resize( value );
    kltFlag_.resize( value );
    kltCoeffs_.resize( value );
    kltMean_.resize( value );
    kltStdDev_.resize( value );
    numOfRetainComponentsMinus1_.resize( value, 0 );
  }

  uint8_t getNumOfRetainComponentsMinus1( size_t i ) { return numOfRetainComponentsMinus1_[i]; }
  uint8_t getNumOfRetainComponents( size_t i ) { return numOfRetainComponentsMinus1_[i] + 1; }
  void    setNumOfRetainComponentsMinus1( size_t i, uint8_t value ) { numOfRetainComponentsMinus1_[i] = value; }

  uint8_t getRowTrSizeMinus1( size_t i ) { return rowTrSizeMinus1_[i]; }
  void    setRowTrSizeMinus1( size_t i, uint8_t value ) {
    rowTrSizeMinus1_[i] = value;
    trRowGroupIdx_[i].resize( value + 1 );
    if ( kltFlag_[i] ) {
      kltCoeffs_[i].resize( 3 * ( value + size_t{ 1 } ) );
      kltMean_[i].resize( 3 * ( value + size_t{ 1 } ) );
      kltStdDev_[i].resize( 3 * ( value + size_t{ 1 } ) );
      for ( size_t j = 0; j < 3 * ( value + size_t{ 1 } ); j++ ) {
        kltCoeffs_[i][j].resize( 3 * ( value + size_t{ 1 } ) );
      }
    }
  }

  uint8_t getTrRowGroupIdx( size_t i, size_t j ) { return trRowGroupIdx_[i][j]; }
  void    setTrRowGroupIdx( size_t i, size_t j, uint8_t value ) { trRowGroupIdx_[i][j] = value; }

  uint8_t getKltFlag( size_t i ) { return kltFlag_[i]; }
  void    setKltFlag( size_t i, uint8_t value ) {
    kltFlag_[i] = value;
    if ( kltFlag_[i] ) {
      // Resize kltMean_[i] and kltStdDev_[i] when kltFlag_[i] is true
      // The size depends on rowTrSizeMinus1_[i], which should have been set by setRowTrSizeMinus1
      // Check if rowTrSizeMinus1_[i] is already set to avoid accessing out of bounds
      if ( i < rowTrSizeMinus1_.size() ) {
        kltMean_[i].resize( 3 * ( rowTrSizeMinus1_[i] + size_t{ 1 } ) );
        kltStdDev_[i].resize( 3 * ( rowTrSizeMinus1_[i] + size_t{ 1 } ) );
        kltCoeffs_[i].resize( 3 * ( rowTrSizeMinus1_[i] + size_t{ 1 } ) );
        for ( size_t j = 0; j < 3 * ( rowTrSizeMinus1_[i] + size_t{ 1 } ); j++ ) {
          kltCoeffs_[i][j].resize( 3 * ( rowTrSizeMinus1_[i] + size_t{ 1 } ) );
        }
      } else {
        // This should not happen if parsing order is correct, but add an assert for safety
        assert( false && "rowTrSizeMinus1_ not set for current index when kltFlag is true" );
      }
    }
  }

  std::vector<std::vector<float>> getKltCoeff( size_t i ) { return kltCoeffs_[i]; }
  std::vector<float>              getKltCoeff( size_t i, size_t m ) { return kltCoeffs_[i][m]; }
  float                           getKltCoeff( size_t i, size_t m, size_t n ) { return kltCoeffs_[i][m][n]; }
  void setKltCoeff( size_t i, size_t m, size_t n, float value ) { kltCoeffs_[i][m][n] = value; }

  std::vector<float> getKltMean( size_t i ) { return kltMean_[i]; }
  float              getKltMean( size_t i, size_t m ) { return kltMean_[i][m]; }
  void               setKltMean( size_t i, size_t m, float value ) { kltMean_[i][m] = value; }

  std::vector<float> getKltStdDev( size_t i ) { return kltStdDev_[i]; }
  float              getKltStdDev( size_t i, size_t m ) { return kltStdDev_[i][m]; }
  void               setKltStdDev( size_t i, size_t m, float value ) { kltStdDev_[i][m] = value; }

  uint8_t getTrRowType( size_t i ) { return trRowType_[i]; }
  void    setTrRowType( size_t i, uint8_t value ) { trRowType_[i] = value; }

  bool getDimensionReductionEnableFlag() const { return dimensionReductionEnableFlag_; }
  void setDimensionReductionEnableFlag( bool value ) { dimensionReductionEnableFlag_ = value; }

 private:
  std::vector<uint8_t>                         trColType_;
  uint8_t                                      trRowNumGroups_;
  std::vector<uint8_t>                         rowTrSizeMinus1_;
  std::vector<std::vector<uint8_t>>            trRowGroupIdx_;
  std::vector<uint8_t>                         kltFlag_;
  std::vector<std::vector<std::vector<float>>> kltCoeffs_;
  std::vector<std::vector<float>>              kltMean_;
  std::vector<std::vector<float>>              kltStdDev_;
  std::vector<uint8_t>                         trRowType_;
  std::vector<uint8_t>                         numOfRetainComponentsMinus1_;
  bool                                         dimensionReductionEnableFlag_;
};

// H.8.3.6.3.12	GS Geometry Prediction syntax
class GsGeometryPrediction {
 public:
  GsGeometryPrediction() : geometryPredictionFlag_( 0 ), geometryPredictionSegmentVal_( 0 ) {}
  ~GsGeometryPrediction() {}
  GsGeometryPrediction &operator=( const GsGeometryPrediction & ) = default;

  bool operator==( const GsGeometryPrediction &other ) const {
    return geometryPredictionFlag_ == other.geometryPredictionFlag_;
  }

  bool operator!=( const GsGeometryPrediction &other ) const { return !( *this == other ); }

  bool getGeometryPredictionEnableFlag() const { return geometryPredictionFlag_; }
  void setGeometryPredictionEnableFlag( bool value ) { geometryPredictionFlag_ = value; }

  uint16_t getGeometryPredictionSegmentVal() const { return geometryPredictionSegmentVal_; }
  void     setGeometryPredictionSegmentVal( uint16_t value ) { geometryPredictionSegmentVal_ = value; }

 private:
  bool     geometryPredictionFlag_;
  uint16_t geometryPredictionSegmentVal_;
};

// H.8.3.6.3.7  Gaussian splat spherical harmonics prediction syntax
class GsPrediction {
 public:
  GsPrediction() : shNumComponents_( 0 ) {}
  ~GsPrediction() {}
  GsPrediction &operator=( const GsPrediction & ) = default;

  bool operator==( const GsPrediction &other ) const {
    bool retVal = true;
    if ( shNumComponents_ != other.shNumComponents_ || usePredictionFlag_ != other.usePredictionFlag_ ||
         numPredSrcMinus1_ != other.numPredSrcMinus1_ || predSrcIdx_ != other.predSrcIdx_ ||
         predCoeff_ != other.predCoeff_ ) {
      retVal = false;
    }
    return retVal;
  }
  bool operator!=( const GsPrediction &other ) const { return !( *this == other ); }

  uint8_t getShNumComponents() { return shNumComponents_; }
  void    setShNumComponents( uint8_t value ) {
    shNumComponents_ = value;
    usePredictionFlag_.resize( value, false );
    numPredSrcMinus1_.resize( value, 0 );
    predSrcIdx_.resize( value );
    predCoeff_.resize( value );
  }

  bool getUsePredictionFlag( size_t i ) { return usePredictionFlag_[i]; }
  void setUsePredictionFlag( size_t i, bool value ) { usePredictionFlag_[i] = value; }

  uint8_t getNumPredSrcMinus1( size_t i ) { return numPredSrcMinus1_[i]; }
  void    setNumPredSrcMinus1( size_t i, uint8_t value ) {
    numPredSrcMinus1_[i] = value;
    predSrcIdx_[i].resize( value + 1, 0 );
    predCoeff_[i].resize( value + 1, 0.0f );
  }

  uint8_t getPredSrcIdx( size_t i, size_t j ) { return predSrcIdx_[i][j]; }
  void    setPredSrcIdx( size_t i, size_t j, uint8_t value ) { predSrcIdx_[i][j] = value; }

  float getPredCoeff( size_t i, size_t j ) { return predCoeff_[i][j]; }
  void  setPredCoeff( size_t i, size_t j, float value ) { predCoeff_[i][j] = value; }

 private:
  uint8_t                           shNumComponents_;
  std::vector<bool>                 usePredictionFlag_;
  std::vector<uint8_t>              numPredSrcMinus1_;
  std::vector<std::vector<uint8_t>> predSrcIdx_;
  std::vector<std::vector<float>>   predCoeff_;
};

// H.8.3.6.3.9  GS scale transform syntax
class GsScaleTransform {
 public:
  GsScaleTransform() {
    for ( int i = 0; i < 3; i++ ) {
      scaleScaling_[i] = 1.f;
      scalePower_[i]   = 5.f;
    }
  }
  ~GsScaleTransform() {}
  GsScaleTransform &operator=( const GsScaleTransform & ) = default;

  bool operator==( const GsScaleTransform &other ) const {
    for ( int i = 0; i < 3; i++ ) {
      if ( other.scaleScaling_[i] != scaleScaling_[i] ) return false;
      if ( other.scalePower_[i] != scalePower_[i] ) return false;
    }
    return true;
  }
  bool operator!=( const GsScaleTransform &other ) const { return !( *this == other ); }

  bool  getSameScalingFlag() { return sameScalingFlag_; }
  bool  getSamePowerTransformFlag() { return samePowerTransformFlag_; }
  float getScaleScaling( int i ) { return scaleScaling_[i]; }
  float getScalePower( int i ) { return scalePower_[i]; }

  void setSameScalingFlag( bool value ) { sameScalingFlag_ = value; }
  void setSamePowerTransformFlag( bool value ) { samePowerTransformFlag_ = value; }
  void setScaleScaling( int i, float value ) { scaleScaling_[i] = value; }
  void setScalePower( int i, float value ) { scalePower_[i] = value; }

 private:
  bool  sameScalingFlag_        = true;
  bool  samePowerTransformFlag_ = true;
  float scaleScaling_[3];
  float scalePower_[3];
};

// H.8.3.6.3.4	AAPS V-PCC GS extension syntax
class AapsVpccGsExtension {
 public:
  AapsVpccGsExtension() :
      dequantizationPresentFlag_( false ),
      adaptiveQuantizationEnableFlag_( false ),
      shPropertyPresentFlag_( false ),
      transform2DPresentFlag_( false ),
      GeometryPredictionEnableFlag_( false ),
      scaleTransformPresentFlag_( false ),
      rawPatchDataArrangementMethod_( 0 ),
      linearOpacityFlag_( false ) {}
  ~AapsVpccGsExtension() {}
  AapsVpccGsExtension &operator=( const AapsVpccGsExtension & ) = default;

  bool              getDequantizationPresentFlag() { return dequantizationPresentFlag_; }
  GsDequantization &getGsDequantization() { return gsDequantization_; }
  void              setDequantizationPresentFlag( bool value ) { dequantizationPresentFlag_ = value; }

  bool getAdaptiveQuantizationEnableFlag() { return adaptiveQuantizationEnableFlag_; }
  void setAdaptiveQuantizationEnableFlag( bool value ) { adaptiveQuantizationEnableFlag_ = value; }

  bool                  getGeometryPredictionPresentFlag() { return GeometryPredictionEnableFlag_; }
  GsGeometryPrediction &getGsGeometryPrediction() { return gsGeometryPrediction_; }
  void                  setGeometryPredictionPresentFlag( bool value ) { GeometryPredictionEnableFlag_ = value; }

  bool                          getShPropertyPresentFlag() { return shPropertyPresentFlag_; }
  GsSphericalHarmonicsProperty &getGsShProperty() { return shSphericalHarmonicsProperty_; }
  void                          setShPropertyPresentFlag( bool value ) { shPropertyPresentFlag_ = value; }

  bool           getTransform2DPresentFlag() { return transform2DPresentFlag_; }
  Gs2DTransform &getGs2DTransform() { return gs2DTransform_; }
  void           setTransform2DPresentFlag( bool value ) { transform2DPresentFlag_ = value; }

  bool              getScaleTransformPresentFlag() { return scaleTransformPresentFlag_; }
  GsScaleTransform &getGsScaleTransform() { return gsScaleTransform_; }
  void              setScaleTransformPresentFlag( bool value ) { scaleTransformPresentFlag_ = value; }

  int  getRawPatchDataArrangementMethod() { return rawPatchDataArrangementMethod_; }
  void setRawPatchDataArrangementMethod( int value ) { rawPatchDataArrangementMethod_ = value; }

  bool getLinearOpacityFlag() { return linearOpacityFlag_; }
  void setLinearOpacityFlag( bool value ) { linearOpacityFlag_ = value; }

  bool          getPredEnableFlag() { return predEnableFlag_; }
  GsPrediction &getGsPrediction() { return gsPrediction_; }
  void          setPredEnableFlag( bool value ) { predEnableFlag_ = value; }

  uint8_t getReservedZero8bits() const { return reservedZero8bits_; }
  void    setReservedZero8bits( uint8_t value ) { reservedZero8bits_ = value; }

 private:
  bool                         dequantizationPresentFlag_{};
  bool                         adaptiveQuantizationEnableFlag_{};
  bool                         GeometryPredictionEnableFlag_{};
  bool                         shPropertyPresentFlag_{};
  bool                         transform2DPresentFlag_{};
  bool                         scaleTransformPresentFlag_{};
  bool                         rawPatchDataArrangementFlag_{};
  bool                         linearOpacityFlag_{};
  bool                         predEnableFlag_{};
  int                          rawPatchDataArrangementMethod_{};
  GsPrediction                 gsPrediction_{};
  GsDequantization             gsDequantization_{};
  GsSphericalHarmonicsProperty shSphericalHarmonicsProperty_{};
  Gs2DTransform                gs2DTransform_{};
  GsScaleTransform             gsScaleTransform_{};
  GsGeometryPrediction         gsGeometryPrediction_;
  uint8_t                      reservedZero8bits_{};
};
};  // namespace pcc

#endif  //~PCC_BITSTREAM_VPCCEXTENSION_H