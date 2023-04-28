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
#ifndef PCC_BITSTREAM_VPCCEXTENSION_H
#define PCC_BITSTREAM_VPCCEXTENSION_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// H.7.3.4.1	VPS V-PCC extension syntax
class VpsVpccExtension {
 public:
  VpsVpccExtension() {}
  ~VpsVpccExtension() {}
  VpsVpccExtension& operator=( const VpsVpccExtension& ) = default;

 private:
};

// H.7.3.6.1.1	ASPS V-PCC extension syntax
class AspsVpccExtension {
 public:
  AspsVpccExtension() : removeDuplicatePointEnableFlag_( true ), surfaceThicknessMinus1_( 3 ) {}
  ~AspsVpccExtension() {}
  AspsVpccExtension& operator=( const AspsVpccExtension& ) = default;

  bool    getRemoveDuplicatePointEnableFlag() { return removeDuplicatePointEnableFlag_; }
  uint8_t getSurfaceThicknessMinus1() { return surfaceThicknessMinus1_; }
  void    setRemoveDuplicatePointEnableFlag( bool value ) { removeDuplicatePointEnableFlag_ = value; }
  void    setSurfaceThicknessMinus1( uint8_t value ) { surfaceThicknessMinus1_ = value; }

 private:
  bool    removeDuplicatePointEnableFlag_;
  uint8_t surfaceThicknessMinus1_;
};

// H.7.3.6.2.1	AFPS V-PCC extension syntax
class AfpsVpccExtension {
 public:
  AfpsVpccExtension() {}
  ~AfpsVpccExtension() {}
  AfpsVpccExtension& operator=( const AfpsVpccExtension& ) = default;

 private:
};

// H.7.3.6.2.2	Atlas camera parameters syntax
class AtlasCameraParameters {
 public:
  AtlasCameraParameters() :
      cameraModel_( 0 ),
      scaleEnabledFlag_( false ),
      offsetEnabledFlag_( false ),
      rotationEnabledFlag_( false ) {}
  ~AtlasCameraParameters() {}
  AtlasCameraParameters& operator=( const AtlasCameraParameters& ) = default;

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

// H.7.3.6.2.1	AAPS V-PCC extension syntax
class AapsVpccExtension {
 public:
  AapsVpccExtension() {}
  ~AapsVpccExtension() {}
  AapsVpccExtension& operator=( const AapsVpccExtension& ) = default;

  bool                   getCameraParametersPresentFlag() { return cameraParametersPresentFlag_; }
  AtlasCameraParameters& getAtlasCameraParameters() { return atlasCameraParameters_; }
  void                   setCameraParametersPresentFlag( bool value ) { cameraParametersPresentFlag_ = value; }

 private:
  bool                  cameraParametersPresentFlag_;
  AtlasCameraParameters atlasCameraParameters_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_VPCCEXTENSION_H