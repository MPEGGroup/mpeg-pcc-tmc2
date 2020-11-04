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
#ifndef PCC_BITSTREAM_GEOMETRYINFORMATION_H
#define PCC_BITSTREAM_GEOMETRYINFORMATION_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 8.3.4.4 Geometry information Syntax
class GeometryInformation {
 public:
  GeometryInformation() :
      geometryCodecId_( 0 ),
      Geometry2dBitdepthMinus1_( 10 ),
      geometryMSBAlignFlag_( false ),
      geometry3dCoordinatesBitdepthMinus1_( 9 ),
      auxiliaryGeometryCodecId_( 0 ) {}
  ~GeometryInformation() {}
  GeometryInformation& operator=( const GeometryInformation& ) = default;

  uint8_t getGeometryCodecId() { return geometryCodecId_; }
  uint8_t getGeometry2dBitdepthMinus1() { return Geometry2dBitdepthMinus1_; }
  bool    getGeometryMSBAlignFlag() { return geometryMSBAlignFlag_; }
  uint8_t getGeometry3dCoordinatesBitdepthMinus1() { return geometry3dCoordinatesBitdepthMinus1_; }
  uint8_t getAuxiliaryGeometryCodecId() { return auxiliaryGeometryCodecId_; }
  void    setGeometryCodecId( uint8_t value ) { geometryCodecId_ = value; }
  void    setGeometry2dBitdepthMinus1( uint8_t value ) { Geometry2dBitdepthMinus1_ = value; }
  void    setGeometryMSBAlignFlag( bool value ) { geometryMSBAlignFlag_ = value; }
  void    setGeometry3dCoordinatesBitdepthMinus1( uint8_t value ) { geometry3dCoordinatesBitdepthMinus1_ = value; }
  void    setAuxiliaryGeometryCodecId( uint8_t value ) { auxiliaryGeometryCodecId_ = value; }

 private:
  uint8_t geometryCodecId_;
  uint8_t Geometry2dBitdepthMinus1_;
  bool    geometryMSBAlignFlag_;
  uint8_t geometry3dCoordinatesBitdepthMinus1_;
  uint8_t auxiliaryGeometryCodecId_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_GEOMETRYINFORMATION_H