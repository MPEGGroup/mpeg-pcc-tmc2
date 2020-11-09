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
#ifndef PCC_BITSTREAM_OCCUPANCYINFORMATION_H
#define PCC_BITSTREAM_OCCUPANCYINFORMATION_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 8.3.4.3 Occupancy information Set Syntax
class OccupancyInformation {
 public:
  OccupancyInformation() :
      occupancyCodecId_( 0 ),
      LossyOccupancyCompressionThreshold_( 0 ),
      Occupancy2DBitdepthMinus1_( 10 ),
      occupancyMSBAlignFlag_( false ) {}
  ~OccupancyInformation() {}
  OccupancyInformation& operator=( const OccupancyInformation& ) = default;
  void                  init( uint8_t codecId, uint8_t threshold, uint8_t nominal2DBitdepth, bool msbAlignFlag ) {
    occupancyCodecId_                   = codecId;
    LossyOccupancyCompressionThreshold_ = threshold;
    Occupancy2DBitdepthMinus1_          = nominal2DBitdepth;
    occupancyMSBAlignFlag_              = msbAlignFlag;
  }
  uint8_t getOccupancyCodecId() { return occupancyCodecId_; }
  uint8_t getLossyOccupancyCompressionThreshold() { return LossyOccupancyCompressionThreshold_; }
  uint8_t getOccupancy2DBitdepthMinus1() { return Occupancy2DBitdepthMinus1_; }
  bool    getOccupancyMSBAlignFlag() { return occupancyMSBAlignFlag_; }
  void    setOccupancyCodecId( uint8_t value ) { occupancyCodecId_ = value; }
  void    setLossyOccupancyCompressionThreshold( uint8_t value ) { LossyOccupancyCompressionThreshold_ = value; }
  void    setOccupancy2DBitdepthMinus1( uint8_t value ) { Occupancy2DBitdepthMinus1_ = value; }
  void    setOccupancyMSBAlignFlag( bool value ) { occupancyMSBAlignFlag_ = value; }

 private:
  uint8_t occupancyCodecId_;
  uint8_t LossyOccupancyCompressionThreshold_;
  uint8_t Occupancy2DBitdepthMinus1_;
  bool    occupancyMSBAlignFlag_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_OCCUPANCYINFORMATION_H