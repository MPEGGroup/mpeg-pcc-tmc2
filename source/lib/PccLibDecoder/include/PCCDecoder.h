
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
#ifndef PCCDecoder_h
#define PCCDecoder_h

#include "PCCCommon.h"
#include "PCCDecoderParameters.h"
#include "PCCCodec.h"

namespace pcc {

class PCCBitstream;
class PCCContext;
class PCCFrameContext;
class PCCGroupOfFrames; 
template <typename T, size_t N>
class PCCVideo;
typedef pcc::PCCVideo<uint8_t, 3> PCCVideo3B;
template <typename T, size_t N>
class PCCImage;
typedef pcc::PCCImage<uint8_t, 3> PCCImage3B;

class PCCDecoder : public PCCCodec {
public:
  PCCDecoder();
  ~PCCDecoder();

  int decompress( PCCBitstream &bitstream, PCCContext &context, PCCGroupOfFrames& reconstruct );
  void setParameters( PCCDecoderParameters value );
  
private:
 
  int  decompressHeader( PCCContext &context, PCCBitstream &bitstream );
  void decompressOccupancyMap( PCCContext &context, PCCBitstream& bitstream );
  void decompressOccupancyMap( PCCFrameContext& frame, PCCBitstream &bitstream );

  PCCDecoderParameters params_;
  
  uint16_t width_;
  uint16_t height_;
  uint8_t  occupancyResolution_;
  uint8_t  radius2Smoothing_;
  uint8_t  neighborCountSmoothing_;
  uint8_t  radius2BoundaryDetection_;
  uint8_t  thresholdSmoothing_;
  uint8_t  losslessGeo_;
  uint8_t  losslessTexture_;
  uint8_t  noAttributes_;
  bool     absoluteD1_;
  
};
}; //~namespace

#endif /* PCCDecoder_h */
