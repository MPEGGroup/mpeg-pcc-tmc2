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
#include "PCCMath.h"
#include "PCCPatch.h"

namespace pcc {

class PCCContext;
class PCCFrameContext;
class PCCGroupOfFrames;
class PCCPatch;
class PatchFrameGeometryParameterSet;
class GeometryPatchParameterSet;
class V3CParameterSet;
class PLRData;

template <typename T, size_t N>
class PCCImage;
typedef pcc::PCCImage<uint8_t, 3> PCCImageOccupancyMap;

class PCCDecoder : public PCCCodec {
 public:
  PCCDecoder();
  ~PCCDecoder();

  int decode( PCCContext& context, PCCGroupOfFrames& reconstruct, int32_t atlasIndex );

  void setParameters( const PCCDecoderParameters& params );
  void setReconstructionParameters( const PCCDecoderParameters& params );
  void setPostProcessingSeiParameters( GeneratePointCloudParameters& gpcParams, PCCContext& context, size_t atglIndex );
  void setGeneratePointCloudParameters( GeneratePointCloudParameters& gpcParams,
                                        PCCContext&                   context,
                                        size_t                        atglIndex );
  void createPatchFrameDataStructure( PCCContext& context );
  void createPatchFrameDataStructure( PCCContext& context, size_t atglIndex );

 private:
  void       setPointLocalReconstruction( PCCContext& context );
  void       setPLRData( PCCFrameContext& tile, PCCPatch& patch, PLRData& plrd, size_t occupancyPackingBlockSize );
  void       setTilePartitionSizeAfti( PCCContext& context );
  size_t     setTileSizeAndLocation( PCCContext& context, size_t frameIndex, AtlasTileHeader& atgh );
  bool       compareHashSEIMD5( std::vector<uint8_t>& encMD5, std::vector<uint8_t>& decMD5 );
  bool       compareHashSEICrc( uint16_t encCrc, uint16_t decCrc );
  bool       compareHashSEICheckSum( uint32_t encCheckSum, uint32_t decCheckSum );
  void       createHashSEI( PCCContext& context, int frameIndex, SEIDecodedAtlasInformationHash& sei );
  void       createHlsAtlasTileLogFiles( PCCContext& context, int frameIndex );
  void       setConsitantFourCCCode( PCCContext& context, size_t atglIndex );
  PCCCodecId getCodedCodecId( PCCContext& context, const uint8_t codecCodecId, const std::string& videoDecoderPath );

  PCCDecoderParameters     params_;
  std::vector<std::string> consitantFourCCCode_;
};

};  // namespace pcc

#endif /* PCCDecoder_h */
