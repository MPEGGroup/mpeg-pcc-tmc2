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
#ifndef PCCBitstreamEncoderNewSyntax_H
#define PCCBitstreamEncoderNewSyntax_H

#include "PCCCommon.h"
#include "PCCEncoderParameters.h"
#include "PCCCodec.h"
#include "ArithmeticCodec.h"
#include "PCCMetadata.h"

#include <map>

namespace pcc {

class PCCGroupOfFrames;
class PCCBitstream;
class PCCContext;
class PCCFrameContext;
class PCCEncoderParameters;
class PCCPatch;
class ArithmeticCodec;
class ProfileTierLevel;
class SequenceParameterSet;
class OccupancyParameterSet;
class GeometrySequenceParams;
class AttributeParameterSet;
class GeometryParameterSet; 
class AttributeSequenceParams;


class PCCBitstreamEncoderNewSyntax {
 public:
  PCCBitstreamEncoderNewSyntax();
  ~PCCBitstreamEncoderNewSyntax();

  int  encode( PCCContext& context, PCCBitstream& bitstream );
  void setParameters( PCCEncoderParameters params );

 private:
  void EncodeUInt32( const uint32_t           value,
                     const uint32_t           bitCount,
                     o3dgc::Arithmetic_Codec& arithmeticEncoder,
                     o3dgc::Static_Bit_Model& bModel0 );

  // 7.3.2 V-PCC unit syntax
  void vpccUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType );

  // 7.3.3 V-PCC unit header syntax
  void vpccUnitHeader( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType );

  // 7.3.4 PCM separate video data syntax
  void pcmSeparateVideoData( PCCContext& context, PCCBitstream& bitstream, uint8_t bitCount );

  // 7.3.5 V-PCC unit payload syntax
  void vpccUnitPayload( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType );

  void vpccVideoDataUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType );

  // 7.3.6 Sequence parameter set syntax
  void vpccSequenceParameterSet( SequenceParameterSet& sequenceParameterSet,
                                 PCCBitstream&         bitstream );

  // 7.3.7 Byte alignment syntax
  void byteAlignment( PCCBitstream& bitstream );

  // 7.3.8 Profile, tier, and level syntax
  void profileTierLevel( ProfileTierLevel& profileTierLevel, PCCBitstream& bitstream );

  // 7.3.9 Occupancy parameter set syntax
  void occupancyParameterSet( OccupancyParameterSet& occupancyParameterSet,
                              PCCBitstream&          bitstream );

  // 7.3.10 Geometry parameter set syntax
  void geometryParameterSet( GeometryParameterSet& geometryParameterSet,
                             SequenceParameterSet& sequenceParameterSet,
                             PCCBitstream&         bitstream );

  // 7.3.11 Geometry sequence parameter syntax
  void geometrySequenceParams( GeometrySequenceParams& geometrySequenceParams,
                               PCCBitstream&           bitstream );

  // 7.3.12 Attribute parameter set syntax
  void attributeParameterSet( AttributeParameterSet& attributeParameterSet,
                              SequenceParameterSet&  sequenceParameterSet,
                              PCCBitstream&          bitstream );

  // 7.3.13 Attribute sequence parameter syntax
  void attributeSequenceParams( AttributeSequenceParams& attributeSequenceParams,
                                uint8_t                  dimension,
                                PCCBitstream&            bitstream );

  // 7.3.14 Patch sequence data unit syntax
  void patchSequenceDataUnit( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.15 Patch sequence unit payload syntax
  void patchSequenceUnitPayload( PCCContext& context, PCCBitstream& bitstream, size_t frameIndex );

  // 7.3.16 Patch sequence parameter set syntax
  void patchSequenceParameterSet( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.17 Geometry frame parameter set syntax
  void geometryFrameParameterSet( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.18 Geometry frame parameter syntax
  void geometryFrameParams( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.19 Attribute frame parameter set syntax
  void attributeFrameParameterSet( PCCContext&   context,
                                   PCCBitstream& bitstream,
                                   size_t        attributeIndex );

  // 7.3.20 Attribute frame parameter syntax
  void attributeFrameParams( PCCContext&   context,
                             PCCBitstream& bitstream,
                             size_t        attributeIndex,
                             size_t        attributeDimension );

  // 7.3.21 Geometry patch parameter set syntax
  void geometryPatchParameterSet( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.22 Geometry patch parameter syntax
  void geometryPatchParams( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.23 Attribute patch parameter set syntax
  void attributePatchParameterSet( PCCContext&   context,
                                   PCCBitstream& bitstream,
                                   size_t        attributeIndex );

  // 7.3.24 Attribute patch parameter syntax
  void attributePatchParams( PCCContext&   context,
                             PCCBitstream& bitstream,
                             size_t        attributeIndex,
                             size_t        attributeDimension );

  // 7.3.25 Patch frame parameter set syntax
  void patchFrameParameterSet( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.26 Patch frame layer unit syntax
  void patchFrameLayerUnit( PCCContext& context, PCCBitstream& bitstream, size_t frameIndex );

  // 7.3.27 Patch frame header syntax
  void patchFrameHeader( PCCContext& context, PCCBitstream& bitstream, size_t frameIndex );

  // 7.3.28 Reference list structure syntax
  void refListStruct( PCCContext& context, PCCBitstream& bitstream, size_t referenceListIndex );

  // 7.3.29 Patch frame data unit syntax
  void patchFrameDataUnit( PCCContext& context, PCCBitstream& bitstream, size_t frameIndex );

  // 7.3.30 Patch information data syntax
  void patchInformationData( PCCContext&   context,
                             PCCBitstream& bitstream,
                             size_t        frameIndex,
                             size_t        patchIndex,
                             size_t        patchMode );

  // 7.3.31 Patch data unit syntax
  void patchDataUnit( PCCContext&   context,
                      PCCBitstream& bitstream,
                      size_t        frameIndex,
                      size_t        patchIndex );

  // 7.3.32  Delta Patch data unit syntax
  void deltaPatchDataUnit( PCCContext&   context,
                           PCCBitstream& bitstream,
                           size_t        frameIndex,
                           size_t        patchIndex );

  // 7.3.33 PCM patch data unit syntax
  void pcmPatchDataUnit( PCCContext&   context,
                         PCCBitstream& bitstream,
                         size_t        frameIndex,
                         size_t        patchIndex );

  // 7.3.34 Point local reconstruction syntax
  void pointLocalReconstruction( PCCContext& context, PCCBitstream& bitstream );

  PCCEncoderParameters params_;
};

};  // namespace pcc

#endif /* PCCBitstreamEncoderNewSyntax_H */