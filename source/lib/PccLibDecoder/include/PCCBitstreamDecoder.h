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

#ifndef PCCBitstreamDecoder_h
#define PCCBitstreamDecoder_h

#include "PCCCommon.h"
#include "PCCDecoderParameters.h"
#include "PCCCodec.h"
#include "PCCMath.h"
#include "PCCMetadata.h"
#include "PCCPatch.h"

namespace pcc {

class PCCBitstream;
class PCCContext;
class ProfileTierLevel;
class SequenceParameterSet;
class OccupancyInformation;
class GeometryInformation;
class AttributeInformation;
class PatchDataGroup;
class PatchSequenceParameterSet;
class PatchFrameGeometryParameterSet;
class PatchFrameAttributeParameterSet;
class GeometryPatchParameterSet;
class GeometryPatchParams;
class AttributePatchParameterSet;
class AttributeFrameParams;
class AttributePatchParams;
class GeometryFrameParams;
class PatchTileGroupLayerUnit;
class PatchFrameParameterSet;
class PatchTileGroupHeader;
class RefListStruct;
class PatchTileGroupDataUnit;
class PatchInformationData;
class PatchDataUnit;
class DeltaPatchDataUnit;
class PCMPatchDataUnit;
class PointLocalReconstructionInformation;
class PointLocalReconstructionData;
class SeiMessage;
class PatchFrameTileInformation;

class PCCBitstreamDecoder {
 public:
  PCCBitstreamDecoder();
  ~PCCBitstreamDecoder();

  int32_t decode( PCCBitstream& bitstream, PCCContext& context );

 private:
  // 7.3.2.1 General V-PCC unit syntax
  void vpccUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType );

  // 7.3.2.2 V-PCC unit header syntax
  void vpccUnitHeader( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType );

  // 7.3.2.3 PCM separate video data syntax
  void pcmSeparateVideoData( PCCContext& context, PCCBitstream& bitstream, uint8_t bitCount );

  // 7.3.2.4 V-PCC unit payload syntax
  void vpccUnitPayload( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType );

  void vpccVideoDataUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType );

  // 7.3.4.1 General Sequence parameter set syntax
  void sequenceParameterSet( SequenceParameterSet& sequenceParameterSet, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.4.2 Byte alignment syntax
  void byteAlignment( PCCBitstream& bitstream );

  // 7.3.4.2 Profile, tier, and level syntax
  void profileTierLevel( ProfileTierLevel& profileTierLevel, PCCBitstream& bitstream );

  // 7.3.4.3 Occupancy parameter set syntax
  void occupancyInformation( OccupancyInformation& occupancyInformation, PCCBitstream& bitstream );

  // 7.3.4.4 Geometry parameter set syntax
  void geometryInformation( GeometryInformation&  geometryInformation,
                            SequenceParameterSet& sequenceParameterSet,
                            PCCBitstream&         bitstream );

  // 7.3.4.5 Attribute parameter set syntax
  void attributeInformation( AttributeInformation& attributeInformation,
                             SequenceParameterSet& sequenceParameterSet,
                             PCCBitstream&         bitstream );

  // 7.3.5.1 General patch data group unit syntax  
  void patchDataGroup( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.5.2 Patch data group unit payload syntax 
  void patchDataGroupUnitPayload( PatchDataGroup& patchDataGroup,
                                  PDGUnitType     unitType,
                                  size_t          frameIndex,
                                  PCCContext&     context,
                                  PCCBitstream&   bitstream );


  // 7.3.5.3 Patch sequence parameter set syntax
  void patchSequenceParameterSet( PatchDataGroup& pdg, PCCBitstream& bitstream );

  // 7.3.5.4 Patch frame geometry parameter set syntax 
  void patchFrameGeometryParameterSet( PatchDataGroup&       patchDataGroup,
                                       SequenceParameterSet& sequenceParameterSet,
                                       PCCBitstream&         bitstream );

  // 7.3.5.5 Geometry frame Params syntax
  void geometryFrameParams( GeometryFrameParams& geometryFrameParams, PCCBitstream& bitstream );

  // 7.3.5.6 Patch frame attribute parameter set syntax 
  void patchFrameAttributeParameterSet( PatchDataGroup&       patchDataGroup,
                                        SequenceParameterSet& sequenceParameterSet,
                                        PCCBitstream&         bitstream );

  // 7.3.5.7 Attribute frame Params syntax
  void attributeFrameParams( AttributeFrameParams& attributeFrameParams, size_t dimension, PCCBitstream& bitstream );

  // 7.3.5.8 Geometry patch parameter set syntax
  void geometryPatchParameterSet( PatchDataGroup& patchDataGroup, PCCBitstream& bitstream );

  // 7.3.5.9 Geometry patch Params
  void geometryPatchParams( GeometryPatchParams&            geometryPatchParams,
                            PatchFrameGeometryParameterSet& patchFrameGeometryParameterSet,
                            PCCBitstream&                   bitstream );

  // 7.3.5.10 Attribute patch parameter set syntax
  void attributePatchParameterSet( PatchDataGroup&       patchDataGroup,
                                   SequenceParameterSet& sequenceParameterSet,
                                   PCCBitstream&         bitstream );

  // 7.3.5.11 Attribute patch Params syntax
  void attributePatchParams( AttributePatchParams&            attributePatchParams,
                             PatchFrameAttributeParameterSet& afps,
                             size_t                           dimension,
                             PCCBitstream&                    bitstream );

  // 7.3.5.12 Patch frame parameter set syntax
  void patchFrameParameterSet( PatchDataGroup&       patchDataGroup,
                               SequenceParameterSet& sequenceParameterSet,
                               PCCBitstream&         bitstream );

  void patchFrameTileInformation( PatchFrameTileInformation& pfti, SequenceParameterSet& sps, PCCBitstream& bitstream );

  // 7.3.5.13 Patch frame layer unit syntax
  void patchTileGroupLayerUnit( PatchDataGroup& pdg,
                                uint32_t        frameIndex,
                                PCCContext&     context,
                                PCCBitstream&   bitstream );

  // 7.3.5.14 Patch frame header syntax
  void patchTileGroupHeader( PatchTileGroupHeader& ptgh,
                             PatchTileGroupHeader& pfhPrev,
                             PCCContext&           context,
                             PCCBitstream&         bitstream );

  // 7.3.5.15 Reference list structure syntax
  void refListStruct( RefListStruct&             refListStruct,
                      PatchSequenceParameterSet& patchSequenceParameterSet,
                      PCCBitstream&              bitstream );

  // 7.3.5.16 Patch frame data unit syntax
  void patchTileGroupDataUnit( PatchTileGroupDataUnit& ptgdu,
                               PatchTileGroupHeader&   ptgh,
                               PCCContext&             context,
                               PCCBitstream&           bitstream );

  // 7.3.5.17 Patch information data syntax
  void patchInformationData( PatchInformationData& pid,
                             size_t                patchMode,
                             PatchTileGroupHeader& ptgh,
                             PCCContext&           context,
                             PCCBitstream&         bitstream );

  // 7.3.5.18 Patch data unit syntax
  void patchDataUnit( PatchDataUnit& pdu, PatchTileGroupHeader& ptgh, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.5.19  Delta Patch data unit syntax
  void deltaPatchDataUnit( DeltaPatchDataUnit&   dpdu,
                           PatchTileGroupHeader& ptgh,
                           PCCContext&           context,
                           PCCBitstream&         bitstream );

  // 7.3.5.20 PCM patch data unit syntax
  void pcmPatchDataUnit( PCMPatchDataUnit&     ppdu,
                         PatchTileGroupHeader& ptgh,
                         PCCContext&           context,
                         PCCBitstream&         bitstream );

  // 7.3.5.21 Point local reconstruction syntax
  void pointLocalReconstructionInformation( PointLocalReconstructionInformation& plri,
                                            PCCContext&                          context,
                                            PCCBitstream&                        bitstream );
  void pointLocalReconstructionData( PointLocalReconstructionData& plrd, PCCContext& context, PCCBitstream& bitstream );


  // 7.3.5.22 Supplemental enhancement information message syntax 
  void seiMessage( PatchDataGroup& pdg, PCCContext& context, PCCBitstream& bitstream );
  
  int32_t prevPatchSizeU_;
  int32_t prevPatchSizeV_;
  int32_t predPatchIndex_;
  int32_t predFramePatchTileGroupLayerUnitIndex_;
};

};  // namespace pcc

#endif /* PCCBitstreamDecoder_h */
