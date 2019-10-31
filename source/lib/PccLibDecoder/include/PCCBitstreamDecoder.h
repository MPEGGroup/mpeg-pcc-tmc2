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
class VpccParameterSet;
class OccupancyInformation;
class GeometryInformation;
class AttributeInformation;

class AtlasTileGroupHeader;
class AtlasTileGroupDataUnit;

class PatchDataGroup;
class PatchVpccParameterSet;
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
class InterPatchDataUnit;
class MergePatchDataUnit;
class SkipPatchDataUnit;
class DeltaPatchDataUnit;
class RawPatchDataUnit;
class EOMPatchDataUnit;
class PointLocalReconstructionInformation;
class PointLocalReconstructionData;
class SeiMessage;
class AtlasFrameTileInformation;
class SampleStreamNalUnit;
class NalUnit;

class PCCBitstreamDecoder {
 public:
  PCCBitstreamDecoder();
  ~PCCBitstreamDecoder();

  // JR: OLD
  int32_t decode( PCCBitstream& bitstream, PCCContext& context );

  // JR: NEW
  void    read( PCCBitstream& bitstream, SampleStreamNalUnit& ssnu );
  int32_t decode( SampleStreamNalUnit& ssnu, PCCContext& context );

#ifdef BITSTREAM_TRACE  
  void setTraceFile( FILE* traceFile ){
    traceFile_ = traceFile;
  }
#endif
 private:
  // 7.3.2.1 General V-PCC unit syntax
  void vpccUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType );

  // 7.3.2.2 V-PCC unit header syntax
  void vpccUnitHeader( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType );

  // 7.3.2.3 V-PCC unit payload syntax
  void vpccUnitPayload( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType );

  void videoSubStream( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType );

  // 7.3.3 Byte alignment syntax
  void byteAlignment( PCCBitstream& bitstream );

  // 7.3.4.1 General Sequence parameter set syntax
  void vpccParameterSet( VpccParameterSet& vpccParameterSet, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.4.2 Profile, tier, and level syntax
  void profileTierLevel( ProfileTierLevel& profileTierLevel, PCCBitstream& bitstream );

  // 7.3.4.3 Occupancy parameter set syntax
  void occupancyInformation( OccupancyInformation& occupancyInformation, PCCBitstream& bitstream );

  // 7.3.4.4 Geometry parameter set syntax
  void geometryInformation( GeometryInformation& geometryInformation,
                            VpccParameterSet&    vpccParameterSet,
                            PCCBitstream&        bitstream );

  // 7.3.4.5 Attribute parameter set syntax
  void attributeInformation( AttributeInformation& attributeInformation,
                             VpccParameterSet&     vpccParameterSet,
                             PCCBitstream&         bitstream );

  // jkei: newly added RBSP syntax
  // TODO: fill the functions
  bool moreRbspData( PCCBitstream& bitstream );
  void rbspTrailingBits( PCCBitstream& bitstream );
  // 7.3.5.1  General NAL unit syntax
  // 7.3.5.2  NAL unit header syntax
  // 7.3.6.1  Atlas sequence parameter set RBSP syntax
  void atlasSequenceParameterSetRBSP( AtlasSequenceParameterSetRBSP& asps,
                                      PCCContext&                    context,
                                      PCCBitstream&                  bitstream );
  // 7.3.6.2  Point local reconstruction information syntax (NEW, comes from below)
  void pointLocalReconstructionInformation( AtlasSequenceParameterSetRBSP& asps,
                                            PCCContext&                    context,
                                            PCCBitstream&                  bitstream );

  // 7.3.6.3  Atlas frame parameter set RBSP syntax
  void atlasFrameParameterSetRbsp( AtlasFrameParameterSetRbsp& afps, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.6.4  Atlas frame tile information syntax
  void atlasFrameTileInformation( AtlasFrameTileInformation& pfti, VpccParameterSet& sps, PCCBitstream& bitstream );

  // 7.3.6.5  Supplemental enhancement information RBSP syntax
  void seiRbsp( PCCContext& context, PCCBitstream& bitstream );
  // 7.3.6.6  Access unit delimiter RBSP syntax
  void auDelimiterRbsp( AccessUnitDelimiterRbsp& audrbsp, PCCContext& context, PCCBitstream& bitstream );
  // 7.3.6.7  End of sequence RBSP syntax
  void endOfSequenceRbsp( EndOfSequenceRbsp& eosbsp, PCCContext& context, PCCBitstream& bitstream );
  // 7.3.6.8  End of bitstream RBSP syntax
  void endOfBitstreamRbsp( EndOfBitstreamRbsp& eobrbsp, PCCContext& context, PCCBitstream& bitstream );
  // 7.3.6.9  Filler data RBSP syntax
  void fillerDataRbsp( FillerDataRbsp& fdrbsp, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.6.10  Atlas tile group layer RBSP syntax = patchTileGroupLayerUnit
  void atlasTileGroupLayerRbsp( AtlasTileGroupLayerRbsp& atgl, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.6.11  Atlas tile group header syntax
  void atlasTileGroupHeader( AtlasTileGroupHeader& atgh, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.6.12  Reference list structure syntax
  void refListStruct( RefListStruct& rls, AtlasSequenceParameterSetRBSP& asps, PCCBitstream& bitstream );
  // 7.3.7.1  General atlas tile group data unit syntax =patchTileGroupDataUnit
  void atlasTileGroupDataUnit( AtlasTileGroupDataUnit& atgdu,
                               AtlasTileGroupHeader&   atgh,
                               PCCContext&             context,
                               PCCBitstream&           bitstream );

  // 7.3.7.2  Patch information data syntax
  void patchInformationData( PatchInformationData& pid,
                             size_t                patchMode,
                             AtlasTileGroupHeader& atgh,
                             PCCContext&           context,
                             PCCBitstream&         bitstream );

  // 7.3.7.3  Patch data unit syntax : AtlasTileGroupHeader instead of PatchTileGroupHeader
  void patchDataUnit( PatchDataUnit& pdu, AtlasTileGroupHeader& atgh, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.7.4  Skip patch data unit syntax
  void skipPatchDataUnit( SkipPatchDataUnit&    spdu,
                          AtlasTileGroupHeader& atgh,
                          PCCContext&           context,
                          PCCBitstream&         bitstream );

  // 7.3.7.5  Merge patch data unit syntax
  void mergePatchDataUnit( MergePatchDataUnit&   mpdu,
                           AtlasTileGroupHeader& atgh,
                           PCCContext&           context,
                           PCCBitstream&         bitstream );

  // 7.3.7.6  Inter patch data unit syntax
  void interPatchDataUnit( InterPatchDataUnit&   ipdu,
                           AtlasTileGroupHeader& atgh,
                           PCCContext&           context,
                           PCCBitstream&         bitstream );

  // 7.3.7.7  Raw patch data unit syntax
  void rawPatchDataUnit( RawPatchDataUnit&     rpdu,
                         AtlasTileGroupHeader& ptgh,
                         PCCContext&           context,
                         PCCBitstream&         bitstream );
  // 7.3.7.8  EOM patch data unit syntax
  void eomPatchDataUnit( EOMPatchDataUnit&     epdu,
                         AtlasTileGroupHeader& ptgh,
                         PCCContext&           context,
                         PCCBitstream&         bitstream );

  // 7.3.7.9  Point local reconstruction data syntax
  void pointLocalReconstructionData( PointLocalReconstructionData& plrd, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.8  Supplemental enhancement information message syntax
  void seiMessage( PatchDataGroup& pdg, PCCContext& context, PCCBitstream& bitstream );

  // jkei: <------- added up to this pointOLD PST.Oct30th

  // jkei: OLD--->
  // 7.3.5.1 General patch data group unit syntax
  void atlasSubStream( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.5.2 Patch data group unit payload syntax
  void atlasSubStreamUnitPayload( PatchDataGroup& atlasSubStream,
                                  PDGUnitType     unitType,
                                  size_t          frameIndex,
                                  PCCContext&     context,
                                  PCCBitstream&   bitstream );

  // 7.3.5.3 Patch sequence parameter set syntax
  void patchVpccParameterSet( PatchDataGroup& pdg, PCCBitstream& bitstream );

  // 7.3.5.4 Patch frame geometry parameter set syntax
  void patchFrameGeometryParameterSet( PatchDataGroup&   atlasSubStream,
                                       VpccParameterSet& vpccParameterSet,
                                       PCCBitstream&     bitstream );

  // 7.3.5.5 Geometry frame Params syntax
  void geometryFrameParams( GeometryFrameParams& geometryFrameParams, PCCBitstream& bitstream );

  // 7.3.5.6 Patch frame attribute parameter set syntax
  void patchFrameAttributeParameterSet( PatchDataGroup&   atlasSubStream,
                                        VpccParameterSet& vpccParameterSet,
                                        PCCBitstream&     bitstream );

  // 7.3.5.7 Attribute frame Params syntax
  void attributeFrameParams( AttributeFrameParams& attributeFrameParams, size_t dimension, PCCBitstream& bitstream );

  // 7.3.5.8 Geometry patch parameter set syntax
  void geometryPatchParameterSet( PatchDataGroup& atlasSubStream, PCCBitstream& bitstream );

  // 7.3.5.9 Geometry patch Params
  void geometryPatchParams( GeometryPatchParams&            geometryPatchParams,
                            PatchFrameGeometryParameterSet& patchFrameGeometryParameterSet,
                            PCCBitstream&                   bitstream );

  // 7.3.5.10 Attribute patch parameter set syntax
  void attributePatchParameterSet( PatchDataGroup&   atlasSubStream,
                                   VpccParameterSet& vpccParameterSet,
                                   PCCBitstream&     bitstream );

  // 7.3.5.11 Attribute patch Params syntax
  void attributePatchParams( AttributePatchParams&            attributePatchParams,
                             PatchFrameAttributeParameterSet& afps,
                             size_t                           dimension,
                             PCCBitstream&                    bitstream );

  // 7.3.5.12 Patch frame parameter set syntax
  void patchFrameParameterSet( PatchDataGroup&   atlasSubStream,
                               VpccParameterSet& vpccParameterSet,
                               PCCBitstream&     bitstream );

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

  // 7.3.5.15 Reference list structure syntax (OLD)
  void refListStruct( RefListStruct&         refListStruct,
                      PatchVpccParameterSet& patchVpccParameterSet,
                      PCCBitstream&          bitstream );

  // 7.3.5.15 Reference list structure syntax (NEW: jkei: this went up)
  // void  refListStruct( RefListStruct&             rls,
  //                    AtlasSequenceParameterSetRBSP& asps,
  //                    PCCBitstream&              bitstream );

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

  // 7.3.7.3 Patch data unit syntax
  void patchDataUnit( PatchDataUnit& pdu, PatchTileGroupHeader& ptgh, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.5.19  Delta Patch data unit syntax
  void deltaPatchDataUnit( DeltaPatchDataUnit&   dpdu,
                           PatchTileGroupHeader& ptgh,
                           PCCContext&           context,
                           PCCBitstream&         bitstream );

  // 7.3.5.20 raw patch data unit syntax
  void pcmPatchDataUnit( RawPatchDataUnit&     ppdu,
                         PatchTileGroupHeader& ptgh,
                         PCCContext&           context,
                         PCCBitstream&         bitstream );

  // 7.3.5.x EOM patch data unit syntax
  void eomPatchDataUnit( EOMPatchDataUnit&     epdu,
                         PatchTileGroupHeader& ptgh,
                         PCCContext&           context,
                         PCCBitstream&         bitstream );

  // 7.3.5.21 Point local reconstruction syntax (OLD)
  void pointLocalReconstructionInformation( PointLocalReconstructionInformation& plri,
                                            PCCContext&                          context,
                                            PCCBitstream&                        bitstream );

  // 7.3.5.21 Point local reconstruction syntax (NEW jkei: this went up)
  //  void pointLocalReconstructionInformation( AtlasSequenceParameterSetRBSP& asps,
  //                                            PCCContext&                    context,
  //                                            PCCBitstream&                  bitstream );
  //
  //  void pointLocalReconstructionData( PointLocalReconstructionData& plrd, PCCContext& context, PCCBitstream&
  //  bitstream );

  // 7.3.5.22 Supplemental enhancement information message syntax (jkei: this went up)
  // void seiMessage( PatchDataGroup& pdg, PCCContext& context, PCCBitstream& bitstream );

  // JR TODO: continue

  // C.2 Sample stream NAL unit syntax and semantics
  // C.2.1 Sample stream NAL header syntax
  void sampleStreamVpccHeader( PCCBitstream& bitstream, SampleStreamNalUnit& sampleStreamNalUnit );
  // C.2.2 Sample stream NAL unit syntax
  void sampleStreamNalUnit( PCCBitstream& bitstream, SampleStreamNalUnit& sampleStreamNalUnit );
  // 7.3.5 NAL unit syntax
  // 7.3.5.1 General NAL unit syntax
  void nalUnit( PCCBitstream& bitstream, NalUnit& nalUnit );
  // 7.3.5.2 NAL unit header syntax
  void nalUnitHeader( PCCBitstream& bitstream, NalUnit& nalUnit );

  // F.2.1 VUI parameters syntax
  void vuiParameters();
  // F.2.2 HRD parameters syntax
  void hrdParameters();
  // F.2.3 Sub-layer HRD parameters syntax
  void hrdSubLayerParameters();

  // JR TODO: remove
  // 7.3.2.3 raw separate video data syntax - TODO: remove this
  void pcmSeparateVideoData( PCCContext& context, PCCBitstream& bitstream, uint8_t bitCount );

  int32_t prevPatchSizeU_;
  int32_t prevPatchSizeV_;
  int32_t predPatchIndex_;
  int32_t predFramePatchTileGroupLayerUnitIndex_;

#ifdef BITSTREAM_TRACE  
  FILE* traceFile_; 
#endif
};

};  // namespace pcc

#endif /* PCCBitstreamDecoder_h */
