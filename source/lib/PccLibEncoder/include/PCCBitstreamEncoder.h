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
#ifndef PCCBitstreamEncoder_H
#define PCCBitstreamEncoder_H

#include "PCCCommon.h"
#include "PCCEncoderParameters.h"
#include "PCCCodec.h"
#include <map>

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
class PatchDataGroupUnitPayload;
class PatchVpccParameterSet;
class PatchFrameGeometryParameterSet;
class PatchFrameAttributeParameterSet;
class GeometryPatchParameterSet;
class GeometryPatchParams;
class AttributePatchParameterSet;
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
class RawPatchDataUnit;
class EOMPatchDataUnit;
class AttributeSequenceParams;
class AttributeFrameParams;
class PointLocalReconstructionInformation;
class PointLocalReconstructionData;
class SeiMessage;
class AtlasFrameTileInformation;
class SampleStreamNalUnit;
class NalUnit;

class PCCBitstreamEncoder {
 public:
  PCCBitstreamEncoder();
  ~PCCBitstreamEncoder();

  int32_t write( SampleStreamNalUnit& ssnu, PCCBitstream& bitstream );
  size_t  write( SampleStreamVpccUnit& ssvu, PCCBitstream& bitstream );
  int     encode( PCCContext& context, SampleStreamVpccUnit& ssvu );
  void    setParameters( PCCEncoderParameters params );

#ifdef BITSTREAM_TRACE
  void setTraceFile( FILE* traceFile ) { traceFile_ = traceFile; }
#endif
 private:
  // 7.3.2.1 General V-PCC unit syntax
  void vpccUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType );

  // 7.3.2.2 V-PCC unit header syntax
  void vpccUnitHeader( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType );

  // 7.3.2.4 V-PCC unit payload syntax
  void vpccUnitPayload( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType );

  void videoSubStream( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType );

  // 7.3.3	Byte alignment syntax
  void byteAlignment( PCCBitstream& bitstream );

  // 7.3.4.1 General Sequence parameter set syntax
  void vpccParameterSet( VpccParameterSet& vpccParameterSet, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.4.2 Profile, tier, and level
  void profileTierLevel( ProfileTierLevel& profileTierLevel, PCCBitstream& bitstream );

  // 7.3.4.3 Occupancy parameter information syntax
  void occupancyInformation( OccupancyInformation& occupancyInformation, PCCBitstream& bitstream );

  // 7.3.4.4 Geometry information syntax
  void geometryInformation( GeometryInformation& geometryInformation,
                            VpccParameterSet&    vpccParameterSet,
                            PCCBitstream&        bitstream );

  // 7.3.4.5 Attribute information syntax
  void attributeInformation( AttributeInformation& attributeInformation,
                             VpccParameterSet&     vpccParameterSet,
                             PCCBitstream&         bitstream );

  // 7.2 Specification of syntax functions and descriptors
  bool byteAligned( PCCBitstream& bitstream );
  bool moreDataInPayload( PCCBitstream& bitstream );
  bool moreRbspData( PCCBitstream& bitstream );
  bool moreRbspTrailingData( PCCBitstream& bitstream );
  bool moreDataInVpccUnit( PCCBitstream& bitstream );
  void rbspTrailingBits( PCCBitstream& bitstream );
  bool payloadExtensionPresent( PCCBitstream& bitstream );

  // 7.3.5 NAL unit syntax
  // 7.3.5.1 General NAL unit syntax
  void nalUnit( PCCBitstream& bitstream, NalUnit& nalUnit );
  // 7.3.5.2 NAL unit header syntax
  void nalUnitHeader( PCCBitstream& bitstream, NalUnit& nalUnit );

  // 7.3.5.2  NAL unit header syntax
  // 7.3.6.1  Atlas sequence parameter set RBSP syntax
  void atlasSequenceParameterSetRBSP( AtlasSequenceParameterSetRBSP& asps,
                                      PCCContext&                    context,
                                      PCCBitstream&                  bitstream );
  // 7.3.6.2  Point local reconstruction information syntax
  void pointLocalReconstructionInformation( AtlasSequenceParameterSetRBSP& asps,
                                            PCCContext&                    context,
                                            PCCBitstream&                  bitstream );

  // 7.3.6.3  Atlas frame parameter set RBSP syntax
  void atlasFrameParameterSetRbsp( AtlasFrameParameterSetRbsp& afps, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.6.4  Atlas frame tile information syntax
  void atlasFrameTileInformation( AtlasFrameTileInformation& pfti, VpccParameterSet& sps, PCCBitstream& bitstream );

  // 7.3.6.5  Supplemental enhancement information RBSP syntax
  void seiRbsp( PCCContext& context, PCCBitstream& bitstream, SEI& sei, NalUnitType nalUnitType );

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
  void pointLocalReconstructionData( PointLocalReconstructionData&  plrd,
                                     PCCContext&                    context,
                                     AtlasSequenceParameterSetRBSP& asps,
                                     PCCBitstream&                  bitstream );

  // 7.3.8 Supplemental enhancement information message syntax
  void seiMessage( PCCBitstream& bitstream, PCCContext& context, SEI& sei, NalUnitType nalUnitType );

  // 7.3.5.1 General patch data group unit syntax
  void atlasSubStream( PCCContext& context, PCCBitstream& bitstream );

  // 7.3.5.3 Patch sequence parameter set syntax
  void patchVpccParameterSet( PatchDataGroup& pdg, size_t index, PCCBitstream& bitstream );

  // 7.3.5.4 Patch frame geometry parameter set syntax
  void patchFrameGeometryParameterSet( PatchDataGroup&   pdg,
                                       size_t            index,
                                       VpccParameterSet& vpccParameterSet,
                                       PCCBitstream&     bitstream );

  // 7.3.5.5 Geometry frame Params syntax
  void geometryFrameParams( GeometryFrameParams& geometryFrameParams, PCCBitstream& bitstream );

  // 7.3.5.6 Patch frame attribute parameter set syntax
  void patchFrameAttributeParameterSet( PatchDataGroup&   pdg,
                                        size_t            index,
                                        VpccParameterSet& vpccParameterSet,
                                        PCCBitstream&     bitstream );

  // 7.3.5.7 Attribute frame Params syntax
  void attributeFrameParams( AttributeFrameParams& attributeFrameParams, size_t dimension, PCCBitstream& bitstream );

  // 7.3.5.8 Geometry patch parameter set syntax
  void geometryPatchParameterSet( PatchDataGroup& atlasSubStream, size_t index, PCCBitstream& bitstream );

  // 7.3.5.9 Geometry patch Params syntax
  void geometryPatchParams( GeometryPatchParams&            geometryPatchParams,
                            PatchFrameGeometryParameterSet& patchFrameGeometryParameterSet,
                            PCCBitstream&                   bitstream );

  // 7.3.5.10 Attribute patch parameter set syntax
  void attributePatchParameterSet( PatchDataGroup&   pdg,
                                   size_t            index,
                                   VpccParameterSet& vpccParameterSet,
                                   PCCBitstream&     bitstream );

  // 7.3.5.11 Attribute patch Params syntax
  void attributePatchParams( AttributePatchParams&            attributePatchParams,
                             PatchFrameAttributeParameterSet& afps,
                             size_t                           dimension,
                             PCCBitstream&                    bitstream );

  // 7.3.5.12 Patch frame parameter set syntax
  void patchFrameParameterSet( PatchDataGroup&   pdg,
                               size_t            index,
                               VpccParameterSet& vpccParameterSet,
                               PCCBitstream&     bitstream );

  // 7.3.5.13 Patch frame layer unit syntax
  void patchTileGroupLayerUnit( PatchDataGroup& pdg, size_t index, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.5.14 Patch frame header syntax
  void patchTileGroupHeader( PatchTileGroupHeader& patchTileGroupHeader,
                             PatchTileGroupHeader& pfhPrev,
                             PCCContext&           context,
                             PCCBitstream&         bitstream );

  // 7.3.5.15 Reference list structure syntax
  void refListStruct( RefListStruct&         refListStruct,
                      PatchVpccParameterSet& patchVpccParameterSet,
                      PCCBitstream&          bitstream );

  // 7.3.5.16 Patch frame data unit syntax
  void patchTileGroupDataUnit( PatchTileGroupDataUnit& ptgdu,
                               PatchTileGroupHeader&   patchTileGroupHeader,
                               PCCContext&             context,
                               PCCBitstream&           bitstream );

  // 7.3.5.17 Patch information data syntax
  void patchInformationData( PatchInformationData& pid,
                             size_t                patchIndex,
                             size_t                patchMode,
                             PatchTileGroupHeader& ptgh,
                             PCCContext&           context,
                             PCCBitstream&         bitstream );

  // 7.3.5.18 Patch data unit syntax
  void patchDataUnit( PatchDataUnit& pdu, PatchTileGroupHeader& ptgh, PCCContext& context, PCCBitstream& bitstream );

  // 7.3.5.19  Delta Patch data unit
  void deltaPatchDataUnit( InterPatchDataUnit&   dpdu,
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

  // 7.3.5.22 Supplemental enhancement information message syntax
  void seiMessage( PatchDataGroup& pdg, size_t index, PCCContext& context, PCCBitstream& bitstream );

  // B.2 Sample stream V-PCC unit syntax and semantics
  // B.2.1 Sample stream V-PCC header syntax
  void sampleStreamVpccHeader( PCCBitstream& bitstream, SampleStreamVpccUnit& ssvu );
  // B.2.2 Sample stream NAL unit syntax
  void sampleStreamVpccUnit( PCCBitstream& bitstream, SampleStreamVpccUnit& ssvu, VpccUnit& vpccUnit );

  // C.2 Sample stream NAL unit syntax and semantics
  // C.2.1 Sample stream NAL header syntax
  void sampleStreamNalHeader( PCCBitstream& bitstream, SampleStreamNalUnit& ssnu );
  // C.2.2 Sample stream NAL unit syntax
  void sampleStreamNalUnit( PCCContext&          context,
                            PCCBitstream&        bitstream,
                            SampleStreamNalUnit& ssnu,
                            NalUnit&             nalUnit,
                            size_t               frameIdx = 0 );

  // E.2  SEI payload syntax
  // E.2.1  General SEI message syntax
  void seiPayload( PCCBitstream& bitstream, PCCContext& context, SEI& sei, NalUnitType nalUnitType );

  // E.2.2  Filler payload SEI message syntax
  void fillerPayload( PCCBitstream& bitstream, SEI& sei );
  // E.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
  void userDataRegisteredItuTT35( PCCBitstream& bitstream, SEI& sei );
  // E.2.4  User data unregistered SEI message syntax
  void userDataUnregistered( PCCBitstream& bitstream, SEI& sei );
  // E.2.5  Recovery point SEI message syntax
  void recoveryPoint( PCCBitstream& bitstream, SEI& sei );
  // E.2.6  No display SEI message syntax
  void noDisplay( PCCBitstream& bitstream, SEI& sei );
  // E.2.7  Reserved SEI message syntax
  void reservedSeiMessage( PCCBitstream& bitstream, SEI& sei );
  // E.2.8  SEI manifest SEI message syntax
  void seiManifest( PCCBitstream& bitstream, SEI& sei );
  // E.2.9  SEI prefix indication SEI message syntax
  void seiPrefixIndication( PCCBitstream& bitstream, SEI& sei );
  // E.2.10  Geometry transformation parameters SEI message syntax
  void geometryTransformationParams( PCCBitstream& bitstream, SEI& sei );
  // E.2.11  Attribute transformation parameters SEI message syntax
  void attributeTransformationParams( PCCBitstream& bitstream, SEI& sei );
  // E.2.12  Active substreams SEI message syntax
  void activeSubstreams( PCCBitstream& bitstream, SEI& sei );
  // E.2.13  Component codec mapping SEI message syntax
  void componentCodecMapping( PCCBitstream& bitstream, SEI& sei );
  // E.2.14  Volumetric Tiling SEI message syntax
  // E.2.14.1  General
  void volumetricTilingInfo( PCCBitstream& bitstream, SEI& sei );
  // E.2.14.2  Volumetric Tiling Info Labels
  void volumetricTilingInfoLabels( PCCBitstream& bitstream, SEIVolumetricTilingInfo& sei );
  // E.2.14.3  Volumetric Tiling Info Objects
  void volumetricTilingInfoObjects( PCCBitstream& bitstream, SEIVolumetricTilingInfo& sei );
  // E.2.15  Buffering period SEI message syntax
  void bufferingPeriod( PCCBitstream&        bitstream,
                        SEI&                 sei,
                        bool                 NalHrdBpPresentFlag,
                        bool                 AclHrdBpPresentFlag,
                        std::vector<uint8_t> hrdCabCntMinus1 );
  // E.2.16  Atlas frame timing SEI message syntax
  void atlasFrameTiming( PCCBitstream& bitstream, SEI& sei, bool CabDabDelaysPresentFlag );
  // E.2.17  Presentation inforomation SEI message syntax
  void presentationInformation( PCCBitstream& bitstream, SEI& sei );
  // E.2.18  Smoothing parameters SEI message syntax
  void smoothingParameters( PCCBitstream& bitstream, SEI& sei );

  // F.2  VUI syntax
  // F.2.1  VUI parameters syntax
  void vuiParameters( PCCBitstream& bitstream, VUIParameters& vp );
  // F.2.2  HRD parameters syntax
  void hrdParameters( PCCBitstream& bitstream, HrdParameters& hp );
  // F.2.3  Sub-layer HRD parameters syntax
  void hrdSubLayerParameters( PCCBitstream& bitstream, HrdSubLayerParameters& hlsp, size_t cabCnt );

  PCCEncoderParameters params_;
#ifdef BITSTREAM_TRACE
  FILE* traceFile_;
#endif
};

};  // namespace pcc

#endif /* PCCBitstreamEncoder_H */
