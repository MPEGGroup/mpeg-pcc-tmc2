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

#include "PCCCommon.h"
#include "PCCVideoBitstream.h"
#include "PCCBitstream.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"

#include "PCCBitstreamDecoderNewSyntax.h"

using namespace pcc;

PCCBitstreamDecoderNewSyntax::PCCBitstreamDecoderNewSyntax() {}
PCCBitstreamDecoderNewSyntax::~PCCBitstreamDecoderNewSyntax() {}

int PCCBitstreamDecoderNewSyntax::decode( PCCBitstream& bitstream, PCCContext& context ) {
  VPCCUnitType vpccUnitType;
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_SPS
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_PSD
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_OVD
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_GVD
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_AVD
  return 1;
}

void PCCBitstreamDecoderNewSyntax::vpccVideoDataUnit( PCCContext&   context,
                                                      PCCBitstream& bitstream,
                                                      VPCCUnitType& vpccUnitType ) {
  TRACE_BITSTREAM( "%s start \n", __func__ );
  auto& sps = context.getSps();
  if ( vpccUnitType == VPCC_OVD ) {
    TRACE_BITSTREAM( "OccupancyMap \n" );
    bitstream.read( context.createVideoBitstream( PCCVideoType::OccupancyMap ) );
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( !sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) {
      bitstream.read( context.createVideoBitstream( PCCVideoType::GeometryD0 ) );
      bitstream.read( context.createVideoBitstream( PCCVideoType::GeometryD1 ) );
    } else {
      TRACE_BITSTREAM( "Geometry \n" );
      bitstream.read( context.createVideoBitstream( PCCVideoType::Geometry ) );
    }
    if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
      bitstream.read( context.createVideoBitstream( PCCVideoType::GeometryMP ) );
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( sps.getAttributeCount() ) {
      TRACE_BITSTREAM( "Texture \n" );
      bitstream.read( context.createVideoBitstream( PCCVideoType::Texture ) );
      if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
        bitstream.read( context.createVideoBitstream( PCCVideoType::TextureMP ) );
      }
    }
  }
  TRACE_BITSTREAM( "%s done \n", __func__ );
}

uint32_t PCCBitstreamDecoderNewSyntax::DecodeUInt32( const uint32_t           bitCount,
                                                     o3dgc::Arithmetic_Codec& arithmeticDecoder,
                                                     o3dgc::Static_Bit_Model& bModel0 ) {
  uint32_t decodedValue = 0;
  for ( uint32_t i = 0; i < bitCount; ++i ) {
    decodedValue += ( arithmeticDecoder.decode( bModel0 ) << i );
  }
  return PCCFromLittleEndian<uint32_t>( decodedValue );
}

uint32_t PCCBitstreamDecoderNewSyntax::DecodeUInt32( const uint32_t           bitCount,
                                                     o3dgc::Arithmetic_Codec& arithmeticDecoder,
                                                     o3dgc::Static_Bit_Model& bModel0,
                                                     PCCBitstream&            bitstream ) {
  uint32_t decodedValue = 0;
  for ( uint32_t i = 0; i < bitCount; ++i ) {
    uint32_t v = arithmeticDecoder.decode( bModel0 );
    decodedValue += ( v << i );
    TRACE_BITSTREAM_NH( "CodeUInt32: Bit %u / %u: bit = %u \n", i, bitCount, v );
  }
  TRACE_BITSTREAM_NH( "CodeUInt32: value = %u \n", PCCFromLittleEndian<uint32_t>( decodedValue ) );
  return PCCFromLittleEndian<uint32_t>( decodedValue );
}

// 7.3.2 V-PCC unit syntax
void PCCBitstreamDecoderNewSyntax::vpccUnit( PCCContext&   context,
                                             PCCBitstream& bitstream,
                                             VPCCUnitType& vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  vpccUnitHeader( context, bitstream, vpccUnitType );
  vpccUnitPayload( context, bitstream, vpccUnitType );
}

// 7.3.3 V-PCC unit header syntax
void PCCBitstreamDecoderNewSyntax::vpccUnitHeader( PCCContext&   context,
                                                   PCCBitstream& bitstream,
                                                   VPCCUnitType& vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  auto& vpcc   = context.getVPCC();
  vpccUnitType = (VPCCUnitType)bitstream.read( 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD ||
       vpccUnitType == VPCC_PSD ) {
    vpcc.setSequenceParameterSetId( bitstream.read( 4 ) );  // u(4)
  }
  if ( vpccUnitType == VPCC_AVD ) {
    auto& sps = context.getSps();
    vpcc.setAttributeIndex( bitstream.read( 7 ) );  // u(7)
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      vpcc.setLayerIndex( bitstream.read( 4 ) );  // u(4)
      pcmSeparateVideoData( context, bitstream, 11 );
    } else {
      pcmSeparateVideoData( context, bitstream, 15 );
    }
  } else if ( vpccUnitType == VPCC_GVD ) {
    auto& sps = context.getSps();
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      vpcc.setLayerIndex( bitstream.read( 4 ) );  // u(4)
      pcmSeparateVideoData( context, bitstream, 18 );
    } else {
      pcmSeparateVideoData( context, bitstream, 22 );
    }
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_PSD ) {
    bitstream.read( 23 );  // u(23)
  } else {
    bitstream.read( 27 );  // u(27)
  }
}

// 7.3.4 PCM separate video data syntax
void PCCBitstreamDecoderNewSyntax::pcmSeparateVideoData( PCCContext&   context,
                                                         PCCBitstream& bitstream,
                                                         uint8_t       bitCount ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  if ( sps.getPcmSeparateVideoPresentFlag() && !vpcc.getLayerIndex() ) {
    vpcc.setPCMVideoFlag( bitstream.read( 1 ) );  // u(1)
    bitstream.read( bitCount );                   // u(bitCount)
  } else {
    bitstream.read( bitCount + 1 );  // u(bitCount + 1)
  }
}

// 7.3.5 V-PCC unit payload syntax
void PCCBitstreamDecoderNewSyntax::vpccUnitPayload( PCCContext&   context,
                                                    PCCBitstream& bitstream,
                                                    VPCCUnitType& vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  auto& sps  = context.getSps();  // jkei[!] just for consistancy
  auto& psdu = context.getPatchSequenceDataUnit();
  if ( vpccUnitType == VPCC_SPS ) {
    sequenceParameterSet( sps, bitstream );
  } else if ( vpccUnitType == VPCC_PSD ) {
    patchSequenceDataUnit( context, bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    TRACE_BITSTREAM( "vpccUnitType = %d \n", (int32_t)vpccUnitType );
    vpccVideoDataUnit( context, bitstream, vpccUnitType );
  }
}

// 7.3.6 Sequence parameter set syntax
void PCCBitstreamDecoderNewSyntax::sequenceParameterSet( SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  sps.setSequenceParameterSetId( bitstream.read( 4 ) );   // u(4)
  sps.setFrameWidth( bitstream.read( 16 ) );              // u(16)
  sps.setFrameHeight( bitstream.read( 16 ) );             // u(16)
  sps.setAvgFrameRatePresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( sps.getAvgFrameRatePresentFlag() ) {
    sps.setAvgFrameRate( bitstream.read( 16 ) );  // u(16)
  }
  sps.setEnhancedOccupancyMapForDepthFlag( bitstream.read( 1 ) );  // u(1)
  sps.setLayerCountMinus1( bitstream.read( 4 ) );                  // u(4)
  if ( sps.getLayerCountMinus1() > 0 ) {
    sps.setMultipleLayerStreamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  }
  sps.allocate();
  for ( size_t i = 0; i < sps.getLayerCountMinus1(); i++ ) {
    sps.setLayerAbsoluteCodingEnabledFlag( i + 1, bitstream.read( 1 ) );  // u(1)
    if ( ( sps.getLayerAbsoluteCodingEnabledFlag( i + 1 ) == 0 ) ) {
      if ( i > 0 ) {
        sps.setLayerPredictorIndexDiff( i + 1, bitstream.readUvlc() );  // ue(v)
      } else {
        sps.setLayerPredictorIndexDiff( i + 1, 0 );
      }
    }
  }

  TRACE_BITSTREAM( " sps.getLayerCountMinus1() = %lu \n", sps.getLayerCountMinus1() );
  TRACE_BITSTREAM( " AbsoluteCoding L0 = %lu \n", sps.getLayerAbsoluteCodingEnabledFlag( 0 ) );
  TRACE_BITSTREAM( " AbsoluteCoding L1 = %lu \n", sps.getLayerAbsoluteCodingEnabledFlag( 1 ) );

  sps.setPcmPatchEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( sps.getPcmPatchEnabledFlag() ) {
    sps.setPcmSeparateVideoPresentFlag( bitstream.read( 1 ) );  // u(1)
  }
  occupancyParameterSet( sps.getOccupancyParameterSet(), bitstream );
  geometryParameterSet( sps.getGeometryParameterSet(), sps, bitstream );
  sps.setAttributeCount( bitstream.read( 16 ) );  // u(16)

  sps.allocate();
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    attributeParameterSet( sps.getAttributeParameterSet( i ), sps, bitstream );
  }
  sps.setPatchSequenceOrientationEnabledFlag( bitstream.read( 1 ) );  // u(1)
  sps.setPatchInterPredictionEnabledFlag( bitstream.read( 1 ) );      // u(1)
  sps.setPixelDeinterleavingFlag( bitstream.read( 1 ) );              // u(1)
  sps.setPointLocalReconstructionEnabledFlag( bitstream.read( 1 ) );  // u(1)
  sps.setRemoveDuplicatePointEnabledFlag( bitstream.read( 1 ) );      // u(1)

  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
  sps.setLosslessGeo444( bitstream.read( 1 ) );    // u(1)
  sps.setLosslessGeo( bitstream.read( 1 ) );       // u(1)
  sps.setLosslessTexture( bitstream.read( 1 ) );   // u(1)
  sps.setMinLevel( bitstream.read( 8 ) );          // u(8)
  sps.setSurfaceThickness( bitstream.read( 8 ) );  // u(8)
  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
  byteAlignment( bitstream );
}

// 7.3.7 Byte alignment syntax
void PCCBitstreamDecoderNewSyntax::byteAlignment( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  bitstream.read( 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.read( 1 );  // f(1): equal to 0
  }
}

// 7.3.8 Profile, tier, and level syntax
void PCCBitstreamDecoderNewSyntax::profileTierLevel( ProfileTierLevel& ptl,
                                                     PCCBitstream&     bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ptl.setTierFlag( bitstream.read( 1 ) );    // u(1)
  ptl.setProfileIdc( bitstream.read( 1 ) );  // u(7)
  bitstream.read( 48 );                      // u(48)
  ptl.setLevelIdc( bitstream.read( 8 ) );    // u(8)
}

// 7.3.9 Occupancy parameter set syntax
void PCCBitstreamDecoderNewSyntax::occupancyParameterSet( OccupancyParameterSet& ops,
                                                          PCCBitstream&          bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ops.setOccupancyCodecId( bitstream.read( 8 ) );           // u(8)
  ops.setOccupancyPackingBlockSize( bitstream.read( 8 ) );  // u(8)
}

// 7.3.10 Geometry parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryParameterSet( GeometryParameterSet& gps,
                                                         SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  gps.setGeometryCodecId( bitstream.read( 8 ) );                      // u(8)
  gps.setGeometryNominal2dBitdepthMinus1( bitstream.read( 5 ) );      // u(5)
  gps.setGeometry3dCoordinatesBitdepthMinus1( bitstream.read( 5 ) );  // u(5)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    gps.setPcmGeometryCodecId( bitstream.read( 1 ) );  // u(8)
  }
  gps.setGeometryParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( gps.getGeometryParamsEnabledFlag() ) {
    geometrySequenceParams( gps.getGeometrySequenceParams(), bitstream );
  }

  gps.setGeometryPatchParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( gps.getGeometryPatchParamsEnabledFlag() ) {
    gps.setGeometryPatchScaleParamsEnabledFlag( bitstream.read( 1 ) );     // u(1)
    gps.setGeometryPatchOffsetParamsEnabledFlag( bitstream.read( 1 ) );    // u(1)
    gps.setGeometryPatchRotationParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
    gps.setGeometryPatchPointSizeInfoEnabledFlag( bitstream.read( 1 ) );   // u(1)
    gps.setGeometryPatchPointShapeInfoEnabledFlag( bitstream.read( 1 ) );  // u(1)
  }
}

// 7.3.11 Geometry sequence Params syntax
void PCCBitstreamDecoderNewSyntax::geometrySequenceParams( GeometrySequenceParams& gsp,
                                                           PCCBitstream&           bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  gsp.setGeometrySmoothingParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  gsp.setGeometryScaleParamsPresentFlag( bitstream.read( 1 ) );      // u(1)
  gsp.setGeometryOffsetParamsPresentFlag( bitstream.read( 1 ) );     // u(1)
  gsp.setGeometryRotationParamsPresentFlag( bitstream.read( 1 ) );   // u(1)
  gsp.setGeometryPointSizeInfoPresentFlag( bitstream.read( 1 ) );    // u(1)
  gsp.setGeometryPointShapeInfoPresentFlag( bitstream.read( 1 ) );   // u(1)
  if ( gsp.getGeometrySmoothingParamsPresentFlag() ) {
    gsp.setGeometrySmoothingEnabledFlag( bitstream.read( 1 ) );  // u(8)
    if ( gsp.getGeometrySmoothingEnabledFlag() ) {
      gsp.setGeometrySmoothingGridSize( bitstream.read( 8 ) );   // u(8)
      gsp.setGeometrySmoothingThreshold( bitstream.read( 8 ) );  // u(8)
    }
  }
  if ( gsp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gsp.setGeometryScaleOnAxis( d, bitstream.read( 32 ) );  // u(32)
    }
    if ( gsp.getGeometryOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gsp.setGeometryOffsetOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
      }
    }
    if ( gsp.getGeometryRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gsp.setGeometryRotationOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
      }
    }
    if ( gsp.getGeometryPointSizeInfoPresentFlag() ) {
      gsp.setGeometryPointSizeInfo( bitstream.read( 8 ) );  // u(8)
    }
    if ( gsp.getGeometryPointShapeInfoPresentFlag() ) {
      gsp.setGeometryPointShapeInfo( bitstream.read( 8 ) );  // u(8)
    }
  }
  TRACE_BITSTREAM( "  GeometrySmoothingParamsPresentFlag = %d  \n",
                   gsp.getGeometrySmoothingParamsPresentFlag() );
  TRACE_BITSTREAM( "  GeometryScaleParamsPresentFlag     = %d  \n",
                   gsp.getGeometryScaleParamsPresentFlag() );
  TRACE_BITSTREAM( "  GeometryOffsetParamsPresentFlag    = %d  \n",
                   gsp.getGeometryOffsetParamsPresentFlag() );
  TRACE_BITSTREAM( "  GeometryRotationParamsPresentFlag  = %d  \n",
                   gsp.getGeometryRotationParamsPresentFlag() );
  TRACE_BITSTREAM( "  GeometryPointSizeInfoPresentFlag   = %d  \n",
                   gsp.getGeometryPointSizeInfoPresentFlag() );
  TRACE_BITSTREAM( "  GeometryPointShapeInfoPresentFlag  = %d  \n",
                   gsp.getGeometryPointShapeInfoPresentFlag() );
  TRACE_BITSTREAM( "  getGeometrySmoothingEnabledFlag    = %d  \n",
                   gsp.getGeometrySmoothingEnabledFlag() );
  TRACE_BITSTREAM( "  getGeometrySmoothingGridSize       = %u  \n",
                   gsp.getGeometrySmoothingGridSize() );
  TRACE_BITSTREAM( "  getGeometrySmoothingThreshold      = %u  \n",
                   gsp.getGeometrySmoothingThreshold() );
}

// 7.3.12 Attribute parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributeParameterSet( AttributeParameterSet& aps,
                                                          SequenceParameterSet&  sps,
                                                          PCCBitstream&          bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  aps.setAttributeTypeId( bitstream.read( 4 ) );           // u(4)
  aps.setAttributeDimensionMinus1( bitstream.read( 8 ) );  // u(8)
  aps.setAttributeCodecId( bitstream.read( 8 ) );          // u(8)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    aps.setPcmAttributeCodecId( bitstream.read( 8 ) );  // u(8)
  }
  aps.setAttributeParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( aps.getAttributeParamsEnabledFlag() ) {
    attributeSequenceParams( aps.getAttributeSequenceParams(), aps.getAttributeDimensionMinus1(),
                             bitstream );
  }
  aps.setAttributePatchParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( aps.getAttributePatchParamsEnabledFlag() ) {
    aps.setAttributePatchScaleParamsEnabledFlag( bitstream.read( 1 ) );   // u(1)
    aps.setAttributePatchOffsetParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  }
}

// 7.3.13 Attribute sequence Params syntax
void PCCBitstreamDecoderNewSyntax::attributeSequenceParams( AttributeSequenceParams& asp,
                                                            uint8_t                  dimension,
                                                            PCCBitstream&            bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  asp.setAttributeSmoothingParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  asp.setAttributeScaleParamsPresentFlag( bitstream.read( 1 ) );      // u(1)
  asp.setAttributeOffsetParamsPresentFlag( bitstream.read( 1 ) );     // u(1)
  if ( asp.getAttributeSmoothingParamsPresentFlag() ) {
    asp.setAttributeSmoothingRadius( bitstream.read( 8 ) );                    // u(8)
    asp.setAttributeSmoothingNeighbourCount( bitstream.read( 8 ) );            // u(8)
    asp.setAttributeSmoothingRadius2BoundaryDetection( bitstream.read( 8 ) );  // u(8)
    asp.setAttributeSmoothingThreshold( bitstream.read( 8 ) );                 // u(8)
    asp.setAttributeSmoothingThresholdLocalEntropy( bitstream.read( 8 ) );     // u(3)
  }
  if ( asp.getAttributeScaleParamsPresentFlag() ) {
    for ( size_t i = 0; i < dimension; i++ ) {
      asp.addAttributeScale( bitstream.read( 32 ) );  // u(32)
    }
  }
  if ( asp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < dimension; i++ ) {
      asp.addAttributeOffset( bitstream.read( 32 ) );  // i(32)
    }
  }
}

// 7.3.14 Patch sequence data unit syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceDataUnit( PCCContext&   context,
                                                          PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t i    = 0;
  auto&  sps  = context.getSps();
  auto&  psdu = context.getPatchSequenceDataUnit();
  psdu.setFrameCount( 0 );
  bool terminatePatchSequenceInformationFlag = false;
  while ( !terminatePatchSequenceInformationFlag ) {
    PatchSequenceUnitPayload& psup = psdu.addPatchSequenceUnitPayload();

    psup.setUnitType( (PSDUnitType)bitstream.readUvlc() );  // ue(v)

    TRACE_BITSTREAM( "PSDU %u : type = %u <=> %s \n", i, (uint32_t)psup.getUnitType(),
                     psup.strUnitType().c_str() );
    psup.setFrameIndex( psdu.getFrameCount() );
    patchSequenceUnitPayload( psup, context, bitstream );

    TRACE_BITSTREAM( "  %lu type = %s frameIndex = %lu  \n", i, psup.strUnitType().c_str(),
                     psup.getFrameIndex() );

    if ( psup.getUnitType() == PSD_PFLU ) { psdu.setFrameCount( psdu.getFrameCount() + 1 ); }
    terminatePatchSequenceInformationFlag = bitstream.read( 1 );  // u(1)

    TRACE_BITSTREAM( " type = %s Frame = %lu End = %d \n", psup.strUnitType().c_str(),
                     psup.getFrameIndex(), terminatePatchSequenceInformationFlag );
    i++;
  }
  byteAlignment( bitstream );

  TRACE_BITSTREAM( "%s done \n", __func__ );
}

// 7.3.15 Patch sequence unit payload syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceUnitPayload( PatchSequenceUnitPayload& psup,
                                                             PCCContext&               context,
                                                             PCCBitstream&             bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( " patchSequenceUnitPayload:  type = %u frameIndex = %u\n", psup.getUnitType(),
                   psup.getFrameIndex() );
  if ( psup.getUnitType() == PSD_SPS ) {
    auto&         psdu = context.getPatchSequenceDataUnit();  // perhaps I need to allocate this
    auto&         psps = psdu.getPatchSequenceParameterSet( 0 );
    RefListStruct rls;
    psps.addRefListStruct( rls );
    patchSequenceParameterSet( psup.getPatchSequenceParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_GPPS ) {
    geometryPatchParameterSet( psup.getGeometryPatchParameterSet(),
                               psup.getGeometryFrameParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_APPS ) {
    auto& sps = context.getSps();
    for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
      AttributePatchParameterSet apps;
      psup.addAttributePatchParameterSet( apps );
    }
    auto& psdu = context.getPatchSequenceDataUnit();
    for ( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributePatchParameterSet(
          psup.getAttributePatchParameterSet( attributeIndex ),
          sps.getAttributeParameterSet( attributeIndex ),
          psdu.getAttributeFrameParameterSet( psup.getFrameIndex(), attributeIndex ), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_FPS ) {
    auto& sps = context.getSps();
    patchFrameParameterSet( psup.getPatchFrameParameterSet(), sps, bitstream );
    TRACE_BITSTREAM( " PatchOrientationPresentFlag = %lu \n",
                     psup.getPatchFrameParameterSet().getPatchOrientationPresentFlag() );

  } else if ( psup.getUnitType() == PSD_AFPS ) {
    auto& sps = context.getSps();
    for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
      AttributeFrameParameterSet afps;
      psup.addAttributeFrameParameterSet( afps );
    }
    for ( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributeFrameParameterSet( psup.getAttributeFrameParameterSet( attributeIndex ),
                                  sps.getAttributeParameterSet( attributeIndex ), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_GFPS ) {
    auto& sps = context.getSps();
    geometryFrameParameterSet( psup.getGeometryFrameParameterSet(), sps.getGeometryParameterSet(),
                               bitstream );
  } else if ( psup.getUnitType() == PSD_PFLU ) {
    patchFrameLayerUnit( psup.getPatchFrameLayerUnit(), context, bitstream );
  }
}

// 7.3.16 Patch sequence parameter set syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceParameterSet( PatchSequenceParameterSet& psps,
                                                              PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  psps.setPatchSequenceParameterSetId( bitstream.readUvlc() );         // ue(v)
  psps.setLog2MaxPatchFrameOrderCntLsbMinus4( bitstream.readUvlc() );  // ue(v)
  psps.setMaxDecPatchFrameBufferingMinus1( bitstream.readUvlc() );     // ue(v)
  psps.setLongTermRefPatchFramesFlag( bitstream.read( 1 ) );           // u(1)
  psps.setNumRefPatchFrameListsInSps( bitstream.readUvlc() );          // ue(v)
  psps.allocate( psps.getNumRefPatchFrameListsInSps() );
  for ( size_t i = 0; i < psps.getNumRefPatchFrameListsInSps(); i++ ) {
    refListStruct( psps.getRefListStruct( i ), psps, bitstream );
  }
}

// 7.3.17 Geometry frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryFrameParameterSet( GeometryFrameParameterSet& gfps,
                                                              GeometryParameterSet&      gps,
                                                              PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  gfps.setGeometryFrameParameterSetId( bitstream.readUvlc() );  // ue(v)
  gfps.setPatchSequenceParameterSetId( bitstream.readUvlc() );  // ue(v)
  if ( gps.getGeometryParamsEnabledFlag() ) {
    gfps.setOverrideGeometryParamsFlag( bitstream.read( 1 ) );  // u(1)
    if ( gfps.getOverrideGeometryParamsFlag() ) {
      geometryFrameParams( gfps.getGeometryFrameParams(), bitstream );
    }
  }

  if ( gps.getGeometryPatchParamsEnabledFlag() ) {
    gfps.setOverrideGeometryPatchParamsFlag( bitstream.read( 1 ) );  // u(1)
    if ( gfps.getOverrideGeometryPatchParamsFlag() ) {
      gfps.setGeometryPatchScaleParamsEnabledFlag( bitstream.read( 1 ) );     // u(1)
      gfps.setGeometryPatchOffsetParamsEnabledFlag( bitstream.read( 1 ) );    // u(1)
      gfps.setGeometryPatchRotationParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
      gfps.setGeometryPatchPointSizeInfoEnabledFlag( bitstream.read( 1 ) );   // u(1)
      gfps.setGeometryPatchPointShapeInfoEnabledFlag( bitstream.read( 1 ) );  // u(1)
    }
  }
  byteAlignment( bitstream );
}

// 7.3.18 Geometry frame Params syntax
void PCCBitstreamDecoderNewSyntax::geometryFrameParams( GeometryFrameParams& gfp,
                                                        PCCBitstream&        bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  gfp.setGeometrySmoothingParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  gfp.setGeometryScaleParamsPresentFlag( bitstream.read( 1 ) );      // u(1)
  gfp.setGeometryOffsetParamsPresentFlag( bitstream.read( 1 ) );     // u(1)
  gfp.setGeometryRotationParamsPresentFlag( bitstream.read( 1 ) );   // u(1)
  gfp.setGeometryPointSizeInfoPresentFlag( bitstream.read( 1 ) );    // u(1)
  gfp.setGeometryPointShapeInfoPresentFlag( bitstream.read( 1 ) );   // u(1)

  if ( gfp.getGeometrySmoothingParamsPresentFlag() ) {
    gfp.setGeometrySmoothingEnabledFlag( bitstream.read( 1 ) );  // u(1)
    if ( gfp.getGeometrySmoothingEnabledFlag() ) {
      gfp.setGeometrySmoothingGridSize( bitstream.read( 8 ) );   // u(8)
      gfp.setGeometrySmoothingThreshold( bitstream.read( 8 ) );  // u(8)
    }
  }
  if ( gfp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gfp.setGeometryScaleOnAxis( d, bitstream.read( 32 ) );  // u(32)
    }
  }
  if ( gfp.getGeometryOffsetParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gfp.setGeometryOffsetOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
    }
  }
  if ( gfp.getGeometryRotationParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gfp.setGeometryRotationOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
    }
  }
  if ( gfp.getGeometryPointSizeInfoPresentFlag() ) {
    gfp.setGeometryPointSizeInfo( bitstream.read( 16 ) );  // u(16)
  }
  if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
    gfp.setGeometryPointShapeInfo( bitstream.read( 4 ) );  // u(4)
  }
}

// 7.3.19 Attribute frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributeFrameParameterSet( AttributeFrameParameterSet& afps,
                                                               AttributeParameterSet&      aps,
                                                               PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  afps.setAttributeFrameParameterSetId( bitstream.readUvlc() );  // ue(v)
  afps.setPatchSequencParameterSetId( bitstream.readUvlc() );
  size_t attributeDimension = aps.getAttributeDimensionMinus1() + 1;
  if ( aps.getAttributeParamsEnabledFlag() ) {
    afps.setOverrideAttributeParamsFlag( bitstream.read( 1 ) );  // u(1)
    if ( afps.getOverrideAttributeParamsFlag() ) {
      attributeFrameParams( afps.getAttributeFrameParams(), attributeDimension, bitstream );
    }
  }
  if ( aps.getAttributePatchParamsEnabledFlag() ) {
    afps.setOverrideAttributePatchParamsFlag( bitstream.read( 1 ) );  // u(1)
    if ( afps.getOverrideAttributePatchParamsFlag() ) {
      afps.setAttributePatchScaleParamsEnabledFlag( bitstream.read( 1 ) );   // u(1)
      afps.setAttributePatchOffsetParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
    }
  }
  byteAlignment( bitstream );
}

// 7.3.20 Attribute frame Params syntax
void PCCBitstreamDecoderNewSyntax::attributeFrameParams( AttributeFrameParams& afp,
                                                         size_t                attributeDimension,
                                                         PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  afp.setAttributeSmoothingParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  afp.setAttributeScaleParamsPresentFlag( bitstream.read( 1 ) );      // u(1)
  afp.setAttributeOffsetParamsPresentFlag( bitstream.read( 1 ) );     // u(1)
  if ( afp.getAttributeSmoothingParamsPresentFlag() ) {
    afp.setAttributeSmoothingRadius( bitstream.read( 8 ) );                    // u(8)
    afp.setAttributeSmoothingNeighbourCount( bitstream.read( 8 ) );            // u(8)
    afp.setAttributeSmoothingRadius2BoundaryDetection( bitstream.read( 8 ) );  // u(8)
    afp.setAttributeSmoothingThreshold( bitstream.read( 8 ) );                 // u(8)
    afp.setAttributeSmoothingThresholdLocalEntropy( bitstream.read( 3 ) );     // u(3)
  }
  if ( afp.getAttributeScaleParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ )
      afp.setAttributeScale( i, bitstream.read( 32 ) );  // u32
  }
  if ( afp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ )
      afp.setAttributeOffset( i, convertToInt( bitstream.read( 32 ) ) );  // i32
  }
}

// 7.3.21 Geometry patch parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchParameterSet( GeometryPatchParameterSet& gpps,
                                                              GeometryFrameParameterSet& gfps,
                                                              PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  gpps.setGeometryPatchParameterSetId( bitstream.readUvlc() );  // ue(v)
  gpps.setGeometryFrameParameterSetId( bitstream.readUvlc() );  // ue(v)
  if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ||
       gfps.getGeometryPatchOffsetParamsEnabledFlag() ||
       gfps.getGeometryPatchRotationParamsEnabledFlag() ||
       gfps.getGeometryPatchPointSizeInfoEnabledFlag() ||
       gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    gpps.setGeometryPatchParamsPresentFlag( bitstream.read( 32 ) );  // u(32)
    if ( gpps.getGeometryPatchParamsPresentFlag() )
      geometryPatchParams( gpps.getGeometryPatchParams(), gfps, bitstream );
  }
  byteAlignment( bitstream );
}

// 7.3.22 Geometry patch Params syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchParams( GeometryPatchParams&       gpp,
                                                        GeometryFrameParameterSet& gfps,
                                                        PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ) {
    gpp.setGeometryPatchScaleParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( gpp.getGeometryPatchScaleParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gpp.setGeometryPatchScaleOnAxis( d, bitstream.read( 32 ) );  // u(32)
      }
    }
  }
  if ( gfps.getGeometryPatchOffsetParamsEnabledFlag() ) {
    gpp.setGeometryPatchOffsetParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( gpp.getGeometryPatchOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gpp.setGeometryPatchOffsetOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i32
      }
    }
  }
  if ( gfps.getGeometryPatchRotationParamsEnabledFlag() ) {
    gpp.setGeometryPatchRotationParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( gpp.getGeometryPatchRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gpp.setGeometryPatchRotationOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
      }
    }
  }
  if ( gfps.getGeometryPatchPointSizeInfoEnabledFlag() ) {
    gpp.setGeometryPatchPointSizeInfoPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( gpp.getGeometryPatchPointSizeInfoPresentFlag() ) {
      gpp.setGeometryPatchPointSizeInfo( bitstream.read( 16 ) );  // u(16)
    }
  }
  if ( gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    gpp.setGeometryPatchPointShapeInfoPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( gpp.getGeometryPatchPointShapeInfoPresentFlag() ) {
      gpp.setGeometryPatchPointShapeInfo( bitstream.read( 4 ) );  // u(4)
    }
  }
}

// 7.3.23 Attribute patch parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributePatchParameterSet( AttributePatchParameterSet& apps,
                                                               AttributeParameterSet&      aps,
                                                               AttributeFrameParameterSet& afps,
                                                               PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  apps.setAttributePatchParameterSetId( bitstream.readUvlc() );  // ue(v)
  apps.setAttributeFrameParameterSetId( bitstream.readUvlc() );  // ue(v)
  size_t attributeDimension = aps.getAttributeDimensionMinus1() + 1;
  if ( afps.getAttributePatchScaleParamsEnabledFlag() ||
       afps.getAttributePatchOffsetParamsEnabledFlag() ) {
    apps.setAttributePatchParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( apps.getAttributePatchParamsPresentFlag() ) {
      attributePatchParams( apps.getAttributePatchParams(), afps, attributeDimension, bitstream );
    }
  }
  byteAlignment( bitstream );
}

// 7.3.24 Attribute patch Params syntax
void PCCBitstreamDecoderNewSyntax::attributePatchParams( AttributePatchParams&       app,
                                                         AttributeFrameParameterSet& afps,
                                                         size_t                      dimension,
                                                         PCCBitstream&               bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( afps.getAttributePatchScaleParamsEnabledFlag() ) {
    app.setAttributePatchScaleParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( app.getAttributePatchScaleParamsPresentFlag() ) {
      for ( size_t i = 0; i < dimension; i++ ) {
        app.setAttributePatchScale( i, bitstream.read( 32 ) );  // u(32)
      }
    }
  }
  if ( afps.getAttributePatchOffsetParamsEnabledFlag() ) {
    app.setAttributePatchOffsetParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( app.getAttributePatchOffsetParamsPresentFlag() ) {
      for ( size_t i = 0; i < dimension; i++ ) {
        app.setAttributePatchOffset( i, convertToInt( bitstream.read( 32 ) ) );  // i(32)
      }
    }
  }
}

// 7.3.25 Patch frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::patchFrameParameterSet( PatchFrameParameterSet& pfps,
                                                           SequenceParameterSet&   sps,
                                                           PCCBitstream&           bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  pfps.setPatchFrameParameterSetId( bitstream.readUvlc() );             // ue(v)
  pfps.setPatchSequenceParameterSetId( bitstream.readUvlc() );          // ue(v)
  pfps.setLocalOverrideGeometryPatchEnableFlag( bitstream.read( 1 ) );  // u(1)
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    pfps.setLocalOverrideAttributePatchEnableFlag( i, bitstream.read( 1 ) );  // u(1)
  }
  pfps.setAdditionalLtPfocLsbLen( bitstream.readUvlc() );  // ue(v)
  if ( sps.getPatchSequenceOrientationEnabledFlag() ) {
    pfps.setPatchOrientationPresentFlag( bitstream.read( 1 ) );  // u(1)
    TRACE_BITSTREAM( "code PatchOrientationPresentFlag = %lu \n",
                     pfps.getPatchOrientationPresentFlag() );
  } else {
    pfps.setPatchOrientationPresentFlag( true );
    TRACE_BITSTREAM( "else PatchOrientationPresentFlag = %lu \n",
                     pfps.getPatchOrientationPresentFlag() );
  }
  byteAlignment( bitstream );
}

// 7.3.26 Patch frame layer unit syntax
void PCCBitstreamDecoderNewSyntax::patchFrameLayerUnit( PatchFrameLayerUnit& pflu,
                                                        PCCContext&          context,
                                                        PCCBitstream&        bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& pfh  = pflu.getPatchFrameHeader();
  auto& pfdu = pflu.getPatchFrameDataUnit();
  patchFrameHeader( pfh, context, bitstream );
  patchFrameDataUnit( pfdu, pfh, context, bitstream );
}

// 7.3.27 Patch frame header syntax
void PCCBitstreamDecoderNewSyntax::patchFrameHeader( PatchFrameHeader& pfh,
                                                     PCCContext&       context,
                                                     PCCBitstream&     bitstream ) {
  TRACE_BITSTREAM( "%s start \n", __func__ );
  auto& psdu = context.getPatchSequenceDataUnit();
  auto& psps = psdu.getPatchSequenceParameterSet( pfh.getPatchFrameParameterSetId() );

  pfh.setPatchFrameParameterSetId( bitstream.readUvlc() );  // ue( v )
  pfh.setAddress( bitstream.readUvlc() );                   // ue( v )
  pfh.setType( bitstream.readUvlc() );                      // ue( v )
  pfh.setPatchFrameOrderCntLsb( bitstream.readUvlc() );     // ue( v )

  TRACE_BITSTREAM( "Id     = %u \n", pfh.getPatchFrameParameterSetId() );
  TRACE_BITSTREAM( "Adress = %u \n", pfh.getAddress() );
  TRACE_BITSTREAM( "Type   = %u \n", pfh.getType() );
  TRACE_BITSTREAM( "POC    = %u \n", pfh.getPatchFrameOrderCntLsb() );
  TRACE_BITSTREAM( "psps.getNumRefPatchFrameListsInSps() = %lu\n",
                   psps.getNumRefPatchFrameListsInSps() );
  TRACE_BITSTREAM( "psps.getNumRefPatchFrameListsInSps() = %lu \n",
                   psps.getNumRefPatchFrameListsInSps() );

  if ( psps.getNumRefPatchFrameListsInSps() > 0 ) {
    pfh.setRefPatchFrameListSpsFlag( bitstream.read( 1 ) );  // u( 1 )
    if ( pfh.getRefPatchFrameListSpsFlag() ) {
      if ( psps.getNumRefPatchFrameListsInSps() > 1 ) {
        pfh.setRefPatchFrameListIdx( bitstream.readUvlc() );  // u( v )
      } else {
        psps.getRefListStruct( psps.getNumRefPatchFrameListsInSps() );
      }
    }
    uint8_t rlsIdx = psps.getNumRefPatchFrameListsInSps() ? pfh.getRefPatchFrameListIdx()
                                                          : psps.getNumRefPatchFrameListsInSps();
    size_t numLtrpEntries = 0;
    for ( size_t i = 0; i < psps.getRefListStruct( rlsIdx ).getNumRefEntries(); i++ ) {
      if ( !psps.getRefListStruct( rlsIdx ).getStRefPatchFrameFlag( i ) ) { numLtrpEntries++; }
    }

    for ( size_t j = 0; j < numLtrpEntries; j++ ) {
      pfh.setAdditionalPfocLsbPresentFlag( j, bitstream.read( 1 ) );  // u( 1 )
      if ( pfh.getAdditionalPfocLsbPresentFlag( j ) ) {
        pfh.setAdditionalPfocLsbVal( j, bitstream.readUvlc() );
      }  // ue( v )
    }

    if ( pfh.getType() == P_PATCH_FRAME &&
         psps.getRefListStruct( rlsIdx ).getNumRefEntries() > 1 ) {
      pfh.setNumRefIdxActiveOverrideFlag( bitstream.read( 1 ) );  // u( 1 )
      if ( pfh.getNumRefIdxActiveOverrideFlag() ) {
        pfh.setNumRefIdxActiveMinus1( bitstream.readUvlc() );
      }  // ue( v )
    }
  }
  if ( pfh.getType() == I_PATCH_FRAME ) {
    pfh.setInterPredictPatch2dShiftUBitCountMinus1( bitstream.read( 8 ) );              // u( 8 )
    pfh.setInterPredictPatch2dShiftVBitCountMinus1( bitstream.read( 8 ) );              // u( 8 )
    pfh.setInterPredictPatch3dShiftTangentAxisBitCountMinus1( bitstream.read( 8 ) );    // u( 8 )
    pfh.setInterPredictPatch3dShiftBitangentAxisBitCountMinus1( bitstream.read( 8 ) );  // u( 8 )
    pfh.setInterPredictPatch3dShiftNormalAxisBitCountMinus1( bitstream.read( 8 ) );     // u( 8 )
    pfh.setInterPredictPatchLodBitCountFlag( bitstream.read( 8 ) );                     // u( 8 )
  } else {
    pfh.setInterPredictPatchBitCountFlag( bitstream.read( 1 ) );  // u( 1 )
    if ( pfh.getInterPredictPatchBitCountFlag() ) {
      pfh.setInterPredictPatch2dShiftUBitCountFlag( bitstream.read( 1 ) );  // u( 1 )
      if ( pfh.getInterPredictPatch2dShiftUBitCountFlag() ) {
        pfh.setInterPredictPatch2dShiftUBitCountMinus1( bitstream.read( 8 ) );
      }                                                                     // u( 8 )
      pfh.setInterPredictPatch2dShiftVBitCountFlag( bitstream.read( 1 ) );  // u( 1 )
      if ( pfh.getInterPredictPatch2dShiftVBitCountFlag() ) {
        pfh.setInterPredictPatch2dShiftVBitCountMinus1( bitstream.read( 8 ) );
      }                                                                               // u( 8 )
      pfh.setInterPredictPatch3dShiftTangentAxisBitCountFlag( bitstream.read( 1 ) );  // u( 1 )
      if ( pfh.getInterPredictPatch3dShiftTangentAxisBitCountFlag() ) {
        pfh.setInterPredictPatch3dShiftTangentAxisBitCountMinus1( bitstream.read( 8 ) );  // u( 8 )
      }
      pfh.setInterPredictPatch3dShiftBitangentAxisBitCountFlag( bitstream.read( 1 ) );  // u( 1 )
      if ( pfh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag() ) {
        pfh.setInterPredictPatch3dShiftBitangentAxisBitCountMinus1(
            bitstream.read( 8 ) );  // u( 8 )
      }
      pfh.setInterPredictPatch3dShiftNormalAxisBitCountFlag( bitstream.read( 1 ) );  // u( 1 )
      if ( pfh.getInterPredictPatch3dShiftNormalAxisBitCountFlag() ) {
        pfh.setInterPredictPatch3dShiftNormalAxisBitCountMinus1( bitstream.read( 8 ) );  // u( 8 )
      }
      pfh.setInterPredictPatchLodBitCountFlag( bitstream.read( 1 ) );  // u( 1 )
      if ( pfh.getInterPredictPatchLodBitCountFlag() ) {
        pfh.setInterPredictPatchLodBitCount( bitstream.read( 8 ) + 1 );
      }  // u( 8 )
    }
  }
  TRACE_BITSTREAM( "InterPredictPatchBitCountFlag                          = %d \n",
                   pfh.getInterPredictPatchBitCountFlag() );
  TRACE_BITSTREAM( "InterPredictPatch2dShiftUBitCountFlag                  = %d \n",
                   pfh.getInterPredictPatch2dShiftUBitCountFlag() );
  TRACE_BITSTREAM( "InterPredictPatch2dShiftVBitCountFlag                  = %d \n",
                   pfh.getInterPredictPatch2dShiftVBitCountFlag() );
  TRACE_BITSTREAM( "InterPredictPatch3dShiftTangentAxisBitCountFlag        = %d \n",
                   pfh.getInterPredictPatch3dShiftTangentAxisBitCountFlag() );
  TRACE_BITSTREAM( "InterPredictPatch3dShiftBitangentAxisBitCountFlag      = %d \n",
                   pfh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag() );
  TRACE_BITSTREAM( "InterPredictPatch3dShiftNormalAxisBitCountFlag         = %d \n",
                   pfh.getInterPredictPatch3dShiftNormalAxisBitCountFlag() );
  TRACE_BITSTREAM( "InterPredictPatchLodBitCountFlag                       = %d \n",
                   pfh.getInterPredictPatchLodBitCountFlag() );
  TRACE_BITSTREAM( "getInterPredictPatch2dShiftUBitCountMinus1             = %u \n",
                   pfh.getInterPredictPatch2dShiftUBitCountMinus1() );
  TRACE_BITSTREAM( "getInterPredictPatch2dShiftVBitCountMinus1             = %u \n",
                   pfh.getInterPredictPatch2dShiftVBitCountMinus1() );
  TRACE_BITSTREAM( "getInterPredictPatch3dShiftTangentAxisBitCountMinus1   = %u \n",
                   pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() );
  TRACE_BITSTREAM( "getInterPredictPatch3dShiftBitangentAxisBitCountMinus1 = %u \n",
                   pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() );
  TRACE_BITSTREAM( "getInterPredictPatch3dShiftNormalAxisBitCountMinus1    = %u \n",
                   pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1() );
  byteAlignment( bitstream );
  TRACE_BITSTREAM( "%s done \n", __func__ );
}

// 7.3.28 Reference list structure syntax
void PCCBitstreamDecoderNewSyntax::refListStruct( RefListStruct&             rls,
                                                  PatchSequenceParameterSet& psps,
                                                  PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  rls.setNumRefEntries( bitstream.readUvlc() );  // ue(v)
  rls.allocate();
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( psps.getLongTermRefPatchFramesFlag() ) {
      rls.setStRefPatchFrameFlag( i, bitstream.read( 1 ) );  // u(1)
      if ( rls.getStRefPatchFrameFlag( i ) ) {
        rls.setAbsDeltaPfocSt( i, bitstream.readUvlc() );  // ue(v)
        if ( rls.getAbsDeltaPfocSt( i ) > 0 ) {
          rls.setStrpfEntrySignFlag( i, bitstream.read( 1 ) );  // u(1)
        } else {
          rls.setPfocLsbLt( i, bitstream.readUvlc() );  // u(v)
        }
      }
    }
  }
}

// 7.3.29 Patch frame data unit syntax
void PCCBitstreamDecoderNewSyntax::patchFrameDataUnit( PatchFrameDataUnit& pfdu,
                                                       PatchFrameHeader&   pfh,
                                                       PCCContext&         context,
                                                       PCCBitstream&       bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "patchFrameDataUnit start \n" );
  TRACE_BITSTREAM( "pfh.getType()        = %lu \n", pfh.getType() );
  uint8_t                 puCount = -1;
  o3dgc::Arithmetic_Codec arithmeticDecoder;
  uint32_t                compressedBitstreamSize;
  compressedBitstreamSize = bitstream.read<uint32_t>();
  TRACE_BITSTREAM_NH( "start arithmetic coder \n" );
  assert( compressedBitstreamSize + bitstream.size() <= bitstream.capacity() );
  arithmeticDecoder.set_buffer( uint32_t( bitstream.capacity() - bitstream.size() ),
                                bitstream.buffer() + bitstream.size() );
  arithmeticDecoder.start_decoder();
  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelMoreAvailablePatchFlag;
  bool moreAvailablePatchFlag = arithmeticDecoder.decode( bModelMoreAvailablePatchFlag );  // ae(v)
  TRACE_BITSTREAM_NH( "moreAvailablePatchFlag = %d \n", moreAvailablePatchFlag );
  pfdu.init();
  const uint8_t bitCountPatchMode = ( PATCH_FRAME_TYPE( pfh.getType() ) ) == I_PATCH_FRAME ? 1 : 2;
  TRACE_BITSTREAM_NH( "bitCountPatchMode = %u \n", bitCountPatchMode );
  while ( moreAvailablePatchFlag ) {
    uint8_t patchMode = DecodeUInt32( bitCountPatchMode, arithmeticDecoder, bModel,
                                      bitstream );  // ae(v) need to write the model
    TRACE_BITSTREAM_NH( "patchMode = %lu \n", patchMode );
    pfdu.addPatchMode( patchMode );
    auto& pid = pfdu.addPatchInformationData();
    patchInformationData( pid, patchMode, pfh, context, bitstream, arithmeticDecoder );
    moreAvailablePatchFlag = arithmeticDecoder.decode( bModelMoreAvailablePatchFlag );  // ae(v)
    TRACE_BITSTREAM_NH( "moreAvailablePatchFlag = %d \n", moreAvailablePatchFlag );
  }
  auto& sps = context.getSps();
  if ( sps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plr = pfdu.getPointLocalReconstruction();
    pointLocalReconstruction( plr, context, bitstream, arithmeticDecoder );
  }
  arithmeticDecoder.stop_decoder();
  bitstream += (uint64_t)compressedBitstreamSize;
  byteAlignment( bitstream );
}

// 7.3.30 Patch information data syntax
void PCCBitstreamDecoderNewSyntax::patchInformationData(
    PatchInformationData&    pid,
    size_t                   patchMode,
    PatchFrameHeader&        pfh,
    PCCContext&              context,
    PCCBitstream&            bitstream,
    o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
  TRACE_BITSTREAM_NH( "%s start: FrameType = %lu PatchMode = %lu \n", __func__, pfh.getType(),
                      patchMode );

  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelOverrideGeometryPatchFlag;
  o3dgc::Adaptive_Bit_Model bModelOverrideAttributePatchFlag;
  auto&                     sps  = context.getSps();
  auto&                     psdu = context.getPatchSequenceDataUnit();
  auto&         pfps = psdu.getPatchFrameParameterSet( pfh.getPatchFrameParameterSetId() );
  const uint8_t bitCountGAppsId = 6;
  if ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == P_PATCH_FRAME && patchMode == P_SKIP ) {
    // skip mode: currently not supported but added it for convenience. Could easily be removed
  } else if ( ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == I_PATCH_FRAME && patchMode == I_INTRA ) ||
              ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == P_PATCH_FRAME && patchMode == P_INTRA ) ) {
    if ( pfps.getLocalOverrideGeometryPatchEnableFlag() ) {
      const bool overrideGeometryPatchFlag =
          arithmeticDecoder.decode( bModelOverrideGeometryPatchFlag );  // ae(v)
      pid.setOverrideGeometryPatchFlag( overrideGeometryPatchFlag );
      if ( pid.getOverrideGeometryPatchFlag() ) {
        uint8_t gppsId = DecodeUInt32( bitCountGAppsId, arithmeticDecoder, bModel );
        pid.setGeometryPatchParameterSetId( gppsId );

        TRACE_BITSTREAM_NH( " gppsId = %lu \n", pid.getGeometryPatchParameterSetId() );
      }
    }
    TRACE_BITSTREAM_NH( " sps.getAttributeCount() = %lu \n", sps.getAttributeCount() );
    pfps.allocatePatchFrame( sps.getAttributeCount() );
    TRACE_BITSTREAM_NH( " allocate done %lu \n", sps.getAttributeCount() );
    for ( int i = 0; i < sps.getAttributeCount(); i++ ) {
      TRACE_BITSTREAM_NH( " overight flag = %lu \n",
                          pfps.getLocalOverrideAttributePatchEnableFlag( i ) );
      if ( pfps.getLocalOverrideAttributePatchEnableFlag( i ) ) {
        const bool overrideAttributePatchFlag =
            arithmeticDecoder.decode( bModelOverrideAttributePatchFlag );
        TRACE_BITSTREAM_NH( " overrideAttributePatchFlag = %lu \n", overrideAttributePatchFlag );
        pid.addOverrideAttributePatchFlag( overrideAttributePatchFlag );
      } else {
        pid.addOverrideAttributePatchFlag( false );
      }
      TRACE_BITSTREAM_NH( " overight patch flag = %lu \n", pid.getOverrideAttributePatchFlag( i ) );
      if ( pid.getOverrideAttributePatchFlag( i ) ) {
        uint8_t appsId = DecodeUInt32( bitCountGAppsId, arithmeticDecoder, bModel );  // ae(v)
        pid.addAttributePatchParameterSetId( appsId );
        TRACE_BITSTREAM_NH( " AttributePatchParameterSetId = %lu \n",
                            pid.getAttributePatchParameterSetId( i ) );
      } else {
        pid.addAttributePatchParameterSetId( 0 );
      }
    }
    auto& pdu = pid.getPatchDataUnit();
    patchDataUnit( pdu, pfh, context, bitstream, arithmeticDecoder );
  } else if ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == P_PATCH_FRAME && patchMode == P_INTER ) {
    auto& dpdu = pid.getDeltaPatchDataUnit();
    deltaPatchDataUnit( dpdu, pfh, context, bitstream, arithmeticDecoder );
  } else if ( ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == I_PATCH_FRAME && patchMode == I_PCM ) ||
              ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == P_PATCH_FRAME && patchMode == P_PCM ) ) {
    auto& ppdu = pid.getPCMPatchDataUnit();
    pcmPatchDataUnit( ppdu, pfh, context, bitstream, arithmeticDecoder );
  }

  TRACE_BITSTREAM_NH( "%s done \n", __func__ );
}

// 7.3.31 Patch data unit syntax
void PCCBitstreamDecoderNewSyntax::patchDataUnit( PatchDataUnit&           pdu,
                                                  PatchFrameHeader&        pfh,
                                                  PCCContext&              context,
                                                  PCCBitstream&            bitstream,
                                                  o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
  TRACE_BITSTREAM_NH( "%s start \n", __func__ );

  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelOrientationSwapFlag,
      bModelProjectionFlag;

  size_t val;

  TRACE_BITSTREAM_NH( "BitCount = %lu %lu  \n", pfh.getInterPredictPatch2dShiftUBitCountMinus1(),
                      pfh.getInterPredictPatch2dShiftVBitCountMinus1() );

  pdu.set2DShiftU( DecodeUInt32( pfh.getInterPredictPatch2dShiftUBitCountMinus1() + 1,
                                 arithmeticDecoder, bModel ) );
  pdu.set2DShiftV( DecodeUInt32( pfh.getInterPredictPatch2dShiftVBitCountMinus1() + 1,
                                 arithmeticDecoder, bModel ) );

  const int64_t deltaSizeU0 =
      o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelSizeU0 ) );
  pdu.set2DDeltaSizeU( deltaSizeU0 );
  const int64_t deltaSizeV0 =
      o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelSizeV0 ) );
  pdu.set2DDeltaSizeV( deltaSizeV0 );

  TRACE_BITSTREAM_NH( "UV = %lu %lu Delta size = %ld %ld \n", pdu.get2DShiftU(), pdu.get2DShiftV(),
                      pdu.get2DDeltaSizeU(), pdu.get2DDeltaSizeU() );

  pdu.set3DShiftTangentAxis( DecodeUInt32(
      pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() + 1, arithmeticDecoder, bModel ) );
  pdu.set3DShiftBiTangentAxis(
      DecodeUInt32( pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() + 1,
                    arithmeticDecoder, bModel ) );
  pdu.set3DShiftNormalAxis( DecodeUInt32(
      pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1() + 1, arithmeticDecoder, bModel ) );

  TRACE_BITSTREAM_NH( "Axis = %lu %lu  %lu \n", pdu.get3DShiftTangentAxis(),
                      pdu.get3DShiftBiTangentAxis(), pdu.get3DShiftNormalAxis() );

  o3dgc::Adaptive_Data_Model orientationModel( 4 );
  pcc::PCCAxis3              normalAxis;
  normalAxis = pcc::PCCAxis3( arithmeticDecoder.decode( orientationModel ) );
  pdu.setNormalAxis( normalAxis );

  auto& pfps = context.getPatchSequenceDataUnit().getPatchFrameParameterSet( 0 );
  if ( pfps.getPatchOrientationPresentFlag() ) {
    pdu.setOrientationSwapFlag( arithmeticDecoder.decode( bModel ) );  // v(1)
  }

  TRACE_BITSTREAM_NH( "getInterPredictPatchLodBitCount = %u \n",
                      pfh.getInterPredictPatchLodBitCount() );

  if ( pfh.getInterPredictPatchLodBitCount() > 0 ) {
    uint8_t lod = DecodeUInt32( pfh.getInterPredictPatchLodBitCount() + 1, arithmeticDecoder,
                                bModel );  // ae(v)
    pdu.setLod( lod );

    TRACE_BITSTREAM_NH( "PatchLod = %lu \n", pdu.getLod() );
  }
  bool  projectionFlag = 0;
  int   i              = 0;
  auto& sps            = context.getSps();
  while ( i < sps.getLayerCountMinus1() + 1 && projectionFlag == 0 ) {
    TRACE_BITSTREAM_NH( " AbsoluteCoding Layer %d =%d \n", i,
                        sps.getLayerAbsoluteCodingEnabledFlag( i ) );

    projectionFlag = projectionFlag | sps.getLayerAbsoluteCodingEnabledFlag( i );
    i++;
  }

  TRACE_BITSTREAM_NH( "projectionFlag =%d \n", projectionFlag );

  if ( projectionFlag ) {
    pdu.setProjectionMode( arithmeticDecoder.decode( bModelProjectionFlag ) );  // ae(v)
    TRACE_BITSTREAM_NH( "projectionMode =%d \n", pdu.getProjectionMode() );
  }

  TRACE_BITSTREAM_NH( "patch UV %4lu %4lu S=%4ld %4ld P=%lu O=%d A=%lu %lu %lu \n",
                      pdu.get2DShiftU(), pdu.get2DShiftV(), pdu.get2DDeltaSizeU(),
                      pdu.get2DDeltaSizeV(), pdu.getProjectionMode(), pdu.getOrientationSwapFlag(),
                      pdu.get3DShiftTangentAxis(), pdu.get3DShiftBiTangentAxis(),
                      pdu.get3DShiftNormalAxis() );

  TRACE_BITSTREAM_NH( "%s done \n", __func__ );
}

// 7.3.32  Delta Patch data unit syntax
void PCCBitstreamDecoderNewSyntax::deltaPatchDataUnit(
    DeltaPatchDataUnit&      dpdu,
    PatchFrameHeader&        pfh,
    PCCContext&              context,
    PCCBitstream&            bitstream,
    o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
  TRACE_BITSTREAM_NH( "%s \n", __func__ );
  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelPatchIdx, bModelPatch2DShiftU, bModelPatch2DShiftV,
      bModelPatchSizeU, bModelPatchSizeV;
  o3dgc::Adaptive_Bit_Model bModelPatch3DShiftNorm, bModelPatch3DShiftTan, bModelPatch3DShiftBitan;
  o3dgc::Adaptive_Bit_Model bModelProjectionFlag;

  dpdu.setDeltaPatchIdx( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchIdx ) ) );  // ae(v)
  dpdu.set2DDeltaShiftU( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch2DShiftU ) ) );  // ae(v)
  dpdu.set2DDeltaShiftV( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch2DShiftV ) ) );  // ae(v)
  dpdu.set2DDeltaSizeU( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchSizeU ) ) );  // ae(v)
  dpdu.set2DDeltaSizeV( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchSizeV ) ) );  // ae(v)
  dpdu.set3DDeltaShiftTangentAxis( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch3DShiftTan ) ) );  // ae(v)
  dpdu.set3DDeltaShiftBiTangentAxis( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch3DShiftBitan ) ) );  // ae(v)
  dpdu.set3DDeltaShiftNormalAxis( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch3DShiftNorm ) ) );  // ae(v)

  bool  projectionFlag = 0;
  int   i              = 0;
  auto& sps            = context.getSps();

  while ( i < sps.getLayerCountMinus1() + 1 && projectionFlag == 0 ) {
    projectionFlag = projectionFlag | sps.getLayerAbsoluteCodingEnabledFlag( i );
    i++;
  }
  if ( projectionFlag ) {
    dpdu.setProjectionMode( arithmeticDecoder.decode( bModelProjectionFlag ) );  // ae(v)
  }
  TRACE_BITSTREAM_NH(
      "DeltaIdx = %u ShiftUV = %ld %ld DeltaSize = %ld %ld Axis = %ld %ld %ld Proj = %d Or = %d \n",
      dpdu.getDeltaPatchIdx(), dpdu.get2DDeltaShiftU(), dpdu.get2DDeltaShiftV(),
      dpdu.get2DDeltaSizeU(), dpdu.get2DDeltaSizeV(), dpdu.get3DDeltaShiftTangentAxis(),
      dpdu.get3DDeltaShiftBiTangentAxis(), dpdu.get3DDeltaShiftNormalAxis(),
      dpdu.getProjectionMode(), dpdu.getOrientationSwapFlag() );
}

// 7.3.33 PCM patch data unit syntax
void PCCBitstreamDecoderNewSyntax::pcmPatchDataUnit( PCMPatchDataUnit&        ppdu,
                                                     PatchFrameHeader&        pfh,
                                                     PCCContext&              context,
                                                     PCCBitstream&            bitstream,
                                                     o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
  TRACE_BITSTREAM_NH( "%s \n", __func__ );
  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelVideoPCMFlag;
  o3dgc::Adaptive_Bit_Model bModelPatchSizeU, bModelPatchSizeV, bModelPcmPoints;
  auto&                     sps = context.getSps();
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    ppdu.setPatchInPcmVideoFlag( arithmeticDecoder.decode( bModelVideoPCMFlag ) );  // ae(v)
  }
  ppdu.set2DShiftU( DecodeUInt32( pfh.getInterPredictPatch2dShiftUBitCountMinus1() + 1,
                                  arithmeticDecoder, bModel ) );  // ae(v)
  ppdu.set2DShiftV( DecodeUInt32( pfh.getInterPredictPatch2dShiftVBitCountMinus1() + 1,
                                  arithmeticDecoder, bModel ) );  // ae(v)
  ppdu.set2DDeltaSizeU( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchSizeU ) ) );  // ae(v)
  ppdu.set2DDeltaSizeV( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchSizeV ) ) );  // ae(v)
  ppdu.setPcmPoints( o3dgc::UIntToInt(
      arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPcmPoints ) ) );  // ae(v)

  TRACE_BITSTREAM_NH( "UBitCount       = %lu \n",
                      pfh.getInterPredictPatch2dShiftUBitCountMinus1() );
  TRACE_BITSTREAM_NH( "VBitCount       = %lu \n",
                      pfh.getInterPredictPatch2dShiftVBitCountMinus1() );
  TRACE_BITSTREAM_NH( "2DShiftU        = %lu \n", ppdu.get2DShiftU() );
  TRACE_BITSTREAM_NH( "2DShiftV        = %lu \n", ppdu.get2DShiftV() );
  TRACE_BITSTREAM_NH( "2DDeltaSizeU    = %ld \n", ppdu.get2DDeltaSizeU() );
  TRACE_BITSTREAM_NH( "2DDeltaSizeV    = %ld \n", ppdu.get2DDeltaSizeV() );
  TRACE_BITSTREAM_NH( "PcmPoints       = %u  \n", ppdu.getPcmPoints() );
  TRACE_BITSTREAM_NH( "PatchInPcmVideo = %u  \n", ppdu.getPatchInPcmVideoFlag() );
}

// 7.3.34 Point local reconstruction syntax
void PCCBitstreamDecoderNewSyntax::pointLocalReconstruction(
    PointLocalReconstruction& plr,
    PCCContext&               context,
    PCCBitstream&             bitstream,
    o3dgc::Arithmetic_Codec&  arithmeticDecoder ) {
  TRACE_BITSTREAM_NH( "%s \n", __func__ );
  o3dgc::Adaptive_Bit_Model bModelInterpolateFlag, bModelNeighbour, bModelMinDepth,
      bModelFillingFlag;
  for ( uint64_t j = 0; j < plr.getBlockToPatchMapHeight(); j++ ) {
    for ( uint64_t i = 0; i < plr.getBlockToPatchMapWidth(); i++ ) {
      if ( plr.getBlockToPatchMap( i, j ) >= 0 ) {
        plr.setModeInterpolateFlag( i, j,
                                    arithmeticDecoder.decode( bModelInterpolateFlag ) );  // ae(v)
        if ( plr.getModeInterpolateFlag( i, j ) ) {
          plr.setModeNeighbourMinus1( i, j, arithmeticDecoder.decode( bModelNeighbour ) );  // ae(v)
        }
        plr.setModeMinimumDepthMinus1( i, j, arithmeticDecoder.decode( bModelMinDepth ) );  // ae(v)
        if ( ( plr.getModeMinimumDepthMinus1( i, j ) > 0 ) ||
             ( plr.getModeInterpolateFlag( i, j ) ) ) {
          plr.setModeFillingFlag( i, j, arithmeticDecoder.decode( bModelFillingFlag ) );  // ae(v)
        }
      }
    }
  }
}
