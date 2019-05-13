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
#include "PCCBitstream.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"
#include "PCCGroupOfFrames.h"
#include "PCCEncoderParameters.h"
#include <tbb/tbb.h>
#include "PCCChrono.h"

#include "PCCBitstreamEncoder.h"

using namespace std;
using namespace pcc;

PCCBitstreamEncoder::PCCBitstreamEncoder() {}
PCCBitstreamEncoder::~PCCBitstreamEncoder() {}

void PCCBitstreamEncoder::setParameters( PCCEncoderParameters params ) { params_ = params; }

int PCCBitstreamEncoder::encode( PCCContext& context, PCCBitstream& bitstream ) {
  bitstream.getBitStreamStat().newGOF();
  vpccUnit( context, bitstream, VPCC_SPS );
  vpccUnit( context, bitstream, VPCC_PSD );
  vpccUnit( context, bitstream, VPCC_OVD );
  vpccUnit( context, bitstream, VPCC_GVD );
  vpccUnit( context, bitstream, VPCC_AVD );
  std::cout << " occupancy map  ->" << bitstream.getBitStreamStat().getTotalMetadata() << " B " << std::endl;
  return 0;
}

void PCCBitstreamEncoder::vpccVideoDataUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sps = context.getSps();
  if ( vpccUnitType == VPCC_OVD ) {
    TRACE_BITSTREAM( "OccupancyMap \n" );
    bitstream.write( context.getVideoBitstream( VIDEO_OCCUPANCY ) );
  } else if ( vpccUnitType == VPCC_GVD ) {
    TRACE_BITSTREAM( "Geometry \n" );
    if ( !sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_D0 ) );
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_D1 ) );
    } else {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY ) );
    }
    if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_MP ) );
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( sps.getAttributeCount() > 0 ) {
      TRACE_BITSTREAM( "Texture \n" );
      bitstream.write( context.getVideoBitstream( VIDEO_TEXTURE ) );
      if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
        bitstream.write( context.getVideoBitstream( VIDEO_TEXTURE_MP ) );
      }
    }
  }
}

// 7.3.2.1 General V-PCC unit syntax
void PCCBitstreamEncoder::vpccUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  int32_t position = bitstream.size();
  vpccUnitHeader( context, bitstream, vpccUnitType );
  vpccUnitPayload( context, bitstream, vpccUnitType );
  bitstream.getBitStreamStat().setVpccUnitSize( vpccUnitType, (int32_t)bitstream.size() - position );
}

// 7.3.2.2 V-PCC unit header syntax TODO: write attributeDimensionIndex_, modify pcmSeparateVideoData parameters for VPCC_AVD case
void PCCBitstreamEncoder::vpccUnitHeader( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  bitstream.write( vpccUnitType, 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_PSD ) {
    bitstream.write( (uint32_t)vpcc.getSequenceParameterSetId(), 4 );  // u(4)
  }
  if ( vpccUnitType == VPCC_AVD ) {
    bitstream.write( (uint32_t)vpcc.getAttributeIndex(), 7 );  // u(7)
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      bitstream.write( (uint32_t)vpcc.getLayerIndex(), 4 );  // u(4)
      pcmSeparateVideoData( context, bitstream, 11 );
    } else {
      pcmSeparateVideoData( context, bitstream, 15 );
    }
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      bitstream.write( vpcc.getLayerIndex(), 4 );  // u(4)
      pcmSeparateVideoData( context, bitstream, 18 );
    } else {
      pcmSeparateVideoData( context, bitstream, 22 );
    }
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_PSD ) {
    bitstream.write( (uint32_t)0, 23 );  // u(23)
  } else {
    bitstream.write( (uint32_t)0, 27 );  // u(27)
  }
}

// 7.3.2.3 PCM separate video data syntax
void PCCBitstreamEncoder::pcmSeparateVideoData( PCCContext& context, PCCBitstream& bitstream, uint8_t bitCount ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  if ( sps.getPcmSeparateVideoPresentFlag() && !vpcc.getLayerIndex() ) {
    bitstream.write( (uint32_t)vpcc.getPCMVideoFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)0, bitCount );                // u(bitCount)
  } else {
    bitstream.write( (uint32_t)0, bitCount + 1 );  // u(bitCount + 1)
  }
}

// 7.3.2.4 V-PCC unit payload syntax
void PCCBitstreamEncoder::vpccUnitPayload( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sps = context.getSps();
  TRACE_BITSTREAM( "vpccUnitType = %d \n", (int32_t)vpccUnitType );
  if ( vpccUnitType == VPCC_SPS ) {
    sequenceParameterSet( sps, context, bitstream );
  } else if ( vpccUnitType == VPCC_PSD ) {
    patchSequenceDataUnit( context, bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    vpccVideoDataUnit( context, bitstream, vpccUnitType );
  }
}

// 7.3.4.1 General Sequence parameter set syntax TODO: remove sps.setAttributeCount() and loop over attributes
void PCCBitstreamEncoder::sequenceParameterSet( SequenceParameterSet& sps, PCCContext& context, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  bitstream.write( (uint32_t)sps.getSequenceParameterSetId(), 4 );   // u(4)
  bitstream.write( (uint32_t)sps.getFrameWidth(), 16 );              // u(16)
  bitstream.write( (uint32_t)sps.getFrameHeight(), 16 );             // u(16)
  bitstream.write( (uint32_t)sps.getAvgFrameRatePresentFlag(), 1 );  // u(1)
  if ( sps.getAvgFrameRatePresentFlag() ) {
    bitstream.write( (uint32_t)sps.getAvgFrameRate(), 16 );  // u(16)
  }
  bitstream.write( (uint32_t)sps.getEnhancedOccupancyMapForDepthFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)sps.getLayerCountMinus1(), 4 );                  // u(4)
  if ( sps.getLayerCountMinus1() > 0 ) {
    bitstream.write( (uint32_t)sps.getMultipleLayerStreamsPresentFlag(), 1 );  // u(1)
  }
  for ( size_t i = 0; i < sps.getLayerCountMinus1(); i++ ) {
    bitstream.write( (uint32_t)sps.getLayerAbsoluteCodingEnabledFlag( i + 1 ), 1 );  // u(1)
    if ( ( sps.getLayerAbsoluteCodingEnabledFlag( i + 1 ) == 0 ) ) {
      if ( i > 0 ) {
        bitstream.writeUvlc( (uint32_t)sps.getLayerPredictorIndexDiff( i + 1 ) );  // ue(v)
      }
    }
  }
  TRACE_BITSTREAM( " LayerCountMinus1  = %lu \n", sps.getLayerCountMinus1() );
  TRACE_BITSTREAM( " AbsoluteCoding L0 = %lu \n", sps.getLayerAbsoluteCodingEnabledFlag( 0 ) );
  TRACE_BITSTREAM( " AbsoluteCoding L1 = %lu \n", sps.getLayerAbsoluteCodingEnabledFlag( 1 ) );
  bitstream.write( (uint32_t)sps.getPcmPatchEnabledFlag(), 1 );  // u(1)
  if ( sps.getPcmPatchEnabledFlag() ) {
    bitstream.write( (uint32_t)sps.getPcmSeparateVideoPresentFlag(), 1 );  // u(1)
  }
  occupancyParameterSet( sps.getOccupancyParameterSet(), bitstream );
  geometryParameterSet( sps.getGeometryParameterSet(), sps, bitstream );
  bitstream.write( (uint32_t)sps.getAttributeCount(), 16 );  // u(16)
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    attributeParameterSet( sps.getAttributeParameterSet( i ), sps, bitstream );
  }
  bitstream.write( (uint32_t)sps.getPatchInterPredictionEnabledFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)sps.getPixelDeinterleavingFlag(), 1 );              // u(1)
  bitstream.write( (uint32_t)sps.getPointLocalReconstructionEnabledFlag(), 1 );  // u(1)
  if ( sps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstruction( sps.getPointLocalReconstruction(), context, bitstream );
  }
  bitstream.write( (uint32_t)sps.getRemoveDuplicatePointEnabledFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)sps.getProjection45DegreeEnableFlag(), 1 );         // u(1)

  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
  bitstream.write( (uint32_t)sps.getLosslessGeo444(), 1 );    // u(1)
  bitstream.write( (uint32_t)sps.getLosslessGeo(), 1 );       // u(1)
  bitstream.write( (uint32_t)sps.getLosslessTexture(), 1 );   // u(1)
  bitstream.write( (uint32_t)sps.getMinLevel(), 8 );          // u(8)
  bitstream.write( (uint32_t)sps.getSurfaceThickness(), 8 );  // u(8)
  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE

  byteAlignment( bitstream );
}

// 7.3.4.2 Byte alignment syntax
void PCCBitstreamEncoder::byteAlignment( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( 1, 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.write( 0, 1 );  // f(1): equal to 0
  }
}

// 7.3.4.2 Profile, tier, and level syntax TODO: change profileIdc to profileCodecGroupIdc, missing profilePCCToolsetIdc, profileReconstructionIdc
void PCCBitstreamEncoder::profileTierLevel( ProfileTierLevel& ptl, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)ptl.getTierFlag(), 1 );    // u(1)
  bitstream.write( (uint32_t)ptl.getProfileIdc(), 1 );  // u(7)
  bitstream.write( (uint32_t)0, 48 );                   // u(48)
  bitstream.write( (uint32_t)ptl.getLevelIdc(), 8 );    // u(8)
}

// 7.3.4.3 Occupancy parameter set syntax TODO: remove ops.getOccupancyPackingBlockSize
void PCCBitstreamEncoder::occupancyParameterSet( OccupancyParameterSet& ops, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)ops.getOccupancyCodecId(), 8 );           // u(8)
  bitstream.write( (uint32_t)ops.getOccupancyLossyThreshold(), 8);     // u(8)
  bitstream.write( (uint32_t)ops.getOccupancyPackingBlockSize(), 8 );  // u(8)
  TRACE_BITSTREAM( "  OccupancyLossyThreshold = %d  \n", ops.getOccupancyLossyThreshold() );
}

// 7.3.4.4 Geometry parameter set syntax TODO: remove gps.getGeometryPatchScaleParamsEnabledFlag, gps.getGeometryPatchOffsetParamsEnabledFlag, gps.getGeometryPatchRotationParamsEnabledFlag, gps.getGeometryPatchPointSizeInfoEnabledFlag, gps.getGeometryPatchPointShapeInfoEnabledFlag
void PCCBitstreamEncoder::geometryParameterSet( GeometryParameterSet& gps,
                                                SequenceParameterSet& sps,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)gps.getGeometryCodecId(), 8 );
  bitstream.write( (uint32_t)gps.getGeometryNominal2dBitdepthMinus1(), 5 );            // u(5)
  bitstream.write( ( uint32_t )( gps.getGeometry3dCoordinatesBitdepthMinus1() ), 5 );  // u(5)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    bitstream.write( (uint32_t)gps.getPcmGeometryCodecId(), 1 );  // u(8)
  }
  bitstream.write( (uint32_t)gps.getGeometryParamsEnabledFlag(), 1 );  // u(1)
  if ( gps.getGeometryParamsEnabledFlag() ) { geometrySequenceParams( gps.getGeometrySequenceParams(), bitstream ); }
  bitstream.write( (uint32_t)gps.getGeometryPatchParamsEnabledFlag(), 1 );  // u(1)
  if ( gps.getGeometryPatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gps.getGeometryPatchScaleParamsEnabledFlag(), 1 );     // u(1)
    bitstream.write( (uint32_t)gps.getGeometryPatchOffsetParamsEnabledFlag(), 1 );    // u(1)
    bitstream.write( (uint32_t)gps.getGeometryPatchRotationParamsEnabledFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)gps.getGeometryPatchPointSizeInfoEnabledFlag(), 1 );   // u(1)
    bitstream.write( (uint32_t)gps.getGeometryPatchPointShapeInfoEnabledFlag(), 1 );  // u(1)
  }
}

// OLD 7.3.11 Geometry sequence metadata syntax TODO: remove 
void PCCBitstreamEncoder::geometrySequenceParams( GeometrySequenceParams& gsp, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)gsp.getGeometrySmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryOffsetParamsPresentFlag(), 1 );     // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryRotationParamsPresentFlag(), 1 );   // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryPointSizeInfoPresentFlag(), 1 );    // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryPointShapeInfoPresentFlag(), 1 );   // u(1)
  if ( gsp.getGeometrySmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)gsp.getGeometrySmoothingEnabledFlag(), 1 );  // u(8)
    if ( gsp.getGeometrySmoothingEnabledFlag() ) {
      bitstream.write( (uint32_t)gsp.getGeometrySmoothingGridSize(), 8 );   // u(8)
      bitstream.write( (uint32_t)gsp.getGeometrySmoothingThreshold(), 8 );  // u(8)
    }
  }
  if ( gsp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.write( (int32_t)gsp.getGeometryScaleOnAxis( d ), 32 );  // u(32)
    }
    if ( gsp.getGeometryOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.writeS( gsp.getGeometryOffsetOnAxis( d ), 32 );  // i(32)
      }
    }
    if ( gsp.getGeometryRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.writeS( gsp.getGeometryRotationOnAxis( d ), 32 );  // i(32)
      }
    }
    if ( gsp.getGeometryPointSizeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gsp.getGeometryPointSizeInfo(), 8 );  // u(8)
    }
    if ( gsp.getGeometryPointShapeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gsp.getGeometryPointShapeInfo(), 8 );  // u(8)
    }
  }
  TRACE_BITSTREAM( "  GeometrySmoothingParamsPresentFlag = %d  \n", gsp.getGeometrySmoothingParamsPresentFlag() );
  TRACE_BITSTREAM( "  GeometryScaleParamsPresentFlag     = %d  \n", gsp.getGeometryScaleParamsPresentFlag() );
  TRACE_BITSTREAM( "  GeometryOffsetParamsPresentFlag    = %d  \n", gsp.getGeometryOffsetParamsPresentFlag() );
  TRACE_BITSTREAM( "  GeometryRotationParamsPresentFlag  = %d  \n", gsp.getGeometryRotationParamsPresentFlag() );
  TRACE_BITSTREAM( "  GeometryPointSizeInfoPresentFlag   = %d  \n", gsp.getGeometryPointSizeInfoPresentFlag() );
  TRACE_BITSTREAM( "  GeometryPointShapeInfoPresentFlag  = %d  \n", gsp.getGeometryPointShapeInfoPresentFlag() );
  TRACE_BITSTREAM( "  getGeometrySmoothingEnabledFlag    = %d  \n", gsp.getGeometrySmoothingEnabledFlag() );
  TRACE_BITSTREAM( "  getGeometrySmoothingGridSize       = %u  \n", gsp.getGeometrySmoothingGridSize() );
  TRACE_BITSTREAM( "  getGeometrySmoothingThreshold      = %u  \n", gsp.getGeometrySmoothingThreshold() );
}

// 7.3.4.5 Attribute parameter set syntax TODO: add aps.getAttributeCount, and loop to receive the parameters per attribute, add condition to aps.getAttributeParamsEnabledFlag and setAttributePatchParamsEnabledFlag, remove attributeSequenceParams(),  aps.getAttributePatchScaleParamsEnabledFlag, aps.getAttributePatchOffsetParamsEnabledFlag, add channel partitions
void PCCBitstreamEncoder::attributeParameterSet( AttributeParameterSet& aps,
                                                 SequenceParameterSet&  sps,
                                                 PCCBitstream&          bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)aps.getAttributeTypeId(), 4 );           // u(4)
  bitstream.write( (uint32_t)aps.getAttributeDimensionMinus1(), 8 );  // u(8)
  bitstream.write( (uint32_t)aps.getAttributeCodecId(), 8 );          // u(8)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    bitstream.write( (uint32_t)aps.getPcmAttributeCodecId(), 8 );  // u(8)
  }
  bitstream.write( (uint32_t)aps.getAttributeParamsEnabledFlag(), 1 );  // u(1)
  if ( aps.getAttributeParamsEnabledFlag() ) {
    attributeSequenceParams( aps.getAttributeSequenceParams(), aps.getAttributeDimensionMinus1(), bitstream );
  }
  bitstream.write( (uint32_t)aps.getAttributeParamsEnabledFlag(), 1 );  // u(1)
  if ( aps.getAttributeParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)aps.getAttributePatchScaleParamsEnabledFlag(), 1 );   // u(1)
    bitstream.write( (uint32_t)aps.getAttributePatchOffsetParamsEnabledFlag(), 1 );  // u(1)
  }
}

// OLD 7.3.13 Attribute sequence Params syntax TODO: remove
void PCCBitstreamEncoder::attributeSequenceParams( AttributeSequenceParams& asp,
                                                   uint8_t                  dimension,
                                                   PCCBitstream&            bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)asp.getAttributeSmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)asp.getAttributeScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)asp.getAttributeOffsetParamsPresentFlag(), 1 );     // u(1)
  if ( asp.getAttributeSmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)asp.getAttributeSmoothingRadius(), 8 );                    // u(8)
    bitstream.write( (uint32_t)asp.getAttributeSmoothingNeighbourCount(), 8 );            // u(8)
    bitstream.write( (uint32_t)asp.getAttributeSmoothingRadius2BoundaryDetection(), 8 );  // u(8)
    bitstream.write( (uint32_t)asp.getAttributeSmoothingThreshold(), 8 );                 // u(8)
    bitstream.write( (uint32_t)asp.getAttributeSmoothingThresholdLocalEntropy(), 3 );     // u(3)
  }
  if ( asp.getAttributeScaleParamsPresentFlag() ) {
    for ( size_t i = 0; i < dimension; i++ ) {
      bitstream.write( (uint32_t)asp.getAttributeScale( i ), 32 );  // u(32)
    }
  }
  if ( asp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < dimension; i++ ) {
      bitstream.writeS( asp.getAttributeOffset( i ), 32 );  // i(32)
    }
  }
}

// 7.3.5.1 General patch data group unit syntax TODO: rename(?)
void PCCBitstreamEncoder::patchSequenceDataUnit( PCCContext& context, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t prevPFLUindex = 0;
  auto&  psdu          = context.getPatchSequenceDataUnit();
  size_t psdFrameCount = 0;
  for ( uint8_t i = 0; i < psdu.getPatchSequenceDataUnitSize(); i++ ) {
    auto& psup = psdu.getPatchSequenceUnitPayloadElement( i );
    bitstream.writeUvlc( ( (uint32_t)psup.getUnitType() ) );  // ue(v)
    TRACE_BITSTREAM( "PSDU %u : type = %u <=> %s \n", i, (uint32_t)psup.getUnitType(), psup.strUnitType().c_str() );
    patchSequenceUnitPayload( psup, psdu.getPatchSequenceUnitPayloadElement( prevPFLUindex ), psdFrameCount, context,
                              bitstream );
    if ( psup.getUnitType() == PSD_PFLU ) {
      psdFrameCount++;
      prevPFLUindex = i;
    }
    TRACE_BITSTREAM( "  %lu type = %s frameIndex = %u  \n", i, psup.strUnitType().c_str(), psup.getFrameIndex() );
    bitstream.write( ( uint32_t )( ( i + 1 ) == psdu.getPatchSequenceDataUnitSize() ), 1 );  // u(1)
    TRACE_BITSTREAM( " type = %s Frame = %u End = %d \n", psup.strUnitType().c_str(), psup.getFrameIndex(),
                     ( i + 1 ) == psdu.getPatchSequenceDataUnitSize() );
  }
  assert( psdFrameCount == psdu.getFrameCount() );
  byteAlignment( bitstream );
}

// 7.3.5.2 Patch data group unit payload syntax TODO: rename(?), rename enum (PSD_->PDGU_?), remove loop for APPS and AFPS, add PDGU_PREFIX_SEI and PDU_SUFFIX_SEI
void PCCBitstreamEncoder::patchSequenceUnitPayload( PatchSequenceUnitPayload& psup,
                                                    PatchSequenceUnitPayload& psupPrevPFLU,
                                                    size_t                    frameIndex,
                                                    PCCContext&               context,
                                                    PCCBitstream&             bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "  type = %u frameIndex = %u\n", psup.getUnitType(), psup.getFrameIndex() );
  auto& sps = context.getSps();
  if ( psup.getUnitType() == PSD_SPS ) {
    patchSequenceParameterSet( psup.getPatchSequenceParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_GPPS ) {
    geometryPatchParameterSet( psup.getGeometryPatchParameterSet(), psup.getGeometryFrameParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_APPS ) {
    auto& psdu = context.getPatchSequenceDataUnit();

    for ( uint16_t attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributePatchParameterSet( psup.getAttributePatchParameterSet( attributeIndex ),
                                  sps.getAttributeParameterSet( attributeIndex ),
                                  psdu.getAttributeFrameParameterSet( 0, attributeIndex ), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_FPS ) {
    patchFrameParameterSet( psup.getPatchFrameParameterSet(), sps, bitstream );
  } else if ( psup.getUnitType() == PSD_AFPS ) {
    for ( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributeFrameParameterSet( psup.getAttributeFrameParameterSet( attributeIndex ),
                                  sps.getAttributeParameterSet( attributeIndex ), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_GFPS ) {
    geometryFrameParameterSet( psup.getGeometryFrameParameterSet(), sps.getGeometryParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_PFLU ) {
    //psup.getPatchFrameLayerUnit().setFrameIndex(frameIndex);
    patchFrameLayerUnit( psup.getPatchFrameLayerUnit(), psupPrevPFLU.getPatchFrameLayerUnit(), context, bitstream );
  }
}

// 7.3.5.3 Patch sequence parameter set syntax TODO: add psps.setLog2PatchPackingBlockSize u(3), byteAlignment(bitstream) missing
void PCCBitstreamEncoder::patchSequenceParameterSet( PatchSequenceParameterSet& psps, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( psps.getPatchSequenceParameterSetId() );         // ue(v)
  bitstream.writeUvlc( psps.getLog2MaxPatchFrameOrderCntLsbMinus4() );  // ue(v)
  bitstream.writeUvlc( psps.getMaxDecPatchFrameBufferingMinus1() );     // ue(v)
  bitstream.write( psps.getLongTermRefPatchFramesFlag(), 1 );           // u(1)
  bitstream.writeUvlc( psps.getNumRefPatchFrameListsInSps() );          // ue(v)
  for ( size_t i = 0; i < psps.getNumRefPatchFrameListsInSps(); i++ ) {
    refListStruct( psps.getRefListStruct( i ), psps, bitstream );
  }
  bitstream.write(psps.getUseEightOrientationsFlag(), 1);           // u(1)
}

// 7.3.5.4 Patch frame geometry parameter set syntax TODO: rename(?), remove gfps.getOverrideGeometryParamsFlag, gfps.getOverrideGeometryPatchParamsFlag
void PCCBitstreamEncoder::geometryFrameParameterSet( GeometryFrameParameterSet& gfps,
                                                     GeometryParameterSet&      gps,
                                                     PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( gfps.getGeometryFrameParameterSetId() );  // ue(v)
  bitstream.writeUvlc( gfps.getPatchSequenceParameterSetId() );  // ue(v)
  if ( gps.getGeometryParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gfps.getOverrideGeometryParamsFlag(), 1 );  // u(1)
    if ( gfps.getOverrideGeometryParamsFlag() ) { geometryFrameParams( gfps.getGeometryFrameParams(), bitstream ); }
  }
  if ( gps.getGeometryPatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gfps.getOverrideGeometryPatchParamsFlag(), 1 );  // u(1)
    if ( gfps.getOverrideGeometryPatchParamsFlag() ) {
      bitstream.write( (uint32_t)gfps.getGeometryPatchScaleParamsEnabledFlag(), 1 );
      bitstream.write( (uint32_t)gfps.getGeometryPatchOffsetParamsEnabledFlag(), 1 );
      bitstream.write( (uint32_t)gfps.getGeometryPatchRotationParamsEnabledFlag(), 1 );
      bitstream.write( (uint32_t)gfps.getGeometryPatchPointSizeInfoEnabledFlag(), 1 );
      bitstream.write( (uint32_t)gfps.getGeometryPatchPointShapeInfoEnabledFlag(), 1 );
    }
  }
  byteAlignment( bitstream );
}

// 7.3.5.5 Geometry frame Params syntax TODO: gfp.setGeometrySmoothingGridSize wrotes 7 bits only, rotation should use quaternions
void PCCBitstreamEncoder::geometryFrameParams( GeometryFrameParams& gfp, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)gfp.getGeometrySmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryOffsetParamsPresentFlag(), 1 );     // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryRotationParamsPresentFlag(), 1 );   // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryPointSizeInfoPresentFlag(), 1 );    // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryPointShapeInfoPresentFlag(), 1 );   // u(1)

  if ( gfp.getGeometrySmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometrySmoothingEnabledFlag(), 1 );
    if ( gfp.getGeometrySmoothingEnabledFlag() ) {
      bitstream.write( (uint32_t)gfp.getGeometrySmoothingGridSize(), 8 );
      bitstream.write( (uint32_t)gfp.getGeometrySmoothingThreshold(), 8 );
    }
  }
  if ( gfp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.write( (uint32_t)gfp.getGeometryScaleOnAxis( d ), 32 );  // u(32)
    }
  }
  if ( gfp.getGeometryOffsetParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.writeS( gfp.getGeometryOffsetOnAxis( d ), 32 );  // i32
    }
  }
  if ( gfp.getGeometryRotationParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.writeS( gfp.getGeometryRotationOnAxis( d ), 32 );  // i32
    }
  }
  if ( gfp.getGeometryPointSizeInfoPresentFlag() ) { bitstream.write( (uint32_t)gfp.getGeometryPointSizeInfo(), 16 ); }
  if ( gfp.getGeometryPointShapeInfoPresentFlag() ) { bitstream.write( (uint32_t)gfp.getGeometryPointShapeInfo(), 4 ); }
}

// 7.3.5.6 Patch frame attribute parameter set syntax TODO: rename(?), remove afps.getOverrideAttributeParamsFlag, afps.setOverrideAttributePatchParamsFlag
void PCCBitstreamEncoder::attributeFrameParameterSet( AttributeFrameParameterSet& afps,
                                                      AttributeParameterSet&      aps,
                                                      PCCBitstream&               bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  // jkei : this is attributeIndex-th afps
  bitstream.writeUvlc( afps.getAttributeFrameParameterSetId() );  // ue(v)
  bitstream.writeUvlc( afps.getPatchSequencParameterSetId() );    //  ue(v)

  size_t attributeDimension = aps.getAttributeDimensionMinus1() + 1;
  if ( aps.getAttributeParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)afps.getOverrideAttributeParamsFlag(), 1 );  // u(1)
    if ( afps.getOverrideAttributeParamsFlag() ) {
      attributeFrameParams( afps.getAttributeFrameParams(), attributeDimension, bitstream );
    }
  }
  if ( aps.getAttributePatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)afps.getOverrideAttributePatchParamsFlag(),
                     1 );  //    u(1)
    if ( afps.getOverrideAttributePatchParamsFlag() ) {
      bitstream.write( (uint32_t)afps.getAttributePatchScaleParamsEnabledFlag(), 1 );   //  u(1)
      bitstream.write( (uint32_t)afps.getAttributePatchOffsetParamsEnabledFlag(), 1 );  // u(1)
    }
  }

  byteAlignment( bitstream );
}

// 7.3.5.7 Attribute frame Params syntax TODO: add attributeDimensions loop, remove afp.getAttributeSmoothingRadius, afp.getAttributeSmoothingNeighbourCount and afp.getAttributeSmoothingRadius2BoundaryDetection, add three missing parameters
void PCCBitstreamEncoder::attributeFrameParams( AttributeFrameParams& afp,
                                                size_t                attributeDimension,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)afp.getAttributeSmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)afp.getAttributeScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)afp.getAttributeOffsetParamsPresentFlag(), 1 );     // u(1)
  if ( afp.getAttributeSmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)afp.getAttributeSmoothingRadius(), 8 );                    //  u(8)
    bitstream.write( (uint32_t)afp.getAttributeSmoothingNeighbourCount(), 8 );            //   u(8)
    bitstream.write( (uint32_t)afp.getAttributeSmoothingRadius2BoundaryDetection(), 8 );  //  u(8)
    bitstream.write( (uint32_t)afp.getAttributeSmoothingThreshold(), 8 );                 // u(8)
    bitstream.write( (uint32_t)afp.getAttributeSmoothingThresholdLocalEntropy(), 3 );     // u(3)
  }

  if ( afp.getAttributeScaleParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ )
      bitstream.write( (uint32_t)afp.getAttributeScale( i ), 32 );  // u(32)
  }
  if ( afp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ ) bitstream.writeS( afp.getAttributeOffset( i ), 32 );  // i32
  }
}

// 7.3.5.8 Geometry patch parameter set syntax 
void PCCBitstreamEncoder::geometryPatchParameterSet( GeometryPatchParameterSet& gpps,
                                                     GeometryFrameParameterSet& gfps,
                                                     PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( gpps.getGeometryPatchParameterSetId() );  // ue(v)
  bitstream.writeUvlc( gpps.getGeometryFrameParameterSetId() );  //  ue(v)
  if ( gfps.getGeometryPatchScaleParamsEnabledFlag() || gfps.getGeometryPatchOffsetParamsEnabledFlag() ||
       gfps.getGeometryPatchRotationParamsEnabledFlag() || gfps.getGeometryPatchPointSizeInfoEnabledFlag() ||
       gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpps.getGeometryPatchParamsPresentFlag(), 1 );  // u(1)
    if ( gpps.getGeometryPatchParamsPresentFlag() )
      geometryPatchParams( gpps.getGeometryPatchParams(), gfps, bitstream );
  }

  byteAlignment( bitstream );
}

// 7.3.5.9 Geometry patch Params syntax TODO: rotation should be defined using quaternions (dimension 4)
void PCCBitstreamEncoder::geometryPatchParams( GeometryPatchParams&       gpp,
                                               GeometryFrameParameterSet& gfps,
                                               PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchScaleParamsPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchScaleParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.write( (uint32_t)gpp.getGeometryPatchScaleOnAxis( d ), 32 );  // u(32)
      }
    }
  }
  if ( gfps.getGeometryPatchOffsetParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchOffsetParamsPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.writeS( gpp.getGeometryPatchOffsetOnAxis( d ), 32 );  // i(32)
      }
    }
  }
  if ( gfps.getGeometryPatchRotationParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchRotationParamsPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.writeS( gpp.getGeometryPatchRotationOnAxis( d ), 32 );  // i(32)
      }
    }
  }
  if ( gfps.getGeometryPatchPointSizeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchPointSizeInfoPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchPointSizeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gpp.getGeometryPatchPointSizeInfo(), 16 );  // u(16)
    }
  }
  if ( gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchPointShapeInfoPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchPointShapeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gpp.getGeometryPatchPointShapeInfo(), 4 );  // u(14
    }
  }
}

// 7.3.5.10 Attribute patch parameter set syntax TODO: add apps.setAttributeDimensionMinus1() u(8)
void PCCBitstreamEncoder::attributePatchParameterSet( AttributePatchParameterSet& apps,
                                                      AttributeParameterSet&      aps,
                                                      AttributeFrameParameterSet& afps,
                                                      PCCBitstream&               bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( apps.getAttributePatchParameterSetId() );
  bitstream.writeUvlc( apps.getAttributeFrameParameterSetId() );
  size_t attributeDimension = aps.getAttributeDimensionMinus1() + 1;
  if ( afps.getAttributePatchScaleParamsEnabledFlag() || afps.getAttributePatchOffsetParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)apps.getAttributePatchParamsPresentFlag(), 1 );  // u(1)
    if ( apps.getAttributePatchParamsPresentFlag() ) {
      attributePatchParams( apps.getAttributePatchParams(), afps, attributeDimension, bitstream );
    }
  }
  byteAlignment( bitstream );
}

// 7.3.5.11 Attribute patch Params syntax (apps)
void PCCBitstreamEncoder::attributePatchParams( AttributePatchParams&       app,
                                                AttributeFrameParameterSet& afps,
                                                size_t                      dimension,
                                                PCCBitstream&               bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( afps.getAttributePatchScaleParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)app.getAttributePatchScaleParamsPresentFlag(), 1 );  //  u(1)
    if ( app.getAttributePatchScaleParamsPresentFlag() ) {
      for ( size_t i = 0; i < dimension; i++ ) {
        bitstream.write( (uint32_t)app.getAttributePatchScale( i ), 32 );  //  u(32)
      }
    }
  }
  if ( afps.getAttributePatchOffsetParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)app.getAttributePatchOffsetParamsPresentFlag(), 1 );  //   u(1)
    if ( app.getAttributePatchOffsetParamsPresentFlag() ) {
      for ( size_t i = 0; i < dimension; i++ ) {
        bitstream.writeS( app.getAttributePatchOffset( i ), 32 );  // i(32)
      }
    }
  }
}

// 7.3.5.12 Patch frame parameter set syntax TODO: add pfps.getGeometryPatchFrameParameterSetId and pfps.getAttributePatchFrameParameterSetId[attributeCount], and remove pfps.getLocalOverrideGeometryPatchEnableFlag and pfps.getLocalOverrideAttributePatchEnableFlag
void PCCBitstreamEncoder::patchFrameParameterSet( PatchFrameParameterSet& pfps,
                                                  SequenceParameterSet&   sps,
                                                  PCCBitstream&           bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( pfps.getPatchFrameParameterSetId() );             // ue(v)
  bitstream.writeUvlc( pfps.getPatchSequenceParameterSetId() );          // ue(v)
  bitstream.write( pfps.getLocalOverrideGeometryPatchEnableFlag(), 1 );  // u(1)
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    bitstream.write( pfps.getLocalOverrideAttributePatchEnableFlag( i ), 1 );  // u(1)
  }
  bitstream.writeUvlc( pfps.getAdditionalLtPfocLsbLen() );  // ue(v)
  if ( sps.getProjection45DegreeEnableFlag() ) {
    bitstream.write( pfps.getProjection45DegreeEnableFlag(), 1);  // u(1)
  }

  byteAlignment( bitstream );
}

// 7.3.5.13 Patch frame layer unit syntax
void PCCBitstreamEncoder::patchFrameLayerUnit( PatchFrameLayerUnit& pflu,
                                               PatchFrameLayerUnit& pfluPrev,
                                               PCCContext&          context,
                                               PCCBitstream&        bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  pflu.getPatchFrameHeader().setFrameIndex(pflu.getFrameIndex());
  pflu.getPatchFrameDataUnit().setFrameIndex(pflu.getFrameIndex());
  patchFrameHeader( pflu.getPatchFrameHeader(), pfluPrev.getPatchFrameHeader(), context, bitstream );
  patchFrameDataUnit( pflu.getPatchFrameDataUnit(), pflu.getPatchFrameHeader(), context, bitstream );
}

// 7.3.5.14 Patch frame header syntax TODO: difference between ue(v) and u(v)?
void PCCBitstreamEncoder::patchFrameHeader( PatchFrameHeader& pfh,
                                            PatchFrameHeader& pfhPrev,
                                            PCCContext&       context,
                                            PCCBitstream&     bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& psdu = context.getPatchSequenceDataUnit();
  auto& psps = psdu.getPatchSequenceParameterSet( pfh.getPatchFrameParameterSetId() );
  auto& pfps = psdu.getPatchFrameParameterSet( pfh.getPatchFrameParameterSetId() );
  auto& gps = context.getSps().getGeometryParameterSet();
  auto& sps = context.getSps();

  bitstream.writeUvlc( pfh.getPatchFrameParameterSetId() );  // ue(v )
  bitstream.writeUvlc( pfh.getAddress() );                   // u( v )
  bitstream.writeUvlc( pfh.getType() );                      // ue( v )
  bitstream.writeUvlc( pfh.getPatchFrameOrderCntLsb() );     // u( v )

  TRACE_BITSTREAM( "Id     = %u \n", pfh.getPatchFrameParameterSetId() );
  TRACE_BITSTREAM( "Adress = %u \n", pfh.getAddress() );
  TRACE_BITSTREAM( "Type   = %u \n", pfh.getType() );
  TRACE_BITSTREAM( "POC    = %u \n", pfh.getPatchFrameOrderCntLsb() );
  TRACE_BITSTREAM( "psps.getNumRefPatchFrameListsInSps() = %lu\n", psps.getNumRefPatchFrameListsInSps() );
  TRACE_BITSTREAM( "psps.getNumRefPatchFrameListsInSps() = %lu \n", psps.getNumRefPatchFrameListsInSps() );

  if ( psps.getNumRefPatchFrameListsInSps() > 0 ) {
    bitstream.write( (uint32_t)pfh.getRefPatchFrameListSpsFlag(), 1 );  // u( 1 )
  }
  if ( pfh.getRefPatchFrameListSpsFlag() ) {
    if ( psps.getNumRefPatchFrameListsInSps() > 1 ) {
      bitstream.writeUvlc( pfh.getRefPatchFrameListIdx() );  // u( v )
    } else {
      psps.getRefListStruct( psps.getNumRefPatchFrameListsInSps() );
    }
    uint8_t rlsIdx =
        psps.getNumRefPatchFrameListsInSps() ? pfh.getRefPatchFrameListIdx() : psps.getNumRefPatchFrameListsInSps();
    size_t numLtrpEntries = 0;
    for ( size_t i = 0; i < psps.getRefListStruct( rlsIdx ).getNumRefEntries(); i++ ) {
      if ( !psps.getRefListStruct( rlsIdx ).getStRefPatchFrameFlag( i ) ) { numLtrpEntries++; }
    }

    for ( size_t j = 0; j < numLtrpEntries; j++ ) {
      bitstream.write( (uint32_t)pfh.getAdditionalPfocLsbPresentFlag( j ), 1 );  // u( 1 )
      if ( pfh.getAdditionalPfocLsbPresentFlag( j ) ) {
        bitstream.writeUvlc( pfh.getAdditionalPfocLsbVal( j ) );  // u( v )
      }
    }

    if ( pfh.getType() == PATCH_FRAME_P && psps.getRefListStruct( rlsIdx ).getNumRefEntries() > 1 ) {
      bitstream.write( (uint32_t)pfh.getNumRefIdxActiveOverrideFlag(), 1 );  // u( 1 )
      if ( pfh.getNumRefIdxActiveOverrideFlag() ) {
        bitstream.writeUvlc( pfh.getNumRefIdxActiveMinus1() );  // u( v )
      }
    }
  }
  auto          geometryBitDepth2D     = context.getSps().getGeometryParameterSet().getGeometryNominal2dBitdepthMinus1()+1;
  const uint8_t maxBitCountForMaxDepth = uint8_t( geometryBitDepth2D - gbitCountSize[context.getSps().getMinLevel()] + 1 ); //8

  if( pfps.getProjection45DegreeEnableFlag() == 0){
  pfh.setInterPredictPatch2dDeltaSizeDBitCountMinus1( maxBitCountForMaxDepth );
  }
  else {
    pfh.setInterPredictPatch2dDeltaSizeDBitCountMinus1( maxBitCountForMaxDepth + 1 );
  }

  if ( pfh.getType() == PATCH_FRAME_I ) {
    bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftUBitCountMinus1(), 8 );              // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftVBitCountMinus1(), 8 );              // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(), 8 );    // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), 8 );  // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1(), 8 );     // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatchLodBitCount(), 8 );                         // u( 8 )
  } else {
    bool countFlag[6] = {0, 0, 0, 0, 0, 0};
    bool countSumFlag = 0;
    countFlag[0] =
        pfh.getInterPredictPatch2dShiftUBitCountMinus1() > pfhPrev.getInterPredictPatch2dShiftUBitCountMinus1();
    countFlag[1] =
        pfh.getInterPredictPatch2dShiftVBitCountMinus1() > pfhPrev.getInterPredictPatch2dShiftVBitCountMinus1();
    countFlag[2] = pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() >
                   pfhPrev.getInterPredictPatch3dShiftTangentAxisBitCountMinus1();
    countFlag[3] = pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() >
                   pfhPrev.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1();
    countFlag[4] = pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1() >
                   pfhPrev.getInterPredictPatch3dShiftNormalAxisBitCountMinus1();
    countFlag[5] = pfh.getInterPredictPatchLodBitCount() > pfhPrev.getInterPredictPatchLodBitCount();
    for ( size_t i = 0; i < 6; i++ ) countSumFlag |= countFlag[i];
    pfh.setInterPredictPatchBitCountFlag( countSumFlag );
    pfh.setInterPredictPatch2dShiftUBitCountFlag( countFlag[0] );
    pfh.setInterPredictPatch2dShiftVBitCountFlag( countFlag[1] );
    pfh.setInterPredictPatch3dShiftTangentAxisBitCountFlag( countFlag[2] );
    pfh.setInterPredictPatch3dShiftBitangentAxisBitCountFlag( countFlag[3] );
    pfh.setInterPredictPatch3dShiftNormalAxisBitCountFlag( countFlag[4] );
    pfh.setInterPredictPatchLodBitCountFlag( countFlag[5] );
    if ( !pfh.getInterPredictPatchBitCountFlag() || !pfh.getInterPredictPatch2dShiftUBitCountFlag() ) {
      pfh.setInterPredictPatch2dShiftUBitCountMinus1( pfhPrev.getInterPredictPatch2dShiftUBitCountMinus1() );
    }
    if ( !pfh.getInterPredictPatchBitCountFlag() || !pfh.getInterPredictPatch2dShiftVBitCountFlag() ) {
      pfh.setInterPredictPatch2dShiftVBitCountMinus1( pfhPrev.getInterPredictPatch2dShiftVBitCountMinus1() );
    }
    if ( !pfh.getInterPredictPatchBitCountFlag() || !pfh.getInterPredictPatch3dShiftTangentAxisBitCountFlag() ) {
      pfh.setInterPredictPatch3dShiftTangentAxisBitCountMinus1(
          pfhPrev.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() );
    }
    if ( !pfh.getInterPredictPatchBitCountFlag() || !pfh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag() ) {
      pfh.setInterPredictPatch3dShiftBitangentAxisBitCountMinus1(
          pfhPrev.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() );
    }
    if ( !pfh.getInterPredictPatchBitCountFlag() || !pfh.getInterPredictPatch3dShiftNormalAxisBitCountFlag() ) {
      pfh.setInterPredictPatch3dShiftNormalAxisBitCountMinus1(
          pfhPrev.getInterPredictPatch3dShiftNormalAxisBitCountMinus1() );
    }
    if ( !pfh.getInterPredictPatchBitCountFlag() || !pfh.getInterPredictPatchLodBitCountFlag() ) {
      pfh.setInterPredictPatchLodBitCount( pfhPrev.getInterPredictPatchLodBitCount() );
    }
    bitstream.write( (uint32_t)pfh.getInterPredictPatchBitCountFlag(), 1 );  // u( 1 )
    if ( pfh.getInterPredictPatchBitCountFlag() ) {
      bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftUBitCountFlag(), 1 );  // u( 1 )
      if ( pfh.getInterPredictPatch2dShiftUBitCountFlag() ) {
        bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftUBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftVBitCountFlag(), 1 );  // u( 1 )
      if ( pfh.getInterPredictPatch2dShiftVBitCountFlag() ) {
        bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftVBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftTangentAxisBitCountFlag(), 1 );  // u( 1 )
      if ( pfh.getInterPredictPatch3dShiftTangentAxisBitCountFlag() ) {
        bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag(), 1 );  // u( 1 )
      if ( pfh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag() ) {
        bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftNormalAxisBitCountFlag(), 1 );  // u( 1 )
      if ( pfh.getInterPredictPatch3dShiftNormalAxisBitCountFlag() ) {
        bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)pfh.getInterPredictPatchLodBitCountFlag(), 1 );  // u( 1 )
      if ( pfh.getInterPredictPatchLodBitCountFlag() ) {
        bitstream.write( (uint32_t)pfh.getInterPredictPatchLodBitCount(), 8 );  // u( 8 )
      }
    }
  }
  if (sps.getPcmPatchEnabledFlag()) {
    bitstream.write((uint32_t)pfh.getPcm3dShiftBitCountPresentFlag(), 1);  // u( 1 )
    if (pfh.getPcm3dShiftBitCountPresentFlag()) {
      //bitstream.write((uint32_t)pfh.getPcm3dShiftAxisBitCountMinus1(), 11);
      bitstream.write((uint32_t)pfh.getPcm3dShiftAxisBitCountMinus1(), gps.getGeometry3dCoordinatesBitdepthMinus1());
    }
  }
  TRACE_BITSTREAM( "InterPredictPatchBitCount Flag %d %d %d %d %d %d %d %d Count = %u %u %u %u %u %u %u \n",
                   pfh.getInterPredictPatchBitCountFlag(), pfh.getInterPredictPatch2dShiftUBitCountFlag(),
                   pfh.getInterPredictPatch2dShiftVBitCountFlag(),
                   pfh.getInterPredictPatch3dShiftTangentAxisBitCountFlag(),
                   pfh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag(),
                   pfh.getInterPredictPatch3dShiftNormalAxisBitCountFlag(), pfh.getInterPredictPatchLodBitCountFlag(), pfh.getPcm3dShiftBitCountPresentFlag(),
                   pfh.getInterPredictPatch2dShiftUBitCountMinus1(), pfh.getInterPredictPatch2dShiftVBitCountMinus1(),
                   pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(),
                   pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(),
                   pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1(), pfh.getInterPredictPatchLodBitCount(), 
                    pfh.getPcm3dShiftAxisBitCountMinus1());
  byteAlignment( bitstream );
}

// 7.3.5.15 Reference list structure syntax TODO: difference between ue(v) and u(v)?
void PCCBitstreamEncoder::refListStruct( RefListStruct&             rls,
                                         PatchSequenceParameterSet& psps,
                                         PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( rls.getNumRefEntries() );  // ue(v)
  rls.allocate();
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( psps.getLongTermRefPatchFramesFlag() ) {
      bitstream.write( rls.getStRefPatchFrameFlag( i ), 1 );  // u(1)
      if ( rls.getStRefPatchFrameFlag( i ) ) {
        bitstream.writeUvlc( rls.getAbsDeltaPfocSt( i ) );  // ue(v)
        if ( rls.getAbsDeltaPfocSt( i ) > 0 ) {
          bitstream.write( rls.getStrpfEntrySignFlag( i ), 1 );  // u(1)
        } else {
          bitstream.writeUvlc( rls.getPfocLsbLt( i ) );  // u(v)
        }
      }
    }
  }
}

// 7.3.5.16 Patch frame data unit syntax TODO: modify loop to use patchMode instead of flag
void PCCBitstreamEncoder::patchFrameDataUnit( PatchFrameDataUnit& pfdu,
                                              PatchFrameHeader&   pfh,
                                              PCCContext&         context,
                                              PCCBitstream&       bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "pfh.getType()        = %lu \n", pfh.getType() );
  uint8_t       moreAvailablePatchFlag = pfdu.getPatchCount() > 0;
  const uint8_t bitCountPatchMode      = ( PCCPatchFrameType( pfh.getType() ) ) == PATCH_FRAME_I ? 1 : 2;
  bitstream.write( moreAvailablePatchFlag, 1 );
  TRACE_BITSTREAM( "moreAvailablePatchFlag = %d \n", moreAvailablePatchFlag );
  TRACE_BITSTREAM( "bitCountPatchMode = %u \n", bitCountPatchMode );
  for ( size_t puCount = 0; puCount < pfdu.getPatchCount(); puCount++ ) {
    bitstream.write( uint32_t( pfdu.getPatchMode( puCount ) ), bitCountPatchMode );
    TRACE_BITSTREAM( "patchMode = %lu \n", pfdu.getPatchMode( puCount ) );
    pfdu.getPatchInformationData( puCount ).setFrameIndex(pfdu.getFrameIndex());
    pfdu.getPatchInformationData( puCount ).setPatchIndex(puCount);
    patchInformationData( pfdu.getPatchInformationData( puCount ), pfdu.getPatchMode( puCount ), pfh, context,
                          bitstream );
    moreAvailablePatchFlag = !( ( puCount + 1 ) == pfdu.getPatchCount() );
    bitstream.write( moreAvailablePatchFlag, 1 );  // ae(v)
    TRACE_BITSTREAM( "moreAvailablePatchFlag = %d \n", moreAvailablePatchFlag );
  }
  byteAlignment( bitstream );
}

// 7.3.5.17 Patch information data syntax TODO: gppsId and appsId using u(v) instead of ue(v) ?
void PCCBitstreamEncoder::patchInformationData( PatchInformationData& pid,
                                                size_t                patchMode,
                                                PatchFrameHeader&     pfh,
                                                PCCContext&           context,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sps             = context.getSps();
  auto&         psdu            = context.getPatchSequenceDataUnit();
  auto&         pfps            = psdu.getPatchFrameParameterSet( pfh.getPatchFrameParameterSetId() );
  const uint8_t bitCountGAppsId = 6;
  if ( ( PCCPatchFrameType( pfh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_SKIP ) {
    // skip mode.
    // currently not supported but added it for convenience. Could easily be removed
  } else if ( ( ( PCCPatchFrameType( pfh.getType() ) ) == PATCH_FRAME_I && patchMode == PATCH_MODE_I_INTRA ) ||
              ( ( PCCPatchFrameType( pfh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_INTRA ) ) {
    if ( pfps.getLocalOverrideGeometryPatchEnableFlag() ) {
      bitstream.write( pid.getOverrideGeometryPatchFlag(), 1 ); // u(1)
      if ( pid.getOverrideGeometryPatchFlag() ) {
        bitstream.write( uint32_t( pid.getGeometryPatchParameterSetId() ), bitCountGAppsId ); // ue(v)
        TRACE_BITSTREAM( " gppsId = %lu \n", pid.getGeometryPatchParameterSetId() );
      }
    }
    TRACE_BITSTREAM( " sps.getAttributeCount() = %lu \n", sps.getAttributeCount() );
    for ( int i = 0; i < sps.getAttributeCount(); i++ ) {
      TRACE_BITSTREAM( " overight flag = %lu \n", pfps.getLocalOverrideAttributePatchEnableFlag( i ) );
      if ( pfps.getLocalOverrideAttributePatchEnableFlag( i ) ) {
        bitstream.write( pid.getOverrideAttributePatchFlag( i ), 1 );
        TRACE_BITSTREAM( " overrideAttributePatchFlag = %lu \n", pid.getOverrideAttributePatchFlag( i ) );
      }
      TRACE_BITSTREAM( " overight patch flag = %lu \n", pid.getOverrideAttributePatchFlag( i ) );
      if ( pid.getOverrideAttributePatchFlag( i ) ) {
        bitstream.write( uint32_t( pid.getAttributePatchParameterSetId( i ) ), bitCountGAppsId );
        TRACE_BITSTREAM( " AttributePatchParameterSetId = %lu \n", pid.getAttributePatchParameterSetId( i ) );
      }
    }
    auto& pdu = pid.getPatchDataUnit();
    pdu.setPduFrameIndex(pid.getFrameIndex());
    pdu.setPduPatchIndex(pid.getPatchIndex());
    patchDataUnit( pdu, pfh, context, bitstream );
  } else if ( ( PCCPatchFrameType( pfh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_INTER ) {
    auto& dpdu = pid.getDeltaPatchDataUnit();
    dpdu.setDpduFrameIndex(pid.getFrameIndex());
    dpdu.setDpduPatchIndex(pid.getPatchIndex());
    deltaPatchDataUnit( dpdu, pfh, context, bitstream );
  } else if ( ( ( PCCPatchFrameType( pfh.getType() ) ) == PATCH_FRAME_I && patchMode == PATCH_MODE_I_PCM ) ||
              ( ( PCCPatchFrameType( pfh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_PCM ) ) {
    auto& ppdu = pid.getPCMPatchDataUnit();
    ppdu.setPpduFrameIndex(pid.getFrameIndex());
    ppdu.setPpduPatchIndex(pid.getPatchIndex());
    pcmPatchDataUnit( ppdu, pfh, context, bitstream );
  }
}

// 7.3.5.18 Patch data unit syntax
void PCCBitstreamEncoder::patchDataUnit( PatchDataUnit&    pdu,
                                         PatchFrameHeader& pfh,
                                         PCCContext&       context,
                                         PCCBitstream&     bitstream ) {
  auto& sps = context.getSps();
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( uint32_t( pdu.get2DShiftU() ), pfh.getInterPredictPatch2dShiftUBitCountMinus1() + 1 );
  bitstream.write( uint32_t( pdu.get2DShiftV() ), pfh.getInterPredictPatch2dShiftVBitCountMinus1() + 1 );
  bitstream.write( uint32_t( pdu.get2DDeltaSizeD() ), pfh.getInterPredictPatch2dDeltaSizeDBitCountMinus1() + 1 );
  bitstream.writeSvlc( int32_t( pdu.get2DDeltaSizeU() ) );  // The way it is implemented in TM
  bitstream.writeSvlc( int32_t( pdu.get2DDeltaSizeV() ) );  // The way it is implemented in TM
  bitstream.write( uint32_t( pdu.get3DShiftTangentAxis() ),
                   pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() + 1 );
  bitstream.write( uint32_t( pdu.get3DShiftBiTangentAxis() ),
                   pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() + 1 );
  bitstream.write( uint32_t( pdu.get3DShiftNormalAxis() ),
                   pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1() + 1 );

  bitstream.write( uint32_t(pdu.getProjectPlane()), 3 );  // 0,1,2(near 0,1,2) 3,4,5(far 0,1,2)
  auto& psdu = context.getPatchSequenceDataUnit();
  auto& psps = psdu.getPatchSequenceParameterSet(0);
  if ( psps.getUseEightOrientationsFlag() ) { 
    bitstream.write(pdu.getOrientationIndex(), 3);
  }
  else {
    bitstream.write(pdu.getOrientationIndex(), 1);
  }
  if ( pfh.getInterPredictPatchLodBitCount() > 0 ) {
    bitstream.write( uint32_t( pdu.getLod() ), pfh.getInterPredictPatchLodBitCount() );
  }
  if( sps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionData( pdu.getPointLocalReconstructionData(), context, bitstream );
  }
  auto& pfps = psdu.getPatchFrameParameterSet(0);
  if ( pfps.getProjection45DegreeEnableFlag() ) {
    bitstream.write( uint32_t( pdu.get45DegreeProjectionPresentFlag() ), 1 );
  }
  if ( pdu.get45DegreeProjectionPresentFlag() ) {
    bitstream.write( uint32_t( pdu.get45DegreeProjectionRotationAxis() ), 2 );
  }

  TRACE_BITSTREAM( "Patch => UV %4lu %4lu S=%4ld %4ld %4ld P=%lu O=%d A=%lu %lu %lu P45= %d %d \n ", pdu.get2DShiftU(),
                  pdu.get2DShiftV(), pdu.get2DDeltaSizeU(), pdu.get2DDeltaSizeV(), pdu.get2DDeltaSizeD(),
                  pdu.getProjectPlane(),
                  pdu.getOrientationIndex(), pdu.get3DShiftTangentAxis(), pdu.get3DShiftBiTangentAxis(),
                  pdu.get3DShiftNormalAxis(),pdu.get45DegreeProjectionPresentFlag(), pdu.get45DegreeProjectionRotationAxis()  );

}

// 7.3.5.19  Delta Patch data unit syntax TODO: Missing 10-projection syntax element?
void PCCBitstreamEncoder::deltaPatchDataUnit( DeltaPatchDataUnit& dpdu,
                                              PatchFrameHeader&   pfh,
                                              PCCContext&         context,
                                              PCCBitstream&       bitstream ) {
  auto& sps = context.getSps();
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeSvlc( int32_t( dpdu.getDeltaPatchIdx() ) );
  bitstream.writeSvlc( int32_t( dpdu.get2DDeltaShiftU() ) );
  bitstream.writeSvlc( int32_t( dpdu.get2DDeltaShiftV() ) );
  bitstream.writeSvlc( int32_t( dpdu.get2DDeltaSizeU() ) );
  bitstream.writeSvlc( int32_t( dpdu.get2DDeltaSizeV() ) );
  bitstream.writeSvlc( int32_t( dpdu.get2DDeltaSizeD() ) );
  bitstream.writeSvlc( int32_t( dpdu.get3DDeltaShiftTangentAxis() ) );
  bitstream.writeSvlc( int32_t( dpdu.get3DDeltaShiftBiTangentAxis() ) );
  bitstream.writeSvlc( int32_t( dpdu.get3DDeltaShiftNormalAxis() ) );
  if ( sps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionData( dpdu.getPointLocalReconstructionData(), context, bitstream );
  }

  TRACE_BITSTREAM(
                  "DeltaPatch => DeltaIdx = %u ShiftUV = %ld %ld DeltaSize = %ld %ld %ld Axis = %ld %ld %ld \n",
                  dpdu.getDeltaPatchIdx(), dpdu.get2DDeltaShiftU(), dpdu.get2DDeltaShiftV(), dpdu.get2DDeltaSizeU(),
                  dpdu.get2DDeltaSizeV(), dpdu.get2DDeltaSizeD(), dpdu.get3DDeltaShiftTangentAxis(), dpdu.get3DDeltaShiftBiTangentAxis(),
                  dpdu.get3DDeltaShiftNormalAxis() );
}

// 7.3.5.20 PCM patch data unit syntax TODO: getPcmPoints is u(v) in CD, but is currently se(v)
void PCCBitstreamEncoder::pcmPatchDataUnit( PCMPatchDataUnit& ppdu,
                                            PatchFrameHeader& pfh,
                                            PCCContext&       context,
                                            PCCBitstream&     bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  auto& sps = context.getSps();
  auto& gps = context.getSps().getGeometryParameterSet();
  if ( sps.getPcmSeparateVideoPresentFlag() ) { bitstream.write( ppdu.getPatchInPcmVideoFlag(), 1 ); }
  bitstream.write( uint32_t( ppdu.get2DShiftU() ), pfh.getInterPredictPatch2dShiftUBitCountMinus1() + 1 );
  bitstream.write( uint32_t( ppdu.get2DShiftV() ), pfh.getInterPredictPatch2dShiftVBitCountMinus1() + 1 );
  bitstream.writeSvlc( int32_t( ppdu.get2DDeltaSizeU() ) );
  bitstream.writeSvlc( int32_t( ppdu.get2DDeltaSizeV() ) );

  if (pfh.getPcm3dShiftBitCountPresentFlag()) {
    bitstream.write(uint32_t(ppdu.get3DShiftTangentAxis()),
      pfh.getPcm3dShiftAxisBitCountMinus1() + 1);
    bitstream.write(uint32_t(ppdu.get3DShiftBiTangentAxis()),
      pfh.getPcm3dShiftAxisBitCountMinus1() + 1);
    bitstream.write(uint32_t(ppdu.get3DShiftNormalAxis()),
      pfh.getPcm3dShiftAxisBitCountMinus1() + 1);
  }
  else {
    size_t bitCountPcmU1V1D1 = gps.getGeometry3dCoordinatesBitdepthMinus1()  - gps.getGeometryNominal2dBitdepthMinus1();
    bitstream.write(uint32_t(ppdu.get3DShiftTangentAxis()),      bitCountPcmU1V1D1);
    bitstream.write(uint32_t(ppdu.get3DShiftBiTangentAxis()),    bitCountPcmU1V1D1);
    bitstream.write(uint32_t(ppdu.get3DShiftNormalAxis()),       bitCountPcmU1V1D1);
  }

  bitstream.writeSvlc( int32_t( ppdu.getPcmPoints() ) );
  TRACE_BITSTREAM( "PCM Patch => UV %4lu %4lu  S=%4ld %4ld  UVD1=%4ld %4ld %4ld NumPcmPoints=%lu PatchInPcmVideoFlag=%d \n",
                   ppdu.get2DShiftU(), ppdu.get2DShiftV(), ppdu.get2DDeltaSizeU(), ppdu.get2DDeltaSizeV(),
                   ppdu.get3DShiftBiTangentAxis(), ppdu.get3DShiftBiTangentAxis(), ppdu.get3DShiftNormalAxis(),
                   ppdu.getPcmPoints(), ppdu.getPatchInPcmVideoFlag() );
}

// 7.3.5.21 Point local reconstruction syntax
void PCCBitstreamEncoder::pointLocalReconstruction( PointLocalReconstruction& plr,
                                                    PCCContext&               context,
                                                    PCCBitstream&             bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( uint32_t( plr.getPlrlNumberOfModesMinus1() ), 4 );
  TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plr.getPlrlNumberOfModesMinus1() );
  for ( size_t i = 0; i <= plr.getPlrlNumberOfModesMinus1(); i++ ) {
    bitstream.write( uint32_t( plr.getPlrlInterpolateFlag( i ) ), 1 );
    bitstream.write( uint32_t( plr.getPlrlFillingFlag( i ) ), 1 );
    bitstream.write( uint32_t( plr.getPlrlMinimumDepth( i ) ), 2 );
    bitstream.write( uint32_t( plr.getPlrlNeighbourMinus1( i ) ), 2 );
    TRACE_BITSTREAM( "  Mode[%lu] = I = %d F = %d D = %u N = %u \n", i, plr.getPlrlInterpolateFlag( i ),
                     plr.getPlrlFillingFlag( i ), plr.getPlrlMinimumDepth( i ), plr.getPlrlNeighbourMinus1( i ) );
  }
  bitstream.writeUvlc( uint32_t( plr.getPlrBlockThresholdPerPatchMinus1() ) );
  TRACE_BITSTREAM( "  BlockThresholdPerPatchMinus1 = %u \n", plr.getPlrBlockThresholdPerPatchMinus1() );
}

void PCCBitstreamEncoder::pointLocalReconstructionData( PointLocalReconstructionData& plrd,
                                                        PCCContext&                   context,
                                                        PCCBitstream&                 bitstream ) {
  auto& plr = context.getSps().getPointLocalReconstruction();
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "WxH= %lu x %lu \n", plrd.getPlrBlockToPatchMapWidth(), plrd.getPlrBlockToPatchMapHeight() );

  const size_t  blockCount   = plrd.getPlrBlockToPatchMapWidth() * plrd.getPlrBlockToPatchMapHeight();
  const uint8_t bitCountMode = uint8_t( getFixedLengthCodeBitsCount( uint32_t( plr.getPlrlNumberOfModesMinus1() ) ) );
  TRACE_BITSTREAM( "  bitCountMode = %u \n", bitCountMode );

  if ( blockCount > plr.getPlrBlockThresholdPerPatchMinus1() + 1 ) {
    bitstream.write( uint32_t( plrd.getPlrLevelFlag() ), 1 );
  }
  TRACE_BITSTREAM( "  LevelFlag = %u \n", plrd.getPlrLevelFlag() );
  if ( plrd.getPlrLevelFlag() ) {
    bitstream.write( uint32_t( plrd.getPlrPresentFlag() ), 1 );
    if ( plrd.getPlrPresentFlag() ) { bitstream.write( uint32_t( plrd.getPlrModeMinus1() ), bitCountMode ); }
    TRACE_BITSTREAM( "  ModePatch: Present = %d ModeMinus1 = %d \n", plrd.getPlrPresentFlag(),
                     plrd.getPlrPresentFlag() ? (int32_t)plrd.getPlrModeMinus1() : -1 );
  } else {
    for ( size_t i = 0; i < blockCount; i++ ) {
      bitstream.write( uint32_t( plrd.getPlrBlockPresentFlag( i ) ), 1 );
      if ( plrd.getPlrBlockPresentFlag( i ) ) {
        bitstream.write( uint32_t( plrd.getPlrBlockModeMinus1( i ) ), bitCountMode );
      }
      TRACE_BITSTREAM( "  Mode[ %4lu / %4lu ]: Present = %d ModeMinus1 = %d \n", i, blockCount,
                       plrd.getPlrBlockPresentFlag( i ),
                       plrd.getPlrBlockPresentFlag( i ) ? plrd.getPlrBlockModeMinus1( i ) : -1 );
    }
  }
#ifdef BITSTREAM_TRACE
  for ( size_t v0 = 0; v0 < plrd.getPlrBlockToPatchMapHeight(); ++v0 ) {
    for ( size_t u0 = 0; u0 < plrd.getPlrBlockToPatchMapWidth(); ++u0 ) {
      size_t i = v0 * plrd.getPlrBlockToPatchMapWidth() + u0;
      TRACE_BITSTREAM( "Patch Block[ %2lu %2lu <=> %4lu ] / [ %2lu %2lu ] Level = %d Present = %d Mode = %d \n", u0, v0,
                       i, plrd.getPlrBlockToPatchMapWidth(), plrd.getPlrBlockToPatchMapHeight(), plrd.getPlrLevelFlag(),
                       plrd.getPlrLevelFlag() ? plrd.getPlrPresentFlag() : plrd.getPlrBlockPresentFlag( i ),
                       plrd.getPlrLevelFlag()
                           ? plrd.getPlrPresentFlag() ? (int32_t)plrd.getPlrModeMinus1() : -1
                           : plrd.getPlrBlockPresentFlag( i ) ? plrd.getPlrBlockModeMinus1( i ) : -1 );
    }
  }
#endif
}
