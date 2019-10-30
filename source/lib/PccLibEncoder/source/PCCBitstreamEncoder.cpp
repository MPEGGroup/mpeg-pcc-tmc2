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

void PCCBitstreamEncoder::rbspRailingBits() {  }  // TODO
bool PCCBitstreamEncoder::moreRbspData() { return false; }  // TODO

void PCCBitstreamEncoder::setParameters( PCCEncoderParameters params ) { params_ = params; }

int PCCBitstreamEncoder::encode( PCCContext& context, PCCBitstream& bitstream ) {
  bitstream.getBitStreamStat().newGOF();
  vpccUnit( context, bitstream, VPCC_VPS );
  vpccUnit( context, bitstream, VPCC_AD );
  vpccUnit( context, bitstream, VPCC_OVD );
  vpccUnit( context, bitstream, VPCC_GVD );
  vpccUnit( context, bitstream, VPCC_AVD );
  std::cout << " occupancy map  ->" << bitstream.getBitStreamStat().getTotalMetadata() << " B " << std::endl;
  return 0;
}

void PCCBitstreamEncoder::videoSubStream( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sps = context.getSps();
  size_t atlasIndex = 0; 
  if ( vpccUnitType == VPCC_OVD ) {
    TRACE_BITSTREAM( "OccupancyMap \n" );
    bitstream.write( context.getVideoBitstream( VIDEO_OCCUPANCY ) );
  } else if ( vpccUnitType == VPCC_GVD ) {
    TRACE_BITSTREAM( "Geometry \n" );
    if ( sps.getMapCountMinus1(atlasIndex) > 0 && !sps.getMapAbsoluteCodingEnableFlag( atlasIndex, 1 ) ) {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_D0 ) );
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_D1 ) );
    } else {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY ) );
    }
    if ( sps.getRawPatchEnabledFlag(atlasIndex) && sps.getRawSeparateVideoPresentFlag(atlasIndex) ) {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_MP ) );
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( sps.getAttributeInformation(atlasIndex).getAttributeCount() > 0 ) {
      TRACE_BITSTREAM( "Texture \n" );
      bitstream.write( context.getVideoBitstream( VIDEO_TEXTURE ) );
      if ( sps.getRawPatchEnabledFlag(atlasIndex) && sps.getRawSeparateVideoPresentFlag(atlasIndex) ) {
        bitstream.write( context.getVideoBitstream( VIDEO_TEXTURE_MP ) );
      }
    }
  }
}

// 7.3.2.1 General V-PCC unit syntax
void PCCBitstreamEncoder::vpccUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  int32_t position = (int32_t)bitstream.size();
  vpccUnitHeader( context, bitstream, vpccUnitType );
  vpccUnitPayload( context, bitstream, vpccUnitType );
  // while( moreDataInVpccUnit() ) { bitstream.write( 0, 8 ); }
  bitstream.getBitStreamStat().setVpccUnitSize( vpccUnitType, (int32_t)bitstream.size() - position );
}

// 7.3.2.2 V-PCC unit header syntax
void PCCBitstreamEncoder::vpccUnitHeader( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  bitstream.write( vpccUnitType, 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_AD ) {
    bitstream.write( (uint32_t)vpcc.getVpccParameterSetId(), 4 );  // u(4)
    bitstream.write( (uint32_t)vpcc.getAtlasId(), 6 );  // u(6)
  }
  if ( vpccUnitType == VPCC_AVD ) {
    bitstream.write( (uint32_t)vpcc.getAttributeIndex(), 7 );           // u(7)
    bitstream.write( (uint32_t)vpcc.getAttributeDimensionIndex(), 5 );  // u(5)
    bitstream.write( (uint32_t)vpcc.getMapIndex(), 4 );                 // u(4)
    bitstream.write( (uint32_t)vpcc.getRawVideoFlag(), 1 );             // u(1)

  } else if ( vpccUnitType == VPCC_GVD ) {
    bitstream.write( (uint32_t)vpcc.getMapIndex(), 4 );      // u(4)
    bitstream.write( (uint32_t)vpcc.getRawVideoFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)0, 12 );                      // u(12)

  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_AD ) {
    bitstream.write( (uint32_t)0, 17 );  // u(17)
  } else {
    bitstream.write( (uint32_t)0, 27 );  // u(27)
  }
}

// 7.3.2.3 V-PCC unit payload syntax
void PCCBitstreamEncoder::vpccUnitPayload( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sps = context.getSps();
  TRACE_BITSTREAM( "vpccUnitType = %d \n", (int32_t)vpccUnitType );
  if ( vpccUnitType == VPCC_VPS ) {
    vpccParameterSet( sps, context, bitstream );
  } else if ( vpccUnitType == VPCC_AD ) {
    atlasSubStream( context, bitstream ); 
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    videoSubStream( context, bitstream, vpccUnitType ); 
  }
}

// 7.3.3 Byte alignment syntax
void PCCBitstreamEncoder::byteAlignment( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( 1, 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.write( 0, 1 );  // f(1): equal to 0
  }
}

// 7.3.4.1 General V-PCC parameter set syntax
void PCCBitstreamEncoder::vpccParameterSet( VpccParameterSet& sps, PCCContext& context, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  bitstream.write( (uint32_t)sps.getVpccParameterSetId(), 4 );  // u(4)
  bitstream.write( (uint32_t)sps.getAtlasCountMinus1(), 6 );    // u(6)
  for ( int j = 0; j < sps.getAtlasCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "Atlas = %lu \n", j );
    bitstream.write( (uint32_t)sps.getFrameWidth( j ), 16 );                    // u(16)
    bitstream.write( (uint32_t)sps.getFrameHeight( j ), 16 );                   // u(16)
    bitstream.write( (uint32_t)sps.getMapCountMinus1( j ), 4 );                 // u(4)
    TRACE_BITSTREAM( " MapCountMinus1 = %lu \n", sps.getMapCountMinus1( j ) );
    if ( sps.getMapCountMinus1(j ) > 0 ) {
      bitstream.write( (uint32_t)sps.getMultipleMapStreamsPresentFlag( j ), 1 );  // u(1)
      TRACE_BITSTREAM( "MultipleMapStreamsPresentFlag = %lu \n", sps.getMultipleMapStreamsPresentFlag( j ) );
    }    
    for ( size_t i = 1; i <= sps.getMapCountMinus1(j ); i++ ) {
      if ( sps.getMultipleMapStreamsPresentFlag( j ) ) {
        bitstream.write( (uint32_t)sps.getMapAbsoluteCodingEnableFlag( j, i ), 1 );  // u(1)
      }
      if ( sps.getMapAbsoluteCodingEnableFlag( j, i ) == 0 ) {
        if( i > 0 ){
          bitstream.writeUvlc( (uint32_t)sps.getMapPredictorIndexDiff( j, i ) );  // ue(v)
        }
      }
    }
    bitstream.write( (uint32_t)sps.getRawPatchEnabledFlag( j ), 1 );  // u(1)
    TRACE_BITSTREAM( " RawPatchEnabledFlag = %lu \n", sps.getRawPatchEnabledFlag( j ) );
    if ( sps.getRawPatchEnabledFlag( j ) ) {
      bitstream.write( (uint32_t)sps.getRawSeparateVideoPresentFlag( j ), 1 );  // u(1)
      TRACE_BITSTREAM( "RawSeparateVideoPresentFlag = %lu \n", sps.getRawSeparateVideoPresentFlag( j ) );
    }
#ifdef BITSTREAM_TRACE
    for ( size_t i = 0; i < sps.getMapCountMinus1( j ) + 1; i++ ) {
      TRACE_BITSTREAM( " AbsoluteCoding L%lu = %lu \n", i, sps.getMapAbsoluteCodingEnableFlag( j, i ) );
    }
#endif
    occupancyInformation( sps.getOccupancyInformation( j ), bitstream );
    geometryInformation( sps.getGeometryInformation( j ), sps, bitstream );
    attributeInformation( sps.getAttributeInformation(j), sps, bitstream );
  }
  bitstream.write( (uint32_t)sps.getExtensionPresentFlag(), 1 );  // u(1)
  if ( sps.getExtensionPresentFlag() ) {
    bitstream.writeUvlc( (uint32_t)sps.getExtensionLength() );  // ue(v)
    for ( size_t i = 0; i < sps.getExtensionLength(); i++ ) {
      bitstream.write( (uint32_t)sps.getExtensionDataByte( i ), 8 );  // u(8)
    }
  }

  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
#ifdef BITSTREAM_TRACE
  bitstream.trace( "  Depreciated1\n" );
#endif
  bitstream.write( (uint32_t)sps.getLosslessGeo444(), 1 );    // u(1) TODO: remove?
  bitstream.write( (uint32_t)sps.getLosslessGeo(), 1 );       // u(1) TODO: remove?
  
  bitstream.write( (uint32_t)sps.getMinLevel(), 8 );          // u(8) TODO: remove?
  bitstream.write( (uint32_t)sps.getSurfaceThickness(), 8 );  // u(8) TODO: remove?

  bitstream.write( (uint32_t)sps.getPatchInterPredictionEnabledFlag(), 1 );      // u(1) TODO: remove?
  bitstream.write( (uint32_t)sps.getPixelDeinterleavingFlag(), 1 );              // u(1) TODO: remove?
  bitstream.write( (uint32_t)sps.getPointLocalReconstructionEnabledFlag(), 1 );  // u(1) TODO: remove?
  if ( sps.getPointLocalReconstructionEnabledFlag() ) {// TODO: remove?
    pointLocalReconstructionInformation( sps.getPointLocalReconstructionInformation(), context, bitstream );// TODO: remove?
  }// TODO: remove?
  bitstream.write( (uint32_t)sps.getRemoveDuplicatePointEnabledFlag(), 1 );  // u(1) TODO: remove?
  bitstream.write( (uint32_t)sps.getProjection45DegreeEnableFlag(), 1 );     // u(1) TODO: remove?
  bitstream.write( (uint32_t)sps.getPatchPrecedenceOrderFlag(), 1 );         // u(1) TODO: remove?
#ifdef BITSTREAM_TRACE
  bitstream.trace( "  Depreciated2\n" );
#endif
  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
  byteAlignment( bitstream );
}

// 7.3.4.2 Profile, tier, and level syntax
void PCCBitstreamEncoder::profileTierLevel( ProfileTierLevel& ptl, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)ptl.getTierFlag(), 1 );                  // u(1)
  bitstream.write( (uint32_t)ptl.getProfileCodecGroupIdc(), 7 );      // u(7)
  bitstream.write( (uint32_t)ptl.getProfilePccToolsetIdc(), 8 );      // u(8)
  bitstream.write( (uint32_t)ptl.getProfileReconctructionIdc(), 8 );  // u(8)
  bitstream.write( (uint32_t)0, 32 );                                 // u(32)
  bitstream.write( (uint32_t)ptl.getLevelIdc(), 8 );                  // u(8)
}

// 7.3.4.3 Occupancy parameter set syntax
void PCCBitstreamEncoder::occupancyInformation( OccupancyInformation& oi, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)oi.getOccupancyCodecId(), 8 );                       // u(8)
  bitstream.write( (uint32_t)oi.getLossyOccupancyMapCompressionThreshold(), 8 );  // u(8)
  bitstream.write( (uint32_t)oi.getOccupancyNominal2DBitdepthMinus1(), 5 );       // u(5)
  bitstream.write( (uint32_t)oi.getOccupancyMSBAlignFlag(), 1 );                  // u(1)
  TRACE_BITSTREAM( "  OccupancyLossyThreshold = %d  \n", oi.getLossyOccupancyMapCompressionThreshold() );
}

// 7.3.4.4 Geometry parameter set syntax
void PCCBitstreamEncoder::geometryInformation( GeometryInformation&  gi,
                                               VpccParameterSet& sps,
                                               PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  bitstream.write( (uint32_t)gi.getGeometryCodecId(), 8 );                            // u(8)
  bitstream.write( (uint32_t)gi.getGeometryNominal2dBitdepthMinus1(), 5 );            // u(5)
  bitstream.write( (uint32_t)gi.getGeometryMSBAlignFlag(), 1 );                       // u(1)
  bitstream.write( ( uint32_t )( gi.getGeometry3dCoordinatesBitdepthMinus1() ), 5 );  // u(5)
  if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
    bitstream.write( (uint32_t)gi.getRawGeometryCodecId(), 8 );  // u(8)
  }
  // JR TODO:  Remove 
  bitstream.write( (uint32_t)gi.getGeometryParamsEnabledFlag(), 1 );       // u(1)
  bitstream.write( (uint32_t)gi.getGeometryPatchParamsEnabledFlag(), 1 );  // u(1)
  TRACE_BITSTREAM( "GeometryParamsEnabledFlag = %d \n", gi.getGeometryParamsEnabledFlag() );
  TRACE_BITSTREAM( "GeometryPatchParamsEnabledFlag = %d \n", gi.getGeometryPatchParamsEnabledFlag() );
}

// 7.3.4.5 Attribute information
void PCCBitstreamEncoder::attributeInformation( AttributeInformation& ai,
                                                VpccParameterSet& sps,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  bitstream.write( (uint32_t)ai.getAttributeCount(), 7 );  // u(7)
  TRACE_BITSTREAM( "AttributeCount = %u  \n", ai.getAttributeCount() );
  for ( uint32_t i = 0; i < ai.getAttributeCount(); i++ ) {
    bitstream.write( (uint32_t)ai.getAttributeTypeId( i ), 4 );   // u(4)
    bitstream.write( (uint32_t)ai.getAttributeCodecId( i ), 8 );  // u(8)
    if ( sps.getRawSeparateVideoPresentFlag(atlasIndex) ) {
      bitstream.write( (uint32_t)ai.getRawAttributeCodecId( i ), 8 );  // u(8) 
    }
		for(int32_t j=0; j< sps.getMapCountMinus1(atlasIndex);j++){
		  if( sps.getMapAbsoluteCodingEnableFlag( atlasIndex, j )==0) {
        bitstream.write( (uint32_t)ai.getAttributeMapAbsoluteCodingEnabledFlag( i ), 1 );  // u(1)
      }
    }
    bitstream.write( (uint32_t)ai.getAttributeDimensionMinus1( i ), 6 );  // u(6)

    if ( ai.getAttributeDimensionMinus1( i ) > 0 ) {
      bitstream.write( (uint32_t)ai.getAttributeDimensionPartitionsMinus1( i ), 6 );  // u(6) 
      int32_t remainingDimensions = ai.getAttributeDimensionMinus1( i );
      int32_t k                   = ai.getAttributeDimensionPartitionsMinus1( i );
      for ( int32_t j = 0; j < k; j++ ) {
        if ( k - j != remainingDimensions ) {
          bitstream.writeUvlc( (uint32_t)ai.getAttributePartitionChannelsMinus1( i, j ) );  // ue(v)
        }
        remainingDimensions -= ai.getAttributePartitionChannelsMinus1( i, j ) + 1;
      }
    }
    bitstream.write( (uint32_t)ai.getAttributeNominal2dBitdepthMinus1( i ), 5 );  // u(5)
  }
  if ( ai.getAttributeCount() > 0 ) {
    bitstream.write( (uint32_t)ai.getAttributeMSBAlignFlag(), 1 );            // u(1) 
    // bitstream.write( (uint32_t)ai.getAttributeParamsEnabledFlag(), 1 );       // u(1) TODO: remove?
    // bitstream.write( (uint32_t)ai.getAttributePatchParamsEnabledFlag(), 1 );  // u(1) TODO: remove?
  }
}


// 7.3.6.1 Atlas sequence parameter set RBSP
void PCCBitstreamEncoder::atlasSequenceParameterSetRBSP( AtlasSequenceParameterSetRBSP& asps,
                                                         PCCContext&                    context,
                                                         PCCBitstream&                  bitstream ) {
  bitstream.writeUvlc( (uint32_t)asps.getAltasSequenceParameterSetId() ) ;  // ue(v)     
  bitstream.write( (uint32_t)asps.getFrameWidth(), 16 );  // u(16)
  bitstream.write( (uint32_t)asps.getFrameHeight(), 16 );  // u(16)
  bitstream.write( (uint32_t) asps.getLog2PatchPackingBlockSize(),  3  );  // u(3)
  bitstream.writeUvlc( (uint32_t)asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() );  // ue(v)
  bitstream.writeUvlc( (uint32_t)asps.getMaxDecAtlasFrameBufferingMinus1() );  // ue(v)
  bitstream.write( (uint32_t)asps.getLongTermRefAtlasFramesFlag(),  1 );  // u(1)
  bitstream.writeUvlc( (uint32_t)asps.getNumRefAtlasFrameListsInAsps() );  // ue(v)  
  for ( size_t i = 0; i < asps.getNumRefAtlasFrameListsInAsps(); i++ ) {
    refListStruct( asps.getRefListStruct( i ), asps, bitstream );
  }
  bitstream.write( (uint32_t)asps.getLongTermRefAtlasFramesFlag(),  1 );  // u(1)
  bitstream.write( (uint32_t)asps.get45DegreeProjectionPatchPresentFlag(),  1 );  // u(1)
  bitstream.write( (uint32_t)asps.getNormalAxisLimitsQuantizationEnabledFlag(),  1 );  // u(1)
  bitstream.write( (uint32_t)asps.getNormalAxisMaxDeltaValueEnabledFlag(),  1 );  // u(1)
  bitstream.write( (uint32_t)asps.getRemoveDuplicatePointEnabledFlag(),  1 );  // u(1)
  bitstream.write( (uint32_t)asps.getPixelDeinterleavingFlag(),  1 );  // u(1)
  bitstream.write( (uint32_t)asps.getPatchPrecedenceOrderFlag(),  1 );  // u(1)
  bitstream.write( (uint32_t)asps.getPatchSizeQuantizerPresentFlag(),  1 );  // u(1)
  bitstream.write( (uint32_t)asps.getEnhancedOccupancyMapForDepthFlag(),  1 );  // u(1)
  bitstream.write( (uint32_t)asps.getPointLocalReconstructionEnabledFlag(),  1 );  // u(1)

  bitstream.write( (uint32_t)asps.getMapCountMinus1(),  4 );  // u(4)
  if( asps.getEnhancedOccupancyMapForDepthFlag() && asps.getMapCountMinus1() == 0 ){  
    bitstream.write( (uint32_t)asps.getEnhancedOccupancyMapFixBitCountMinus1(),  4 );  // u(4)
  }
  if( asps.getPointLocalReconstructionEnabledFlag() ){
    asps.allocatePointLocalReconstructionInformation();
    pointLocalReconstructionInformation( asps, context, bitstream ); 
  }
  if( asps.getPixelDeinterleavingFlag() || asps.getPointLocalReconstructionEnabledFlag() ){
    bitstream.write( (uint32_t)asps.getSurfaceThicknessMinus1(), 8 );  // u(8)
  }
   bitstream.write( (uint32_t)asps.getVuiParametersPresentFlag(), 1 );  // u(1)
  if( asps.getVuiParametersPresentFlag() ){
    vuiParameters(); // TODO
  }
  bitstream.write( (uint32_t)asps.getExtensionPresentFlag(), 1 );  // u(1)
  if( asps.getExtensionPresentFlag() ){
    while( moreRbspData() ){
      bitstream.write( (uint32_t)asps.getExtensionPresentFlag(), 1 );  // u(1)    
    }
  }
  rbspRailingBits();
}

// 7.3.4.6 Point local reconstruction information syntax (OLD)
void PCCBitstreamEncoder::pointLocalReconstructionInformation( PointLocalReconstructionInformation& plri,
                                                               PCCContext&                          context,
                                                               PCCBitstream&                        bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( uint32_t( plri.getNumberOfModesMinus1() ), 4 );  // u(4)
  TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plri.getNumberOfModesMinus1() );
  for ( size_t i = 0; i < plri.getNumberOfModesMinus1(); i++ ) {
    bitstream.write( uint32_t( plri.getInterpolateFlag( i ) ), 1 );  // u(1)
    bitstream.write( uint32_t( plri.getFillingFlag( i ) ), 1 );      // u(1)
    bitstream.write( uint32_t( plri.getMinimumDepth( i ) ), 2 );     // u(2)
    bitstream.write( uint32_t( plri.getNeighbourMinus1( i ) ), 2 );  // u(2)
    TRACE_BITSTREAM( "  Mode[%lu] = I = %d F = %d D = %u N = %u \n", i, plri.getInterpolateFlag( i ),
                     plri.getFillingFlag( i ), plri.getMinimumDepth( i ), plri.getNeighbourMinus1( i ) );
  }
  bitstream.writeUvlc( uint32_t( plri.getBlockThresholdPerPatchMinus1() ) );  // ue(v)
  TRACE_BITSTREAM( "  BlockThresholdPerPatchMinus1 = %u \n", plri.getBlockThresholdPerPatchMinus1() );
}

// 7.3.4.6 Point local reconstruction information syntax (NEW)
void PCCBitstreamEncoder::pointLocalReconstructionInformation( AtlasSequenceParameterSetRBSP& asps,
                                                               PCCContext&                    context,
                                                               PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );  
  for( size_t j = 0; j < asps.getMapCountMinus1() + 1 ; j ++ ) { 
    auto& plri = asps.getPointLocalReconstructionInformation( j );
    bitstream.write( (uint32_t) plri.getMapEnabledFlag(), 1  );  // u(1)
    if( plri.getMapEnabledFlag() ) {
      bitstream.write( uint32_t( plri.getNumberOfModesMinus1() ), 4 );  // u(4)
      TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plri.getNumberOfModesMinus1() );
      for ( size_t i = 0; i < plri.getNumberOfModesMinus1(); i++ ) {
        bitstream.write( (uint32_t) plri.getInterpolateFlag( i ), 1 );  // u(1)
        bitstream.write( (uint32_t) plri.getFillingFlag( i ), 1 );      // u(1)
        bitstream.write( (uint32_t) plri.getMinimumDepth( i ), 2 );     // u(2)
        bitstream.write( (uint32_t) plri.getNeighbourMinus1( i ), 2 );  // u(2)
        TRACE_BITSTREAM( "  Mode[%lu] = I = %d F = %d D = %u N = %u \n", i, plri.getInterpolateFlag( i ),
                        plri.getFillingFlag( i ), plri.getMinimumDepth( i ), plri.getNeighbourMinus1( i ) );
      }
      bitstream.write( (uint32_t) plri.getBlockThresholdPerPatchMinus1(), 6 );  // u(6)
      TRACE_BITSTREAM( "  BlockThresholdPerPatchMinus1 = %u \n", plri.getBlockThresholdPerPatchMinus1() );
    }
  }
}

// 7.3.5.1 General patch data group unit syntax
void PCCBitstreamEncoder::atlasSubStream( PCCContext& context, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& pdg = context.getPatchDataGroup();
  for ( uint8_t unitType = (uint8_t)PDG_PSPS; unitType < (uint8_t)PDG_PTGLU; unitType++ ) {
    bitstream.writeUvlc( ( (uint32_t)unitType ) );  // ue(v)
    TRACE_BITSTREAM( "%s (%u): frame = %u \n", strUnitType( (PDGUnitType)unitType ).c_str(), (uint32_t)unitType, 0 );
    atlasSubStreamUnitPayload( pdg, (PDGUnitType)unitType, 0, 0, context, bitstream );
    bitstream.write( (uint32_t)0, 1 );  // u(1)
  }
  for ( uint8_t i = 0; i < pdg.getPatchTileGroupLayerUnitSize(); i++ ) {
    bitstream.writeUvlc( ( (uint32_t)PDG_PTGLU ) );  // ue(v)
    TRACE_BITSTREAM( "%s (%u): frame = %u \n", strUnitType( PDG_PTGLU ).c_str(), (uint32_t)PDG_PTGLU, i );
    atlasSubStreamUnitPayload( pdg, PDG_PTGLU, i, i, context, bitstream );
    bitstream.write( ( uint32_t )( i + 1 == pdg.getPatchTileGroupLayerUnitSize() ), 1 );  // u(1)
  }
  byteAlignment( bitstream );
}

// 7.3.5.2 Patch data group unit payload syntax
void PCCBitstreamEncoder::atlasSubStreamUnitPayload( PatchDataGroup& pdg,
                                                     PDGUnitType     unitType,
                                                     size_t          index,
                                                     size_t          frameIndex,
                                                     PCCContext&     context,
                                                     PCCBitstream&   bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "  type = %u frameIndex = %u \n", (uint8_t)unitType, frameIndex );
  auto& sps = context.getSps();
  switch ( unitType ) {
    case PDG_PSPS: patchVpccParameterSet( pdg, index, bitstream ); break;
    case PDG_GPPS: geometryPatchParameterSet( pdg, index, bitstream ); break;
    case PDG_APPS: attributePatchParameterSet( pdg, index, sps, bitstream ); break;
    case PDG_PFPS: patchFrameParameterSet( pdg, index, sps, bitstream ); break;
    case PDG_PFAPS: patchFrameAttributeParameterSet( pdg, index, sps, bitstream ); break;
    case PDG_PFGPS: patchFrameGeometryParameterSet( pdg, index, sps, bitstream ); break;
    case PDG_PTGLU: patchTileGroupLayerUnit( pdg, index, context, bitstream ); break;
    case PDG_PREFIX_SEI: seiMessage( pdg, index, context, bitstream ); break;
    case PDG_SUFFIX_SEI: seiMessage( pdg, index, context, bitstream ); break;
    default: assert( 0 ); break;
  }
}

// 7.3.5.3 Patch sequence parameter set syntax
void PCCBitstreamEncoder::patchVpccParameterSet( PatchDataGroup& pdg, size_t index, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& psps = pdg.getPatchVpccParameterSet( index );
  assert( psps.getPatchVpccParameterSetId() == index );
  bitstream.writeUvlc( psps.getPatchVpccParameterSetId() );         // ue(v)
  bitstream.write( psps.getLog2PatchPackingBlockSize(), 3 );            // u(3)
  bitstream.writeUvlc( psps.getLog2MaxPatchFrameOrderCntLsbMinus4() );  // ue(v)
  bitstream.writeUvlc( psps.getMaxDecPatchFrameBufferingMinus1() );     // ue(v)
  bitstream.write( psps.getLongTermRefPatchFramesFlag(), 1 );           // u(1)
  bitstream.writeUvlc( psps.getNumRefPatchFrameListsInPsps() );         // ue(v)
  for ( size_t i = 0; i < psps.getNumRefPatchFrameListsInPsps(); i++ ) {
    refListStruct( psps.getRefListStruct( i ), psps, bitstream );
  }
  bitstream.write( psps.getUseEightOrientationsFlag(), 1 );                // u(1)
  bitstream.write( psps.getNormalAxisLimitsQuantizationEnableFlag(), 1 );  // u(1)
  bitstream.write( psps.getNormalAxisMaxDeltaValueEnableFlag(), 1 );       // u(1)
}

// 7.3.5.4 Patch frame geometry parameter set syntax
void PCCBitstreamEncoder::patchFrameGeometryParameterSet( PatchDataGroup&       pdg,
                                                          size_t                index,
                                                          VpccParameterSet& sps,
                                                          PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  auto& gi    = sps.getGeometryInformation( atlasIndex );
  auto& pfgps = pdg.getPatchFrameGeometryParameterSet( index );
  assert( pfgps.getPatchFrameGeometryParameterSetId() == index );
  bitstream.writeUvlc( pfgps.getPatchFrameGeometryParameterSetId() );  // ue(v)
  bitstream.writeUvlc( pfgps.getPatchVpccParameterSetId() );       // ue(v)
  TRACE_BITSTREAM( "GeometryParamsEnabledFlag = %d \n", gi.getGeometryParamsEnabledFlag() );
  if ( gi.getGeometryParamsEnabledFlag() ) { geometryFrameParams( pfgps.getGeometryFrameParams(), bitstream ); }
  TRACE_BITSTREAM( "GeometryPatchParamsEnabledFlag = %d \n", gi.getGeometryPatchParamsEnabledFlag() );
  if ( gi.getGeometryPatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)pfgps.getGeometryPatchScaleParamsEnabledFlag(), 1 );     // u(1)
    bitstream.write( (uint32_t)pfgps.getGeometryPatchOffsetParamsEnabledFlag(), 1 );    // u(1)
    bitstream.write( (uint32_t)pfgps.getGeometryPatchRotationParamsEnabledFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)pfgps.getGeometryPatchPointSizeInfoEnabledFlag(), 1 );   // u(1)
    bitstream.write( (uint32_t)pfgps.getGeometryPatchPointShapeInfoEnabledFlag(), 1 );  // u(1)
  }
  byteAlignment( bitstream );
}

// 7.3.5.5 Geometry frame Params syntax
void PCCBitstreamEncoder::geometryFrameParams( GeometryFrameParams& gfp, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)gfp.getGeometrySmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryOffsetParamsPresentFlag(), 1 );     // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryRotationParamsPresentFlag(), 1 );   // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryPointSizeInfoPresentFlag(), 1 );    // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryPointShapeInfoPresentFlag(), 1 );   // u(1)
  if ( gfp.getGeometrySmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometrySmoothingEnabledFlag(), 1 );  // u(1)
    if ( gfp.getGeometrySmoothingEnabledFlag() ) {
      bitstream.write( (uint32_t)gfp.getGeometrySmoothingGridSizeMinus2(), 7 );  // u(7)
      bitstream.write( (uint32_t)gfp.getGeometrySmoothingThreshold(), 8 );       // u(8)
    }
    bitstream.write( (uint32_t)gfp.getGeometryPatchBlockFilteringEnableFlag(), 1 );  // u(1)
    if ( gfp.getGeometryPatchBlockFilteringEnableFlag() ) {
      bitstream.write( (uint32_t)gfp.getGeometryPatchBlockFilteringLog2ThresholdMinus1(), 2 );  // u(2)
      bitstream.write( (uint32_t)gfp.getGeometryPatchBlockFilteringPassesCountMinus1(), 2 );    // u(3)
      bitstream.write( (uint32_t)gfp.getGeometryPatchBlockFilteringFilterSizeMinus1(), 3 );     // u(3)
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
    for ( size_t d = 0; d < 4; d++ ) {
      bitstream.writeS( gfp.getGeometryRotationQuaternion( d ),
                        32 );  // i32 -> TODO: This should be i(16) according to the CD
    }
  }
  if ( gfp.getGeometryPointSizeInfoPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometryPointSizeInfo(), 16 );  // u(16)
  }
  if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometryPointShapeInfo(), 4 );  // u(4)
  }
}

// 7.3.5.6 Patch frame attribute parameter set syntax
void PCCBitstreamEncoder::patchFrameAttributeParameterSet( PatchDataGroup&       pdg,
                                                           size_t                index,
                                                           VpccParameterSet& sps,
                                                           PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  auto& ai    = sps.getAttributeInformation(atlasIndex);
  auto& pfaps = pdg.getPatchFrameAttributeParameterSet( index );
  assert( pfaps.getPatchFrameAttributeParameterSetId() == index );
  bitstream.writeUvlc( pfaps.getPatchFrameAttributeParameterSetId() );  // ue(v)
  bitstream.writeUvlc( pfaps.getPatchSequencParameterSetId() );         //  ue(v)

  TRACE_BITSTREAM( "PatchFrameAttributeParameterSetId = %u  \n", pfaps.getPatchFrameAttributeParameterSetId() );
  TRACE_BITSTREAM( "PatchSequencParameterSetId       = %u  \n", pfaps.getPatchSequencParameterSetId() );

  size_t attributeDimension = ai.getAttributeDimensionMinus1( pfaps.getPatchFrameAttributeParameterSetId() ) + 1;
  TRACE_BITSTREAM( "attributeDimension = %lu \n", attributeDimension );
  if ( ai.getAttributeParamsEnabledFlag() ) {
    attributeFrameParams( pfaps.getAttributeFrameParams(), attributeDimension, bitstream );
  }
  if ( ai.getAttributePatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)pfaps.getAttributePatchScaleParamsEnabledFlag(), 1 );   //  u(1)
    bitstream.write( (uint32_t)pfaps.getAttributePatchOffsetParamsEnabledFlag(), 1 );  // u(1)
  }
  byteAlignment( bitstream );
}

// 7.3.5.7 Attribute frame Params syntax
void PCCBitstreamEncoder::attributeFrameParams( AttributeFrameParams& afp,
                                                size_t                attributeDimension,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  for ( size_t i = 0; i < attributeDimension; i++ ) {
    bitstream.write( (uint32_t)afp.getAttributeSmoothingParamsPresentFlag( i ), 1 );  // u(1)
  }
  bitstream.write( (uint32_t)afp.getAttributeScaleParamsPresentFlag(), 1 );   // u(1)
  bitstream.write( (uint32_t)afp.getAttributeOffsetParamsPresentFlag(), 1 );  // u(1)
  for ( size_t i = 0; i < attributeDimension; i++ ) {
    if ( afp.getAttributeSmoothingParamsPresentFlag( i ) ) {
      bitstream.write( (uint32_t)afp.getAttributeSmoothingGridSizeMinus2( i ), 8 );                // u(8)
      bitstream.write( (uint32_t)afp.getAttributeSmoothingThreshold( i ), 8 );                     // u(8)
      bitstream.write( (uint32_t)afp.getAttributeSmoothingLocalEntropyThreshold( i ), 3 );         // u(3)
      bitstream.write( (uint32_t)afp.getAttributeSmoothingThresholdAttributeVariation( i ), 8 );   // u(8)
      bitstream.write( (uint32_t)afp.getAttributeSmoothingThresholdAttributeDifference( i ), 8 );  // u(8)
    }
  }
  if ( afp.getAttributeScaleParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ ) {
      bitstream.write( (uint32_t)afp.getAttributeScale( i ), 32 );  // u(32)
    }
  }
  if ( afp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ ) bitstream.writeS( afp.getAttributeOffset( i ), 32 );  // i32
  }
}

// 7.3.5.8 Geometry patch parameter set syntax
void PCCBitstreamEncoder::geometryPatchParameterSet( PatchDataGroup& pdg, size_t index, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& gpps  = pdg.getGeometryPatchParameterSet( index );
  auto& pfgps = pdg.getPatchFrameGeometryParameterSet( gpps.getPatchFrameGeometryParameterSetId() );
  assert( gpps.getGeometryPatchParameterSetId() == index );
  bitstream.writeUvlc( gpps.getGeometryPatchParameterSetId() );       // ue(v)
  bitstream.writeUvlc( gpps.getPatchFrameGeometryParameterSetId() );  //  ue(v)
  if ( pfgps.getGeometryPatchScaleParamsEnabledFlag() || pfgps.getGeometryPatchOffsetParamsEnabledFlag() ||
       pfgps.getGeometryPatchRotationParamsEnabledFlag() || pfgps.getGeometryPatchPointSizeInfoEnabledFlag() ||
       pfgps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpps.getGeometryPatchParamsPresentFlag(), 1 );  // u(1)
    if ( gpps.getGeometryPatchParamsPresentFlag() )
      geometryPatchParams( gpps.getGeometryPatchParams(), pfgps, bitstream );
  }
  byteAlignment( bitstream );
}

// 7.3.5.9 Geometry patch Params syntax
void PCCBitstreamEncoder::geometryPatchParams( GeometryPatchParams&            gpp,
                                               PatchFrameGeometryParameterSet& gfps,
                                               PCCBitstream&                   bitstream ) {
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
      for ( size_t d = 0; d < 4; d++ ) {
        bitstream.writeS( gpp.getGeometryPatchRotationQuaternion( d ),
                          32 );  // i(32) -> TODO: i(16) according to the CD
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

// 7.3.5.10 Attribute patch parameter set syntax
void PCCBitstreamEncoder::attributePatchParameterSet( PatchDataGroup&       pdg,
                                                      size_t                index,
                                                      VpccParameterSet& sps,
                                                      PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  auto& apps  = pdg.getAttributePatchParameterSet( index );
  auto& pfaps = pdg.getPatchFrameAttributeParameterSet( apps.getPatchFrameAttributeParameterSetId() );
  assert( apps.getAttributePatchParameterSetId() == index );
  bitstream.writeUvlc( apps.getAttributePatchParameterSetId() );
  bitstream.writeUvlc( apps.getPatchFrameAttributeParameterSetId() );
  bitstream.write( apps.getAttributeDimensionMinus1(), 8 );
  size_t attributeDimension = apps.getAttributeDimensionMinus1() + 1;
  if ( pfaps.getAttributePatchScaleParamsEnabledFlag() || pfaps.getAttributePatchOffsetParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)apps.getAttributePatchParamsPresentFlag(), 1 );  // u(1)
    if ( apps.getAttributePatchParamsPresentFlag() ) {
      attributePatchParams( apps.getAttributePatchParams(), pfaps, attributeDimension, bitstream );
    }
  }
  byteAlignment( bitstream );
}

// 7.3.5.11 Attribute patch Params syntax (apps)
void PCCBitstreamEncoder::attributePatchParams( AttributePatchParams&            app,
                                                PatchFrameAttributeParameterSet& afps,
                                                size_t                           dimension,
                                                PCCBitstream&                    bitstream ) {
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

// 7.3.5.12 Patch frame parameter set syntax
void PCCBitstreamEncoder::patchFrameParameterSet( PatchDataGroup&       pdg,
                                                  size_t                index,
                                                  VpccParameterSet& sps,
                                                  PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  auto& ai   = sps.getAttributeInformation(atlasIndex);
  auto& pfps = pdg.getPatchFrameParameterSet( index );
  assert( pfps.getPatchFrameParameterSetId() == index );
  bitstream.writeUvlc( pfps.getPatchFrameParameterSetId() );          // ue(v)
  bitstream.writeUvlc( pfps.getPatchVpccParameterSetId() );       // ue(v)
  bitstream.writeUvlc( pfps.getGeometryPatchFrameParameterSetId() );  // ue(v)

  TRACE_BITSTREAM( " ai.getAttributeCount() = %u \n", ai.getAttributeCount() );

  for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
    bitstream.writeUvlc( pfps.getAttributePatchFrameParameterSetId( i ) );  // ue(v)
  }
  atlasFrameTileInformation( pfps.getAtlasFrameTileInformation(), sps, bitstream );

  bitstream.write( pfps.getLocalOverrideGeometryPatchEnableFlag(), 1 );  // u(1)
  for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
    bitstream.write( pfps.getLocalOverrideAttributePatchEnableFlag( i ), 1 );  // u(1)
  }
  bitstream.writeUvlc( pfps.getAdditionalLtPfocLsbLen() );  // ue(v)
  if ( sps.getProjection45DegreeEnableFlag() ) {
    bitstream.write( pfps.getProjection45DegreeEnableFlag(), 1 );  // u(1)
  }
  bitstream.write( pfps.getLodModeEnableFlag(), 1 );  // u(1)
  byteAlignment( bitstream );
}

void PCCBitstreamEncoder::atlasFrameTileInformation(
                                                    AtlasFrameTileInformation& pfti,
                                                     VpccParameterSet&      sps,
                                                     PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( pfti.getSingleTileInAtlasFrameFlag(), 1 );  // u(1)
  if ( !pfti.getSingleTileInAtlasFrameFlag() ) {
    bitstream.write( pfti.getUniformTileSpacingFlag(), 1 );  // u(1)
    if ( pfti.getUniformTileSpacingFlag() ) {
      bitstream.writeUvlc( pfti.getTileColumnWidthMinus1( 0 ) );  //  ue(v)
      bitstream.writeUvlc( pfti.getTileRowHeightMinus1( 0 ) );    //  ue(v)
    } else {
      bitstream.writeUvlc( pfti.getNumTileColumnsMinus1() );  //  ue(v)
      bitstream.writeUvlc( pfti.getNumTileRowsMinus1() );     //  ue(v)

      for ( size_t i = 0; i < pfti.getNumTileColumnsMinus1(); i++ ) {
        bitstream.writeUvlc( pfti.getTileColumnWidthMinus1( i ) );  //  ue(v)
      }
      for ( size_t i = 0; i < pfti.getNumTileRowsMinus1(); i++ ) {
        bitstream.writeUvlc( pfti.getTileRowHeightMinus1( i ) );  //  ue(v)
      }
    }
  }
  bitstream.write( pfti.getSingleTilePerTileGroupFlag(), 1 );  //  u(1)
  if ( !pfti.getSingleTilePerTileGroupFlag() ) {
    uint32_t NumTilesInPatchFrame     = ( pfti.getNumTileColumnsMinus1() + 1 ) * ( pfti.getNumTileRowsMinus1() + 1 );
    bitstream.writeUvlc( pfti.getNumTileGroupsInAtlasFrameMinus1() );  // ue(v)
    for ( size_t i = 0; i <= pfti.getNumTileGroupsInAtlasFrameMinus1(); i++ ) {
      uint8_t bitCount = getFixedLengthCodeBitsCount( NumTilesInPatchFrame + 1 );
      if ( i > 0 ) {
        bitstream.write( pfti.getTopLeftTileIdx( i ), bitCount );
      }  // u(v) : Ceil( Log2( NumTilesInPatchFrame )
      bitCount = getFixedLengthCodeBitsCount( NumTilesInPatchFrame - pfti.getTopLeftTileIdx( i ) + 1 );
      bitstream.write( pfti.getBottomRightTileIdxDelta( i ),
                       bitCount );  // u(v) : Ceil( Log2( NumTilesInPatchFrame − pfti_top_left_tile_idx[ i ] ) )
    }
  }
  bitstream.write( pfti.getSignalledTileGroupIdFlag(), 1 );  // u(1)
  if ( pfti.getSignalledTileGroupIdFlag() ) {
    bitstream.writeUvlc( pfti.getSignalledTileGroupIdLengthMinus1() );  // ue(v)
    for ( size_t i = 0; i <= pfti.getSignalledTileGroupIdLengthMinus1(); i++ ) {
      uint8_t bitCount = pfti.getSignalledTileGroupIdLengthMinus1() + 1;
      bitstream.write( pfti.getTileGroupId( i ), bitCount );  // pfti_signalled_tile_group_id_length_minus1 + 1  bits
    }
  }
}

// 7.3.5.14 Patch tile group layer unit syntax
void PCCBitstreamEncoder::patchTileGroupLayerUnit( PatchDataGroup& pdg,
                                                   size_t          index,
                                                   PCCContext&     context,
                                                   PCCBitstream&   bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& ptglu    = pdg.getPatchTileGroupLayerUnit( index );
  auto& pfluPrev = pdg.getPatchTileGroupLayerUnit( ( std::max )( 0, (int32_t)index - 1 ) );
  ptglu.getPatchTileGroupHeader().setFrameIndex( ptglu.getFrameIndex() );
  ptglu.getPatchTileGroupDataUnit().setFrameIndex( ptglu.getFrameIndex() );
  patchTileGroupHeader( ptglu.getPatchTileGroupHeader(), pfluPrev.getPatchTileGroupHeader(), context, bitstream );
  patchTileGroupDataUnit( ptglu.getPatchTileGroupDataUnit(), ptglu.getPatchTileGroupHeader(), context, bitstream );
}

// 7.3.5.15 Patch tile group header syntax
void PCCBitstreamEncoder::patchTileGroupHeader( PatchTileGroupHeader& ptgh,
                                                PatchTileGroupHeader& pfhPrev,
                                                PCCContext&           context,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& pdg  = context.getPatchDataGroup();
  size_t atlasIndex = 0;
  auto& pfps = pdg.getPatchFrameParameterSet( ptgh.getPatchFrameParameterSetId() );
  auto& psps = pdg.getPatchVpccParameterSet( pfps.getPatchVpccParameterSetId() );
  auto& gi   = context.getSps().getGeometryInformation(atlasIndex);
  auto& sps  = context.getSps();

  bitstream.writeUvlc( ptgh.getPatchFrameParameterSetId() );  // ue(v )
  auto&   pfti     = pfps.getAtlasFrameTileInformation();
  uint8_t bitCount = pfti.getSignalledTileGroupIdLengthMinus1() + 1;
  bitstream.write( ptgh.getAddress(), bitCount );  // u(v)
  // The length of ptgh_address is pfti_signalled_tile_group_id_length_minus1 + 1 bits.
  // If pfti_signalled_tile_group_id_flag is equal to 0, the value of ptgh_address shall be in the range of 0 to
  // pfti_num_tile_groups_in_patch_frame_minus1, inclusive. Otherwise, the value of ptgh_address shall be in the range
  // of 0 to 2( pfti_signalled_tile_group_id_length_minus1 + 1 ) − 1, inclusive.
  bitstream.writeUvlc( ptgh.getType() );  // ue( v )
  bitCount = psps.getLog2MaxPatchFrameOrderCntLsbMinus4() + 4;
  bitstream.write( ptgh.getPatchFrameOrderCntLsb(), bitCount );  // u( v )
  // The length of the ptgh_patch_frm_order_cnt_lsb syntax element is equal to
  // psps_log2_max_patch_frame_order_cnt_lsb_minus4 + 4 bits. The value of the ptgh_patch_frm_order_cnt_lsb shall be in
  // the range of 0 to MaxPatchFrmOrderCntLsb − 1, inclusive.

  TRACE_BITSTREAM( "Id     = %u \n", ptgh.getPatchFrameParameterSetId() );
  TRACE_BITSTREAM( "Adress = %u \n", ptgh.getAddress() );
  TRACE_BITSTREAM( "Type   = %u \n", ptgh.getType() );
  TRACE_BITSTREAM( "POC    = %u \n", ptgh.getPatchFrameOrderCntLsb() );
  TRACE_BITSTREAM( "psps.getNumRefPatchFrameListsInPsps() = %lu\n", psps.getNumRefPatchFrameListsInPsps() );
  TRACE_BITSTREAM( "psps.getNumRefPatchFrameListsInPsps() = %lu \n", psps.getNumRefPatchFrameListsInPsps() );
  TRACE_BITSTREAM( "gi.getGeometry3dCoordinatesBitdepthMinus1() = %lu \n",
                   gi.getGeometry3dCoordinatesBitdepthMinus1() );

  if ( psps.getNumRefPatchFrameListsInPsps() > 0 ) {
    bitstream.write( (uint32_t)ptgh.getRefPatchFrameListSpsFlag(), 1 );  // u( 1 )
  }
  if ( ptgh.getRefPatchFrameListSpsFlag() ) {
    if ( psps.getNumRefPatchFrameListsInPsps() > 1 ) {
      bitCount = getFixedLengthCodeBitsCount( psps.getNumRefPatchFrameListsInPsps() + 1 );
      bitstream.write( ptgh.getRefPatchFrameListIdx(),
                       bitCount );  // u( v ) :Ceil( Log2( psps_num_ref_patch_frame_lists_in_psps ) )
    }
  } else {
    refListStruct( psps.getRefListStruct( psps.getNumRefPatchFrameListsInPsps() ), psps, bitstream );
  }
  uint8_t rlsIdx =
      psps.getNumRefPatchFrameListsInPsps() ? ptgh.getRefPatchFrameListIdx() : psps.getNumRefPatchFrameListsInPsps();
  size_t numLtrpEntries = 0;
  for ( size_t i = 0; i < psps.getRefListStruct( rlsIdx ).getNumRefEntries(); i++ ) {
    if ( !psps.getRefListStruct( rlsIdx ).getStRefAtalsFrameFlag( i ) ) { numLtrpEntries++; }
  }

  for ( size_t j = 0; j < numLtrpEntries; j++ ) {                               // numLtrpEntries=numLtrPatchFrmEntries
    bitstream.write( (uint32_t)ptgh.getAdditionalPfocLsbPresentFlag( j ), 1 );  // u( 1 )
    if ( ptgh.getAdditionalPfocLsbPresentFlag( j ) ) {
      uint8_t bitCount = pfps.getAdditionalLtPfocLsbLen();
      bitstream.write( ptgh.getAdditionalPfocLsbVal( j ), bitCount );  // u( v ) pfps_additional_lt_pfoc_lsb_len
    }
  }

  ptgh.setNormalAxisMinValueQuantizer( 0 );
  ptgh.setNormalAxisMaxDeltaValueQuantizer( 0 );
  if ( psps.getNormalAxisLimitsQuantizationEnableFlag() ) {
    ptgh.setNormalAxisMinValueQuantizer( gbitCountSize[context.getSps().getMinLevel()] );
    ptgh.setNormalAxisMaxDeltaValueQuantizer( gbitCountSize[context.getSps().getMinLevel()] );

    bitstream.write( ptgh.getNormalAxisMinValueQuantizer(), 5 );  // u(5)
    if ( psps.getNormalAxisMaxDeltaValueEnableFlag() ) {
      bitstream.write( ptgh.getNormalAxisMaxDeltaValueQuantizer(), 5 );  // u(5)
    }
  }
  const uint8_t maxBitCountForMinDepth = uint8_t( gi.getGeometry3dCoordinatesBitdepthMinus1() );
  const uint8_t maxBitCountForMaxDepth = uint8_t( gi.getGeometry3dCoordinatesBitdepthMinus1() );
  // min
  ptgh.setInterPredictPatch3dShiftNormalAxisBitCountMinus1( maxBitCountForMinDepth );

  // max
  if ( pfps.getProjection45DegreeEnableFlag() == 0 ) {
    ptgh.setInterPredictPatch2dDeltaSizeDBitCountMinus1( maxBitCountForMaxDepth );
  } else {
    ptgh.setInterPredictPatch2dDeltaSizeDBitCountMinus1( maxBitCountForMaxDepth + 1 );
  }

  if ( ptgh.getType() == PATCH_FRAME_P && psps.getRefListStruct( rlsIdx ).getNumRefEntries() > 1 ) {
    bitstream.write( (uint32_t)ptgh.getNumRefIdxActiveOverrideFlag(), 1 );  // u( 1 )
    if ( ptgh.getNumRefIdxActiveOverrideFlag() ) {
      bitstream.writeUvlc( ptgh.getNumRefIdxActiveMinus1() );  // ue(v)
    }
  }
  if ( ptgh.getType() == PATCH_FRAME_I ) {
    bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftUBitCountMinus1(), 8 );              // u( 8 )
    bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftVBitCountMinus1(), 8 );              // u( 8 )
    bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(), 8 );    // u( 8 )
    bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), 8 );  // u( 8 )
    bitstream.write( (uint32_t)ptgh.getInterPredictPatchLodBitCount(), 8 );                         // u( 8 )
  } else {
    bool countFlag[6] = {0, 0, 0, 0, 0, 0};
    bool countSumFlag = 0;
    countFlag[0] =
        ptgh.getInterPredictPatch2dShiftUBitCountMinus1() > pfhPrev.getInterPredictPatch2dShiftUBitCountMinus1();
    countFlag[1] =
        ptgh.getInterPredictPatch2dShiftVBitCountMinus1() > pfhPrev.getInterPredictPatch2dShiftVBitCountMinus1();
    countFlag[2] = ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() >
                   pfhPrev.getInterPredictPatch3dShiftTangentAxisBitCountMinus1();
    countFlag[3] = ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() >
                   pfhPrev.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1();
    countFlag[4] = ptgh.getInterPredictPatchLodBitCount() > pfhPrev.getInterPredictPatchLodBitCount();
    for ( size_t i = 0; i < 5; i++ ) countSumFlag |= countFlag[i];
    ptgh.setInterPredictPatchBitCountFlag( countSumFlag );
    ptgh.setInterPredictPatch2dShiftUBitCountFlag( countFlag[0] );
    ptgh.setInterPredictPatch2dShiftVBitCountFlag( countFlag[1] );
    ptgh.setInterPredictPatch3dShiftTangentAxisBitCountFlag( countFlag[2] );
    ptgh.setInterPredictPatch3dShiftBitangentAxisBitCountFlag( countFlag[3] );
    ptgh.setInterPredictPatchLodBitCountFlag( countFlag[4] );
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatch2dShiftUBitCountFlag() ) {
      ptgh.setInterPredictPatch2dShiftUBitCountMinus1( pfhPrev.getInterPredictPatch2dShiftUBitCountMinus1() );
    }
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatch2dShiftVBitCountFlag() ) {
      ptgh.setInterPredictPatch2dShiftVBitCountMinus1( pfhPrev.getInterPredictPatch2dShiftVBitCountMinus1() );
    }
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatch3dShiftTangentAxisBitCountFlag() ) {
      ptgh.setInterPredictPatch3dShiftTangentAxisBitCountMinus1(
          pfhPrev.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() );
    }
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag() ) {
      ptgh.setInterPredictPatch3dShiftBitangentAxisBitCountMinus1(
          pfhPrev.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() );
    }
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatchLodBitCountFlag() ) {
      ptgh.setInterPredictPatchLodBitCount( pfhPrev.getInterPredictPatchLodBitCount() );
    }
    bitstream.write( (uint32_t)ptgh.getInterPredictPatchBitCountFlag(), 1 );  // u( 1 )
    if ( ptgh.getInterPredictPatchBitCountFlag() ) {
      bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftUBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatch2dShiftUBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftUBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftVBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatch2dShiftVBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftVBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftTangentAxisBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatch3dShiftTangentAxisBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), 8 );  // u( 8 )
      }

      bitstream.write( (uint32_t)ptgh.getInterPredictPatchLodBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatchLodBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatchLodBitCount(), 8 );  // u( 8 )
      }
    }
  }
  if ( sps.getEnhancedOccupancyMapForDepthFlag() && sps.getEOMTexturePatch() ) {
    bitstream.write( (uint32_t)ptgh.getEOMPatchNbPatchBitCountMinus1(), 8 );  // u( 8 )
    bitstream.write( (uint32_t)ptgh.getEOMPatchMaxEPBitCountMinus1(), 8 );    // u( 8 )
  }
  TRACE_BITSTREAM("RawPatchEnabledFlag = %d \n", sps.getRawPatchEnabledFlag(atlasIndex) );
  if ( sps.getRawPatchEnabledFlag(atlasIndex) ) {
    bitstream.write( (uint32_t)ptgh.getRaw3dShiftBitCountPresentFlag(), 1 );  // u( 1 )
    if ( ptgh.getRaw3dShiftBitCountPresentFlag() ) {
      bitstream.write( (uint32_t)ptgh.getRaw3dShiftAxisBitCountMinus1(),
                       gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
    }
  } else {
    size_t bitCountPcmU1V1D1 = gi.getGeometry3dCoordinatesBitdepthMinus1() - gi.getGeometryNominal2dBitdepthMinus1();
    ptgh.setRaw3dShiftAxisBitCountMinus1( bitCountPcmU1V1D1 - 1 );
  }
  TRACE_BITSTREAM(
      "InterPredictPatchBitCount Flag %d %d %d %d %d %d %d Count = %u %u %u %u %u %u \n",
      ptgh.getInterPredictPatchBitCountFlag(), ptgh.getInterPredictPatch2dShiftUBitCountFlag(),
      ptgh.getInterPredictPatch2dShiftVBitCountFlag(), ptgh.getInterPredictPatch3dShiftTangentAxisBitCountFlag(),
      ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag(), ptgh.getInterPredictPatchLodBitCountFlag(),
      ptgh.getRaw3dShiftBitCountPresentFlag(), ptgh.getInterPredictPatch2dShiftUBitCountMinus1(),
      ptgh.getInterPredictPatch2dShiftVBitCountMinus1(), ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(),
      ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), ptgh.getInterPredictPatchLodBitCount(),
      ptgh.getRaw3dShiftAxisBitCountMinus1() );
  byteAlignment( bitstream );
}

// 7.3.5.16 Reference list structure syntax (OLD)
void PCCBitstreamEncoder::refListStruct( RefListStruct&             rls,
                                         PatchVpccParameterSet& psps,
                                         PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( rls.getNumRefEntries() );  // ue(v)
  TRACE_BITSTREAM( "NumRefEntries = %lu  \n", rls.getNumRefEntries() );
  rls.allocate();
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( psps.getLongTermRefPatchFramesFlag() ) {
      bitstream.write( rls.getStRefAtalsFrameFlag( i ), 1 );  // u(1)
      if ( rls.getStRefAtalsFrameFlag( i ) ) {
        bitstream.writeUvlc( rls.getAbsDeltaPfocSt( i ) );  // ue(v)
        if ( rls.getAbsDeltaPfocSt( i ) > 0 ) {
          bitstream.write( rls.getStrpfEntrySignFlag( i ), 1 );  // u(1)
        } else {
          uint8_t bitCount = psps.getLog2MaxPatchFrameOrderCntLsbMinus4() + 4;
          bitstream.write( rls.getAfocLsbLt( i ),
                           bitCount );  // u(v) psps_log2_max_patch_frame_order_cnt_lsb_minus4  + 4 bits
        }
      }
    }
  }
  TRACE_BITSTREAM( "%s done \n", __func__ );
}
// 7.3.5.16 Reference list structure syntax (NEW)
void PCCBitstreamEncoder::refListStruct( RefListStruct&                 rls,
                                         AtlasSequenceParameterSetRBSP& asps,
                                         PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( rls.getNumRefEntries() );  // ue(v)
  TRACE_BITSTREAM( "NumRefEntries = %lu  \n", rls.getNumRefEntries() );
  rls.allocate();
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( asps.getLongTermRefAtlasFramesFlag() ) {
      bitstream.write( rls.getStRefAtalsFrameFlag( i ), 1 );  // u(1)
      if ( rls.getStRefAtalsFrameFlag( i ) ) {
        bitstream.writeUvlc( rls.getAbsDeltaPfocSt( i ) );  // ue(v)
        if ( rls.getAbsDeltaPfocSt( i ) > 0 ) {
          bitstream.write( rls.getStrpfEntrySignFlag( i ), 1 );  // u(1)
        } else {
          uint8_t bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
          bitstream.write( rls.getAfocLsbLt( i ),
                           bitCount );  // u(v) psps_log2_max_patch_frame_order_cnt_lsb_minus4  + 4 bits
        }
      }
    }
  }
  TRACE_BITSTREAM( "%s done \n", __func__ );
}

// 7.3.6.1 General patch tile group data unit syntax
void PCCBitstreamEncoder::patchTileGroupDataUnit( PatchTileGroupDataUnit& ptgdu,
                                                  PatchTileGroupHeader&   ptgh,
                                                  PCCContext&             context,
                                                  PCCBitstream&           bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "ptgh.getType()        = %lu \n", ptgh.getType() );
  for ( size_t puCount = 0; puCount < ptgdu.getPatchCount(); puCount++ ) {
    bitstream.writeUvlc( uint32_t( ptgdu.getPatchMode( puCount ) ) );
    TRACE_BITSTREAM( "patchMode = %lu \n", ptgdu.getPatchMode( puCount ) );
    ptgdu.getPatchInformationData( puCount ).setFrameIndex( ptgdu.getFrameIndex() );
    ptgdu.getPatchInformationData( puCount ).setPatchIndex( puCount );
    patchInformationData( ptgdu.getPatchInformationData( puCount ),
                          ptgdu.getPatchInformationData( puCount ).getPatchIndex(), ptgdu.getPatchMode( puCount ), ptgh,
                          context, bitstream );
  }
  TRACE_BITSTREAM( "ptgdu.getPatchCount() = %lu \n", ptgdu.getPatchCount() );
  byteAlignment( bitstream );
}

// 7.3.6.2 Patch information data syntax
void PCCBitstreamEncoder::patchInformationData( PatchInformationData& pid,
                                                size_t                patchIndex,
                                                size_t                patchMode,
                                                PatchTileGroupHeader& ptgh,
                                                PCCContext&           context,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sps  = context.getSps();
  size_t atlasIndex = 0;
  auto& ai   = sps.getAttributeInformation(atlasIndex);
  auto& pdg  = context.getPatchDataGroup();
  auto& pfps = pdg.getPatchFrameParameterSet( ptgh.getPatchFrameParameterSetId() );
  if ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_SKIP ) {
    // skip mode.
    // currently not supported but added it for convenience. Could easily be removed
  } else if ( ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_I && patchMode == PATCH_MODE_I_INTRA ) ||
              ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_INTRA ) ) {
    if ( pfps.getLocalOverrideGeometryPatchEnableFlag() ) {
      bitstream.write( pid.getOverrideGeometryPatchFlag(), 1 );  // u(1)
      if ( pid.getOverrideGeometryPatchFlag() ) {
        bitstream.writeUvlc( uint32_t( pid.getGeometryPatchParameterSetId() ) );  // ue(v)
        TRACE_BITSTREAM( " gppsId = %lu \n", pid.getGeometryPatchParameterSetId() );
      }
    }
    TRACE_BITSTREAM( " ai.getAttributeCount() = %lu \n", ai.getAttributeCount() );
    for ( int i = 0; i < ai.getAttributeCount(); i++ ) {
      TRACE_BITSTREAM( " override flag = %lu \n", pfps.getLocalOverrideAttributePatchEnableFlag( i ) );
      if ( pfps.getLocalOverrideAttributePatchEnableFlag( i ) ) {
        bitstream.write( pid.getOverrideAttributePatchFlag( i ), 1 );  // u(1)
        TRACE_BITSTREAM( " overrideAttributePatchFlag = %lu \n", pid.getOverrideAttributePatchFlag( i ) );
      }
      TRACE_BITSTREAM( " override attribute patch flag = %lu \n", pid.getOverrideAttributePatchFlag( i ) );
      if ( pid.getOverrideAttributePatchFlag( i ) ) {
        bitstream.writeUvlc( uint32_t( pid.getAttributePatchParameterSetId( i ) ) );  // ue(v)
        TRACE_BITSTREAM( " AttributePatchParameterSetId = %lu \n", pid.getAttributePatchParameterSetId( i ) );
      }
    }
    auto& pdu = pid.getPatchDataUnit();
    pdu.setPduFrameIndex( pid.getFrameIndex() );
    pdu.setPduPatchIndex( pid.getPatchIndex() );
    patchDataUnit( pdu, ptgh, context, bitstream );
  } else if ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_INTER ) {
    auto& dpdu = pid.getDeltaPatchDataUnit();
    dpdu.setDpduFrameIndex( pid.getFrameIndex() );
    dpdu.setDpduPatchIndex( pid.getPatchIndex() );
    deltaPatchDataUnit( dpdu, ptgh, context, bitstream );
  } else if ( ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_I && patchMode == PATCH_MODE_I_Raw ) ||
              ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_Raw ) ) {
    auto& ppdu = pid.getRawPatchDataUnit();
    ppdu.setFrameIndex( pid.getFrameIndex() );
    ppdu.setPatchIndex( pid.getPatchIndex() );
    pcmPatchDataUnit( ppdu, ptgh, context, bitstream );
  } else if ( ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_I && patchMode == PATCH_MODE_I_EOM ) ||
              ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_EOM ) ) {
    auto& epdu = pid.getEomPatchDataUnit();
    eomPatchDataUnit( epdu, ptgh, context, bitstream );
  }
}

// 7.3.6.3 Patch data unit syntax
void PCCBitstreamEncoder::patchDataUnit( PatchDataUnit&        pdu,
                                         PatchTileGroupHeader& ptgh,
                                         PCCContext&           context,
                                         PCCBitstream&         bitstream ) {
  auto& sps                          = context.getSps();
  auto  ptghPatchFrameParameterSetId = ptgh.getPatchFrameParameterSetId();
  auto& pfps = context.getPatchDataGroup().getPatchFrameParameterSet( ptghPatchFrameParameterSetId );
  auto  pfpsPatchVpccParameterSetId = pfps.getPatchVpccParameterSetId();
  auto& psps = context.getPatchDataGroup().getPatchVpccParameterSet( pfpsPatchVpccParameterSetId );
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( uint32_t( pdu.get2DShiftU() ), ptgh.getInterPredictPatch2dShiftUBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( pdu.get2DShiftV() ), ptgh.getInterPredictPatch2dShiftVBitCountMinus1() + 1 );  // u(v)
  bitstream.writeSvlc( int32_t( pdu.get2DDeltaSizeU() ) );  // se(v) The way it is implemented in TM
  bitstream.writeSvlc( int32_t( pdu.get2DDeltaSizeV() ) );  // se(v) The way it is implemented in TM
  bitstream.write( uint32_t( pdu.get3DShiftTangentAxis() ),
                   ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( pdu.get3DShiftBiTangentAxis() ),
                   ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( pdu.get3DShiftMinNormalAxis() ),
                   ptgh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1() + 1 );  // u(v)
  if ( psps.getNormalAxisMaxDeltaValueEnableFlag() ) {
    bitstream.write( uint32_t( pdu.get3DShiftDeltaMaxNormalAxis() ),
                     ptgh.getInterPredictPatch2dDeltaSizeDBitCountMinus1() + 1 );  // u(v)
  }
  TRACE_BITSTREAM( "%zu(%zu), %zu(%zu)\n", (size_t)pdu.get3DShiftBiTangentAxis(),
                   (size_t)ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(),
                   (size_t)pdu.get3DShiftDeltaMaxNormalAxis(),
                   (size_t)ptgh.getInterPredictPatch2dDeltaSizeDBitCountMinus1() );
  bitstream.write( uint32_t( pdu.getProjectPlane() ), 3 );  // u(3) 0,1,2(near 0,1,2)
  if ( psps.getUseEightOrientationsFlag() ) {
    bitstream.write( pdu.getPduOrientationIndex(), 3 );  // u(3)
  } else {
    bitstream.write( pdu.getPduOrientationIndex(), 1 );  // u(1)
  }
  if ( pfps.getLodModeEnableFlag()) {
    bitstream.write( uint32_t( pdu.getLodEnableFlag() ), 1 );  // u(1) //    pdu.setLodEnableFlag(bitstream.read( 1 ) ); //u1
    if( pdu.getLodEnableFlag() ) {
      bitstream.writeUvlc( uint32_t( pdu.getLodScaleXminus1() ));
      bitstream.writeUvlc( uint32_t( pdu.getLodScaleY() ));
    }
  }
  if ( pfps.getProjection45DegreeEnableFlag() ) {
    bitstream.write( uint32_t( pdu.get45DegreeProjectionPresentFlag() ), 1 );  // u(1)
  }

  if ( pdu.get45DegreeProjectionPresentFlag() ) {
    bitstream.write( uint32_t( pdu.get45DegreeProjectionRotationAxis() ), 2 );  // u(2)
  }
  if ( sps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionData( pdu.getPointLocalReconstructionData(), context, bitstream );
  }
  TRACE_BITSTREAM( "Patch(%zu/%zu) => UV %4lu %4lu S=%4ld %4ld P=%lu O=%d A=%lu %lu %lu P45= %d %d lod=(%lu) %lu %lu\n ",
                   pdu.getPduPatchIndex(), pdu.getPduFrameIndex(), pdu.get2DShiftU(), pdu.get2DShiftV(),
                   pdu.get2DDeltaSizeU(), pdu.get2DDeltaSizeV(), pdu.getProjectPlane(), pdu.getPduOrientationIndex(),
                   pdu.get3DShiftTangentAxis(), pdu.get3DShiftBiTangentAxis(), pdu.get3DShiftMinNormalAxis(),
                   pdu.get45DegreeProjectionPresentFlag(), pdu.get45DegreeProjectionRotationAxis(),  pdu.getLodEnableFlag(), pdu.getLodScaleXminus1(), pdu.getLodScaleY());
}

// 7.3.6.4  Delta Patch data unit syntax TODO: Missing 10-projection syntax element?
void PCCBitstreamEncoder::deltaPatchDataUnit( DeltaPatchDataUnit&   dpdu,
                                              PatchTileGroupHeader& ptgh,
                                              PCCContext&           context,
                                              PCCBitstream&         bitstream ) {
  auto& sps                          = context.getSps();
  auto  ptghPatchFrameParameterSetId = ptgh.getPatchFrameParameterSetId();
  auto& pfps = context.getPatchDataGroup().getPatchFrameParameterSet( ptghPatchFrameParameterSetId );
  auto  pfpsPatchVpccParameterSetId = pfps.getPatchVpccParameterSetId();
  auto& psps = context.getPatchDataGroup().getPatchVpccParameterSet( pfpsPatchVpccParameterSetId );

  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeSvlc( int32_t( dpdu.getDeltaPatchIdx() ) );              // se(v)
  bitstream.writeSvlc( int32_t( dpdu.get2DDeltaShiftU() ) );              // se(v)
  bitstream.writeSvlc( int32_t( dpdu.get2DDeltaShiftV() ) );              // se(v)
  bitstream.writeSvlc( int32_t( dpdu.get2DDeltaSizeU() ) );               // se(v)
  bitstream.writeSvlc( int32_t( dpdu.get2DDeltaSizeV() ) );               // se(v)
  bitstream.writeSvlc( int32_t( dpdu.get3DDeltaShiftTangentAxis() ) );    // se(v)
  bitstream.writeSvlc( int32_t( dpdu.get3DDeltaShiftBiTangentAxis() ) );  // se(v)
  bitstream.writeSvlc( int32_t( dpdu.get3DDeltaShiftMinNormalAxis() ) );  // se(v)
  if ( psps.getNormalAxisMaxDeltaValueEnableFlag() )
    bitstream.writeSvlc( int32_t( dpdu.get3DShiftDeltaMaxNormalAxis() ) );  // se(v)
  if ( sps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionData( dpdu.getPointLocalReconstructionData(), context, bitstream );
  }

  TRACE_BITSTREAM(
      "%zu frame %zu DeltaPatch => DeltaIdx = %d ShiftUV = %ld %ld DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      dpdu.getDpduFrameIndex(), dpdu.getDpduPatchIndex(), dpdu.getDeltaPatchIdx(), dpdu.get2DDeltaShiftU(),
      dpdu.get2DDeltaShiftV(), dpdu.get2DDeltaSizeU(), dpdu.get2DDeltaSizeV(), dpdu.get3DDeltaShiftTangentAxis(),
      dpdu.get3DDeltaShiftBiTangentAxis(), dpdu.get3DDeltaShiftMinNormalAxis() );
}

// 7.3.6.5 raw patch data unit syntax
void PCCBitstreamEncoder::pcmPatchDataUnit( RawPatchDataUnit&     ppdu,
                                            PatchTileGroupHeader& ptgh,
                                            PCCContext&           context,
                                            PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&  sps        = context.getSps();
  size_t atlasIndex = 0;
  if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
    bitstream.write( ppdu.getPatchInRawVideoFlag(), 1 );
  }                                                                                                           // u(1)
  bitstream.write( uint32_t( ppdu.get2DShiftU() ), ptgh.getInterPredictPatch2dShiftUBitCountMinus1() + 1 );   // u(v)
  bitstream.write( uint32_t( ppdu.get2DShiftV() ), ptgh.getInterPredictPatch2dShiftVBitCountMinus1() + 1 );   // u(v)
  bitstream.writeSvlc( int32_t( ppdu.get2DDeltaSizeU() ) );                                                   // se(v)
  bitstream.writeSvlc( int32_t( ppdu.get2DDeltaSizeV() ) );                                                   // se(v)
  bitstream.write( uint32_t( ppdu.get3DShiftTangentAxis() ), ptgh.getRaw3dShiftAxisBitCountMinus1() + 1 );    // u(v)
  bitstream.write( uint32_t( ppdu.get3DShiftBiTangentAxis() ), ptgh.getRaw3dShiftAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( ppdu.get3DShiftNormalAxis() ), ptgh.getRaw3dShiftAxisBitCountMinus1() + 1 );     // u(v)
  bitstream.writeUvlc( int32_t( ppdu.getRawPoints() ) );
  TRACE_BITSTREAM(
      "Raw Patch => UV %4lu %4lu  S=%4ld %4ld  UVD1=%4ld %4ld %4ld NumPcmPoints=%lu PatchInRawVideoFlag=%d \n",
      ppdu.get2DShiftU(), ppdu.get2DShiftV(), ppdu.get2DDeltaSizeU(), ppdu.get2DDeltaSizeV(),
      ppdu.get3DShiftBiTangentAxis(), ppdu.get3DShiftBiTangentAxis(), ppdu.get3DShiftNormalAxis(), ppdu.getRawPoints(),
      ppdu.getPatchInRawVideoFlag() );
}

// 7.3.6.x EOM patch data unit syntax
void PCCBitstreamEncoder::eomPatchDataUnit( EOMPatchDataUnit&     epdu,
                                            PatchTileGroupHeader& ptgh,
                                            PCCContext&           context,
                                            PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  bitstream.write( uint32_t( epdu.get2DShiftU() ), ptgh.getInterPredictPatch2dShiftUBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( epdu.get2DShiftV() ), ptgh.getInterPredictPatch2dShiftVBitCountMinus1() + 1 );  // u(v)
  bitstream.writeSvlc( int32_t( epdu.get2DDeltaSizeU() ) );                                                  // se(v)
  bitstream.writeSvlc( int32_t( epdu.get2DDeltaSizeV() ) );                                                  // se(v)
  bitstream.write(uint32_t(epdu.getEpduAssociatedPatchesCountMinus1()), 8); //max255
  for(size_t cnt=0; cnt<epdu.getEpduAssociatedPatchesCountMinus1()+1; cnt++)
  {
    bitstream.write(uint32_t(epdu.getEpduAssociatedPatches()[cnt]), 8);
    bitstream.writeUvlc( uint32_t( epdu.getEpduEomPointsPerPatch()[cnt] ) );
  }
  TRACE_BITSTREAM( "EOM Patch => UV %4lu %4lu  N=%4ld\n", epdu.get2DShiftU(),
                  epdu.get2DShiftV(), epdu.getEpduAssociatedPatchesCountMinus1()+1);
  for(size_t cnt=0; cnt<epdu.getEpduAssociatedPatchesCountMinus1()+1; cnt++)
    TRACE_BITSTREAM( "%4ld, %4ld\n", epdu.getEpduAssociatedPatches()[cnt], epdu.getEpduEomPointsPerPatch()[cnt]);
}

// 7.3.6.6 Point local reconstruction data syntax
void PCCBitstreamEncoder::pointLocalReconstructionData( PointLocalReconstructionData& plrd,
                                                        PCCContext&                   context,
                                                        PCCBitstream&                 bitstream ) {
  auto& plri = context.getSps().getPointLocalReconstructionInformation();
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "WxH= %lu x %lu \n", plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight() );

  const size_t  blockCount   = plrd.getBlockToPatchMapWidth() * plrd.getBlockToPatchMapHeight();
  const uint8_t bitCountMode = uint8_t( getFixedLengthCodeBitsCount( uint32_t( plri.getNumberOfModesMinus1() ) ) );
  TRACE_BITSTREAM( "  bitCountMode = %u \n", bitCountMode );

  if ( blockCount > plri.getBlockThresholdPerPatchMinus1() + 1 ) {
    bitstream.write( uint32_t( plrd.getLevelFlag() ), 1 );  // u(1)
  }
  TRACE_BITSTREAM( "  LevelFlag = %u \n", plrd.getLevelFlag() );
  if ( plrd.getLevelFlag() ) {
    bitstream.write( uint32_t( plrd.getPresentFlag() ), 1 );                                                // u(1)
    if ( plrd.getPresentFlag() ) { bitstream.write( uint32_t( plrd.getModeMinus1() ), bitCountMode ); }  // u(v)
    TRACE_BITSTREAM( "  ModePatch: Present = %d ModeMinus1 = %d \n", plrd.getPresentFlag(),
                     plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1 );
  } else {
    for ( size_t i = 0; i < blockCount; i++ ) {
      bitstream.write( uint32_t( plrd.getBlockPresentFlag( i ) ), 1 );  // u(1)
      if ( plrd.getBlockPresentFlag( i ) ) {
        bitstream.write( uint32_t( plrd.getBlockModeMinus1( i ) ), bitCountMode );  // u(v)
      }
      TRACE_BITSTREAM( "  Mode[ %4lu / %4lu ]: Present = %d ModeMinus1 = %d \n", i, blockCount,
                       plrd.getBlockPresentFlag( i ),
                       plrd.getBlockPresentFlag( i ) ? plrd.getBlockModeMinus1( i ) : -1 );
    }
  }
#ifdef BITSTREAM_TRACE
  for ( size_t v0 = 0; v0 < plrd.getBlockToPatchMapHeight(); ++v0 ) {
    for ( size_t u0 = 0; u0 < plrd.getBlockToPatchMapWidth(); ++u0 ) {
      size_t i = v0 * plrd.getBlockToPatchMapWidth() + u0;
      TRACE_BITSTREAM( "Patch Block[ %2lu %2lu <=> %4lu ] / [ %2lu %2lu ] Level = %d Present = %d Mode = %d \n", u0, v0,
                       i, plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight(), plrd.getLevelFlag(),
                       plrd.getLevelFlag() ? plrd.getPresentFlag() : plrd.getBlockPresentFlag( i ),
                       plrd.getLevelFlag()
                           ? plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1
                           : plrd.getBlockPresentFlag( i ) ? plrd.getBlockModeMinus1( i ) : -1 );
    }
  }
#endif
}

// 7.3.5.22 Supplemental enhancement information message syntax TODO: Implement
void PCCBitstreamEncoder::seiMessage( PatchDataGroup& pdg,
                                      size_t          index,
                                      PCCContext&     context,
                                      PCCBitstream&   bitstream ) {}


// JR TODO: continue

// C.2 Sample stream NAL unit syntax and semantics
// C.2.1 Sample stream NAL header syntax
void PCCBitstreamEncoder::sampleStreamNalHeader(  PCCBitstream& bitstream, SampleStreamNalUnit& sampleStreamNalUnit ) {
  bitstream.write( uint32_t( sampleStreamNalUnit.getUnitSizePrecisionBytesMinus1() ), 3 ); // u(3)
  bitstream.write( 0, 5 ); // u(5)  
}

// C.2.2 Sample stream NAL unit syntax
void PCCBitstreamEncoder::sampleStreamNalUnit( PCCBitstream& bitstream, SampleStreamNalUnit& sampleStreamNalUnit, NalUnit& nalu ) {  
  bitstream.write( uint32_t( nalu.getNalUnitSize() ), sampleStreamNalUnit.getUnitSizePrecisionBytesMinus1() + 1 ); // u(v)  
  nalUnit( bitstream, nalu );
}
 
// 7.3.5 NAL unit syntax
// 7.3.5.1 General NAL unit syntax
void PCCBitstreamEncoder::nalUnit( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  nalUnitHeader( bitstream, nalUnit );
  for( size_t i = 0; i < nalUnit.getNalUnitSize() - 2; i++ ) {
    bitstream.write( uint32_t( nalUnit.getNalUnitData( i ) ), 8 ); // b(8)
  }
}

// 7.3.5.2 NAL unit header syntax
void PCCBitstreamEncoder::nalUnitHeader( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  bitstream.write( 0, 1 ); // f(1)
  bitstream.write( uint32_t( nalUnit.getUnitType() ), 6 ); // u(6)
  bitstream.write( uint32_t( nalUnit.getLayerId() ), 6 ); // u(6)
  bitstream.write( uint32_t( nalUnit.getTemporalyIdPlus1() ), 3 ); // u(3)
}


// F.2.1 VUI parameters syntax
void PCCBitstreamEncoder::vuiParameters( ) {
  // vui_timing_info_present_flag //   u(1)
  // if( vui_timing_info_present_flag ) {
  //   vui_num_units_in_tick // u(32)
  //   vui_time_scale // u(32)
  //   vui_poc_proportional_to_timing_flag // u(1)
  //   if( vui_poc_proportional_to_timing_flag ) {
  //     vui_num_ticks_poc_diff_one_minus1 // ue(v)
  //   }
  //   vui_hrd_parameters_present_flag   // u(1)
  //   if( vui_hrd_parameters_present_flag ){
  //     hrdParameters()
  //   }
  // }
}
// F.2.2 HRD parameters syntax
void PCCBitstreamEncoder::hrdParameters( ) {
  // hrd_nal_parameters_present_flag // u(1)
  // hrd_acl_parameters_present_flag // u(1)
  // if( hrd_nal_parameters_present_flag | | hrd_acl_parameters_present_flag ){
  //   hrd_bit_rate_scale // u(4)
  //   hrd_cab_size_scale // u(4)
  //   hrd_initial_cab_removal_delay_length_minus1 // u(5)
  //   hrd_au_cab_removal_delay_length_minus1 // u(5)
  //   hrd_dab_output_delay_length_minus1  // u(5)
  // }
  // maxNumSubLayersMinus1 = 0
  // for( i = 0; i <= maxNumSubLayersMinus1; i++ ) {
  //   hrd_fixed_atlas_rate_general_flag[ i ] // u(1)
  // if( !hrd_fixed_atlas_rate_general_flag[ i ] )
  //   hrd_fixed_atlas_rate_within_cas_flag[ i ] // u(1)
  // if( hrd_fixed_atlas_rate_within_cas_flag[ i ] )
  //   hrd_elemental_duration_in_tc_minus1[ i ] // ue(v)
  // else
  //   hrd_low_delay_flag[ i ] // u(1)
  // if( !hrd_low_delay_flag[ i ] )
  //   hrd_cab_cnt_minus1[ i ] // ue(v)
  // if( hrd_nal_parameters_present_flag )
  //   hrd_sub_layer_parameters( 0, i )
  // if( hrd_acl_parameters_present_flag )
  //   hrd_sub_layer_parameters( 1, i )
  // }
}

// F.2.3 Sub-layer HRD parameters syntax
void PCCBitstreamEncoder::hrdSubLayerParameters() {
  // for( i = 0; i <= CabCnt; i++ ) {
  //  hrd_bit_rate_value_minus1[ type ][ i ] // ue(v)
  //  hrd_cab_size_value_minus1[ type ][ i ]  // ue(v)
  //  hrd_cbr_flag[ type ][ i ] // u(1)
  // }
}


// JR TODO: Remove

// 7.3.2.3 raw separate video data syntax TODO: remove this
void PCCBitstreamEncoder::pcmSeparateVideoData( PCCContext& context, PCCBitstream& bitstream, uint8_t bitCount ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  size_t atlasIndex = 0;
  if ( sps.getRawSeparateVideoPresentFlag(atlasIndex) && !vpcc.getMapIndex() ) {
    bitstream.write( (uint32_t)vpcc.getRawVideoFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)0, bitCount );                // u(bitCount)
  } else {
    bitstream.write( (uint32_t)0, bitCount + 1 );  // u(bitCount + 1)
  }
}
