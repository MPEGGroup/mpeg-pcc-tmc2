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
#include "PCCBitstreamCommon.h"
#include "PCCVideoBitstream.h"
#include "PCCBitstream.h"
#include "PCCHighLevelSyntax.h"

#include "PCCBitstreamReader.h"

using namespace pcc;

PCCBitstreamReader::PCCBitstreamReader() :
    prevPatchSizeU_( 0 ),
    prevPatchSizeV_( 0 ),
    predPatchIndex_( 0 ),
    prevFrameIndex_( 0 ) {}
PCCBitstreamReader::~PCCBitstreamReader() {}

// B.2  Sample stream V-PCC unit syntax
size_t PCCBitstreamReader::read( PCCBitstream& bitstream, SampleStreamVpccUnit& ssvu ) {
  size_t headerSize = 0;
  TRACE_BITSTREAM( "PCCBitstreamXXcoder: SampleStream Vpcc Unit start \n" );
  sampleStreamVpccHeader( bitstream, ssvu );
  headerSize++;
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 %d <=> %d / 8 - 1\n", ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1,
                   ceilLog2( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) );
  size_t unitCount = 0;
  while ( bitstream.moreData() ) {
    auto& vpccUnit = ssvu.addVpccUnit();
    sampleStreamVpccUnit( bitstream, ssvu, vpccUnit );
    TRACE_BITSTREAM( "V-PCC Unit Size(%zuth/%zu)  = %lu \n", unitCount, ssvu.getVpccUnitCount(),
                     vpccUnit.getVpccUnitSize() );
    unitCount++;
    headerSize += ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1;
  }
  TRACE_BITSTREAM( "PCCBitstreamXXcoder: SampleStream Vpcc Unit start done \n" );
  return headerSize;
}

// B.2 Sample stream V-PCC unit syntax and semantics
// B.2.1 Sample stream V-PCC header syntax
void PCCBitstreamReader::sampleStreamVpccHeader( PCCBitstream& bitstream, SampleStreamVpccUnit& ssvu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ssvu.setSsvhUnitSizePrecisionBytesMinus1( bitstream.read( 3 ) );  // u(3)
  bitstream.read( 5 );                                              // u(5)
}

// B.2.2 Sample stream NAL unit syntax
void PCCBitstreamReader::sampleStreamVpccUnit( PCCBitstream& bitstream, SampleStreamVpccUnit& ssvu, VpccUnit& vpccu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  vpccu.setVpccUnitSize( bitstream.read( 8 * ( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) ) );  // u(v)
   auto pos = bitstream.getPosition();
  vpccu.getVpccUnitDataBitstream().copyFrom( bitstream, pos.bytes, vpccu.getVpccUnitSize() );
  uint8_t      vpccUnitType8 = vpccu.getVpccUnitDataBitstream().buffer()[0];
  VPCCUnitType vpccUnitType  = ( VPCCUnitType )( vpccUnitType8 >>= 3 );
  vpccu.setVpccUnitType( vpccUnitType );
  TRACE_BITSTREAM( "vpccUnitType: %hhu VpccUnitSize: %zu\n", vpccUnitType8, vpccu.getVpccUnitSize() );
}

int32_t PCCBitstreamReader::decode( SampleStreamVpccUnit& ssvu, PCCHighLevelSyntax& syntax ) {
  bool endOfGop = false;
  int  numVPS   = 0;  // counter for the atlas information
  syntax.getBitstreamStat().newGOF();
  while ( !endOfGop && ssvu.getVpccUnitCount() > 0 ) {
    auto& VPCCUnit = ssvu.front();    
    VPCCUnitType vpccUnitType = VPCC_VPS;    
    vpccUnit( syntax, VPCCUnit, vpccUnitType );
    if ( vpccUnitType == VPCC_VPS ) {
      numVPS++;
      if ( numVPS > 1 ) {
        endOfGop = true;
      } else {
        ssvu.popFront();  // remove element

      }
    } else {
      ssvu.popFront();  // remove element
    }
  }
  return 1;
}

void PCCBitstreamReader::videoSubStream( PCCHighLevelSyntax& syntax, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  if ( vpccUnitType == VPCC_OVD ) {
    TRACE_BITSTREAM( "OccupancyMap \n" );
    bitstream.read( syntax.createVideoBitstream( VIDEO_OCCUPANCY ) );
    syntax.getBitstreamStat().setVideoBinSize( VIDEO_OCCUPANCY, syntax.getVideoBitstream( VIDEO_OCCUPANCY ).size() );
  } else if ( vpccUnitType == VPCC_GVD ) {
    auto& vuh = syntax.getVpccUnitHeader( (size_t)VPCC_GVD - 1 );
    if ( vuh.getRawVideoFlag() ) {
      TRACE_BITSTREAM( "Geometry raw\n" );
      bitstream.read( syntax.createVideoBitstream( VIDEO_GEOMETRY_RAW ) );
      syntax.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY_RAW,
                                                  syntax.getVideoBitstream( VIDEO_GEOMETRY_RAW ).size() );
    } else {
      auto& vps = syntax.getVps();
      if ( vps.getMapCountMinus1( atlasIndex ) > 0 && vps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
        PCCVideoType geometryIndex = (PCCVideoType)(VIDEO_GEOMETRY_D0 + vuh.getMapIndex());
        TRACE_BITSTREAM("Geometry MAP: %d\n", vuh.getMapIndex());
        bitstream.read(syntax.createVideoBitstream(geometryIndex));
        syntax.getBitstreamStat().setVideoBinSize(geometryIndex, syntax.getVideoBitstream( geometryIndex ).size() );
      } else {
        TRACE_BITSTREAM( "Geometry \n" );
        bitstream.read( syntax.createVideoBitstream( VIDEO_GEOMETRY ) );
        syntax.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY,
                                                    syntax.getVideoBitstream( VIDEO_GEOMETRY ).size() );
      }
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    auto& vuh = syntax.getVpccUnitHeader( (size_t)VPCC_AVD - 1 );
    auto& vps = syntax.getVps();
    if ( vps.getAttributeInformation( atlasIndex ).getAttributeCount() > 0 ) {
      if ( vuh.getRawVideoFlag() ) {
        PCCVideoType textureIndex = (PCCVideoType)(VIDEO_TEXTURE_RAW + vuh.getAttributeDimensionIndex());
        TRACE_BITSTREAM( "Texture raw, PARTITION: %d\n", vuh.getAttributeDimensionIndex() );
        bitstream.read( syntax.createVideoBitstream( textureIndex ) );
        syntax.getBitstreamStat().setVideoBinSize( textureIndex,
                                                    syntax.getVideoBitstream( textureIndex ).size() );
      } else {
        if ( vps.getMapCountMinus1( atlasIndex ) > 0 && vps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
          PCCVideoType textureIndex = (PCCVideoType)(VIDEO_TEXTURE_T0 + vuh.getMapIndex()*MAX_NUM_ATTR_PARTITIONS + vuh.getAttributeDimensionIndex());
          TRACE_BITSTREAM("Texture MAP: %d, PARTITION: %d\n",vuh.getMapIndex(),vuh.getAttributeDimensionIndex());
          bitstream.read( syntax.createVideoBitstream( textureIndex ) );
          syntax.getBitstreamStat().setVideoBinSize( textureIndex,
                                                       syntax.getVideoBitstream( textureIndex ).size() );
        } else {
          PCCVideoType textureIndex = (PCCVideoType)(VIDEO_TEXTURE + vuh.getAttributeDimensionIndex());
          TRACE_BITSTREAM("Texture PARTITION: %d\n", vuh.getAttributeDimensionIndex());
          bitstream.read( syntax.createVideoBitstream( textureIndex ) );
          syntax.getBitstreamStat().setVideoBinSize( textureIndex,
                                                      syntax.getVideoBitstream( textureIndex ).size() );
        }
      }
    }  // if(!noAttribute)
  }    // avd
}

// 7.3.2.1 General V-PCC unit syntax
void PCCBitstreamReader::vpccUnit( PCCHighLevelSyntax& syntax, VpccUnit& currVpccUnit, VPCCUnitType& vpccUnitType ) {
  PCCBitstream& bitstream = currVpccUnit.getVpccUnitDataBitstream();
#ifdef BITSTREAM_TRACE
  bitstream.setTrace( true );
  bitstream.setTraceFile( traceFile_ );
  TRACE_BITSTREAM( "PCCBitstream::(%s)\n", toString( currVpccUnit.getVpccUnitType() ).c_str() );
#endif
  TRACE_BITSTREAM( "%s \n", __func__ );
  int32_t position = (int32_t)bitstream.size();
  vpccUnitHeader( syntax, bitstream, vpccUnitType );
  assert( vpccUnitType == currVpccUnit.getVpccUnitType() );
  vpccUnitPayload( syntax, bitstream, vpccUnitType );
  syntax.getBitstreamStat().setVpccUnitSize( vpccUnitType, (int32_t)bitstream.size() - position );
  TRACE_BITSTREAM( "vpccUnit: vpccUnitType = %d(%s) \n", vpccUnitType, toString( vpccUnitType ).c_str() );
  TRACE_BITSTREAM( "vpccUnit: size [%d ~ %d] \n", position, bitstream.size() );
  TRACE_BITSTREAM( "%s done\n", __func__ );
  std::cout << "<----vpccUnit: vpccUnitType = " << toString( VPCCUnitType( vpccUnitType ) ) << std::endl;  
}

// 7.3.2.2 V-PCC unit header syntax
void PCCBitstreamReader::vpccUnitHeader( PCCHighLevelSyntax& syntax, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  vpccUnitType = (VPCCUnitType)bitstream.read( 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_AD ) {
    auto& vpcc = syntax.getVpccUnitHeader( (int)vpccUnitType - 1 );
    vpcc.setVpccParameterSetId( bitstream.read( 4 ) );  // u(4)
    syntax.setActiveVpsId(vpcc.getVpccParameterSetId());
    vpcc.setAtlasId( bitstream.read( 6 ) );             // u(6)
    syntax.setAtlasIndex(vpcc.getAtlasId());
  }
  if ( vpccUnitType == VPCC_AVD ) {
    auto& vpcc = syntax.getVpccUnitHeader( (int)vpccUnitType - 1 );
    vpcc.setAttributeIndex( bitstream.read( 7 ) );           // u(7)
    vpcc.setAttributeDimensionIndex( bitstream.read( 5 ) );  // u(5)
    vpcc.setMapIndex( bitstream.read( 4 ) );                 // u(4)
    vpcc.setRawVideoFlag( bitstream.read( 1 ) );             // u(1)
  } else if ( vpccUnitType == VPCC_GVD ) {
    auto& vpcc = syntax.getVpccUnitHeader( (int)vpccUnitType - 1 );
    vpcc.setMapIndex( bitstream.read( 4 ) );      // u(4)
    vpcc.setRawVideoFlag( bitstream.read( 1 ) );  // u(1)
    bitstream.read( 12 );                         // u(12)
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_AD ) {
    bitstream.read( 17 );  // u(17)
  } else {
    bitstream.read( 27 );  // u(27)
  }
}

// 7.3.2.3 V-PCC unit payload syntax
void PCCBitstreamReader::vpccUnitPayload( PCCHighLevelSyntax& syntax, PCCBitstream& bitstream, VPCCUnitType& vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "vpccUnitType = %d \n", (int32_t)vpccUnitType );
  if ( vpccUnitType == VPCC_VPS ) {
    auto& vps = syntax.addVpccParameterSet();
    vpccParameterSet( vps, syntax, bitstream );
  } else if ( vpccUnitType == VPCC_AD ) {
    atlasSubStream( syntax, bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    videoSubStream( syntax, bitstream, vpccUnitType );
  }
}

// 7.3.3 Byte alignment syntax
void PCCBitstreamReader::byteAlignment( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.read( 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.read( 1 );  // f(1): equal to 0
  }
}

// 7.3.4.1 General V-PCC parameter set syntax
void PCCBitstreamReader::vpccParameterSet( VpccParameterSet& sps, PCCHighLevelSyntax& syntax, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  sps.setVpccParameterSetId( bitstream.read( 4 ) );  // u(4)
  sps.setAtlasCountMinus1( bitstream.read( 6 ) );    // u(6)
  sps.allocateAtlas();
  syntax.allocateAtlasHLS(sps.getAtlasCountMinus1() + 1);
  for ( int j = 0; j < sps.getAtlasCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "Atlas = %lu \n", j );
    sps.setFrameWidth( j, bitstream.read( 16 ) );     // u(16)
    sps.setFrameHeight( j, bitstream.read( 16 ) );    // u(16)
    sps.setMapCountMinus1( j, bitstream.read( 4 ) );  // u(4)
    TRACE_BITSTREAM( " MapCountMinus1 = %lu \n", sps.getMapCountMinus1( j ) );
    sps.allocateMap( j );
    if ( sps.getMapCountMinus1( j ) > 0 ) {
      sps.setMultipleMapStreamsPresentFlag( j, bitstream.read( 1 ) );  // u(1)
      TRACE_BITSTREAM( "MultipleMapStreamsPresentFlag = %lu \n", sps.getMultipleMapStreamsPresentFlag( j ) );
    }
    sps.setMapAbsoluteCodingEnableFlag( j, 0, 1 );
    for ( size_t i = 1; i <= sps.getMapCountMinus1( j ); i++ ) {
      if ( sps.getMultipleMapStreamsPresentFlag( j ) ) {
        sps.setMapAbsoluteCodingEnableFlag( j, i, bitstream.read( 1 ) );  // u(1)
      } else {
        sps.setMapAbsoluteCodingEnableFlag( j, i, 1 );
      }
      TRACE_BITSTREAM( " AbsoluteCoding Map%lu = %lu \n", i, sps.getMapAbsoluteCodingEnableFlag( j, i ) );
      if ( sps.getMapAbsoluteCodingEnableFlag( j, i ) == 0 ) {
        if ( i > 0 ) {
          sps.setMapPredictorIndexDiff( j, i, bitstream.readUvlc() );  // ue(v)
        } else {
          sps.setMapPredictorIndexDiff( j, i, 0 );
        }
        TRACE_BITSTREAM( " PredictorIndex L%lu = %lu \n", i, sps.getMapPredictorIndexDiff( j, i ) );
      }
    }
    sps.setRawPatchEnabledFlag( j, bitstream.read( 1 ) );  // u(1)
    TRACE_BITSTREAM( " RawPatchEnabledFlag = %lu \n", sps.getRawPatchEnabledFlag( j ) );
    if ( sps.getRawPatchEnabledFlag( j ) ) {
      sps.setRawSeparateVideoPresentFlag( j, bitstream.read( 1 ) );  // u(1)
      TRACE_BITSTREAM( " RawSeparateVideoPresentFlag = %lu \n", sps.getRawSeparateVideoPresentFlag( j ) );
    }
#ifdef BITSTREAM_TRACE
    for ( size_t i = 0; i < sps.getMapCountMinus1( j ) + 1; i++ ) {
      TRACE_BITSTREAM( " AbsoluteCoding L%lu = %lu \n", i, sps.getMapAbsoluteCodingEnableFlag( j, i ) );
    }
#endif
    occupancyInformation( sps.getOccupancyInformation( j ), bitstream );
    geometryInformation( sps.getGeometryInformation( j ), sps, bitstream );
    attributeInformation( sps.getAttributeInformation( j ), sps, bitstream );
  }

  sps.setExtensionPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( sps.getExtensionPresentFlag() ) {
    sps.setExtensionLength( bitstream.readUvlc() );  // ue(v)
    sps.allocateExtensionDataByte();
    for ( size_t i = 0; i < sps.getExtensionLength(); i++ ) {
      sps.setExtensionDataByte( i, bitstream.read( 8 ) );  // u(8)
    }
  }
  byteAlignment( bitstream );
}

// 7.3.4.2 Profile, tier, and level syntax
void PCCBitstreamReader::profileTierLevel( ProfileTierLevel& ptl, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ptl.setTierFlag( bitstream.read( 1 ) );                  // u(1)
  ptl.setProfileCodecGroupIdc( bitstream.read( 7 ) );      // u(7)
  ptl.setProfilePccToolsetIdc( bitstream.read( 8 ) );      // u(8)
  ptl.setProfileReconctructionIdc( bitstream.read( 8 ) );  // u(8)
  bitstream.read( 32 );                                    // u(32)
  ptl.setLevelIdc( bitstream.read( 8 ) );                  // u(8)
}

// 7.3.4.3 Occupancy parameter set syntax
void PCCBitstreamReader::occupancyInformation( OccupancyInformation& oi, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  oi.setOccupancyCodecId( bitstream.read( 8 ) );                       // u(8)
  oi.setLossyOccupancyMapCompressionThreshold( bitstream.read( 8 ) );  // u(8)
  oi.setOccupancyNominal2DBitdepthMinus1( bitstream.read( 5 ) );       // u(5)
  oi.setOccupancyMSBAlignFlag( bitstream.read( 1 ) );                  // u(1)
  TRACE_BITSTREAM( "  OccupancyLossyThreshold = %d  \n", oi.getLossyOccupancyMapCompressionThreshold() );
}

// 7.3.4.4 Geometry parameter set syntax
void PCCBitstreamReader::geometryInformation( GeometryInformation& gi,
                                               VpccParameterSet&    sps,
                                               PCCBitstream&        bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  gi.setGeometryCodecId( bitstream.read( 8 ) );                      // u(8)
  gi.setGeometryNominal2dBitdepthMinus1( bitstream.read( 5 ) );      // u(5)
  gi.setGeometryMSBAlignFlag( bitstream.read( 1 ) );                 // u(1)
  gi.setGeometry3dCoordinatesBitdepthMinus1( bitstream.read( 5 ) );  // u(5)
  if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
    gi.setRawGeometryCodecId( bitstream.read( 8 ) );  // u(8)
  }
}

// 7.3.4.5 Attribute information
void PCCBitstreamReader::attributeInformation( AttributeInformation& ai,
                                                VpccParameterSet&     sps,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  ai.setAttributeCount( bitstream.read( 7 ) );  // u(7)
  TRACE_BITSTREAM( "AttributeCount = %u  \n", ai.getAttributeCount() );
  ai.allocate();
  for ( uint32_t i = 0; i < ai.getAttributeCount(); i++ ) {
    ai.setAttributeTypeId( i, bitstream.read( 4 ) );   // u(4)
    ai.setAttributeCodecId( i, bitstream.read( 8 ) );  // u(8)
    if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
      ai.setRawAttributeCodecId( i, bitstream.read( 8 ) );  // u(8)
    }
    ai.setAttributeMapAbsoluteCodingPersistanceFlag( i, true );
    if ( sps.getMapCountMinus1( atlasIndex ) > 0 ) {
      ai.setAttributeMapAbsoluteCodingPersistanceFlag( i, bitstream.read( 1 ) );  // u(1)
    }
    ai.setAttributeDimensionMinus1( i, bitstream.read( 6 ) );  // u(6)
    if ( ai.getAttributeDimensionMinus1( i ) > 0 ) {
      ai.setAttributeDimensionPartitionsMinus1( i, bitstream.read( 6 ) );  // u(6)
      int32_t remainingDimensions = ai.getAttributeDimensionMinus1( i );
      int32_t k                   = ai.getAttributeDimensionPartitionsMinus1( i );
      for ( int32_t j = 0; j < k; j++ ) {
        if ( k - j == remainingDimensions ) {
          ai.setAttributePartitionChannelsMinus1( i, j, 0 );
        } else {
          ai.setAttributePartitionChannelsMinus1( i, j, bitstream.readUvlc() );  // ue(v)
        }
        remainingDimensions -= ai.getAttributePartitionChannelsMinus1( i, j ) + 1;
      }
      ai.setAttributePartitionChannelsMinus1( i, k, remainingDimensions );
    }
    ai.setAttributeNominal2dBitdepthMinus1( i, bitstream.read( 5 ) );  // u(5)
    ai.setAttributeMSBAlignFlag( i, bitstream.read( 1 ) );  // u(1)
  }
}

// 7.3.6.1 Atlas sequence parameter set Rbsp
void PCCBitstreamReader::atlasSequenceParameterSetRbsp( AtlasSequenceParameterSetRbsp& asps,
                                                         PCCHighLevelSyntax&                    syntax,
                                                         PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  asps.setAltasSequenceParameterSetId( bitstream.readUvlc() );         // ue(v)
  asps.setFrameWidth( bitstream.read( 16 ) );                          // u(16)
  asps.setFrameHeight( bitstream.read( 16 ) );                         // u(16)
  asps.setLog2PatchPackingBlockSize( bitstream.read( 3 ) );            // u(3)
  asps.setLog2MaxAtlasFrameOrderCntLsbMinus4( bitstream.readUvlc() );  // ue(v)
  asps.setMaxDecAtlasFrameBufferingMinus1( bitstream.readUvlc() );     // ue(v)
  asps.setLongTermRefAtlasFramesFlag( bitstream.read( 1 ) );           // u(1)
  asps.setNumRefAtlasFrameListsInAsps( bitstream.readUvlc() );         // ue(v)
  asps.allocateRefListStruct();
  TRACE_BITSTREAM( "ASPS: NumRefListStruct = %u \n", asps.getNumRefAtlasFrameListsInAsps() );
  for ( size_t i = 0; i < asps.getNumRefAtlasFrameListsInAsps(); i++ ) {
    refListStruct( asps.getRefListStruct( i ), asps, bitstream );
  }
  asps.setLongTermRefAtlasFramesFlag( bitstream.read( 1 ) );               // u(1)
  asps.set45DegreeProjectionPatchPresentFlag( bitstream.read( 1 ) );       // u(1)
  asps.setNormalAxisLimitsQuantizationEnabledFlag( bitstream.read( 1 ) );  // u(1)
  asps.setNormalAxisMaxDeltaValueEnabledFlag( bitstream.read( 1 ) );       // u(1)
  asps.setRemoveDuplicatePointEnabledFlag( bitstream.read( 1 ) );          // u(1)
  asps.setPixelDeinterleavingFlag( bitstream.read( 1 ) );                  // u(1)
  asps.setPatchPrecedenceOrderFlag( bitstream.read( 1 ) );                 // u(1)
  asps.setPatchSizeQuantizerPresentFlag( bitstream.read( 1 ) );            // u(1)
  asps.setEnhancedOccupancyMapForDepthFlag( bitstream.read( 1 ) );         // u(1)
  asps.setPointLocalReconstructionEnabledFlag( bitstream.read( 1 ) );      // u(1)
  asps.setMapCountMinus1( bitstream.read( 4 ) );                           // u(4)
  if ( asps.getEnhancedOccupancyMapForDepthFlag() && asps.getMapCountMinus1() == 0 ) {
    asps.setEnhancedOccupancyMapFixBitCountMinus1( bitstream.read( 4 ) );  // u(4)
  }
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionInformation( asps, syntax, bitstream );
  }
  if ( asps.getPixelDeinterleavingFlag() || asps.getPointLocalReconstructionEnabledFlag() ) {
    asps.setSurfaceThicknessMinus1( bitstream.read( 8 ) );  // u(8)
  }
  asps.setVuiParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getVuiParametersPresentFlag() ) { vuiParameters( bitstream, asps.getVuiParameters() ); }
  asps.setExtensionPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getExtensionPresentFlag() ) {
    while ( moreRbspData( bitstream ) ) {
      asps.setExtensionPresentFlag( bitstream.read( 1 ) );  // u(1)
    }
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.2 Point local reconstruction information syntax
void PCCBitstreamReader::pointLocalReconstructionInformation( AtlasSequenceParameterSetRbsp& asps,
                                                               PCCHighLevelSyntax&                    syntax,
                                                               PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "  MapCountMinus1() = %u \n", asps.getMapCountMinus1() );
  asps.allocatePointLocalReconstructionInformation();
  for ( size_t j = 0; j < asps.getMapCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "  PLR info map %lu \n", j );
    auto& plri = asps.getPointLocalReconstructionInformation( j );
    plri.setMapEnabledFlag( bitstream.read( 1 ) );  // u(1)
    if ( plri.getMapEnabledFlag() ) {
      plri.setNumberOfModesMinus1( bitstream.read( 4 ) );  // u(4)
      TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plri.getNumberOfModesMinus1() );
      plri.allocate();
      for ( size_t i = 0; i < plri.getNumberOfModesMinus1(); i++ ) {
        plri.setInterpolateFlag( i, bitstream.read( 1 ) );  // u(1)
        plri.setFillingFlag( i, bitstream.read( 1 ) );      // u(1)
        plri.setMinimumDepth( i, bitstream.read( 2 ) );     // u(2)
        plri.setNeighbourMinus1( i, bitstream.read( 2 ) );  // u(2)
        TRACE_BITSTREAM( "  Mode[%lu] = I = %d F = %d D = %u N = %u \n", i, plri.getInterpolateFlag( i ),
                         plri.getFillingFlag( i ), plri.getMinimumDepth( i ), plri.getNeighbourMinus1( i ) );
      }
      plri.setBlockThresholdPerPatchMinus1( bitstream.read( 6 ) );  // u(6)
      TRACE_BITSTREAM( "  BlockThresholdPerPatchMinus1 = %u \n", plri.getBlockThresholdPerPatchMinus1() );
    }
  }
}

// 7.2 Specification of syntax functions and descriptors
bool PCCBitstreamReader::byteAligned( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return bitstream.byteAligned();
}
bool PCCBitstreamReader::moreDataInPayload( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}
bool PCCBitstreamReader::moreRbspData( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}
bool PCCBitstreamReader::moreRbspTrailingData( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}
bool PCCBitstreamReader::moreDataInVpccUnit( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}
void PCCBitstreamReader::rbspTrailingBits( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.read( 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.read( 1 );  // f(1): equal to 0
  }
}
bool PCCBitstreamReader::payloadExtensionPresent( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}

// 7.3.6.3  Atlas frame parameter set Rbsp syntax
void PCCBitstreamReader::atlasFrameParameterSetRbsp( AtlasFrameParameterSetRbsp& afps,
                                                      PCCHighLevelSyntax&                 syntax,
                                                      PCCBitstream&               bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  afps.setAtlasFrameParameterSetId( bitstream.readUvlc() );     // ue(v)
  afps.setAtlasSequenceParameterSetId( bitstream.readUvlc() );  // ue(v)
  atlasFrameTileInformation( afps.getAtlasFrameTileInformation(), syntax.getVps(), bitstream );
  afps.setAfpsOutputFlagPresentFlag( bitstream.read( 1 ) );
  afps.setAfpsNumRefIdxDefaultActiveMinus1( bitstream.readUvlc() );  // ue(v)
  afps.setAfpsAdditionalLtAfocLsbLen( bitstream.readUvlc() );        // ue(v)
  afps.setAfpsOverrideEomForDepthFlag( bitstream.read( 1 ) );
  if ( afps.getAfpsOverrideEomForDepthFlag() ) {
    afps.setAfpsEomNumberOfPatchBitCountMinus1( bitstream.read( 4 ) );
    afps.setAfpsEomMaxBitCountMinus1( bitstream.read( 4 ) );
  }
  afps.setAfpsRaw3dPosBitCountExplicitModeFlag( bitstream.read( 1 ) );
  afps.setAfpsExtensionPresentFlag( bitstream.read( 1 ) );
  if ( afps.getAfpsExtensionPresentFlag() ) {
    while ( moreRbspData( bitstream ) ) { afps.setAfpsExtensionDataFlag( bitstream.read( 1 ) ); }
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.4  Atlas frame tile information syntax
void PCCBitstreamReader::atlasFrameTileInformation( AtlasFrameTileInformation& afti,
                                                     VpccParameterSet&          sps,
                                                     PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  afti.setSingleTileInAtlasFrameFlag( bitstream.read( 1 ) );  // u(1)
  if ( !afti.getSingleTileInAtlasFrameFlag() ) {
    afti.setUniformTileSpacingFlag( bitstream.read( 1 ) );  // u(1)
    if ( afti.getUniformTileSpacingFlag() ) {
      afti.setTileColumnWidthMinus1( 0, bitstream.readUvlc() );  //  ue(v)
      afti.setTileRowHeightMinus1( 0, bitstream.readUvlc() );    //  ue(v)
    } else {
      afti.setNumTileColumnsMinus1( bitstream.readUvlc() );  //  ue(v)
      afti.setNumTileRowsMinus1( bitstream.readUvlc() );     //  ue(v)
      for ( size_t i = 0; i < afti.getNumTileColumnsMinus1(); i++ ) {
        afti.setTileColumnWidthMinus1( i, bitstream.readUvlc() );  //  ue(v)
      }
      for ( size_t i = 0; i < afti.getNumTileRowsMinus1(); i++ ) {
        afti.setTileRowHeightMinus1( i, bitstream.readUvlc() );  //  ue(v)
      }
    }
  }

  afti.setSingleTilePerTileGroupFlag( bitstream.read( 1 ) );  //  u(1)
  if ( !afti.getSingleTilePerTileGroupFlag() ) {
    uint32_t NumTilesInPatchFrame = ( afti.getNumTileColumnsMinus1() + 1 ) * ( afti.getNumTileRowsMinus1() + 1 );

    afti.setNumTileGroupsInAtlasFrameMinus1( bitstream.readUvlc() );  // ue(v)
    for ( size_t i = 0; i <= afti.getNumTileGroupsInAtlasFrameMinus1(); i++ ) {
      uint8_t bitCount = ceilLog2( NumTilesInPatchFrame );
      if ( i > 0 ) {
        afti.setTopLeftTileIdx( i, bitstream.read( bitCount ) ); // u(v) 
      }  
      bitCount = ceilLog2( NumTilesInPatchFrame - afti.getTopLeftTileIdx( i ) );
      afti.setBottomRightTileIdxDelta( i, bitstream.read( bitCount ) );  // u(v) 
    }
  }
  afti.setSignalledTileGroupIdFlag( bitstream.read( 1 ) );  // u(1)
  if ( afti.getSignalledTileGroupIdFlag() ) {
    afti.setSignalledTileGroupIdLengthMinus1( bitstream.readUvlc() );  // ue(v)
    for ( size_t i = 0; i <= afti.getSignalledTileGroupIdLengthMinus1(); i++ ) {
      uint8_t bitCount = afti.getSignalledTileGroupIdLengthMinus1() + 1;
      afti.setTileGroupId( i, bitstream.read( bitCount ) );  // u(v)
    }
  }
}

// 7.3.6.5  Supplemental enhancement information Rbsp syntax
void PCCBitstreamReader::seiRbsp( PCCHighLevelSyntax& syntax, PCCBitstream& bitstream, NalUnitType nalUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  // do { seiMessage( syntax, bitstream ); } while ( moreRbspData( bitstream ) );
  seiMessage( bitstream, syntax, nalUnitType );
}

// 7.3.6.10  Atlas tile group layer Rbsp syntax = patchTileGroupLayerUnit
void PCCBitstreamReader::atlasTileGroupLayerRbsp( AtlasTileGroupLayerRbsp& atgl,
                                                   PCCHighLevelSyntax&              syntax,
                                                   PCCBitstream&            bitstream ) {
  // setFrameIndex
  TRACE_BITSTREAM( "%s \n", __func__ );
  atlasTileGroupHeader( atgl.getAtlasTileGroupHeader(), syntax, bitstream );
  if ( atgl.getAtlasTileGroupHeader().getAtghType() != SKIP_TILE_GRP ) {
    atlasTileGroupDataUnit( atgl.getAtlasTileGroupDataUnit(), atgl.getAtlasTileGroupHeader(), syntax, bitstream );
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.11  Atlas tile group header syntax
void PCCBitstreamReader::atlasTileGroupHeader( AtlasTileGroupHeader& atgh,
                                                PCCHighLevelSyntax&           syntax,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  atgh.setAtghAtlasFrameParameterSetId( bitstream.readUvlc() );
  size_t                         afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps   = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps   = syntax.getAtlasSequenceParameterSet( aspsId );
  AtlasFrameTileInformation&     afti   = afps.getAtlasFrameTileInformation();

  atgh.setAtghAddress( bitstream.read( afti.getSignalledTileGroupIdLengthMinus1() + 1 ) );
  atgh.setAtghType( PCCTILEGROUP( bitstream.readUvlc() ) );
  if( afps.getAfpsOutputFlagPresentFlag()) {
    atgh.setAtghAtlasOutputFlag( bitstream.read( 1 ) );
  } else {
    atgh.setAtghAtlasOutputFlag( false );
  }
  atgh.setAtghAtlasFrmOrderCntLsb( bitstream.read( asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4 ) );
  TRACE_BITSTREAM( " AtlasFrameParameterSetId: %zu\n", atgh.getAtghAtlasFrameParameterSetId() );
  TRACE_BITSTREAM( " AtlasSequenceParameterSetId: %zu\n", afps.getAtlasSequenceParameterSetId() );
  TRACE_BITSTREAM( " Address       %zu\n", (size_t)atgh.getAtghAddress() );
  TRACE_BITSTREAM( " Type          %zu\n", (size_t)atgh.getAtghType() );
  TRACE_BITSTREAM( " FrameOrderCnt %zu\n", (size_t)atgh.getAtghAtlasFrmOrderCntLsb() );
  if ( asps.getNumRefAtlasFrameListsInAsps() > 0 ) {
    atgh.setAtghRefAtlasFrameListSpsFlag( bitstream.read( 1 ) );
  } else {
    atgh.setAtghRefAtlasFrameListSpsFlag( 0 );
  }
  atgh.setAtghRefAtlasFrameListIdx( 0 );
  if ( atgh.getAtghRefAtlasFrameListSpsFlag() == 0 ) {
    refListStruct( atgh.getRefListStruct(), asps, bitstream );
  } else if ( asps.getNumRefAtlasFrameListsInAsps() > 1 ) {
    size_t bitCount = ceilLog2( asps.getNumRefAtlasFrameListsInAsps() );
    atgh.setAtghRefAtlasFrameListIdx( bitstream.read( bitCount ) );
  }
  if ( atgh.getAtghRefAtlasFrameListSpsFlag() ) {
    atgh.setRefListStruct( asps.getRefListStruct( atgh.getAtghRefAtlasFrameListIdx() ) );
  }
  uint8_t rlsIdx  = atgh.getAtghRefAtlasFrameListIdx();
  auto&   refList = atgh.getAtghRefAtlasFrameListSpsFlag() ? asps.getRefListStruct( rlsIdx ) : atgh.getRefListStruct();
  size_t  numLtrAtlasFrmEntries = 0;
  for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
    if ( !refList.getStRefAtalsFrameFlag( i ) ) { numLtrAtlasFrmEntries++; }
  }
  TRACE_BITSTREAM( " rlsIdx %u numLtrAtlasFrmEntries %zu \n", rlsIdx, (size_t)numLtrAtlasFrmEntries );
  for ( size_t j = 0; j < numLtrAtlasFrmEntries; j++ ) {
    atgh.setAtghAdditionalAfocLsbPresentFlag( j, bitstream.read( 1 ) );
    if ( atgh.getAtghAdditionalAfocLsbPresentFlag( j ) ) {
      uint8_t bitCount = afps.getAfpsAdditionalLtAfocLsbLen();
      atgh.setAtghAdditionalAfocLsbVal( j, bitstream.read( bitCount ) );
    }
  }
  if ( atgh.getAtghType() != SKIP_TILE_GRP ) {
    if ( asps.getNormalAxisLimitsQuantizationEnabledFlag() ) {
      atgh.setAtghPosMinZQuantizer( bitstream.read( 5 ) );
      TRACE_BITSTREAM( " AtghPosMinZQuantizer = %zu \n", atgh.getAtghPosMinZQuantizer() );
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) atgh.setAtghPosDeltaMaxZQuantizer( bitstream.read( 5 ) );
    }
    if ( asps.getPatchSizeQuantizerPresentFlag() ) {
      atgh.setAtghPatchSizeXinfoQuantizer( bitstream.read( 3 ) );
      atgh.setAtghPatchSizeYinfoQuantizer( bitstream.read( 3 ) );
    }
    auto& gi = syntax.getVps().getGeometryInformation( 0 );
    if ( afps.getAfpsRaw3dPosBitCountExplicitModeFlag() ) {
      size_t bitCount = ceilLog2( gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
      atgh.setAtghRaw3dPosAxisBitCountMinus1( bitstream.read( bitCount ) );
    } else {
      atgh.setAtghRaw3dPosAxisBitCountMinus1( gi.getGeometry3dCoordinatesBitdepthMinus1() -
                                              gi.getGeometryNominal2dBitdepthMinus1() - 1 );
    }
    if ( atgh.getAtghType() == P_TILE_GRP && refList.getNumRefEntries() > 1 ) {
      atgh.setAtghNumRefIdxActiveOverrideFlag( bitstream.read( 1 ) );
      if ( atgh.getAtghNumRefIdxActiveOverrideFlag() ) { atgh.setAtghNumRefIdxActiveMinus1( bitstream.readUvlc() ); }
    }
    TRACE_BITSTREAM( "==> AtghRaw3dPosAxisBitCountMinus1 = %lu \n", atgh.getAtghRaw3dPosAxisBitCountMinus1() );
    TRACE_BITSTREAM( "==> AtghNumRefIdxActiveOverrideFlag = %lu \n", atgh.getAtghNumRefIdxActiveOverrideFlag() );
    TRACE_BITSTREAM( "==> AtghNumRefIdxActiveMinus1       = %lu \n", atgh.getAtghNumRefIdxActiveMinus1() );
  }
  byteAlignment( bitstream );
}

// 7.3.6.12  Reference list structure syntax
void PCCBitstreamReader::refListStruct( RefListStruct&                 rls,
                                         AtlasSequenceParameterSetRbsp& asps,
                                         PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  rls.setNumRefEntries( bitstream.readUvlc() );  // ue(v)
  TRACE_BITSTREAM( "NumRefEntries = %lu  \n", rls.getNumRefEntries() );
  rls.allocate();
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( asps.getLongTermRefAtlasFramesFlag() ) {
      rls.setStRefAtalsFrameFlag( i, bitstream.read( 1 ) );  // u(1)
    } else {
      rls.setStRefAtalsFrameFlag( i, 1 );
    }
    if ( rls.getStRefAtalsFrameFlag( i ) ) {
      rls.setAbsDeltaAfocSt( i, bitstream.readUvlc() );  // ue(v)
      if ( rls.getAbsDeltaAfocSt( i ) > 0 ) {
        rls.setStrpfEntrySignFlag( i, bitstream.read( 1 ) );  // u(1)
      } else {
        rls.setStrpfEntrySignFlag( i, 1 );
      }
    } else {
      uint8_t bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
      rls.setAfocLsbLt( i, bitstream.read( bitCount ) );  // u(v)
    }
    TRACE_BITSTREAM( "stRefAtalsFrameFlag = %lu  \n", rls.getStRefAtalsFrameFlag( i ) );
    TRACE_BITSTREAM( "absDeltaAfocSt      = %lu  \n", rls.getAbsDeltaAfocSt( i ) );
    TRACE_BITSTREAM( "strpfEntrySignFlag  = %lu  \n", rls.getStrpfEntrySignFlag( i ) );
  }
}

// 7.3.7.1  General atlas tile group data unit syntax =patchTileGroupDataUnit
void PCCBitstreamReader::atlasTileGroupDataUnit( AtlasTileGroupDataUnit& atgdu,
                                                  AtlasTileGroupHeader&   atgh,
                                                  PCCHighLevelSyntax&             syntax,
                                                  PCCBitstream&           bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  atgdu.init();
  size_t patchIndex    = 0;
  auto   tileGroupType = atgh.getAtghType();
  TRACE_BITSTREAM( "atgh.getAtghType()        = %lu \n", atgh.getAtghType() );
  TRACE_BITSTREAM( "patch %lu : \n", patchIndex );
  uint8_t patchMode = bitstream.readUvlc();  // ue(v)
  TRACE_BITSTREAM( "patchMode = %lu \n", patchMode );
  prevPatchSizeU_ = 0;
  prevPatchSizeV_ = 0;
  predPatchIndex_ = 0;
  while ( !( ( ( PCCTILEGROUP( tileGroupType ) == I_TILE_GRP ) && ( patchMode == PATCH_MODE_I_END ) ) ||
             ( ( PCCTILEGROUP( tileGroupType ) == P_TILE_GRP ) && ( patchMode == PATCH_MODE_P_END ) ) ) ) {
    auto& pid = atgdu.addPatchInformationData( patchMode );
    pid.setFrameIndex( atgdu.getFrameIndex() );
    pid.setPatchIndex( patchIndex );
    patchIndex++;
    patchInformationData( pid, patchMode, atgh, syntax, bitstream );
    TRACE_BITSTREAM( "patch %lu : \n", patchIndex );
    patchMode = bitstream.readUvlc();  // ue(v)
    TRACE_BITSTREAM( "patchMode = %lu \n", patchMode );
  }
  prevFrameIndex_ = atgdu.getFrameIndex();
  if ( ( patchMode == PATCH_MODE_I_END ) || ( patchMode == PATCH_MODE_P_END ) ) {
    TRACE_BITSTREAM( "patchInformationData: AtghType = %lu patchMode = %lu \n", atgh.getAtghType(), patchMode );
  }
  TRACE_BITSTREAM( "atgdu.getPatchCount() including END = %lu \n", atgdu.getPatchCount() + 1 );
  byteAlignment( bitstream );
}

// 7.3.7.2  Patch information data syntax
void PCCBitstreamReader::patchInformationData( PatchInformationData& pid,
                                                size_t                patchMode,
                                                AtlasTileGroupHeader& atgh,
                                                PCCHighLevelSyntax&           syntax,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s: AtghType = %lu patchMode = %lu \n", __func__, atgh.getAtghType(), patchMode );
  if ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == P_TILE_GRP && patchMode == PATCH_MODE_P_SKIP ) {
  } else if ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == P_TILE_GRP && patchMode == PATCH_MODE_P_MERGE ) {
    auto& mpdu = pid.getMergePatchDataUnit();
    mpdu.setFrameIndex( pid.getFrameIndex() );
    mpdu.setPatchIndex( pid.getPatchIndex() );
    mergePatchDataUnit( mpdu, atgh, syntax, bitstream );
  } else if ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == P_TILE_GRP && patchMode == PATCH_MODE_P_INTER ) {
    auto& ipdu = pid.getInterPatchDataUnit();
    ipdu.setFrameIndex( pid.getFrameIndex() );
    ipdu.setPatchIndex( pid.getPatchIndex() );
    interPatchDataUnit( ipdu, atgh, syntax, bitstream );
  } else if ( ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == I_TILE_GRP && patchMode == PATCH_MODE_I_INTRA ) ||
              ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == P_TILE_GRP && patchMode == PATCH_MODE_P_INTRA ) ) {
    auto& pdu = pid.getPatchDataUnit();
    pdu.setFrameIndex( pid.getFrameIndex() );
    pdu.setPatchIndex( pid.getPatchIndex() );
    patchDataUnit( pdu, atgh, syntax, bitstream );
  } else if ( ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == I_TILE_GRP && patchMode == PATCH_MODE_I_RAW ) ||
              ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == P_TILE_GRP && patchMode == PATCH_MODE_P_RAW ) ) {
    auto& rpdu = pid.getRawPatchDataUnit();
    rpdu.setFrameIndex( pid.getFrameIndex() );
    rpdu.setPatchIndex( pid.getPatchIndex() );
    rawPatchDataUnit( rpdu, atgh, syntax, bitstream );
  } else if ( ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == I_TILE_GRP && patchMode == PATCH_MODE_I_EOM ) ||
              ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == P_TILE_GRP && patchMode == PATCH_MODE_P_EOM ) ) {
    auto& epdu = pid.getEomPatchDataUnit();
    epdu.setFrameIndex( pid.getFrameIndex() );
    epdu.setPatchIndex( pid.getPatchIndex() );
    eomPatchDataUnit( epdu, atgh, syntax, bitstream );
  }
}

// 7.3.7.3  Patch data unit syntax : AtlasTileGroupHeader instead of PatchTileGroupHeader
void PCCBitstreamReader::patchDataUnit( PatchDataUnit&        pdu,
                                         AtlasTileGroupHeader& atgh,
                                         PCCHighLevelSyntax&           syntax,
                                         PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps   = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps   = syntax.getAtlasSequenceParameterSet( aspsId );
  pdu.setPduProjectionId( bitstream.read( asps.get45DegreeProjectionPatchPresentFlag() ? 5 : 3 ) ); // u(5 or 3)
  TRACE_BITSTREAM( "PduProjectionId = %lu (45DegreeProjectionPatchPresentFlag = %d ) \n", 
    pdu.getPduProjectionId(),asps.get45DegreeProjectionPatchPresentFlag()  );
  pdu.setPdu2dPosX( bitstream.readUvlc() );  // ue(v)
  pdu.setPdu2dPosY( bitstream.readUvlc() );  // ue(v)
  TRACE_BITSTREAM( " 2dPosXY: %zu,%zu\n", pdu.getPdu2dPosX(), pdu.getPdu2dPosX() );
  pdu.setPdu2dSizeXMinus1( bitstream.readUvlc() );  // ue(v)
  pdu.setPdu2dSizeYMinus1( bitstream.readUvlc() );  // ue(v)
  TRACE_BITSTREAM( " 2dSizeXY: %d,%d\n", int32_t( pdu.getPdu2dSizeXMinus1() + 1 ),
                   int32_t( pdu.getPdu2dSizeYMinus1() +1 ) );
  uint8_t bitCount3DPos=
   syntax.getVps(0).getGeometryInformation(0).getGeometry3dCoordinatesBitdepthMinus1() +1;
  pdu.setPdu3dPosX( bitstream.read( bitCount3DPos ) );  // u(v)
  pdu.setPdu3dPosY( bitstream.read( bitCount3DPos ) );  // u(v)
  TRACE_BITSTREAM( " 3dPosXY: %zu,%zu\n", pdu.getPdu3dPosX(), pdu.getPdu3dPosY() );

  const uint8_t bitCountForMinDepth =
      syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
      atgh.getAtghPosMinZQuantizer() + ( pdu.getPduProjectionId() > 5 ? 2 : 1 );
  pdu.setPdu3dPosMinZ( bitstream.read( bitCountForMinDepth ) );  // u(v)  
  TRACE_BITSTREAM( " Pdu3dPosMinZ: %zu ( bitCountForMinDepth = %u = %u - %u + %u ) \n", 
      pdu.getPdu3dPosMinZ(), bitCountForMinDepth,
      syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1(), 
      atgh.getAtghPosMinZQuantizer(), pdu.getPduProjectionId() > 5 ? 2 : 1  );

  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    uint8_t bitCountForMaxDepth =
        syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
        atgh.getAtghPosDeltaMaxZQuantizer() + ( pdu.getPduProjectionId() > 5 ? 2 : 1 );
    if ( asps.get45DegreeProjectionPatchPresentFlag() ) bitCountForMaxDepth++;
    pdu.setPdu3dPosDeltaMaxZ( bitstream.read( bitCountForMaxDepth ) );  // u(v)
    TRACE_BITSTREAM( " Pdu3dPosDeltaMaxZ: %zu ( bitCountForMaxDepth = %u) \n",
        pdu.getPdu3dPosDeltaMaxZ(), bitCountForMaxDepth  );
  }
  pdu.setPduOrientationIndex( bitstream.read( ( asps.getUseEightOrientationsFlag() ? 3 : 1 ) ) );  // u(3 or 1)
  if ( afps.getLodModeEnableFlag() ) {
    pdu.setLodEnableFlag( bitstream.read( 1 ) );  // u1
    if ( pdu.getLodEnableFlag() ) {
      pdu.setLodScaleXminus1( uint8_t( bitstream.readUvlc() ) );  // uev
      pdu.setLodScaleY( uint8_t( bitstream.readUvlc() ) );        // uev
    }
  } else {
    pdu.setLodEnableFlag( 0 );
    pdu.setLodScaleXminus1( 0 );
    pdu.setLodScaleY( 0 );
  }

  TRACE_BITSTREAM( "PointLocalReconstructionEnabledFlag = %d \n", asps.getPointLocalReconstructionEnabledFlag() );
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plrd = pdu.getPointLocalReconstructionData();
    TRACE_BITSTREAM( "Size = %ld %ld\n",
                     pdu.getPdu2dSizeYMinus1()+1, pdu.getPdu2dSizeYMinus1()+1 );
    plrd.allocate( pdu.getPdu2dSizeXMinus1()+1, pdu.getPdu2dSizeYMinus1()+1 );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
  }
  TRACE_BITSTREAM(
      "Frame %zu, Patch(%zu) => 2Dpos = %4lu %4lu 2Dsize = %4ld %4ld 3Dpos = %ld %ld %ld DeltaMaxZ = %ld Projection = %zu "
      "Orientation = %zu lod= (%lu) %lu %lu\n ",
      pdu.getFrameIndex(), pdu.getPatchIndex(), pdu.getPdu2dPosX(), pdu.getPdu2dPosY(), pdu.getPdu2dSizeXMinus1()+1,
      pdu.getPdu2dSizeYMinus1()+1, pdu.getPdu3dPosX(), pdu.getPdu3dPosY(), pdu.getPdu3dPosMinZ(),
      pdu.getPdu3dPosDeltaMaxZ(), pdu.getPduProjectionId(), pdu.getPduOrientationIndex(), pdu.getLodEnableFlag(),
      pdu.getLodEnableFlag() ? pdu.getLodScaleXminus1() : (uint8_t)0,
      pdu.getLodEnableFlag() ? pdu.getLodScaleY() : (uint8_t)0 );
}

// 7.3.7.4  Skip patch data unit syntax
void PCCBitstreamReader::skipPatchDataUnit( SkipPatchDataUnit&    spdu,
                                             AtlasTileGroupHeader& atgh,
                                             PCCHighLevelSyntax&           syntax,
                                             PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}

// 7.3.7.5  Merge patch data unit syntax
void PCCBitstreamReader::mergePatchDataUnit( MergePatchDataUnit&   mpdu,
                                              AtlasTileGroupHeader& atgh,
                                              PCCHighLevelSyntax&           syntax,
                                              PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId          = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps            = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId          = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps            = syntax.getAtlasSequenceParameterSet( aspsId );
  bool                           overridePlrFlag = false;
  size_t                         numRefIdxActive = syntax.getNumRefIdxActive( atgh );
  if ( numRefIdxActive > 1 ) {
    mpdu.setMpduRefIndex( bitstream.readUvlc() );
  } else {
    mpdu.setMpduRefIndex( 0 );
  }
  mpdu.setMpduOverride2dParamsFlag( bitstream.read( 1 ) );
  if ( mpdu.getMpduOverride2dParamsFlag() ) {
    mpdu.setMpdu2dPosX( bitstream.readSvlc() );        // se(v)
    mpdu.setMpdu2dPosY( bitstream.readSvlc() );        // se(v)
    mpdu.setMpdu2dDeltaSizeX( bitstream.readSvlc() );  // se(v)
    mpdu.setMpdu2dDeltaSizeY( bitstream.readSvlc() );  // se(v)
    if ( asps.getPointLocalReconstructionEnabledFlag() ) overridePlrFlag = true;
  } else {
    mpdu.setMpduOverride3dParamsFlag( bitstream.read( 1 ) );
    if ( mpdu.getMpduOverride3dParamsFlag() ) {
      mpdu.setMpdu3dPosX( bitstream.readSvlc() );     // se(v)
      mpdu.setMpdu3dPosY( bitstream.readSvlc() );     // se(v)
      mpdu.setMpdu3dPosMinZ( bitstream.readSvlc() );  // se(v)
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        mpdu.setMpdu3dPosDeltaMaxZ( bitstream.readSvlc() );  // se(v)
      }
      if ( asps.getPointLocalReconstructionEnabledFlag() ) {
        overridePlrFlag = bitstream.read( 1 );
        mpdu.setMpduOverridePlrFlag( overridePlrFlag );
      }
    }
  } 
  if ( overridePlrFlag && asps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plrd = mpdu.getPointLocalReconstructionData();
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n", prevPatchSizeU_, prevPatchSizeV_,
                     mpdu.getMpdu2dDeltaSizeX(), mpdu.getMpdu2dDeltaSizeY(),
                     prevPatchSizeU_ + mpdu.getMpdu2dDeltaSizeX(), prevPatchSizeV_ + mpdu.getMpdu2dDeltaSizeY() );
    plrd.allocate( prevPatchSizeU_ + mpdu.getMpdu2dDeltaSizeX(), prevPatchSizeV_ + mpdu.getMpdu2dDeltaSizeY() );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
    prevPatchSizeU_ += mpdu.getMpdu2dDeltaSizeX();
    prevPatchSizeV_ += mpdu.getMpdu2dDeltaSizeY();
  }
  TRACE_BITSTREAM(
      "%zu frame %zu MergePatch => Ref Frame Idx = %d ShiftUV = %ld %ld DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      mpdu.getMpduRefIndex(), mpdu.getMpdu2dPosX(), mpdu.getMpdu2dPosY(), mpdu.getMpdu2dDeltaSizeX(),
      mpdu.getMpdu2dDeltaSizeY(), mpdu.getMpdu3dPosX(), mpdu.getMpdu3dPosY(), mpdu.getMpdu3dPosMinZ(),
      mpdu.getMpdu3dPosDeltaMaxZ() );
}

// 7.3.7.6  Inter patch data unit syntax
void PCCBitstreamReader::interPatchDataUnit( InterPatchDataUnit&   ipdu,
                                              AtlasTileGroupHeader& atgh,
                                              PCCHighLevelSyntax&           syntax,
                                              PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId          = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps            = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId          = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps            = syntax.getAtlasSequenceParameterSet( aspsId );
  size_t                         numRefIdxActive = syntax.getNumRefIdxActive( atgh );
  if ( numRefIdxActive > 1 ) {
    ipdu.setIpduRefIndex( bitstream.readUvlc() );
  } else {
    ipdu.setIpduRefIndex( 0 );
  }
  ipdu.setIpduRefPatchIndex( bitstream.readSvlc() );
  ipdu.setIpdu2dPosX( bitstream.readSvlc() );        // se(v)
  ipdu.setIpdu2dPosY( bitstream.readSvlc() );        // se(v)
  ipdu.setIpdu2dDeltaSizeX( bitstream.readSvlc() );  // se(v)
  ipdu.setIpdu2dDeltaSizeY( bitstream.readSvlc() );  // se(v)
  ipdu.setIpdu3dPosX( bitstream.readSvlc() );        // se(v)
  ipdu.setIpdu3dPosY( bitstream.readSvlc() );        // se(v)
  ipdu.setIpdu3dPosMinZ( bitstream.readSvlc() );     // se(v)
  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    ipdu.setIpdu3dPosDeltaMaxZ( bitstream.readSvlc() );  // se(v)
  }
  TRACE_BITSTREAM(
      "%zu frame: numRefIdxActive = %zu reference = frame%zu patch%d 2Dpos = %ld %ld 2DdeltaSize = %ld %ld 3Dpos = %ld "
      "%ld %ld DeltaMaxZ = %ld\n",
      ipdu.getFrameIndex(), numRefIdxActive, ipdu.getIpduRefIndex(), ipdu.getIpduRefPatchIndex(), ipdu.getIpdu2dPosX(),
      ipdu.getIpdu2dPosY(), ipdu.getIpdu2dDeltaSizeX(), ipdu.getIpdu2dDeltaSizeY(), ipdu.getIpdu3dPosX(),
      ipdu.getIpdu3dPosY(), ipdu.getIpdu3dPosMinZ(), ipdu.getIpdu3dPosDeltaMaxZ() );
  
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    auto&   atglPrev      = syntax.getAtlasTileGroupLayer( prevFrameIndex_ );
    auto&   atghPrev      = atglPrev.getAtlasTileGroupHeader();
    auto&   atgdPrev      = atglPrev.getAtlasTileGroupDataUnit();
    auto&   pidPrev       = atgdPrev.getPatchInformationData( ipdu.getIpduRefPatchIndex() + predPatchIndex_ );
    auto    patchModePrev = pidPrev.getPatchMode();
    int32_t sizeU         = ipdu.getIpdu2dDeltaSizeX();
    int32_t sizeV         = ipdu.getIpdu2dDeltaSizeY();
    if ( ( PCCTILEGROUP( atghPrev.getAtghType() ) ) == P_TILE_GRP && patchModePrev == PATCH_MODE_P_SKIP ) {
    } else if ( ( PCCTILEGROUP( atghPrev.getAtghType() ) ) == P_TILE_GRP && patchModePrev == PATCH_MODE_P_MERGE ) {
      auto& plrdPrev = pidPrev.getMergePatchDataUnit().getPointLocalReconstructionData();
      sizeU += plrdPrev.getBlockToPatchMapWidth();
      sizeV += plrdPrev.getBlockToPatchMapHeight();
    } else if ( ( PCCTILEGROUP( atghPrev.getAtghType() ) ) == P_TILE_GRP && patchModePrev == PATCH_MODE_P_INTER ) {
      auto& plrdPrev = pidPrev.getInterPatchDataUnit().getPointLocalReconstructionData();
      sizeU += plrdPrev.getBlockToPatchMapWidth();
      sizeV += plrdPrev.getBlockToPatchMapHeight();
    } else if ( ( ( PCCTILEGROUP( atghPrev.getAtghType() ) ) == I_TILE_GRP && patchModePrev == PATCH_MODE_I_INTRA ) ||
                ( ( PCCTILEGROUP( atghPrev.getAtghType() ) ) == P_TILE_GRP && patchModePrev == PATCH_MODE_P_INTRA ) ) {
      auto& plrdPrev = pidPrev.getPatchDataUnit().getPointLocalReconstructionData();
      sizeU += plrdPrev.getBlockToPatchMapWidth();
      sizeV += plrdPrev.getBlockToPatchMapHeight();
    }
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n", sizeU - ipdu.getIpdu2dDeltaSizeX(),
                     sizeV - ipdu.getIpdu2dDeltaSizeY(), ipdu.getIpdu2dDeltaSizeX(), ipdu.getIpdu2dDeltaSizeY(), sizeU,
                     sizeV );
    auto& plrd = ipdu.getPointLocalReconstructionData();
    plrd.allocate( sizeU, sizeV );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
    prevPatchSizeU_ = sizeU;
    prevPatchSizeV_ = sizeV;
    predPatchIndex_ += ipdu.getIpduRefPatchIndex() + 1;
  }

}

// 7.3.7.7  Raw patch data unit syntax
void PCCBitstreamReader::rawPatchDataUnit( RawPatchDataUnit&     rpdu,
                                            AtlasTileGroupHeader& atgh,
                                            PCCHighLevelSyntax&           syntax,
                                            PCCBitstream&         bitstream ) {
  auto& sps = syntax.getVps();
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                      atlasIndex = 0;
  if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
    rpdu.setRpduPatchInRawVideoFlag( bitstream.read( 1 ) );
  }  // u(1)
  rpdu.setRpdu2dPosX( bitstream.readUvlc() );  // ue(v)
  rpdu.setRpdu2dPosY( bitstream.readUvlc() );  // ue(v)
  rpdu.setRpdu2dSizeXMinus1( bitstream.readUvlc() );                               // ue(v)
  rpdu.setRpdu2dSizeYMinus1( bitstream.readUvlc() );                               // ue(v)
  TRACE_BITSTREAM( " AtghRaw3dPosAxisBitCountMinus1 = %lu \n", atgh.getAtghRaw3dPosAxisBitCountMinus1() );
  rpdu.setRpdu3dPosX( bitstream.read( atgh.getAtghRaw3dPosAxisBitCountMinus1() + 1 ) );  // u(v)
  rpdu.setRpdu3dPosY( bitstream.read( atgh.getAtghRaw3dPosAxisBitCountMinus1() + 1 ) );  // u(v)
  rpdu.setRpdu3dPosZ( bitstream.read( atgh.getAtghRaw3dPosAxisBitCountMinus1() + 1 ) );  // u(v)
  rpdu.setRpduRawPointsMinus1( bitstream.readUvlc() );
  TRACE_BITSTREAM(
      "Raw Patch => UV %4lu %4lu  S=%4ld %4ld  UVD1=%4ld %4ld %4ld NumPcmPoints=%zu PatchInRawVideoFlag=%d \n",
      rpdu.getRpdu2dPosX(), rpdu.getRpdu2dPosY(), rpdu.getRpdu2dSizeXMinus1()+1, rpdu.getRpdu2dSizeYMinus1()+1,
      rpdu.getRpdu3dPosX(), rpdu.getRpdu3dPosY(), rpdu.getRpdu3dPosZ(), (size_t)rpdu.getRpduRawPointsMinus1()+1,
      rpdu.getRpduPatchInRawVideoFlag() );
}

// 7.3.6.x EOM patch data unit syntax
void PCCBitstreamReader::eomPatchDataUnit( EOMPatchDataUnit&     epdu,
                                            AtlasTileGroupHeader& atgh,
                                            PCCHighLevelSyntax&           syntax,
                                            PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  epdu.setEpdu2dPosX( bitstream.readUvlc() );        // ue(v)
  epdu.setEpdu2dPosY( bitstream.readUvlc() );        // ue(v)
  epdu.setEpdu2dSizeXMinus1( bitstream.readUvlc() ); // ue(v)
  epdu.setEpdu2dSizeYMinus1( bitstream.readUvlc() ); // ue(v)
  epdu.setEpduAssociatedPatchesCountMinus1( bitstream.read( 8 ) );
  for ( size_t cnt = 0; cnt < epdu.getEpduAssociatedPatchesCountMinus1() + 1; cnt++ ) {
    epdu.setEpduAssociatedPatches( bitstream.read( 8 ), cnt );
    epdu.setEpduEomPointsPerPatch( bitstream.readUvlc(), cnt );  //???
  }
#ifdef BITSTREAM_TRACE
  TRACE_BITSTREAM( "EOM Patch => UV %4lu %4lu  S=%4ld %4ld  N=%4ld\n", epdu.getEpdu2dPosX(), epdu.getEpdu2dPosY(),epdu.getEpdu2dSizeXMinus1()+1,epdu.getEpdu2dSizeYMinus1()+1,
                   epdu.getEpduAssociatedPatchesCountMinus1() + 1 );
  for ( size_t cnt = 0; cnt < epdu.getEpduAssociatedPatchesCountMinus1() + 1; cnt++ ) {
    TRACE_BITSTREAM( "%4ld, %4ld\n", epdu.getEpduAssociatedPatches()[cnt], epdu.getEpduEomPointsPerPatch()[cnt] );
  }
#endif
}

void PCCBitstreamReader::atlasSubStream( PCCHighLevelSyntax& syntax, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  int64_t             sizeBitstream = bitstream.capacity();
  SampleStreamNalUnit ssnu;
  sampleStreamNalHeader( bitstream, ssnu );
  while ( bitstream.size() < sizeBitstream ) {
    ssnu.addNalUnit();
    sampleStreamNalUnit( syntax, bitstream, ssnu, ssnu.getNalUnit().size() - 1 );
#ifdef BITSTREAM_TRACE
    auto& nu = ssnu.getNalUnit( ssnu.getNalUnit().size() - 1 );
    TRACE_BITSTREAM( "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream written: %llu\n",
                     (int)nu.getNalUnitType(), toString( nu.getNalUnitType() ).c_str(),
                     ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ), nu.getNalUnitSize(), bitstream.size() );
#endif
  }
}

// 7.3.6.6 Point local reconstruction data syntax
void PCCBitstreamReader::pointLocalReconstructionData( PointLocalReconstructionData&  plrd,
                                                        PCCHighLevelSyntax&                    syntax,
                                                        AtlasSequenceParameterSetRbsp& asps,
                                                        PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         plri         = asps.getPointLocalReconstructionInformation( 0 );
  const size_t  blockCount   = plrd.getBlockToPatchMapWidth() * plrd.getBlockToPatchMapHeight();
  const uint8_t bitCountMode = uint8_t( ceilLog2( uint32_t( plri.getNumberOfModesMinus1() ) ) );
  TRACE_BITSTREAM( "WxH= %lu x %lu => blockCount = %lu \n", plrd.getBlockToPatchMapWidth(),
                   plrd.getBlockToPatchMapHeight(), blockCount );
  TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u  => bitCountMode = %u  \n", plri.getNumberOfModesMinus1(),
                   bitCountMode );
  if ( blockCount > plri.getBlockThresholdPerPatchMinus1() + 1 ) {
    plrd.setLevelFlag( bitstream.read( 1 ) );
  } else {
    plrd.setLevelFlag( true );
  }
  TRACE_BITSTREAM( "  LevelFlag = %u \n", plrd.getLevelFlag() );
  if ( plrd.getLevelFlag() ) {
    plrd.setPresentFlag( bitstream.read( 1 ) );
    if ( plrd.getPresentFlag() ) { plrd.setModeMinus1( bitstream.read( bitCountMode ) ); }
    TRACE_BITSTREAM( "  ModePatch: Present = %d ModeMinus1 = %d \n", plrd.getPresentFlag(),
                     plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1 );
  } else {
    for ( size_t i = 0; i < blockCount; i++ ) {
      plrd.setBlockPresentFlag( i, bitstream.read( 1 ) );
      if ( plrd.getBlockPresentFlag( i ) ) { plrd.setBlockModeMinus1( i, bitstream.read( bitCountMode ) ); }
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
                       plrd.getLevelFlag() ? plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1
                                           : plrd.getBlockPresentFlag( i ) ? plrd.getBlockModeMinus1( i ) : -1 );
    }
  }
#endif
}

// 7.3.8 Supplemental enhancement information message syntax
void PCCBitstreamReader::seiMessage( PCCBitstream& bitstream, PCCHighLevelSyntax& syntax, NalUnitType nalUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  int32_t payloadType = 0, payloadSize = 0, byte = 0;
  do {
    byte = bitstream.read( 8 );  // u(8)
    payloadType += byte;
  } while ( byte == 0xff );
  do {
    byte = bitstream.read( 8 );  // u(8)
    payloadSize += byte;
  } while ( byte == 0xff );
  seiPayload( bitstream, syntax, nalUnitType, (SeiPayloadType)payloadType, payloadSize );
}

// 7.3.5 NAL unit syntax
// 7.3.5.1 General NAL unit syntax
void PCCBitstreamReader::nalUnit( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  nalUnitHeader( bitstream, nalUnit );
  for ( size_t i = 0; i < nalUnit.getNalUnitSize() - 2; i++ ) {
    nalUnit.setNalUnitData( i, bitstream.read( 8 ) );  // b(8)
  }
}

// 7.3.5.2 NAL unit header syntax
void PCCBitstreamReader::nalUnitHeader( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.read( 1 );                                         // f(1)
  nalUnit.setNalUnitType( (NalUnitType)bitstream.read( 6 ) );  // u(6)
  nalUnit.setLayerId( bitstream.read( 6 ) );                   // u(6)
  nalUnit.setTemporalyIdPlus1( bitstream.read( 3 ) );          // u(3)
  TRACE_BITSTREAM( " NalUnitSize      = %lu \n", nalUnit.getNalUnitSize() );
  TRACE_BITSTREAM( " NalUnitType      = %hhu \n", nalUnit.getNalUnitType() );
  TRACE_BITSTREAM( " LayerId          = %hhu \n", nalUnit.getLayerId() );
  TRACE_BITSTREAM( " TemporalyIdPlus1 = %hhu \n", nalUnit.getTemporalyIdPlus1() );
}

// C.2 Sample stream NAL unit syntax and semantics
// C.2.1 Sample stream NAL header syntax
void PCCBitstreamReader::sampleStreamNalHeader( PCCBitstream& bitstream, SampleStreamNalUnit& ssnu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ssnu.setUnitSizePrecisionBytesMinus1( bitstream.read( 3 ) );  // u(3)
  bitstream.read( 5 );                                          // u(5)
}

// C.2.2 Sample stream NAL unit syntax
void PCCBitstreamReader::sampleStreamNalUnit( PCCHighLevelSyntax&          syntax,
                                               PCCBitstream&        bitstream,
                                               SampleStreamNalUnit& ssnu,
                                               size_t               index ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& nu = ssnu.getNalUnit( index );
  nu.setNalUnitSize( bitstream.read( 8 * ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ) ) );  // u(v)
  nu.allocate();
  nalUnitHeader( bitstream, nu );
  switch ( nu.getNalUnitType() ) {
    case NAL_ASPS: atlasSequenceParameterSetRbsp( syntax.addAtlasSequenceParameterSet(), syntax, bitstream ); break;
    case NAL_AFPS: atlasFrameParameterSetRbsp( syntax.addAtlasFrameParameterSet(), syntax, bitstream ); break;
    case NAL_TRAIL:
    case NAL_TSA:
    case NAL_STSA:
    case NAL_RADL:
    case NAL_RASL:
    case NAL_SKIP: atlasTileGroupLayerRbsp( syntax.addAtlasTileGroupLayer(), syntax, bitstream ); break;
    case NAL_SUFFIX_SEI: seiRbsp( syntax, bitstream, nu.getNalUnitType() ); break;
    case NAL_PREFIX_SEI: seiRbsp( syntax, bitstream, nu.getNalUnitType() ); break;
    default: fprintf( stderr, "sampleStreamNalUnit type = %d not supported\n", (int32_t)nu.getNalUnitType() );
  }
}

// E.2  SEI payload syntax
// E.2.1  General SEI message syntax
void PCCBitstreamReader::seiPayload( PCCBitstream&  bitstream,
                                      PCCHighLevelSyntax&    syntax,
                                      NalUnitType    nalUnitType,
                                      SeiPayloadType payloadType,
                                      size_t         payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEI& sei = syntax.addSei( nalUnitType, payloadType );
  if ( nalUnitType == NAL_PREFIX_SEI ) {
    if ( payloadType == 0 ) {
      bool                 NalHrdBpPresentFlag = false;
      bool                 AclHrdBpPresentFlag = false;
      std::vector<uint8_t> hrdCabCntMinus1;
      bufferingPeriod( bitstream, sei, payloadSize, NalHrdBpPresentFlag, AclHrdBpPresentFlag, hrdCabCntMinus1 );
    } else if ( payloadType == 1 ) {
      atlasFrameTiming( bitstream, sei, payloadSize, false );
    } else if ( payloadType == 2 ) {
      fillerPayload( bitstream, sei, payloadSize );
    } else if ( payloadType == 3 ) {
      userDataRegisteredItuTT35( bitstream, sei, payloadSize );
    } else if ( payloadType == 4 ) {
      userDataUnregistered( bitstream, sei, payloadSize );
    } else if ( payloadType == 5 ) {
      recoveryPoint( bitstream, sei, payloadSize );
    } else if ( payloadType == 6 ) {
      noDisplay( bitstream, sei, payloadSize );
    } else if ( payloadType == 7 ) {
      // timeCode( bitstream, sei, payloadSize );
    } else if ( payloadType == 8 ) {
      // regionalNesting( bitstream, sei, payloadSize );
    } else if ( payloadType == 9 ) {
      seiManifest( bitstream, sei, payloadSize );
    } else if ( payloadType == 10 ) {
      seiPrefixIndication( bitstream, sei, payloadSize );
    } else if ( payloadType == 11 ) {
      geometryTransformationParams( bitstream, sei, payloadSize );
    } else if ( payloadType == 12 ) {
      attributeTransformationParams( bitstream, sei, payloadSize );
    } else if ( payloadType == 13 ) {
      activeSubstreams( bitstream, sei, payloadSize );
    } else if ( payloadType == 14 ) {
      componentCodecMapping( bitstream, sei, payloadSize );
    } else if ( payloadType == 15 ) {
      volumetricTilingInfo( bitstream, sei, payloadSize );
    } else if ( payloadType == 16 ) {
      presentationInformation( bitstream, sei, payloadSize );
    } else if ( payloadType == 17 ) {
      smoothingParameters( bitstream, sei, payloadSize );
    } else {
      reservedSeiMessage( bitstream, sei, payloadSize );
    }
  } else { /* psdUnitType  ==  NAL_SUFFIX_SEI */
    if ( payloadType == 2 ) {
      fillerPayload( bitstream, sei, payloadSize );
    } else if ( payloadType == 3 ) {
      userDataRegisteredItuTT35( bitstream, sei, payloadSize );
    } else if ( payloadType == 4 ) {
      userDataUnregistered( bitstream, sei, payloadSize );
    } else {
      reservedSeiMessage( bitstream, sei, payloadSize );
    }
  }
  // if ( moreDataInPayload() ) {
  //   if ( payloadExtensionPresent() ) {
  //     spReservedPayloadExtensionData;  // u(v)
  //   }
  //   byteAlignment( bitstream );
  // }
  if(!bitstream.byteAligned()) // this prevents from writing one more byte, in case the payload is already byte aligned (see xWriteByteAlign in HM)
  byteAlignment( bitstream );
}

// E.2.2  Filler payload SEI message syntax
void PCCBitstreamReader::fillerPayload( PCCBitstream& bitstream, SEI& sei, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  for ( size_t k = 0; k < payloadSize; k++ ) {
    bitstream.read( 8 );  // f(8) equal to 0xFF
  }
}

// E.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
void PCCBitstreamReader::userDataRegisteredItuTT35( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIUserDataRegisteredItuTT35& sei = static_cast<SEIUserDataRegisteredItuTT35&>( seiAbstract );
  sei.setItuTT35CountryCode( bitstream.read( 8 ) );  // b(8)
  payloadSize--;
  if ( sei.getItuTT35CountryCode() == 0xFF ) {
    sei.setItuTT35CountryCodeExtensionByte( bitstream.read( 8 ) );  // b(8)
    payloadSize--;
  }
  auto& payload = sei.getItuTT35PayloadByte();
  payload.resize( payloadSize );
  for ( auto& element : payload ) {
    element = bitstream.read( 8 );  // b(8)
  }
}

// E.2.4  User data unregistered SEI message syntax
void PCCBitstreamReader::userDataUnregistered( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIUserDataUnregistered& sei = static_cast<SEIUserDataUnregistered&>( seiAbstract );
  for ( size_t i = 0; i < 16; i++ ) {
    sei.setUuidIsoIec11578( i, bitstream.read( 8 ) );  // u(128) <=> 16 * u(8)
  }
  payloadSize -= 16;
  sei.getUserDataPayloadByte().resize( payloadSize );
  for ( size_t i = 0; i < payloadSize; i++ ) {
    sei.setUserDataPayloadByte( i, bitstream.read( 8 ) );  // b(8)
  }
}

// E.2.5  Recovery point SEI message syntax
void PCCBitstreamReader::recoveryPoint( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIRecoveryPoint& sei = static_cast<SEIRecoveryPoint&>( seiAbstract );
  sei.setRecoveryAfocCnt( bitstream.readSvlc() );  // se(v)
  sei.setExactMatchFlag( bitstream.read( 1 ) );    // u(1)
  sei.setBrokenLinkFlag( bitstream.read( 1 ) );    // u(1)
}

// E.2.6  No display SEI message syntax
void PCCBitstreamReader::noDisplay( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}

// E.2.7  Reserved SEI message syntax
void PCCBitstreamReader::reservedSeiMessage( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIReservedSeiMessage& sei = static_cast<SEIReservedSeiMessage&>( seiAbstract );
  sei.getReservedSeiMessagePayloadByte().resize( payloadSize );
  for ( size_t i = 0; i < payloadSize; i++ ) {
    sei.setReservedSeiMessagePayloadByte( i, bitstream.read( 8 ) );  // b(8)
  }
}

// E.2.8  SEI manifest SEI message syntax
void PCCBitstreamReader::seiManifest( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIManifest& sei = static_cast<SEIManifest&>( seiAbstract );
  sei.setManifestNumSeiMsgTypes( bitstream.read( 16 ) );  // u(16)
  sei.allocate();
  for ( size_t i = 0; i < sei.getManifestNumSeiMsgTypes(); i++ ) {
    sei.setManifestSeiPayloadType( i, bitstream.read( 16 ) );  // u(16)
    sei.setManifestSeiDescription( i, bitstream.read( 8 ) );   // u(8)
  }
}

// E.2.9  SEI prefix indication SEI message syntax
void PCCBitstreamReader::seiPrefixIndication( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIPrefixIndication& sei = static_cast<SEIPrefixIndication&>( seiAbstract );
  sei.setPrefixSeiPayloadType( bitstream.read( 16 ) );          // u(16)
  sei.setNumSeiPrefixIndicationsMinus1( bitstream.read( 8 ) );  // u(8)
  sei.getNumBitsInPrefixIndicationMinus1().resize( sei.getNumSeiPrefixIndicationsMinus1() + 1, 0 );
  sei.getSeiPrefixDataBit().resize( sei.getNumSeiPrefixIndicationsMinus1() + 1 );
  for ( size_t i = 0; i <= sei.getNumSeiPrefixIndicationsMinus1(); i++ ) {
    sei.setNumBitsInPrefixIndicationMinus1( i, bitstream.read( 16 ) );  // u(16)
    sei.getSeiPrefixDataBit( i ).resize( sei.getNumBitsInPrefixIndicationMinus1( i ), false );
    for ( size_t j = 0; j <= sei.getNumBitsInPrefixIndicationMinus1( i ); j++ ) {
      sei.setSeiPrefixDataBit( i, j, bitstream.read( 1 ) );  // u(1)
    }
    while ( !bitstream.byteAligned() ) {
      bitstream.read( 1 );  // f(1): equal to 1
    }
  }
}

// E.2.10  Geometry transformation parameters SEI message syntax
void PCCBitstreamReader::geometryTransformationParams( PCCBitstream& bitstream,
                                                        SEI&          seiAbstract,
                                                        size_t        payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIGeometryTransformationParams& sei = static_cast<SEIGeometryTransformationParams&>( seiAbstract );
  sei.setGtpCancelFlag( bitstream.read( 1 ) );  // u(1)
  if ( !sei.getGtpCancelFlag() ) {
    sei.setGtpScaleEnabledFlag( bitstream.read( 1 ) );     // u(1)
    sei.setGtpOffsetEnabledFlag( bitstream.read( 1 ) );    // u(1)
    sei.setGtpRotationEnabledFlag( bitstream.read( 1 ) );  // u(1)
    sei.setGtpNumCameraInfoMinus1( bitstream.read( 3 ) );  // u(3)
    if ( sei.getGtpScaleEnabledFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        sei.setGtpGeometryScaleOnAxis( d, bitstream.read( 32 ) );  // u(32)
      }
    }
    if ( sei.getGtpOffsetEnabledFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        sei.setGtpGeometryOffsetOnAxis( d, bitstream.readS( 32 ) );  // i(32)
      }
    }
    if ( sei.getGtpRotationEnabledFlag() ) {
      sei.setGtpRotationQx( bitstream.readS( 16 ) );  // i(16)
      sei.setGtpRotationQy( bitstream.readS( 16 ) );  // i(16)
      sei.setGtpRotationQz( bitstream.readS( 16 ) );  // i(16)
    }
    if ( sei.getGtpNumCameraInfoMinus1() != 0 ) {
      for (uint8_t camId = 0; camId < sei.getGtpNumCameraInfoMinus1(); camId++) {
        for (size_t d = 0; d < 3; d++) {
          sei.setGtpCameraOffsetOnAxis(camId, d, bitstream.readS(16));  // i(16)
        }
        for (size_t d = 0; d < 3; d++) {
          sei.setGtpCameraOrientationOnAxis(camId, d, bitstream.readS(16));  // i(16)
        }
      }
    }
  }
}

// E.2.11  Attribute transformation parameters SEI message syntax
void PCCBitstreamReader::attributeTransformationParams( PCCBitstream& bitstream,
                                                         SEI&          seiAbstract,
                                                         size_t        payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIAttributeTransformationParams& sei = static_cast<SEIAttributeTransformationParams&>( seiAbstract );
  sei.setAtpCancelFlag( bitstream.read( 1 ) );  // u(1)
  if ( !sei.getAtpCancelFlag() ) {
    sei.setAtpNumAttributeUpdates( bitstream.readUvlc() );  // ue(v)
    sei.allocate();
    for ( size_t j = 0; j < sei.getAtpNumAttributeUpdates(); j++ ) {
      sei.setAtpAttributeIdx( j, bitstream.read( 8 ) );  // u(8)
      size_t index = sei.getAtpAttributeIdx( j );
      sei.setAtpDimensionMinus1( index, bitstream.read( 8 ) );  // u(8)
      sei.allocate( index );
      for ( size_t i = 0; i < sei.getAtpDimensionMinus1( index ); i++ ) {
        sei.setAtpScaleParamsEnabledFlag( index, i, bitstream.read( 1 ) );   // u(1)
        sei.setAtpOffsetParamsEnabledFlag( index, i, bitstream.read( 1 ) );  // u(1)
        if ( sei.getAtpScaleParamsEnabledFlag( index, i ) ) {
          sei.setAtpAttributeScale( index, i, bitstream.read( 32 ) );  // u(32)
        }
        if ( sei.getAtpOffsetParamsEnabledFlag( index, i ) ) {
          sei.setAtpAttributeOffset( index, i, bitstream.readS( 32 ) );  // i(32)
        }
      }
    }
  }
}

// E.2.12  Active substreams SEI message syntax
void PCCBitstreamReader::activeSubstreams( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIActiveSubstreams& sei = static_cast<SEIActiveSubstreams&>( seiAbstract );
  sei.setActiveAttributesChangesFlag( bitstream.read( 1 ) );    // u(1)
  sei.setActiveMapsChangesFlag( bitstream.read( 1 ) );          // u(1)
  sei.setRawPointsSubstreamsActiveFlag( bitstream.read( 1 ) );  // u(1)
  if ( sei.getActiveAttributesChangesFlag() ) {
    sei.setAllAttributesActiveFlag( bitstream.read( 1 ) );  // u(1)
    if ( !sei.getAllAttributesActiveFlag() ) {
      sei.setActiveAttributeCountMinus1( bitstream.read( 7 ) );  // u(7)
      sei.getActiveAttributeIdx().resize( sei.getActiveAttributeCountMinus1() + 1, 0 );
      for ( size_t i = 0; i <= sei.getActiveAttributeCountMinus1(); i++ ) {
        sei.setActiveAttributeIdx( i, bitstream.read( 7 ) );  // u(7)
      }
    }
  }
  if ( sei.getActiveMapsChangesFlag() ) {
    sei.setAllMapsActiveFlag( bitstream.read( 1 ) );  // u(1)
    if ( !sei.getAllMapsActiveFlag() ) {
      sei.setActiveMapCountMinus1( bitstream.read( 4 ) );  // u(4)
      sei.getActiveMapIdx().resize( sei.getActiveMapCountMinus1() + 1, 0 );
      for ( size_t i = 0; i <= sei.getActiveMapCountMinus1(); i++ ) {
        sei.setActiveMapIdx( i, bitstream.read( 4 ) );  // u(4)
      }
    }
  }
}

// E.2.13  Component codec mapping SEI message syntax
void PCCBitstreamReader::componentCodecMapping( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIComponentCodecMapping& sei = static_cast<SEIComponentCodecMapping&>( seiAbstract );
  sei.setCcmCodecMappingsCountMinus1( bitstream.read( 8 ) );  // u(8)
  sei.allocate();
  for ( size_t i = 0; i <= sei.getCcmCodecMappingsCountMinus1(); i++ ) {
    sei.setCcmCodecId( i, bitstream.read( 8 ) );                           // u(8)
    sei.setCcmCodec4cc( sei.getCcmCodecId( i ), bitstream.readString() );  // st(v)
  }
}

// E.2.14  Volumetric Tiling SEI message syntax
// E.2.14.1  General
void PCCBitstreamReader::volumetricTilingInfo( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIVolumetricTilingInfo& sei = static_cast<SEIVolumetricTilingInfo&>( seiAbstract );
  sei.setVtiCancelFlag( bitstream.read( 1 ) );  // u(1)
  if ( !sei.getVtiCancelFlag() ) {
    sei.setVtiObjectLabelPresentFlag( bitstream.read( 1 ) );           // u(1)
    sei.setVti3dBoundingBoxPresentFlag( bitstream.read( 1 ) );         // u(1)
    sei.setVtiObjectPriorityPresentFlag( bitstream.read( 1 ) );        // u(1)
    sei.setVtiObjectHiddenPresentFlag( bitstream.read( 1 ) );          // u(1)
    sei.setVtiObjectCollisionShapePresentFlag( bitstream.read( 1 ) );  // u(1)
    sei.setVtiObjectDependencyPresentFlag( bitstream.read( 1 ) );      // u(1)
    if ( sei.getVtiObjectLabelPresentFlag() ) { volumetricTilingInfoLabels( bitstream, sei ); }
    if ( sei.getVti3dBoundingBoxPresentFlag() ) {
      sei.setVtiBoundingBoxScaleLog2( bitstream.read( 5 ) );          // u(5)
      sei.setVti3dBoundingBoxScaleLog2( bitstream.read( 5 ) );        // u(5)
      sei.setVti3dBoundingBoxPrecisionMinus8( bitstream.read( 5 ) );  // u(5)
    }
    volumetricTilingInfoObjects( bitstream, sei );
  }
}

// E.2.14.2  Volumetric Tiling Info Labels
void PCCBitstreamReader::volumetricTilingInfoLabels( PCCBitstream& bitstream, SEIVolumetricTilingInfo& sei ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& vtil = sei.getVolumetricTilingInfoLabels();
  vtil.setVtiObjectLabelLanguagePresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( vtil.getVtiObjectLabelLanguagePresentFlag() ) {
    while ( !bitstream.byteAligned() ) {
      bitstream.read( 1 );  // f(1): equal to 0
    }
    vtil.setVtiObjectLabelLanguage( bitstream.readString() );  // st(v)
  }
  vtil.setVtiNumObjectLabelUpdates( bitstream.readUvlc() );  // ue(v)
  vtil.allocate();
  for ( size_t i = 0; i < vtil.getVtiNumObjectLabelUpdates(); i++ ) {
    vtil.setVtiLabelIdx( i, bitstream.readUvlc() );  // ue(v);
    bool cancelFlag = bitstream.read( 1 );           // u(1)
    // LabelAssigned[ vtiLabelIdx[ i] ] = !vtiLabelCancelFlag
    if ( !cancelFlag ) {
      while ( !bitstream.byteAligned() ) {
        bitstream.read( 1 );  // f(1): equal to 0
      }
      vtil.setVtiLabel( vtil.getVtiLabelIdx( i ), bitstream.readString() );  // st(v)
    }
  }
}

// E.2.14.3  Volumetric Tiling Info Objects
void PCCBitstreamReader::volumetricTilingInfoObjects( PCCBitstream& bitstream, SEIVolumetricTilingInfo& sei ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  const int32_t fixedBitcount = 16;
  auto&         vtio          = sei.getVolumetricTilingInfoObjects();
  vtio.setVtiNumObjectUpdates( bitstream.readUvlc() );  // ue(v)
  vtio.allocate();
  for ( size_t i = 0; i <= vtio.getVtiNumObjectUpdates(); i++ ) {
    vtio.setVtiObjectIdx( i, bitstream.readUvlc() );  // ue(v)
    size_t index = vtio.getVtiObjectIdx( i );
    vtio.allocate( index + 1 );
    vtio.setVtiObjectCancelFlag( index, bitstream.read( 1 ) );  // u(1)
    // ObjectTracked[vtiObjectIdx[ i ] ] = ! vtiObjectCancelFlag( index, bitstream.read( x ) )
    if ( !vtio.getVtiObjectCancelFlag( index ) ) {
      vtio.setVtiBoundingBoxUpdateFlag( index, bitstream.read( 1 ) );  // u(1)
      if ( vtio.getVtiBoundingBoxUpdateFlag( index ) ) {
        vtio.setVtiBoundingBoxTop( index, bitstream.read( fixedBitcount ) );     // u(v)
        vtio.setVtiBoundingBoxLeft( index, bitstream.read( fixedBitcount ) );    // u(v)
        vtio.setVtiBoundingBoxWidth( index, bitstream.read( fixedBitcount ) );   // u(v)
        vtio.setVtiBoundingBoxHeight( index, bitstream.read( fixedBitcount ) );  // u(v)
      }
      if ( sei.getVti3dBoundingBoxPresentFlag() ) {
        vtio.setVti3dBoundingBoxUpdateFlag( index, bitstream.read( 1 ) );  // u(1)
        if ( vtio.getVti3dBoundingBoxUpdateFlag( index ) ) {
          vtio.setVti3dBoundingBoxX( index, bitstream.read( fixedBitcount ) );       // u(v)
          vtio.setVti3dBoundingBoxY( index, bitstream.read( fixedBitcount ) );       // u(v)
          vtio.setVti3dBoundingBoxZ( index, bitstream.read( fixedBitcount ) );       // u(v)
          vtio.setVti3dBoundingBoxDeltaX( index, bitstream.read( fixedBitcount ) );  // u(v)
          vtio.setVti3dBoundingBoxDeltaY( index, bitstream.read( fixedBitcount ) );  // u(v)
          vtio.setVti3dBoundingBoxDeltaZ( index, bitstream.read( fixedBitcount ) );  // u(v)
        }
      }
      if ( sei.getVtiObjectPriorityPresentFlag() ) {
        vtio.setVtiObjectPriorityUpdateFlag( index, bitstream.read( 1 ) );  // u(1)
        if ( vtio.getVtiObjectPriorityUpdateFlag( index ) ) {
          vtio.setVtiObjectPriorityValue( index, bitstream.read( 4 ) );  // u(4)
        }
      }
      if ( sei.getVtiObjectHiddenPresentFlag() ) {
        vtio.setVtiObjectHiddenFlag( index, bitstream.read( 1 ) );  // u(1)
      }
      if ( sei.getVtiObjectLabelPresentFlag() ) {
        vtio.setVtiObjectLabelUpdateFlag( index, bitstream.read( 1 ) );  // u(1)
        if ( vtio.getVtiObjectLabelUpdateFlag( index ) ) {
          vtio.setVtiObjectLabelIdx( index, bitstream.read( fixedBitcount ) );  // ue(v)
        }
      }
      if ( sei.getVtiObjectCollisionShapePresentFlag() ) {
        vtio.setVtiObjectCollisionShapeUpdateFlag( index, bitstream.read( 1 ) );  // u(1)
        if ( vtio.getVtiObjectCollisionShapeUpdateFlag( index ) ) {
          vtio.setVtiObjectCollisionShapeId( index, bitstream.read( 16 ) );  // u(16)
        }
      }
      if ( sei.getVtiObjectDependencyPresentFlag() ) {
        vtio.setVtiObjectDependencyUpdateFlag( index, bitstream.read( 1 ) );  // u(1)
        if ( vtio.getVtiObjectDependencyUpdateFlag( index ) ) {
          vtio.setVtiObjectNumDependencies( index, bitstream.read( 4 ) );  // u(4)
          for ( size_t j = 0; j < vtio.getVtiObjectNumDependencies( index ); j++ ) {
            vtio.setVtiObjectDependencyIdx( index, j, bitstream.read( 8 ) );  // u(8)
          }
        }
      }
    }
  }
}

// E.2.15  Buffering period SEI message syntax
void PCCBitstreamReader::bufferingPeriod( PCCBitstream&        bitstream,
                                           SEI&                 seiAbstract,
                                           size_t               payloadSize,
                                           bool                 NalHrdBpPresentFlag,
                                           bool                 AclHrdBpPresentFlag,
                                           std::vector<uint8_t> hrdCabCntMinus1 ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIBufferingPeriod& sei           = static_cast<SEIBufferingPeriod&>( seiAbstract );
  const int32_t       fixedBitcount = 16;
  sei.setBpAtlasSequenceParameterSetId( bitstream.readUvlc() );  // ue(v)
  sei.setBpIrapCabParamsPresentFlag( bitstream.read( 1 ) );      // u(1)
  if ( sei.getBpIrapCabParamsPresentFlag() ) {
    sei.setBpCabDelayOffset( bitstream.read( fixedBitcount ) );  // u(v)
    sei.setBpDabDelayOffset( bitstream.read( fixedBitcount ) );  // u(v)
  }
  sei.setBpConcatenationFlag( bitstream.read( 1 ) );                            // u(1)
  sei.setBpAtlasCabRemovalDelayDeltaMinus1( bitstream.read( fixedBitcount ) );  // u(v)
  sei.setBpMaxSubLayersMinus1( bitstream.read( 3 ) );                           // u(3)
  sei.allocate();
  for ( size_t i = 0; i <= sei.getBpMaxSubLayersMinus1(); i++ ) {
    if ( NalHrdBpPresentFlag ) {
      for ( size_t j = 0; j < hrdCabCntMinus1[i] + 1; j++ ) {
        sei.setBpNalInitialCabRemovalDelay( i, j, bitstream.read( fixedBitcount ) );   // u(v)
        sei.setBpNalInitialCabRemovalOffset( i, j, bitstream.read( fixedBitcount ) );  // u(v)
      }
      if ( sei.getBpIrapCabParamsPresentFlag() ) {
        sei.setBpNalInitialAltCabRemovalDelay( i, bitstream.read( fixedBitcount ) );   // u(v)
        sei.setBpNalInitialAltCabRemovalOffset( i, bitstream.read( fixedBitcount ) );  // u(v)
      }
    }
    if ( AclHrdBpPresentFlag ) {
      for ( size_t j = 0; j < hrdCabCntMinus1[i] + 1; j++ ) {
        sei.setBpAclInitialCabRemovalDelay( i, j, bitstream.read( fixedBitcount ) );   // u(v)
        sei.setBpAclInitialCabRemovalOffset( i, j, bitstream.read( fixedBitcount ) );  // u(v)
      }
      if ( sei.getBpIrapCabParamsPresentFlag() ) {
        sei.setBpAclInitialAltCabRemovalDelay( i, bitstream.read( fixedBitcount ) );   // u(v)
        sei.setBpAclInitialAltCabRemovalOffset( i, bitstream.read( fixedBitcount ) );  // u(v)
      }
    }
  }
}

// E.2.16  Atlas frame timing SEI message syntax
void PCCBitstreamReader::atlasFrameTiming( PCCBitstream& bitstream,
                                            SEI&          seiAbstract,
                                            size_t        payloadSize,
                                            bool          CabDabDelaysPresentFlag ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIAtlasFrameTiming& sei           = static_cast<SEIAtlasFrameTiming&>( seiAbstract );
  const int32_t        fixedBitcount = 16;
  if ( CabDabDelaysPresentFlag ) {
    sei.setAftCabRemovalDelayMinus1( bitstream.read( fixedBitcount ) );  // u(v)
    sei.setAftDabOutputDelay( bitstream.read( fixedBitcount ) );         // u(v)
  }
}

// E.2.17  Presentation inforomation SEI message syntax
void PCCBitstreamReader::presentationInformation( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEIPresentationInformation& sei = static_cast<SEIPresentationInformation&>( seiAbstract );
  sei.setPiUnitOfLengthFlag( bitstream.read( 1 ) );        // u(1)
  sei.setPiOrientationPresentFlag( bitstream.read( 1 ) );  // u(1)
  sei.setPiPivotPresentFlag( bitstream.read( 1 ) );        // u(1)
  sei.setPiDimensionPresentFlag( bitstream.read( 1 ) );    // u(1)
  if ( sei.getPiOrientationPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      sei.setPiUp( d, bitstream.readS( 32 ) );     // i(32)
      sei.setPiFront( d, bitstream.readS( 32 ) );  // i(32)
    }
  }
  if ( sei.getPiPivotPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      int64_t a = bitstream.readS( 32 );
      int64_t b = bitstream.read( 32 );
      sei.setPiPivot( d, ( a << 32 ) & b );  // i(64)
    }
  }
  if ( sei.getPiDimensionPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      uint64_t a = bitstream.readS( 32 );
      uint64_t b = bitstream.read( 32 );
      sei.setPiDimension( d, ( a << 32 ) & b );  // i(64)
    }
  }
}

// E.2.18  Smoothing parameters SEI message syntax
void PCCBitstreamReader::smoothingParameters( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEISmoothingParameters& sei = static_cast<SEISmoothingParameters&>( seiAbstract );
  sei.setSpGeometryCancelFlag( bitstream.read( 1 ) );   // u(1)
  sei.setSpAttributeCancelFlag( bitstream.read( 1 ) );  // u(1)
  if ( !sei.getSpGeometryCancelFlag() ) {
    sei.setSpGeometrySmoothingEnabledFlag( bitstream.read( 1 ) );  // u(1)
    if ( sei.getSpGeometrySmoothingEnabledFlag() == 1 ) {
      sei.setSpGeometrySmoothingId( bitstream.read( 8 ) );  // u(8)
      TRACE_BITSTREAM( "SpGeometrySmoothingId = %u \n", sei.getSpGeometrySmoothingId() );
      if ( sei.getSpGeometrySmoothingId() == 0 ) {
        sei.setSpGeometrySmoothingGridSizeMinus2( bitstream.read( 7 ) );  // u(7)
        sei.setSpGeometrySmoothingThreshold( bitstream.read( 8 ) );       // u(8)
        TRACE_BITSTREAM( "  GridSizeMinus2 = %u \n", sei.getSpGeometrySmoothingGridSizeMinus2() );
        TRACE_BITSTREAM( "  Threshold = %u \n", sei.getSpGeometrySmoothingThreshold() );
      } else if ( sei.getSpGeometrySmoothingId() == 1 ) {
        sei.setSpGeometryPatchBlockFilteringLog2ThresholdMinus1( bitstream.read( 2 ) );  // u(2)
        sei.setSpGeometryPatchBlockFilteringPassesCountMinus1( bitstream.read( 2 ) );    // u(3)
        sei.setSpGeometryPatchBlockFilteringFilterSizeMinus1( bitstream.read( 3 ) );     // u(3)
        TRACE_BITSTREAM( "  Log2ThresholdMinus1 = %u \n", sei.getSpGeometryPatchBlockFilteringLog2ThresholdMinus1() );
        TRACE_BITSTREAM( "  PassesCountMinus1 = %u \n", sei.getSpGeometryPatchBlockFilteringPassesCountMinus1() );
        TRACE_BITSTREAM( "  FilterSizeMinus1 = %u \n", sei.getSpGeometryPatchBlockFilteringFilterSizeMinus1() );
      }
    }
  }
  if ( !sei.getSpAttributeCancelFlag() ) {
    sei.setSpNumAttributeUpdates( bitstream.readUvlc() );  // ue(v)
    sei.allocate();
    for ( size_t j = 0; j < sei.getSpNumAttributeUpdates(); j++ ) {
      sei.setSpAttributeIdx( j, bitstream.read( 8 ) );  // u(8)
      size_t index     = sei.getSpAttributeIdx( j );
      size_t dimensionMinus1 = bitstream.read( 8 );  // u(8)
      sei.allocate( index + 1, dimensionMinus1 + 1 );
      sei.setSpDimensionMinus1( index, dimensionMinus1 );  
      for ( size_t i = 0; i < sei.getSpDimensionMinus1( index ) + 1; i++ ) {
        sei.setSpAttrSmoothingParamsEnabledFlag( index, i, bitstream.read( 1 ) );  // u(1)
        if ( sei.getSpAttrSmoothingParamsEnabledFlag( index, i ) ) {
          sei.setSpAttrSmoothingGridSizeMinus2( index, i, bitstream.read( 8 ) );         // u(8)
          sei.setSpAttrSmoothingThreshold( index, i, bitstream.read( 8 ) );              // u(8)
          sei.setSpAttrSmoothingLocalEntropyThreshold( index, i, bitstream.read( 8 ) );  // u(3)
          sei.setSpAttrSmoothingThresholdVariation( index, i, bitstream.read( 8 ) );     // u(8)
          sei.setSpAttrSmoothingThresholdDifference( index, i, bitstream.read( 8 ) );    // u(8)
        }
      }
    }
  }
}

// F.2  VUI syntax
// F.2.1  VUI parameters syntax
void PCCBitstreamReader::vuiParameters( PCCBitstream& bitstream, VUIParameters& vp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  vp.setVuiWorldCoordinatesInfoPresentFlag( bitstream.read(1) );  // u(1)
  if( vp.getVuiWorldCoordinatesInfoPresentFlag() ) {
    vp.setVuiUnitInMetresFlag( bitstream.read(1) ); // u(1)
    vp.setVuiNumUnitsInBlock( bitstream.read(32) ); // u(32)
    vp.setVuiBlockScale( bitstream.read(32) ); // u(32)
  }
  vp.setVuiDefaultDispalyBoxSize( bitstream.read(1) ); // u(1)
  if( vp.getVuiDefaultDispalyBoxSize() ) {
    vp.setVuiDefDispBoxLeftOffset(bitstream.readUvlc()); // ue(v)
    vp.setVuiDefDispBoxRightOffset(bitstream.readUvlc()); // ue(v)
    vp.setVuiDefDispBoxTopOffset(bitstream.readUvlc()); // ue(v)
    vp.setVuiDefDispBoxBottomOffset(bitstream.readUvlc()); // ue(v)
    vp.setVuiDefDispBoxFrontOffset(bitstream.readUvlc()); // ue(v)
    vp.setVuiDefDispBoxBackOffset(bitstream.readUvlc()); // ue(v)
  }
  vp.setVuiTimingInfoPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( vp.getVuiTimingInfoPresentFlag() ) {
    vp.setVuiNumUnitsInTick( bitstream.read( 32 ) );              // u(32)
    vp.setVuiTimeScale( bitstream.read( 32 ) );                   // u(32)
    vp.setVuiPocProportionalToTimingFlag( bitstream.read( 1 ) );  // u(1)
    if ( vp.getVuiPocProportionalToTimingFlag() ) {
      vp.setVuiNumTicksPocDiffOneMinus1( bitstream.readUvlc() );  // ue(v)
    }
    vp.setVuiHrdParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( vp.getVuiHrdParametersPresentFlag() ) { hrdParameters( bitstream, vp.getHrdParameters() ); }
  }
  vp.setVuiBitstreamRestrictionFlag( bitstream.read( 1 ) );  // u(1)
  if ( vp.getVuiBitstreamRestrictionFlag() ) {
    vp.setVuiTileGroupsRestrictedFlag( bitstream.read( 1 ) );               // u(1)
    vp.setVuiConsistentTilesForVideoComponentsFlag( bitstream.read( 1 ) );  // u(1)
    vp.setVuiMaxNumTileGroupPerAtlas( bitstream.readUvlc() );               // ue(v)
  }
}

// F.2.2  HRD parameters syntax
void PCCBitstreamReader::hrdParameters( PCCBitstream& bitstream, HrdParameters& hp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  hp.setHrdNalParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
  hp.setHrdAclParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( hp.getHrdNalParametersPresentFlag() || hp.getHrdAclParametersPresentFlag() ) {
    hp.setHrdBitRateScale( bitstream.read( 4 ) );                        // u(4)
    hp.setHrdCabSizeScale( bitstream.read( 4 ) );                        // u(4)
    hp.setHrdInitialCabRemovalDelayLengthMinus1( bitstream.read( 5 ) );  // u(5)
    hp.setHrdAuCabRemovalDelayLengthMinus1( bitstream.read( 5 ) );       // u(5)
    hp.setHrdDabOutputDelayLengthMinus1( bitstream.read( 5 ) );          // u(5)
  }
  for ( size_t i = 0; i <= hp.getMaxNumSubLayersMinus1(); i++ ) {
    hp.setHrdFixedAtlasRateGeneralFlag( i, bitstream.read( 1 ) );  // u(1)
    if ( !hp.getHrdFixedAtlasRateGeneralFlag( i ) ) {
      hp.setHrdFixedAtlasRateWithinCasFlag( i, bitstream.read( 1 ) );  // u(1)
    }
    if ( hp.getHrdFixedAtlasRateWithinCasFlag( i ) ) {
      hp.setHrdElementalDurationInTcMinus1( i, bitstream.read( 1 ) );  // ue(v)
    } else {
      hp.setHrdLowDelayFlag( i, bitstream.read( 1 ) );  // u(1)
    }
    if ( !hp.getHrdLowDelayFlag( i ) ) {
      hp.setHrdCabCntMinus1( i, bitstream.read( 1 ) );  // ue(v)
    }
    if ( hp.getHrdNalParametersPresentFlag() ) {
      hrdSubLayerParameters( bitstream, hp.getHrdSubLayerParameters( 0, i ), hp.getHrdCabCntMinus1( i ) );
    }
    if ( hp.getHrdAclParametersPresentFlag() ) {
      hrdSubLayerParameters( bitstream, hp.getHrdSubLayerParameters( 1, i ), hp.getHrdCabCntMinus1( i ) );
    }
  }
}

// F.2.3  Sub-layer HRD parameters syntax
void PCCBitstreamReader::hrdSubLayerParameters( PCCBitstream& bitstream, HrdSubLayerParameters& hlsp, size_t cabCnt ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  hlsp.allocate( cabCnt + 1 );
  for ( size_t i = 0; i <= cabCnt; i++ ) {
    hlsp.setHrdBitRateValueMinus1( i, bitstream.readUvlc() );  // ue(v)
    hlsp.setHrdCabSizeValueMinus1( i, bitstream.readUvlc() );  // ue(v)
    hlsp.setHrdCbrFlag( i, bitstream.read( 1 ) );              // u(1)
  }
}
