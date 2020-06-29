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
#include "PCCAtlasAdaptationParameterSetRbsp.h"

#include "PCCBitstreamReader.h"

using namespace pcc;

PCCBitstreamReader::PCCBitstreamReader() :
    prevPatchSizeU_( 0 ),
    prevPatchSizeV_( 0 ),
    predPatchIndex_( 0 ),
    prevFrameIndex_( 0 ) {}
PCCBitstreamReader::~PCCBitstreamReader() = default;

// B.2  Sample stream V3C unit syntax
size_t PCCBitstreamReader::read( PCCBitstream& bitstream, SampleStreamV3CUnit& ssvu ) {
  size_t headerSize = 0;
  TRACE_BITSTREAM( "PCCBitstreamXXcoder: SampleStream Vpcc Unit start \n" );
  sampleStreamV3CHeader( bitstream, ssvu );
  headerSize++;
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 %d <=> %d / 8 - 1\n", ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1,
                   ceilLog2( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) );
  size_t unitCount = 0;
  while ( bitstream.moreData() ) {
    auto& v3cUnit = ssvu.addV3CUnit();
    sampleStreamV3CUnit( bitstream, ssvu, v3cUnit );
    TRACE_BITSTREAM( "V3C Unit Size(%zuth/%zu)  = %zu \n", unitCount, ssvu.getV3CUnitCount(), v3cUnit.getSize() );
    unitCount++;
    headerSize += ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1;
  }
  TRACE_BITSTREAM( "PCCBitstreamXXcoder: SampleStream Vpcc Unit start done \n" );
  return headerSize;
}

// B.2 Sample stream V3C unit syntax and semantics
// B.2.1 Sample stream V3C header syntax
void PCCBitstreamReader::sampleStreamV3CHeader( PCCBitstream& bitstream, SampleStreamV3CUnit& ssvu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ssvu.setSsvhUnitSizePrecisionBytesMinus1( bitstream.read( 3 ) );  // u(3)
  bitstream.read( 5 );                                              // u(5)
}

// B.2.2 Sample stream NAL unit syntax
void PCCBitstreamReader::sampleStreamV3CUnit( PCCBitstream& bitstream, SampleStreamV3CUnit& ssvu, V3CUnit& v3cUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  v3cUnit.setSize( bitstream.read( 8 * ( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) ) );  // u(v)
  auto pos = bitstream.getPosition();
  v3cUnit.getBitstream().copyFrom( bitstream, pos.bytes_, v3cUnit.getSize() );
  uint8_t v3cUnitType8 = v3cUnit.getBitstream().buffer()[0];
  auto    v3cUnitType  = static_cast<V3CUnitType>( v3cUnitType8 >>= 3 );
  v3cUnit.setType( v3cUnitType );
  TRACE_BITSTREAM( "V3CUnitType: %hhu V3CUnitSize: %zu\n", v3cUnitType8, v3cUnit.getSize() );
}

int32_t PCCBitstreamReader::decode( SampleStreamV3CUnit& ssvu, PCCHighLevelSyntax& syntax ) {
  bool endOfGop = false;
  int  numVPS   = 0;  // counter for the atlas information
  syntax.getBitstreamStat().newGOF();
  while ( !endOfGop && ssvu.getV3CUnitCount() > 0 ) {
    auto&       unit        = ssvu.front();
    V3CUnitType v3cUnitType = V3C_VPS;
    v3cUnit( syntax, unit, v3cUnitType );
    if ( v3cUnitType == V3C_VPS ) {
      numVPS++;
      if ( numVPS > 1 ) {
        // remove the bits counted for the last VPS
        int32_t v3cUnitSize = (int32_t)unit.getBitstream().capacity();
        int32_t statSize    = syntax.getBitstreamStat().getV3CUnitSize( V3C_VPS ) - v3cUnitSize;
        syntax.getBitstreamStat().overwriteV3CUnitSize( V3C_VPS, statSize );
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

void PCCBitstreamReader::videoSubStream( PCCHighLevelSyntax& syntax,
                                         PCCBitstream&       bitstream,
                                         V3CUnitType&        V3CUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  if ( V3CUnitType == V3C_OVD ) {
    TRACE_BITSTREAM( "OccupancyMap \n" );
    bitstream.read( syntax.createVideoBitstream( VIDEO_OCCUPANCY ) );
    syntax.getBitstreamStat().setVideoBinSize( VIDEO_OCCUPANCY, syntax.getVideoBitstream( VIDEO_OCCUPANCY ).size() );
  } else if ( V3CUnitType == V3C_GVD ) {
    auto& vuh = syntax.getV3CUnitHeader( static_cast<size_t>( V3C_GVD ) - 1 );
    if ( vuh.getAuxiliaryVideoFlag() ) {
      TRACE_BITSTREAM( "Geometry raw\n" );
      bitstream.read( syntax.createVideoBitstream( VIDEO_GEOMETRY_RAW ) );
      syntax.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY_RAW,
                                                 syntax.getVideoBitstream( VIDEO_GEOMETRY_RAW ).size() );
    } else {
      auto& vps = syntax.getVps();
      if ( vps.getMapCountMinus1( atlasIndex ) > 0 && vps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
        auto geometryIndex = static_cast<PCCVideoType>( VIDEO_GEOMETRY_D0 + vuh.getMapIndex() );
        TRACE_BITSTREAM( "Geometry MAP: %d\n", vuh.getMapIndex() );
        bitstream.read( syntax.createVideoBitstream( geometryIndex ) );
        syntax.getBitstreamStat().setVideoBinSize( geometryIndex, syntax.getVideoBitstream( geometryIndex ).size() );
      } else {
        TRACE_BITSTREAM( "Geometry \n" );
        bitstream.read( syntax.createVideoBitstream( VIDEO_GEOMETRY ) );
        syntax.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY, syntax.getVideoBitstream( VIDEO_GEOMETRY ).size() );
      }
    }
  } else if ( V3CUnitType == V3C_AVD ) {
    auto& vuh = syntax.getV3CUnitHeader( static_cast<size_t>( V3C_AVD ) - 1 );
    auto& vps = syntax.getVps();
    if ( vps.getAttributeInformation( atlasIndex ).getAttributeCount() > 0 ) {
      if ( vuh.getAuxiliaryVideoFlag() ) {
        auto textureIndex = static_cast<PCCVideoType>( VIDEO_TEXTURE_RAW + vuh.getAttributeDimensionIndex() );
        TRACE_BITSTREAM( "Texture raw, PARTITION: %d\n", vuh.getAttributeDimensionIndex() );
        bitstream.read( syntax.createVideoBitstream( textureIndex ) );
        syntax.getBitstreamStat().setVideoBinSize( textureIndex, syntax.getVideoBitstream( textureIndex ).size() );
      } else {
        if ( vps.getMapCountMinus1( atlasIndex ) > 0 && vps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
          auto textureIndex = static_cast<PCCVideoType>(
              VIDEO_TEXTURE_T0 + vuh.getMapIndex() * MAX_NUM_ATTR_PARTITIONS + vuh.getAttributeDimensionIndex() );
          TRACE_BITSTREAM( "Texture MAP: %d, PARTITION: %d\n", vuh.getMapIndex(), vuh.getAttributeDimensionIndex() );
          bitstream.read( syntax.createVideoBitstream( textureIndex ) );
          syntax.getBitstreamStat().setVideoBinSize( textureIndex, syntax.getVideoBitstream( textureIndex ).size() );
        } else {
          auto textureIndex = static_cast<PCCVideoType>( VIDEO_TEXTURE + vuh.getAttributeDimensionIndex() );
          TRACE_BITSTREAM( "Texture PARTITION: %d\n", vuh.getAttributeDimensionIndex() );
          bitstream.read( syntax.createVideoBitstream( textureIndex ) );
          syntax.getBitstreamStat().setVideoBinSize( textureIndex, syntax.getVideoBitstream( textureIndex ).size() );
        }
      }
    }  // if(!noAttribute)
  }    // avd
}

// 7.3.2.1 General V3C unit syntax
void PCCBitstreamReader::v3cUnit( PCCHighLevelSyntax& syntax, V3CUnit& currV3CUnit, V3CUnitType& v3cUnitType ) {
  PCCBitstream& bitstream = currV3CUnit.getBitstream();
#ifdef BITSTREAM_TRACE
  bitstream.setTrace( true );
  bitstream.setTraceFile( traceFile_ );
  TRACE_BITSTREAM( "PCCBitstream::(%s)\n", toString( currV3CUnit.getType() ).c_str() );
#endif
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto position = static_cast<int32_t>( bitstream.size() );
  v3cUnitHeader( syntax, bitstream, v3cUnitType );
  assert( v3cUnitType == currV3CUnit.getType() );
  v3cUnitPayload( syntax, bitstream, v3cUnitType );
  syntax.getBitstreamStat().setV3CUnitSize( v3cUnitType, static_cast<int32_t>( bitstream.size() ) - position );
  TRACE_BITSTREAM( "v3cUnit: V3CUnitType = %d(%s) \n", v3cUnitType, toString( v3cUnitType ).c_str() );
  TRACE_BITSTREAM( "v3cUnit: size [%d ~ %d] \n", position, bitstream.size() );
  TRACE_BITSTREAM( "%s done\n", __func__ );
  std::cout << "<----v3cUnit: V3CUnitType = " << toString( V3CUnitType( v3cUnitType ) ) << std::endl;
}

// 7.3.2.2 V3C unit header syntax
void PCCBitstreamReader::v3cUnitHeader( PCCHighLevelSyntax& syntax,
                                        PCCBitstream&       bitstream,
                                        V3CUnitType&        v3cUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  v3cUnitType = static_cast<V3CUnitType>( bitstream.read( 5 ) );  // u(5)
  if ( v3cUnitType == V3C_AVD || v3cUnitType == V3C_GVD || v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD ) {
    auto& v3cUnitHeader = syntax.getV3CUnitHeader( static_cast<int>( v3cUnitType ) - 1 );
    v3cUnitHeader.setV3CParameterSetId( bitstream.read( 4 ) );  // u(4)
    syntax.setActiveVpsId( v3cUnitHeader.getV3CParameterSetId() );
    v3cUnitHeader.setAtlasId( bitstream.read( 6 ) );  // u(6)
    syntax.setAtlasIndex( v3cUnitHeader.getAtlasId() );
  }
  if ( v3cUnitType == V3C_AVD ) {
    auto& v3cUnitHeader = syntax.getV3CUnitHeader( static_cast<int>( v3cUnitType ) - 1 );
    v3cUnitHeader.setAttributeIndex( bitstream.read( 7 ) );            // u(7)
    v3cUnitHeader.setAttributeDimensionIndex( bitstream.read( 5 ) );   // u(5)
    v3cUnitHeader.setMapIndex( bitstream.read( 4 ) );                  // u(4)
    v3cUnitHeader.setAuxiliaryVideoFlag( bitstream.read( 1 ) != 0U );  // u(1)
  } else if ( v3cUnitType == V3C_GVD ) {
    auto& vpcc = syntax.getV3CUnitHeader( static_cast<int>( v3cUnitType ) - 1 );
    vpcc.setMapIndex( bitstream.read( 4 ) );                  // u(4)
    vpcc.setAuxiliaryVideoFlag( bitstream.read( 1 ) != 0U );  // u(1)
    bitstream.read( 12 );                                     // u(12)
  } else if ( v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD ) {
    bitstream.read( 17 );  // u(17)
  } else {
    bitstream.read( 27 );  // u(27)
  }
}

// 7.3.2.3 V3C unit payload syntax
void PCCBitstreamReader::v3cUnitPayload( PCCHighLevelSyntax& syntax,
                                         PCCBitstream&       bitstream,
                                         V3CUnitType&        v3cUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "v3cUnitType = %d \n", (int32_t)v3cUnitType );
  if ( v3cUnitType == V3C_VPS ) {
    auto& vps = syntax.addV3CParameterSet();
    v3cParameterSet( vps, syntax, bitstream );
  } else if ( v3cUnitType == V3C_AD ) {
    atlasSubStream( syntax, bitstream );
  } else if ( v3cUnitType == V3C_OVD || v3cUnitType == V3C_GVD || v3cUnitType == V3C_AVD ) {
    videoSubStream( syntax, bitstream, v3cUnitType );
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

// 7.3.4.1 General V3C parameter set syntax
void PCCBitstreamReader::v3cParameterSet( V3CParameterSet& sps, PCCHighLevelSyntax& syntax, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  sps.setV3CParameterSetId( bitstream.read( 4 ) );  // u(4)
  bitstream.read( 8 );                              // u(8)
  sps.setAtlasCountMinus1( bitstream.read( 6 ) );   // u(6)
  sps.allocateAtlas();
  syntax.allocateAtlasHLS( sps.getAtlasCountMinus1() + 1 );
  for ( uint32_t j = 0; j < sps.getAtlasCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "Atlas = %zu \n", j );
    sps.setAtlasId( j, bitstream.read( 6 ) );         // u(6)
    sps.setFrameWidth( j, bitstream.read( 16 ) );     // u(16)
    sps.setFrameHeight( j, bitstream.read( 16 ) );    // u(16)
    sps.setMapCountMinus1( j, bitstream.read( 4 ) );  // u(4)
    TRACE_BITSTREAM( " MapCountMinus1 = %zu \n", sps.getMapCountMinus1( j ) );
    sps.allocateMap( j );
    if ( sps.getMapCountMinus1( j ) > 0 ) {
      sps.setMultipleMapStreamsPresentFlag( j, bitstream.read( 1 ) != 0U );  // u(1)
      TRACE_BITSTREAM( "MultipleMapStreamsPresentFlag = %zu \n", sps.getMultipleMapStreamsPresentFlag( j ) );
    }
    sps.setMapAbsoluteCodingEnableFlag( j, 0, true );
    for ( size_t i = 1; i <= sps.getMapCountMinus1( j ); i++ ) {
      if ( sps.getMultipleMapStreamsPresentFlag( j ) ) {
        sps.setMapAbsoluteCodingEnableFlag( j, i,
                                            bitstream.read( 1 ) != 0U );  // u(1)
      } else {
        sps.setMapAbsoluteCodingEnableFlag( j, i, true );
      }
      TRACE_BITSTREAM( " AbsoluteCoding Map%zu = %zu \n", i, sps.getMapAbsoluteCodingEnableFlag( j, i ) );
      if ( static_cast<int>( sps.getMapAbsoluteCodingEnableFlag( j, i ) ) == 0 ) {
        if ( i > 0 ) {
          sps.setMapPredictorIndexDiff( j, i,
                                        bitstream.readUvlc() != 0U );  // ue(v)
        } else {
          sps.setMapPredictorIndexDiff( j, i, false );
        }
        TRACE_BITSTREAM( " PredictorIndex L%zu = %zu \n", i, sps.getMapPredictorIndexDiff( j, i ) );
      }
    }
    sps.setAuxiliaryVideoPresentFlag( j, bitstream.read( 1 ) );  // u(1)
    sps.setOccupancyVideoPresentFlag( j, bitstream.read( 1 ) );  // u(1)
    sps.setGeometryVideoPresentFlag( j, bitstream.read( 1 ) );   // u(1)
    sps.setAttributeVideoPresentFlag( j, bitstream.read( 1 ) );  // u(1)

    // sps.setRawPatchEnabledFlag( j, bitstream.read( 1 ) != 0U );  // u(1)
    // TRACE_BITSTREAM( " RawPatchEnabledFlag = %zu \n", sps.getRawPatchEnabledFlag( j ) );
    // if ( sps.getRawPatchEnabledFlag( j ) ) {
    //   sps.setRawSeparateVideoPresentFlag( j, bitstream.read( 1 ) != 0U );  // u(1)
    //   TRACE_BITSTREAM( " RawSeparateVideoPresentFlag = %zu \n", sps.getRawSeparateVideoPresentFlag( j ) );
    // }
#ifdef BITSTREAM_TRACE
    for ( size_t i = 0; i < sps.getMapCountMinus1( j ) + 1; i++ ) {
      TRACE_BITSTREAM( " AbsoluteCoding L%zu = %zu \n", i, sps.getMapAbsoluteCodingEnableFlag( j, i ) );
    }
#endif
    if ( sps.getOccupancyVideoPresentFlag( j ) ) {
      occupancyInformation( sps.getOccupancyInformation( j ), bitstream );
    }
    if ( sps.getGeometryVideoPresentFlag( j ) ) {
      geometryInformation( sps.getGeometryInformation( j ), sps, bitstream );
    }
    if ( sps.getAttributeVideoPresentFlag( j ) ) {
      attributeInformation( sps.getAttributeInformation( j ), sps, bitstream );
    }
  }
  sps.setExtensionPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( sps.getExtensionPresentFlag() ) {
    sps.setVpccExtensionFlag( bitstream.read( 1 ) );  // u(1)
    sps.setMivExtensionFlag( bitstream.read( 1 ) );   // u(1)
    sps.setExtension6Bits( bitstream.read( 6 ) );     // u(6)
  }
  if ( sps.getVpccExtensionFlag() ) { vpsVpccExtension( bitstream, sps.getVpsVpccExtension() ); }
  if ( sps.getMivExtensionFlag() ) { vpsMivExtension( bitstream ); }
  if ( sps.getExtension6Bits() ) {
    sps.setExtensionLengthMinus1( bitstream.readUvlc() );  // ue(v)
    sps.allocateExtensionDataByte();
    for ( size_t i = 0; i < sps.getExtensionLengthMinus1() + 1; i++ ) {
      sps.setExtensionDataByte( i, bitstream.read( 8 ) );  // u(8)
    }
  }
  byteAlignment( bitstream );
}

// 7.3.4.2 Profile, tier, and level syntax
void PCCBitstreamReader::profileTierLevel( ProfileTierLevel& ptl, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ptl.setTierFlag( bitstream.read( 1 ) != 0U );            // u(1)
  ptl.setProfileCodecGroupIdc( bitstream.read( 7 ) );      // u(7)
  ptl.setProfileToolsetIdc( bitstream.read( 8 ) );         // u(8)
  ptl.setProfileReconctructionIdc( bitstream.read( 8 ) );  // u(8)
  bitstream.read( 32 );                                    // u(32)
  ptl.setLevelIdc( bitstream.read( 8 ) );                  // u(8)
  ptl.setNumSubProfiles( bitstream.read( 6 ) );            // u(6)
  ptl.setExtendedSubProfileFlag( bitstream.read( 1 ) );    // u(1)
  ptl.allocate();
  for ( size_t i = 0; i < ptl.getNumSubProfiles(); i++ ) {
    size_t v = ptl.getExtendedSubProfileFlag() == 0 ? 32 : 64;
    ptl.setSubProfileIdc( i, bitstream.read( v ) );  // u(v)
  }
  ptl.setToolConstraintsPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( ptl.getToolConstraintsPresentFlag() ) {
    profileToolsetConstraintsInformation( ptl.getProfileToolsetConstraintsInformation(), bitstream );
  }
}

// 7.3.4.3 Occupancy parameter set syntax
void PCCBitstreamReader::occupancyInformation( OccupancyInformation& oi, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  oi.setOccupancyCodecId( bitstream.read( 8 ) );                       // u(8)
  oi.setLossyOccupancyMapCompressionThreshold( bitstream.read( 8 ) );  // u(8)
  oi.setOccupancyNominal2DBitdepthMinus1( bitstream.read( 5 ) );       // u(5)
  oi.setOccupancyMSBAlignFlag( bitstream.read( 1 ) != 0U );            // u(1)
  TRACE_BITSTREAM( "  OccupancyLossyThreshold = %d  \n", oi.getLossyOccupancyMapCompressionThreshold() );
}

// 7.3.4.4 Geometry parameter set syntax
void PCCBitstreamReader::geometryInformation( GeometryInformation& gi, V3CParameterSet& sps, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  gi.setGeometryCodecId( bitstream.read( 8 ) );                      // u(8)
  gi.setGeometryNominal2dBitdepthMinus1( bitstream.read( 5 ) );      // u(5)
  gi.setGeometryMSBAlignFlag( bitstream.read( 1 ) != 0U );           // u(1)
  gi.setGeometry3dCoordinatesBitdepthMinus1( bitstream.read( 5 ) );  // u(5)
  if ( sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
    gi.setAuxiliaryGeometryCodecId( bitstream.read( 8 ) );  // u(8)
  }
}

// 7.3.4.5 Attribute information
void PCCBitstreamReader::attributeInformation( AttributeInformation& ai,
                                               V3CParameterSet&      sps,
                                               PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  ai.setAttributeCount( bitstream.read( 7 ) );  // u(7)
  TRACE_BITSTREAM( "AttributeCount = %u  \n", ai.getAttributeCount() );
  ai.allocate();
  for ( uint32_t i = 0; i < ai.getAttributeCount(); i++ ) {
    ai.setAttributeTypeId( i, bitstream.read( 4 ) );   // u(4)
    ai.setAttributeCodecId( i, bitstream.read( 8 ) );  // u(8)
    if ( sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
      ai.setAuxiliaryAttributeCodecId( i, bitstream.read( 8 ) );  // u(8)
    }
    ai.setAttributeMapAbsoluteCodingPersistenceFlag( i, true );
    if ( sps.getMapCountMinus1( atlasIndex ) > 0 ) {
      ai.setAttributeMapAbsoluteCodingPersistenceFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
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
    ai.setAttributeMSBAlignFlag( i, bitstream.read( 1 ) != 0U );       // u(1)
  }
}

// 7.3.4.6	Profile toolset constraints information syntax
void PCCBitstreamReader::profileToolsetConstraintsInformation( ProfileToolsetConstraintsInformation& ptci,
                                                               PCCBitstream&                         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ptci.setOneFrameOnlyFlag( bitstream.read( 1 ) );                           // u(1)
  ptci.setEnhancedOccupancyMapForDepthContraintFlag( bitstream.read( 1 ) );  // u(1)
  ptci.setMaxMapCountMinus1( bitstream.read( 4 ) );                          // u(4)
  ptci.setMaxAtlasCountMinus1( bitstream.read( 4 ) );                        // u(4)
  ptci.setMultipleMapStreamsConstraintFlag( bitstream.read( 1 ) );           // u(1)
  ptci.setPointLocalReconstructionConstraintFlag( bitstream.read( 1 ) );     // u(1)
  ptci.setAttributeMaxDimensionMinus1( bitstream.read( 6 ) );                // u(6)
  ptci.setAttributeMaxDimensionPartitionsMinus1( bitstream.read( 6 ) );      // u(6)
  ptci.setNoEightOrientationsConstraintFlag( bitstream.read( 1 ) );          // u(1)
  ptci.setNo45degreeProjectionPatchConstraintFlag( bitstream.read( 1 ) );    // u(1)
  bitstream.read( 6 );                                                       // u(6)
  ptci.setNumReservedConstraintByte( bitstream.read( 8 ) );                  // u(8)
  ptci.allocate();
  for ( size_t i = 0; i < ptci.getNumReservedConstraintByte(); i++ ) {
    ptci.setReservedConstraintByte( i, bitstream.read( 8 ) );  // u(8)
  }
}

// 7.3.5 NAL unit syntax
// 7.3.5.1 General NAL unit syntax
void PCCBitstreamReader::nalUnit( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  nalUnitHeader( bitstream, nalUnit );
  for ( size_t i = 2; i < nalUnit.getSize(); i++ ) {
    nalUnit.setData( i, bitstream.read( 8 ) );  // b(8)
  }
}

// 7.3.5.2 NAL unit header syntax
void PCCBitstreamReader::nalUnitHeader( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.read( 1 );                                                 // f(1)
  nalUnit.setType( static_cast<NalUnitType>( bitstream.read( 6 ) ) );  // u(6)
  nalUnit.setLayerId( bitstream.read( 6 ) );                           // u(6)
  nalUnit.setTemporalyIdPlus1( bitstream.read( 3 ) );                  // u(3)
  TRACE_BITSTREAM( " NalUnitSize      = %zu \n", nalUnit.getSize() );
  TRACE_BITSTREAM( " NalUnitType      = %hhu \n", nalUnit.getType() );
  TRACE_BITSTREAM( " LayerId          = %hhu \n", nalUnit.getLayerId() );
  TRACE_BITSTREAM( " TemporalyIdPlus1 = %hhu \n", nalUnit.getTemporalyIdPlus1() );
}

// 7.3.6.1 Atlas sequence parameter set Rbsp
void PCCBitstreamReader::atlasSequenceParameterSetRbsp( AtlasSequenceParameterSetRbsp& asps,
                                                        PCCHighLevelSyntax&            syntax,
                                                        PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  asps.setAtlasSequenceParameterSetId( bitstream.readUvlc() );         // ue(v)
  asps.setFrameWidth( bitstream.read( 16 ) );                          // u(16)
  asps.setFrameHeight( bitstream.read( 16 ) );                         // u(16)
  asps.setLog2MaxAtlasFrameOrderCntLsbMinus4( bitstream.readUvlc() );  // ue(v)
  asps.setMaxDecAtlasFrameBufferingMinus1( bitstream.readUvlc() );     // ue(v)
  asps.setLongTermRefAtlasFramesFlag( bitstream.read( 1 ) != 0U );     // u(1)
  asps.setNumRefAtlasFrameListsInAsps( bitstream.readUvlc() );         // ue(v)
  asps.allocateRefListStruct();
  TRACE_BITSTREAM( "ASPS: NumRefListStruct = %u \n", asps.getNumRefAtlasFrameListsInAsps() );
  for ( size_t i = 0; i < asps.getNumRefAtlasFrameListsInAsps(); i++ ) {
    refListStruct( asps.getRefListStruct( i ), asps, bitstream );
  }
  asps.setUseEightOrientationsFlag( bitstream.read( 1 ) );       // u(1)
  asps.setExtendedProjectionEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getExtendedProjectionEnabledFlag() ) {
    asps.setMaxNumberProjectionsMinus1( bitstream.readUvlc() );  // ue(v)
  }
  asps.setNormalAxisLimitsQuantizationEnabledFlag( bitstream.read( 1 ) );  // u(1)
  asps.setNormalAxisMaxDeltaValueEnabledFlag( bitstream.read( 1 ) );       // u(1)
  asps.setPatchPrecedenceOrderFlag( bitstream.read( 1 ) );                 // u(1)
  asps.setLog2PatchPackingBlockSize( bitstream.read( 3 ) );                // u(3)
  TRACE_BITSTREAM( "Log2PatchPackingBlockSize = %u \n", asps.getLog2PatchPackingBlockSize() );
  asps.setPatchSizeQuantizerPresentFlag( bitstream.read( 1 ) );  // u(1)
  asps.setMapCountMinus1( bitstream.read( 4 ) );                 // u(4)
  asps.setxelDeinterleavingFlag( bitstream.read( 1 ) );          // u(1)
  if ( asps.getPixelDeinterleavingFlag() ) {
    for ( size_t i = 0; i < asps.getMapCountMinus1() + 1; i++ ) {
      asps.setxeDeinterleavingMapFlag( i, bitstream.read( 1 ) );  // u(1)
    }
  }
  asps.setEomPatchEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getEomPatchEnabledFlag() && asps.getMapCountMinus1() == 0 ) {
    asps.setEomFixBitCountMinus1( bitstream.read( 4 ) );  // u(4)
  }
  asps.setRawPatchEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getRawPatchEnabledFlag() || asps.getEomPatchEnabledFlag() ) {
    asps.setAuxiliaryVideoEnabledFlag( bitstream.read( 1 ) );  // u(1)
  }
  asps.setPointLocalReconstructionEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionInformation( asps, syntax, bitstream );
  }
  asps.setVuiParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getVuiParametersPresentFlag() ) { vuiParameters( bitstream, asps.getVuiParameters() ); }

  asps.setExtensionFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getExtensionFlag() ) {
    asps.setVpccExtensionFlag( bitstream.read( 1 ) );  // u(1)
    asps.setMivExtensionFlag( bitstream.read( 1 ) );   // u(1)
    asps.setExtension6Bits( bitstream.read( 6 ) );     // u(6)
  }
  if ( asps.getVpccExtensionFlag() ) { aspsVpccExtension( bitstream, asps, asps.getAspsVpccExtension() ); }
  if ( asps.getMivExtensionFlag() ) { aspsMivExtension( bitstream ); }
  if ( asps.getExtension6Bits() ) {
    while ( moreRbspData( bitstream ) ) {
      bitstream.read( 1 );  // u(1)
    }
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.2 Point local reconstruction information syntax
void PCCBitstreamReader::pointLocalReconstructionInformation( AtlasSequenceParameterSetRbsp& asps,
                                                              PCCHighLevelSyntax&            syntax,
                                                              PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "  MapCountMinus1() = %u \n", asps.getMapCountMinus1() );
  asps.allocatePointLocalReconstructionInformation();
  for ( size_t j = 0; j < asps.getMapCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "  PLR info map %zu \n", j );
    auto& plri = asps.getPointLocalReconstructionInformation( j );
    plri.setMapEnabledFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( plri.getMapEnabledFlag() ) {
      plri.setNumberOfModesMinus1( bitstream.read( 4 ) );  // u(4)
      TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plri.getNumberOfModesMinus1() );
      plri.allocate();
      for ( size_t i = 0; i < plri.getNumberOfModesMinus1(); i++ ) {
        plri.setInterpolateFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
        plri.setFillingFlag( i, bitstream.read( 1 ) != 0U );      // u(1)
        plri.setMinimumDepth( i, bitstream.read( 2 ) );           // u(2)
        plri.setNeighbourMinus1( i, bitstream.read( 2 ) );        // u(2)
        TRACE_BITSTREAM( "  Mode[%zu] = I = %d F = %d D = %u N = %u \n", i, plri.getInterpolateFlag( i ),
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
bool PCCBitstreamReader::moreDataInV3CUnit( PCCBitstream& bitstream ) {
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
                                                     PCCHighLevelSyntax&         syntax,
                                                     PCCBitstream&               bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  afps.setAtlasFrameParameterSetId( bitstream.readUvlc() );     // ue(v)
  afps.setAtlasSequenceParameterSetId( bitstream.readUvlc() );  // ue(v)
  auto& asps = syntax.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  atlasFrameTileInformation( afps.getAtlasFrameTileInformation(), asps, bitstream );
  afps.setOutputFlagPresentFlag( bitstream.read( 1 ) );
  afps.setNumRefIdxDefaultActiveMinus1( bitstream.readUvlc() );  // ue(v)
  afps.setAdditionalLtAfocLsbLen( bitstream.readUvlc() );        // ue(v)
  afps.setLodModeEnableFlag( bitstream.read( 1 ) );              // u(1)
  afps.setRaw3dPosBitCountExplicitModeFlag( bitstream.read( 1 ) );
  afps.setExtensionFlag( bitstream.read( 1 ) );
  if ( afps.getExtensionFlag() ) {
    afps.setVpccExtensionFlag( bitstream.read( 1 ) );  // u(1)
    afps.setMivExtensionFlag( bitstream.read( 1 ) );   // u(1)
    afps.setExtension6Bits( bitstream.read( 6 ) );     // u(6)
  }
  if ( afps.getVpccExtensionFlag() ) { afpsVpccExtension( bitstream, afps.getAfpsVpccExtension() ); }
  if ( afps.getMivExtensionFlag() ) { afpsMivExtension( bitstream ); }
  if ( afps.getExtension6Bits() ) {
    while ( moreRbspData( bitstream ) ) { bitstream.read( 0, 1 ); }
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.4  Atlas frame tile information syntax
void PCCBitstreamReader::atlasFrameTileInformation( AtlasFrameTileInformation&     afti,
                                                    AtlasSequenceParameterSetRbsp& asps,
                                                    PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  afti.setSingleTileInAtlasFrameFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !afti.getSingleTileInAtlasFrameFlag() ) {
    afti.setUniformPartitionSpacingFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( afti.getUniformPartitionSpacingFlag() ) {
      afti.setPartitionColsWidthMinus1( 0, bitstream.readUvlc() );   //  ue(v)
      afti.setPartitionRowsHeightMinus1( 0, bitstream.readUvlc() );  //  ue(v)
      afti.setNumPartitionColumnsMinus1(
          ceil( asps.getFrameWidth() / ( ( afti.getPartitionColsWidthMinus1( 0 ) + 1 ) * 64.0 ) ) - 1 );
      afti.setNumPartitionRowsMinus1(
          ceil( asps.getFrameHeight() / ( ( afti.getPartitionRowsHeightMinus1( 0 ) + 1 ) * 64.0 ) ) - 1 );
    } else {
      afti.setNumPartitionColumnsMinus1( bitstream.readUvlc() );  //  ue(v)
      afti.setNumPartitionRowsMinus1( bitstream.readUvlc() );     //  ue(v)
      for ( size_t i = 0; i < afti.getNumPartitionColumnsMinus1(); i++ ) {
        afti.setPartitionColsWidthMinus1( i, bitstream.readUvlc() );  //  ue(v)
      }
      for ( size_t i = 0; i < afti.getNumPartitionRowsMinus1(); i++ ) {
        afti.setPartitionRowsHeightMinus1( i, bitstream.readUvlc() );  //  ue(v)
      }
    }
    afti.setSinglePartitionPerTileFlag( bitstream.read( 1 ) );  //  u(1)
    if ( afti.getSinglePartitionPerTileFlag() == 0U ) {
      uint32_t NumPartitionsInAtlasFrame =
          ( afti.getNumPartitionColumnsMinus1() + 1 ) * ( afti.getNumPartitionRowsMinus1() + 1 );
      afti.setNumTilesInAtlasFrameMinus1( bitstream.readUvlc() );  // ue(v)
      for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
        uint8_t bitCount = ceilLog2( NumPartitionsInAtlasFrame );
        afti.setTopLeftPartitionIdx( i, bitstream.read( bitCount ) );         // u(v)
        afti.setBottomRightPartitionColumnOffset( i, bitstream.readUvlc() );  // ue(v)
        afti.setBottomRightPartitionRowOffset( i, bitstream.readUvlc() );     // ue(v)
      }
    } else {
      afti.setNumTilesInAtlasFrameMinus1(
          ( afti.getNumPartitionColumnsMinus1() + 1 ) * ( afti.getNumPartitionRowsMinus1() + 1 ) - 1 );  // ue(v)
    }
  } else {
    afti.setNumTilesInAtlasFrameMinus1( 0 );
  }
  if ( asps.getAuxiliaryVideoEnabledFlag() ) {
    afti.setAuxiliaryVideoTileRowWidthMinus1( bitstream.readUvlc() );  // ue(v)
    for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1() + 1; i++ ) {
      afti.setAuxiliaryVideoTileRowHeight( i, bitstream.readUvlc() );  // ue(v)
    }
  }
  afti.setSignalledTileIdFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( afti.getSignalledTileIdFlag() ) {
    afti.setSignalledTileIdLengthMinus1( bitstream.readUvlc() );  // ue(v)
    for ( size_t i = 0; i <= afti.getSignalledTileIdLengthMinus1(); i++ ) {
      uint8_t bitCount = afti.getSignalledTileIdLengthMinus1() + 1;
      afti.setTileId( i, bitstream.read( bitCount ) );  // u(v)
    }
  }
}

// 7.3.6.3	Atlas adaptation parameter set RBSP syntax
// 7.3.6.3.1	General atlas adaptation parameter set RBSP syntax
void PCCBitstreamReader::atlasAdaptationParameterSetRbsp( AtlasAdaptationParameterSetRbsp& aaps,
                                                          PCCBitstream&                    bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  aaps.setAtlasAdaptationParameterSetId( bitstream.readUvlc() );  // ue(v)
  aaps.setLog2MaxAfocPresentFlag( bitstream.read( 1 ) );          // u(1)
  if ( aaps.getLog2MaxAfocPresentFlag() ) {
    aaps.setLog2MaxAtlasFrameOrderCntLsbMinus4( bitstream.readUvlc() );  // ue(v)
  }
  aaps.setExtensionFlag( bitstream.read( 1 ) );  // u(1)
  if ( aaps.getExtensionFlag() ) {
    aaps.setVpccExtensionFlag( bitstream.read( 1 ) );  // u(1)
    aaps.setMivExtensionFlag( bitstream.read( 1 ) );   // u(1)
    aaps.setExtension6Bits( bitstream.read( 6 ) );     // u(6)
  }
  if ( aaps.getVpccExtensionFlag() ) { aapsVpccExtension( bitstream, aaps.getAapsVpccExtension() ); }
  if ( aaps.getMivExtensionFlag() ) { aapsMivExtension( bitstream ); }
  if ( aaps.getExtension6Bits() ) {
    while ( moreRbspData( bitstream ) ) { bitstream.read( 0, 1 ); }  // u(1)
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.5  Supplemental enhancement information Rbsp syntax
void PCCBitstreamReader::seiRbsp( PCCHighLevelSyntax& syntax, PCCBitstream& bitstream, NalUnitType nalUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  // do { seiMessage( syntax, bitstream ); } while ( moreRbspData( bitstream )
  // );
  seiMessage( bitstream, syntax, nalUnitType );
}

// 7.3.6.9  Atlas tile group layer Rbsp syntax
void PCCBitstreamReader::atlasTileLayerRbsp( AtlasTileLayerRbsp& atgl,
                                             PCCHighLevelSyntax& syntax,
                                             PCCBitstream&       bitstream ) {
  // setFrameIndex
  TRACE_BITSTREAM( "%s \n", __func__ );
  atlasTileHeader( atgl.getHeader(), syntax, bitstream );
  if ( atgl.getHeader().getType() != SKIP_TILE ) {
    atlasTileDataUnit( atgl.getDataUnit(), atgl.getHeader(), syntax, bitstream );
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.10	Frame order count RBSP syntax
uint32_t PCCBitstreamReader::frameOrderCountRbsp( PCCBitstream&                    bitstream,
                                                  V3CUnitHeader&                   vuh,
                                                  AtlasSequenceParameterSetRbsp&   asps,
                                                  AtlasAdaptationParameterSetRbsp& aaps ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  uint8_t bitCount = 0;
  if ( vuh.getAtlasId() < 0x3E ) {
    bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
  } else {
    bitCount = aaps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
  }
  uint32_t frmOrderContLsb = bitstream.read( bitCount );
  return frmOrderContLsb;
}

// 7.3.6.12  Atlas tile group header syntax
void PCCBitstreamReader::atlasTileHeader( AtlasTileHeader& ath, PCCHighLevelSyntax& syntax, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ath.setAtlasFrameParameterSetId( bitstream.readUvlc() );       // ue(v)
  ath.setAtlasAdaptationParameterSetId( bitstream.readUvlc() );  // ue(v)
  size_t                         afpsId = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps   = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps   = syntax.getAtlasSequenceParameterSet( aspsId );
  AtlasFrameTileInformation&     afti   = afps.getAtlasFrameTileInformation();

  if ( afti.getSignalledTileIdFlag() )
    ath.setId( bitstream.read( afti.getSignalledTileIdLengthMinus1() + 1 ) );  // u(v)
  else {
    if ( afti.getNumTilesInAtlasFrameMinus1() != 0 )
      ath.setId( bitstream.read( ceilLog2( afti.getNumTilesInAtlasFrameMinus1() + 1 ) ) );  // u(v)
    else
      ath.setId( 0 );
  }
  ath.setType( PCCTileType( bitstream.readUvlc() ) );  // ue(v)
  if ( afps.getOutputFlagPresentFlag() ) {
    ath.setAtlasOutputFlag( bitstream.read( 1 ) != 0U );  // u(1)
  } else {
    ath.setAtlasOutputFlag( false );
  }
  ath.setAtlasFrmOrderCntLsb( bitstream.read( asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4 ) );  // u(v)
  TRACE_BITSTREAM( " AtlasFrameParameterSetId: %zu\n", ath.getAtlasFrameParameterSetId() );
  TRACE_BITSTREAM( " AtlasSequenceParameterSetId: %zu\n", afps.getAtlasSequenceParameterSetId() );
  TRACE_BITSTREAM( " Address       %zu\n", (size_t)ath.getId() );
  TRACE_BITSTREAM( " Type          %zu\n", (size_t)ath.getType() );
  TRACE_BITSTREAM( " FrameOrderCnt %zu\n", (size_t)ath.getAtlasFrmOrderCntLsb() );
  if ( asps.getNumRefAtlasFrameListsInAsps() > 0 ) {
    ath.setRefAtlasFrameListSpsFlag( bitstream.read( 1 ) != 0U );  // u(1)
  } else {
    ath.setRefAtlasFrameListSpsFlag( false );
  }
  ath.setRefAtlasFrameListIdx( 0 );
  if ( static_cast<int>( ath.getRefAtlasFrameListSpsFlag() ) == 0 ) {
    refListStruct( ath.getRefListStruct(), asps, bitstream );
  } else if ( asps.getNumRefAtlasFrameListsInAsps() > 1 ) {
    size_t bitCount = ceilLog2( asps.getNumRefAtlasFrameListsInAsps() );
    ath.setRefAtlasFrameListIdx( bitstream.read( bitCount ) );  // u(v)
  }
  if ( ath.getRefAtlasFrameListSpsFlag() ) {
    ath.setRefListStruct( asps.getRefListStruct( ath.getRefAtlasFrameListIdx() ) );
  }
  uint8_t rlsIdx  = ath.getRefAtlasFrameListIdx();
  auto&   refList = ath.getRefAtlasFrameListSpsFlag() ? asps.getRefListStruct( rlsIdx ) : ath.getRefListStruct();
  size_t  numLtrAtlasFrmEntries = 0;
  for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
    if ( !refList.getStRefAtalsFrameFlag( i ) ) { numLtrAtlasFrmEntries++; }
  }
  TRACE_BITSTREAM( " rlsIdx %u numLtrAtlasFrmEntries %zu \n", rlsIdx, (size_t)numLtrAtlasFrmEntries );
  for ( size_t j = 0; j < numLtrAtlasFrmEntries; j++ ) {
    ath.setAdditionalAfocLsbPresentFlag( j, bitstream.read( 1 ) != 0U );  // u(1)
    if ( ath.getAdditionalAfocLsbPresentFlag( j ) ) {
      uint8_t bitCount = afps.getAdditionalLtAfocLsbLen();
      ath.setAdditionalAfocLsbVal( j, bitstream.read( bitCount ) );  // u(v)
    }
  }
  if ( ath.getType() != SKIP_TILE ) {
    if ( asps.getNormalAxisLimitsQuantizationEnabledFlag() ) {
      ath.setPosMinZQuantizer( bitstream.read( 5 ) );  // u(5)
      TRACE_BITSTREAM( " AtghPosMinZQuantizer = %zu \n", ath.getPosMinZQuantizer() );
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        ath.setPosDeltaMaxZQuantizer( bitstream.read( 5 ) );  // u(5)
      }
    }
    if ( asps.getPatchSizeQuantizerPresentFlag() ) {
      ath.setPatchSizeXinfoQuantizer( bitstream.read( 3 ) );  // u(3)
      ath.setPatchSizeYinfoQuantizer( bitstream.read( 3 ) );  // u(3)
    }

    auto& gi = syntax.getVps().getGeometryInformation( 0 );
    TRACE_BITSTREAM( "Raw3dPosBitCountExplicitModeFlag    = %d \n", afps.getRaw3dPosBitCountExplicitModeFlag() );
    if ( afps.getRaw3dPosBitCountExplicitModeFlag() ) {
      size_t bitCount = ceilLog2( gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
      TRACE_BITSTREAM( "Geometry3dCoordinatesBitdepthMinus1 = %zu \n", gi.getGeometry3dCoordinatesBitdepthMinus1() );
      ath.setRaw3dPosAxisBitCountMinus1( bitstream.read( bitCount ) );  // u(v)
    } else {
      TRACE_BITSTREAM( "Geometry3dCoordinatesBitdepthMinus1 = %zu \n", gi.getGeometry3dCoordinatesBitdepthMinus1() );
      TRACE_BITSTREAM( "GeometryNominal2dBitdepthMinus1     = %zu \n", gi.getGeometryNominal2dBitdepthMinus1() );
      ath.setRaw3dPosAxisBitCountMinus1( gi.getGeometry3dCoordinatesBitdepthMinus1() -
                                         gi.getGeometryNominal2dBitdepthMinus1() - 1 );
    }
    if ( ath.getType() == P_TILE && refList.getNumRefEntries() > 1 ) {
      ath.setNumRefIdxActiveOverrideFlag( bitstream.read( 1 ) != 0U );  // u(1)
      if ( ath.getNumRefIdxActiveOverrideFlag() ) {
        ath.setNumRefIdxActiveMinus1( bitstream.readUvlc() );  // ue(v)
      }
    }
    TRACE_BITSTREAM( "==> Raw3dPosAxisBitCountMinus1  = %zu \n", ath.getRaw3dPosAxisBitCountMinus1() );
    TRACE_BITSTREAM( "==> NumRefIdxActiveOverrideFlag = %zu \n", ath.getNumRefIdxActiveOverrideFlag() );
    TRACE_BITSTREAM( "==> NumRefIdxActiveMinus1       = %zu \n", ath.getNumRefIdxActiveMinus1() );
  }
  byteAlignment( bitstream );
}

// 7.3.6.12  Reference list structure syntax
void PCCBitstreamReader::refListStruct( RefListStruct&                 rls,
                                        AtlasSequenceParameterSetRbsp& asps,
                                        PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  rls.setNumRefEntries( bitstream.readUvlc() );  // ue(v)
  TRACE_BITSTREAM( "NumRefEntries = %zu  \n", rls.getNumRefEntries() );
  rls.allocate();
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( asps.getLongTermRefAtlasFramesFlag() ) {
      rls.setStRefAtalsFrameFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
    } else {
      rls.setStRefAtalsFrameFlag( i, true );
    }
    if ( rls.getStRefAtalsFrameFlag( i ) ) {
      rls.setAbsDeltaAfocSt( i, bitstream.readUvlc() );  // ue(v)
      if ( rls.getAbsDeltaAfocSt( i ) > 0 ) {
        rls.setStrafEntrySignFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
      } else {
        rls.setStrafEntrySignFlag( i, true );
      }
    } else {
      uint8_t bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
      rls.setAfocLsbLt( i, bitstream.read( bitCount ) );  // u(v)
    }
    TRACE_BITSTREAM( "stRefAtalsFrameFlag = %zu  \n", rls.getStRefAtalsFrameFlag( i ) );
    TRACE_BITSTREAM( "absDeltaAfocSt      = %zu  \n", rls.getAbsDeltaAfocSt( i ) );
    TRACE_BITSTREAM( "strpfEntrySignFlag  = %zu  \n", rls.getStrafEntrySignFlag( i ) );
  }
  TRACE_BITSTREAM( "%s done \n", __func__ );
}

// 7.3.7.1  General atlas tile group data unit syntax =patchTileDataUnit
void PCCBitstreamReader::atlasTileDataUnit( AtlasTileDataUnit&  atdu,
                                            AtlasTileHeader&    ath,
                                            PCCHighLevelSyntax& syntax,
                                            PCCBitstream&       bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "Type = %zu \n", ath.getType() );
  atdu.init();
  size_t patchIndex = 0;
  TRACE_BITSTREAM( "patch %zu : \n", patchIndex );
  prevPatchSizeU_   = 0;
  prevPatchSizeV_   = 0;
  predPatchIndex_   = 0;
  uint8_t patchMode = bitstream.readUvlc();  // ue(v)
  TRACE_BITSTREAM( "patchMode = %zu \n", patchMode );
  while ( ( patchMode != I_END ) && ( patchMode != P_END ) ) {
    auto& pid = atdu.addPatchInformationData( patchMode );
    pid.setFrameIndex( atdu.getFrameIndex() );
    pid.setPatchIndex( patchIndex );
    patchIndex++;
    patchInformationData( pid, patchMode, ath, syntax, bitstream );
    TRACE_BITSTREAM( "patch %zu : \n", patchIndex );
    patchMode = bitstream.readUvlc();  // ue(v)
    TRACE_BITSTREAM( "patchMode = %zu \n", patchMode );
  }
  prevFrameIndex_ = atdu.getFrameIndex();
#ifdef BITSTREAM_TRACE
  if ( ( patchMode == I_END ) || ( patchMode == P_END ) ) {
    TRACE_BITSTREAM( "patchInformationData: AtghType = %zu patchMode = %zu \n", ath.getType(), patchMode );
  }
#endif
  TRACE_BITSTREAM( "atdu.getPatchCount() including END = %zu \n", atdu.getPatchCount() + 1 );
  byteAlignment( bitstream );
}

// 7.3.7.2  Patch information data syntax
void PCCBitstreamReader::patchInformationData( PatchInformationData& pid,
                                               size_t                patchMode,
                                               AtlasTileHeader&      ath,
                                               PCCHighLevelSyntax&   syntax,
                                               PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s: AtghType = %zu patchMode = %zu \n", __func__, ath.getType(), patchMode );
  if ( ath.getType() == SKIP_TILE ) {
    // skip mode: currently not supported but added it for convenience. Could
    // easily be removed
  } else if ( ath.getType() == P_TILE ) {
    if ( patchMode == P_SKIP ) {
      // skip mode: currently not supported but added it for convenience. Could
      // easily be removed
    } else if ( patchMode == P_MERGE ) {
      auto& mpdu = pid.getMergePatchDataUnit();
      mpdu.setFrameIndex( pid.getFrameIndex() );
      mpdu.setPatchIndex( pid.getPatchIndex() );
      mergePatchDataUnit( mpdu, ath, syntax, bitstream );
    } else if ( patchMode == P_INTRA ) {
      auto& pdu = pid.getPatchDataUnit();
      pdu.setFrameIndex( pid.getFrameIndex() );
      pdu.setPatchIndex( pid.getPatchIndex() );
      patchDataUnit( pdu, ath, syntax, bitstream );
    } else if ( patchMode == P_INTER ) {
      auto& ipdu = pid.getInterPatchDataUnit();
      ipdu.setFrameIndex( pid.getFrameIndex() );
      ipdu.setPatchIndex( pid.getPatchIndex() );
      interPatchDataUnit( ipdu, ath, syntax, bitstream );
    } else if ( patchMode == P_RAW ) {
      auto& rpdu = pid.getRawPatchDataUnit();
      rpdu.setFrameIndex( pid.getFrameIndex() );
      rpdu.setPatchIndex( pid.getPatchIndex() );
      rawPatchDataUnit( rpdu, ath, syntax, bitstream );
    } else if ( patchMode == P_EOM ) {
      auto& epdu = pid.getEomPatchDataUnit();
      epdu.setFrameIndex( pid.getFrameIndex() );
      epdu.setPatchIndex( pid.getPatchIndex() );
      eomPatchDataUnit( epdu, ath, syntax, bitstream );
    }
  } else if ( ath.getType() == I_TILE ) {
    if ( patchMode == I_INTRA ) {
      auto& pdu = pid.getPatchDataUnit();
      pdu.setFrameIndex( pid.getFrameIndex() );
      pdu.setPatchIndex( pid.getPatchIndex() );
      patchDataUnit( pdu, ath, syntax, bitstream );
    } else if ( patchMode == I_RAW ) {
      auto& rpdu = pid.getRawPatchDataUnit();
      rpdu.setFrameIndex( pid.getFrameIndex() );
      rpdu.setPatchIndex( pid.getPatchIndex() );
      rawPatchDataUnit( rpdu, ath, syntax, bitstream );
    } else if ( patchMode == I_EOM ) {
      auto& epdu = pid.getEomPatchDataUnit();
      epdu.setFrameIndex( pid.getFrameIndex() );
      epdu.setPatchIndex( pid.getPatchIndex() );
      eomPatchDataUnit( epdu, ath, syntax, bitstream );
    }
  }
}

// 7.3.7.3  Patch data unit syntax : AtlasTileHeader instead of
// PatchTileHeader
void PCCBitstreamReader::patchDataUnit( PatchDataUnit&      pdu,
                                        AtlasTileHeader&    ath,
                                        PCCHighLevelSyntax& syntax,
                                        PCCBitstream&       bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps   = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps   = syntax.getAtlasSequenceParameterSet( aspsId );
  pdu.set2dPosX( bitstream.readUvlc() );  // ue(v)
  pdu.set2dPosY( bitstream.readUvlc() );  // ue(v)
  TRACE_BITSTREAM( " 2dPosXY: %zu,%zu\n", pdu.get2dPosX(), pdu.get2dPosX() );
  pdu.set2dSizeXMinus1( bitstream.readUvlc() );  // ue(v)
  pdu.set2dSizeYMinus1( bitstream.readUvlc() );  // ue(v)
  TRACE_BITSTREAM( " 2dSizeXY: %d,%d\n", int32_t( pdu.get2dSizeXMinus1() + 1 ), int32_t( pdu.get2dSizeYMinus1() + 1 ) );
  uint8_t bitCount3DPos = syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() + 1;
  pdu.set3dPosX( bitstream.read( bitCount3DPos ) );  // u(v)
  pdu.set3dPosY( bitstream.read( bitCount3DPos ) );  // u(v)
  TRACE_BITSTREAM( " 3dPosXY: %zu,%zu\n", pdu.get3dPosX(), pdu.get3dPosY() );

  const uint8_t bitCountForMinDepth =
      syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() - ath.getPosMinZQuantizer() +
      2;
  pdu.set3dPosMinZ( bitstream.read( bitCountForMinDepth ) );  // u(v)
  TRACE_BITSTREAM( " Pdu3dPosMinZ: %zu ( bitCountForMinDepth = %u = %u - %u + %u ) \n", pdu.get3dPosMinZ(),
                   bitCountForMinDepth,
                   syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1(),
                   ath.getPosMinZQuantizer(), 2 );

  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
#if EXPAND_RANGE_CONDITIONAL
    uint8_t bitCountForMaxDepth = syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
                                  ath.getPosDeltaMaxZQuantizer() + 1 + asps.getExtendedProjectionEnabledFlag();
#else
#ifdef EXPAND_RANGE_ENCODER
    uint8_t bitCountForMaxDepth = syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
                                  ath.getPosDeltaMaxZQuantizer() + 1;
#else
    uint8_t bitCountForMaxDepth = syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
                                  ath.getPosDeltaMaxZQuantizer() + 2;
#endif
#endif
    // if ( asps.getExtendedProjectionEnabledFlag() ) { bitCountForMaxDepth++; }
    pdu.set3dPosDeltaMaxZ( bitstream.read( bitCountForMaxDepth ) );  // u(v)
    TRACE_BITSTREAM( " Pdu3dPosDeltaMaxZ: %zu ( bitCountForMaxDepth = %u) \n", pdu.get3dPosDeltaMaxZ(),
                     bitCountForMaxDepth );
  }
  pdu.setProjectionId( bitstream.read( ceilLog2( asps.getMaxNumberProjectionsMinus1() + 1 ) ) );  // u(5 or 3)

  TRACE_BITSTREAM( "PduProjectionId = %zu ( MaxNumberProjectionsMinus1 = %d ) \n", pdu.getProjectionId(),
                   asps.getMaxNumberProjectionsMinus1() );
  pdu.setOrientationIndex( bitstream.read( ( asps.getUseEightOrientationsFlag() ? 3 : 1 ) ) );  // u(3 or 1)
  if ( afps.getLodModeEnableFlag() ) {
    pdu.setLodEnableFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( pdu.getLodEnableFlag() ) {
      pdu.setLodScaleXminus1( uint8_t( bitstream.readUvlc() ) );  // ue(v)
      pdu.setLodScaleY( uint8_t( bitstream.readUvlc() ) );        // ue(v)
    }
  } else {
    pdu.setLodEnableFlag( false );
    pdu.setLodScaleXminus1( 0 );
    pdu.setLodScaleY( 0 );
  }
  TRACE_BITSTREAM( "PointLocalReconstructionEnabledFlag = %d \n", asps.getPointLocalReconstructionEnabledFlag() );
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plrd = pdu.getPointLocalReconstructionData();
    TRACE_BITSTREAM( "Size = %ld %ld\n", pdu.get2dSizeXMinus1() + 1, pdu.get2dSizeYMinus1() + 1 );
    plrd.allocate( pdu.get2dSizeXMinus1() + 1, pdu.get2dSizeYMinus1() + 1 );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
  }
#ifdef BITSTREAM_TRACE
  if ( asps.getMivExtensionFlag() ) { TRACE_BITSTREAM( "MivExtension \n" ); }
#endif
  TRACE_BITSTREAM(
      "Frame %zu, Patch(%zu) => 2Dpos = %4zu %4zu 2Dsize = %4ld %4ld 3Dpos = "
      "%ld %ld %ld DeltaMaxZ = %ld Projection = "
      "%zu "
      "Orientation = %zu lod=(%zu) %zu %zu\n ",
      pdu.getFrameIndex(), pdu.getPatchIndex(), pdu.get2dPosX(), pdu.get2dPosY(), pdu.get2dSizeXMinus1() + 1,
      pdu.get2dSizeYMinus1() + 1, pdu.get3dPosX(), pdu.get3dPosY(), pdu.get3dPosMinZ(), pdu.get3dPosDeltaMaxZ(),
      pdu.getProjectionId(), pdu.getOrientationIndex(), pdu.getLodEnableFlag(),
      pdu.getLodEnableFlag() ? pdu.getLodScaleXminus1() : (uint8_t)0,
      pdu.getLodEnableFlag() ? pdu.getLodScaleY() : (uint8_t)0 );
}

// 7.3.7.4  Skip patch data unit syntax
void PCCBitstreamReader::skipPatchDataUnit( SkipPatchDataUnit&  spdu,
                                            AtlasTileHeader&    ath,
                                            PCCHighLevelSyntax& syntax,
                                            PCCBitstream&       bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}

// 7.3.7.5  Merge patch data unit syntax
void PCCBitstreamReader::mergePatchDataUnit( MergePatchDataUnit& mpdu,
                                             AtlasTileHeader&    ath,
                                             PCCHighLevelSyntax& syntax,
                                             PCCBitstream&       bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId          = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps            = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId          = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps            = syntax.getAtlasSequenceParameterSet( aspsId );
  bool                           overridePlrFlag = false;
  size_t                         numRefIdxActive = syntax.getNumRefIdxActive( ath );
  if ( numRefIdxActive > 1 ) {
    mpdu.setRefIndex( bitstream.readUvlc() );  // ue(v)
  } else {
    mpdu.setRefIndex( 0 );
  }
  mpdu.setOverride2dParamsFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( mpdu.getOverride2dParamsFlag() ) {
    mpdu.set2dPosX( bitstream.readSvlc() );        // se(v)
    mpdu.set2dPosY( bitstream.readSvlc() );        // se(v)
    mpdu.set2dDeltaSizeX( bitstream.readSvlc() );  // se(v)
    mpdu.set2dDeltaSizeY( bitstream.readSvlc() );  // se(v)
    if ( asps.getPointLocalReconstructionEnabledFlag() ) { overridePlrFlag = true; }
  } else {
    mpdu.setOverride3dParamsFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( mpdu.getOverride3dParamsFlag() ) {
      mpdu.set3dPosX( bitstream.readSvlc() );     // se(v)
      mpdu.set3dPosY( bitstream.readSvlc() );     // se(v)
      mpdu.set3dPosMinZ( bitstream.readSvlc() );  // se(v)
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        mpdu.set3dPosDeltaMaxZ( bitstream.readSvlc() );  // se(v)
      }
      if ( asps.getPointLocalReconstructionEnabledFlag() ) {
        overridePlrFlag = ( bitstream.read( 1 ) != 0U );  // u(1)
        mpdu.setOverridePlrFlag( static_cast<int64_t>( overridePlrFlag ) );
      }
    }
  }
  if ( overridePlrFlag && asps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plrd = mpdu.getPointLocalReconstructionData();
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n", prevPatchSizeU_, prevPatchSizeV_,
                     mpdu.get2dDeltaSizeX(), mpdu.get2dDeltaSizeY(), prevPatchSizeU_ + mpdu.get2dDeltaSizeX(),
                     prevPatchSizeV_ + mpdu.get2dDeltaSizeY() );
    plrd.allocate( prevPatchSizeU_ + mpdu.get2dDeltaSizeX(), prevPatchSizeV_ + mpdu.get2dDeltaSizeY() );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
    prevPatchSizeU_ += mpdu.get2dDeltaSizeX();
    prevPatchSizeV_ += mpdu.get2dDeltaSizeY();
  }
#ifdef BITSTREAM_TRACE
  if ( asps.getMivExtensionFlag() ) { TRACE_BITSTREAM( "MivExtension \n" ); }
#endif
  TRACE_BITSTREAM(
      "%zu frame %zu MergePatch => Ref Frame Idx = %d ShiftUV = %ld %ld "
      "DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      mpdu.getRefIndex(), mpdu.get2dPosX(), mpdu.get2dPosY(), mpdu.get2dDeltaSizeX(), mpdu.get2dDeltaSizeY(),
      mpdu.get3dPosX(), mpdu.get3dPosY(), mpdu.get3dPosMinZ(), mpdu.get3dPosDeltaMaxZ() );
}

// 7.3.7.6  Inter patch data unit syntax
void PCCBitstreamReader::interPatchDataUnit( InterPatchDataUnit& ipdu,
                                             AtlasTileHeader&    ath,
                                             PCCHighLevelSyntax& syntax,
                                             PCCBitstream&       bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId          = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps            = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId          = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps            = syntax.getAtlasSequenceParameterSet( aspsId );
  size_t                         numRefIdxActive = syntax.getNumRefIdxActive( ath );
  if ( numRefIdxActive > 1 ) {
    ipdu.setRefIndex( bitstream.readUvlc() );  // ue(v)
  } else {
    ipdu.setRefIndex( 0 );
  }
  ipdu.setRefPatchIndex( bitstream.readSvlc() );
  ipdu.set2dPosX( bitstream.readSvlc() );        // se(v)
  ipdu.set2dPosY( bitstream.readSvlc() );        // se(v)
  ipdu.set2dDeltaSizeX( bitstream.readSvlc() );  // se(v)
  ipdu.set2dDeltaSizeY( bitstream.readSvlc() );  // se(v)
  ipdu.set3dPosX( bitstream.readSvlc() );        // se(v)
  ipdu.set3dPosY( bitstream.readSvlc() );        // se(v)
  ipdu.set3dPosMinZ( bitstream.readSvlc() );     // se(v)
  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    ipdu.set3dPosDeltaMaxZ( bitstream.readSvlc() );  // se(v)
  }
  TRACE_BITSTREAM(
      "%zu frame: numRefIdxActive = %zu reference = frame%zu patch%d 2Dpos = "
      "%ld %ld 2DdeltaSize = %ld %ld 3Dpos = %ld "
      "%ld %ld DeltaMaxZ = %ld\n",
      ipdu.getFrameIndex(), numRefIdxActive, ipdu.getRefIndex(), ipdu.getRefPatchIndex(), ipdu.get2dPosX(),
      ipdu.get2dPosY(), ipdu.get2dDeltaSizeX(), ipdu.get2dDeltaSizeY(), ipdu.get3dPosX(), ipdu.get3dPosY(),
      ipdu.get3dPosMinZ(), ipdu.get3dPosDeltaMaxZ() );
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    auto&   atglPrev      = syntax.getAtlasTileLayer( prevFrameIndex_ );
    auto&   atghPrev      = atglPrev.getHeader();
    auto&   atgdPrev      = atglPrev.getDataUnit();
    auto&   pidPrev       = atgdPrev.getPatchInformationData( ipdu.getRefPatchIndex() + predPatchIndex_ );
    auto    patchModePrev = pidPrev.getPatchMode();
    int32_t sizeU         = ipdu.get2dDeltaSizeX();
    int32_t sizeV         = ipdu.get2dDeltaSizeY();
    if ( atghPrev.getType() == P_TILE ) {
      if ( patchModePrev == P_MERGE ) {
        auto& plrdPrev = pidPrev.getMergePatchDataUnit().getPointLocalReconstructionData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      } else if ( patchModePrev == P_INTER ) {
        auto& plrdPrev = pidPrev.getInterPatchDataUnit().getPointLocalReconstructionData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      } else if ( patchModePrev == P_INTRA ) {
        auto& plrdPrev = pidPrev.getPatchDataUnit().getPointLocalReconstructionData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      }
    } else if ( atghPrev.getType() == I_TILE ) {
      if ( patchModePrev == I_INTRA ) {
        auto& plrdPrev = pidPrev.getPatchDataUnit().getPointLocalReconstructionData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      }
    }
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n", sizeU - ipdu.get2dDeltaSizeX(),
                     sizeV - ipdu.get2dDeltaSizeY(), ipdu.get2dDeltaSizeX(), ipdu.get2dDeltaSizeY(), sizeU, sizeV );
    auto& plrd = ipdu.getPointLocalReconstructionData();
    plrd.allocate( sizeU, sizeV );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
    prevPatchSizeU_ = sizeU;
    prevPatchSizeV_ = sizeV;
    predPatchIndex_ += ipdu.getRefPatchIndex() + 1;
  }
#ifdef BITSTREAM_TRACE
  if ( asps.getMivExtensionFlag() ) { TRACE_BITSTREAM( "MivExtension \n" ); }
#endif
}

// 7.3.7.7  Raw patch data unit syntax
void PCCBitstreamReader::rawPatchDataUnit( RawPatchDataUnit&   rpdu,
                                           AtlasTileHeader&    ath,
                                           PCCHighLevelSyntax& syntax,
                                           PCCBitstream&       bitstream ) {
  auto& sps = syntax.getVps();
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t  atlasIndex = 0;
  int32_t bitCount   = ath.getRaw3dPosAxisBitCountMinus1() + 1;
  TRACE_BITSTREAM( " AtghRaw3dPosAxisBitCountMinus1 = %zu => bitcount = %d \n", ath.getRaw3dPosAxisBitCountMinus1(),
                   bitCount );
  if ( 1 /* TODO: evaluate: AuxTileHeight[ TileIdToIndex[ ath_id ] ] > 0 */ ) {
    rpdu.setPatchInAuxiliaryVideoFlag( bitstream.read( 1 ) != 0U );  // u(1)
  }
  rpdu.set2dPosX( bitstream.readUvlc() );           // ue(v)
  rpdu.set2dPosY( bitstream.readUvlc() );           // ue(v)
  rpdu.set2dSizeXMinus1( bitstream.readUvlc() );    // ue(v)
  rpdu.set2dSizeYMinus1( bitstream.readUvlc() );    // ue(v)
  rpdu.set3dPosX( bitstream.read( bitCount ) );     // u(v)
  rpdu.set3dPosY( bitstream.read( bitCount ) );     // u(v)
  rpdu.set3dPosZ( bitstream.read( bitCount ) );     // u(v)
  rpdu.setRawPointsMinus1( bitstream.readUvlc() );  // ue(v)
  TRACE_BITSTREAM(
      "Raw Patch => UV %4zu %4zu  S=%4ld %4ld  UVD1=%4ld %4ld %4ld "
      "NumPcmPoints=%zu PatchInRawVideoFlag=%d \n",
      rpdu.get2dPosX(), rpdu.get2dPosY(), rpdu.get2dSizeXMinus1() + 1, rpdu.get2dSizeYMinus1() + 1, rpdu.get3dPosX(),
      rpdu.get3dPosY(), rpdu.get3dPosZ(), (size_t)rpdu.getRawPointsMinus1() + 1, rpdu.getPatchInAuxiliaryVideoFlag() );
}

// 7.3.6.x EOM patch data unit syntax
void PCCBitstreamReader::eomPatchDataUnit( EOMPatchDataUnit&   epdu,
                                           AtlasTileHeader&    ath,
                                           PCCHighLevelSyntax& syntax,
                                           PCCBitstream&       bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( 1 /* TODO: evaluate: AuxTileHeight[ TileIdToIndex[ ath_id ] ] > 0 */ ) {
    epdu.setPatchInAuxiliaryVideoFlag( bitstream.read( 1 ) != 0U );  // u(1)
  }
  epdu.set2dPosX( bitstream.readUvlc() );            // ue(v)
  epdu.set2dPosY( bitstream.readUvlc() );            // ue(v)
  epdu.set2dSizeXMinus1( bitstream.readUvlc() );     // ue(v)
  epdu.set2dSizeYMinus1( bitstream.readUvlc() );     // ue(v)
  epdu.setPatchCountMinus1( bitstream.readUvlc() );  // ue(v)
  for ( size_t i = 0; i < epdu.getPatchCountMinus1() + 1; i++ ) {
    epdu.setAssociatedPatchesIdx( i, bitstream.readUvlc() );  // ue(v)
    epdu.setPoints( i, bitstream.readUvlc() );                // ue(v)
  }
#ifdef BITSTREAM_TRACE
  TRACE_BITSTREAM( "EOM Patch => UV %4zu %4zu  S=%4ld %4ld  N=%4ld\n", epdu.get2dPosX(), epdu.get2dPosY(),
                   epdu.get2dSizeXMinus1() + 1, epdu.get2dSizeYMinus1() + 1, epdu.getPatchCountMinus1() + 1 );
  for ( size_t i = 0; i < epdu.getPatchCountMinus1() + 1; i++ ) {
    TRACE_BITSTREAM( "%4ld, %4ld\n", epdu.getAssociatedPatchesIdx( i ), epdu.getPoints( i ) );
  }
#endif
}

void PCCBitstreamReader::atlasSubStream( PCCHighLevelSyntax& syntax, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t              sizeBitstream = bitstream.capacity();
  SampleStreamNalUnit ssnu;
  sampleStreamNalHeader( bitstream, ssnu );
  while ( bitstream.size() < sizeBitstream ) {
    ssnu.addNalUnit();
    sampleStreamNalUnit( syntax, bitstream, ssnu, ssnu.getNalUnit().size() - 1 );
#ifdef BITSTREAM_TRACE
    auto& nu = ssnu.getNalUnit( ssnu.getNalUnit().size() - 1 );
    TRACE_BITSTREAM(
        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
        "written: %llu\n",
        (int)nu.getType(), toString( nu.getType() ).c_str(), ( ssnu.getSizePrecisionBytesMinus1() + 1 ), nu.getSize(),
        bitstream.size() );
#endif
  }
}

// 7.3.6.6 Point local reconstruction data syntax
void PCCBitstreamReader::pointLocalReconstructionData( PointLocalReconstructionData&  plrd,
                                                       PCCHighLevelSyntax&            syntax,
                                                       AtlasSequenceParameterSetRbsp& asps,
                                                       PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&        plri         = asps.getPointLocalReconstructionInformation( 0 );
  const size_t blockCount   = plrd.getBlockToPatchMapWidth() * plrd.getBlockToPatchMapHeight();
  const auto   bitCountMode = uint8_t( ceilLog2( uint32_t( plri.getNumberOfModesMinus1() ) ) );
  TRACE_BITSTREAM( "WxH= %zu x %zu => blockCount = %zu \n", plrd.getBlockToPatchMapWidth(),
                   plrd.getBlockToPatchMapHeight(), blockCount );
  TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u  => bitCountMode = %u  \n", plri.getNumberOfModesMinus1(),
                   bitCountMode );
  if ( blockCount > plri.getBlockThresholdPerPatchMinus1() + 1 ) {
    plrd.setLevelFlag( bitstream.read( 1 ) != 0U );  // u(1)
  } else {
    plrd.setLevelFlag( true );
  }
  TRACE_BITSTREAM( "  LevelFlag = %u \n", plrd.getLevelFlag() );
  if ( plrd.getLevelFlag() ) {
    plrd.setPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( plrd.getPresentFlag() ) {
      plrd.setModeMinus1( bitstream.read( bitCountMode ) );  // u(v)
    }
    TRACE_BITSTREAM( "  ModePatch: Present = %d ModeMinus1 = %d \n", plrd.getPresentFlag(),
                     plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1 );
  } else {
    for ( size_t i = 0; i < blockCount; i++ ) {
      plrd.setBlockPresentFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
      if ( plrd.getBlockPresentFlag( i ) ) {
        plrd.setBlockModeMinus1( i, bitstream.read( bitCountMode ) );  // u(v)
      }
      TRACE_BITSTREAM( "  Mode[ %4zu / %4zu ]: Present = %d ModeMinus1 = %d \n", i, blockCount,
                       plrd.getBlockPresentFlag( i ),
                       plrd.getBlockPresentFlag( i ) ? plrd.getBlockModeMinus1( i ) : -1 );
    }
  }
#ifdef BITSTREAM_TRACE
  for ( size_t v0 = 0; v0 < plrd.getBlockToPatchMapHeight(); ++v0 ) {
    for ( size_t u0 = 0; u0 < plrd.getBlockToPatchMapWidth(); ++u0 ) {
      size_t i = v0 * plrd.getBlockToPatchMapWidth() + u0;
      TRACE_BITSTREAM(
          "Patch Block[ %2lu %2lu <=> %4zu ] / [ %2lu %2lu ] Level = %d "
          "Present = %d Mode = %d \n",
          u0, v0, i, plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight(), plrd.getLevelFlag(),
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
  int32_t payloadType = 0;
  int32_t payloadSize = 0;
  int32_t byte        = 0;
  do {
    byte = bitstream.read( 8 );  // u(8)
    payloadType += byte;
  } while ( byte == 0xff );
  do {
    byte = bitstream.read( 8 );  // u(8)
    payloadSize += byte;
  } while ( byte == 0xff );
  seiPayload( bitstream, syntax, nalUnitType, static_cast<SeiPayloadType>( payloadType ), payloadSize );
}

// C.2 Sample stream NAL unit syntax and semantics
// C.2.1 Sample stream NAL header syntax
void PCCBitstreamReader::sampleStreamNalHeader( PCCBitstream& bitstream, SampleStreamNalUnit& ssnu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ssnu.setSizePrecisionBytesMinus1( bitstream.read( 3 ) );  // u(3)
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 = %lu \n", ssnu.getSizePrecisionBytesMinus1() );
  bitstream.read( 5 );  // u(5)
}

// C.2.2 Sample stream NAL unit syntax
void PCCBitstreamReader::sampleStreamNalUnit( PCCHighLevelSyntax&  syntax,
                                              PCCBitstream&        bitstream,
                                              SampleStreamNalUnit& ssnu,
                                              size_t               index ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& nu = ssnu.getNalUnit( index );
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 = %lu \n", ssnu.getSizePrecisionBytesMinus1() );
  nu.setSize( bitstream.read( 8 * ( ssnu.getSizePrecisionBytesMinus1() + 1 ) ) );  // u(v)
  nu.allocate();
  nalUnitHeader( bitstream, nu );
  switch ( nu.getType() ) {
    case NAL_ASPS: atlasSequenceParameterSetRbsp( syntax.addAtlasSequenceParameterSet(), syntax, bitstream ); break;
    case NAL_AFPS: atlasFrameParameterSetRbsp( syntax.addAtlasFrameParameterSet(), syntax, bitstream ); break;
    case NAL_TRAIL_N:
    case NAL_TRAIL_R:
    case NAL_TSA_N:
    case NAL_TSA_R:
    case NAL_STSA_N:
    case NAL_STSA_R:
    case NAL_RADL_N:
    case NAL_RADL_R:
    case NAL_RASL_N:
    case NAL_RASL_R:
    case NAL_SKIP_N:
    case NAL_SKIP_R: atlasTileLayerRbsp( syntax.addAtlasTileLayer(), syntax, bitstream ); break;
    case NAL_SUFFIX_ESEI:
    case NAL_SUFFIX_NSEI: seiRbsp( syntax, bitstream, nu.getType() ); break;
    case NAL_PREFIX_ESEI:
    case NAL_PREFIX_NSEI: seiRbsp( syntax, bitstream, nu.getType() ); break;
    default: fprintf( stderr, "sampleStreamNalUnit type = %d not supported\n", static_cast<int32_t>( nu.getType() ) );
  }
}

// F.2  SEI payload syntax
// F.2.1  General SEI message syntax
void PCCBitstreamReader::seiPayload( PCCBitstream&       bitstream,
                                     PCCHighLevelSyntax& syntax,
                                     NalUnitType         nalUnitType,
                                     SeiPayloadType      payloadType,
                                     size_t              payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEI& sei = syntax.addSei( nalUnitType, payloadType );
  if ( nalUnitType == NAL_PREFIX_ESEI ) {
    if ( payloadType == BUFFERING_PERIOD ) {  // 0
      bool                 NalHrdBpPresentFlag = false;
      bool                 AclHrdBpPresentFlag = false;
      std::vector<uint8_t> hrdCabCntMinus1;
      bufferingPeriod( bitstream, sei, NalHrdBpPresentFlag, AclHrdBpPresentFlag, hrdCabCntMinus1 );
    } else if ( payloadType == ATLAS_FRAME_TIMING ) {  // 1
      atlasFrameTiming( bitstream, sei, false );
    } else if ( payloadType == FILLER_PAYLOAD ) {  // 2
      fillerPayload( bitstream, sei, payloadSize );
    } else if ( payloadType == USER_DATAREGISTERED_ITUTT35 ) {  // 3
      userDataRegisteredItuTT35( bitstream, sei, payloadSize );
    } else if ( payloadType == USER_DATA_UNREGISTERED ) {  // 4
      userDataUnregistered( bitstream, sei, payloadSize );
    } else if ( payloadType == RECOVERY_POINT ) {  // 5
      recoveryPoint( bitstream, sei );
    } else if ( payloadType == NO_DISPLAY ) {  // 6
      noDisplay( bitstream, sei );
    } else if ( payloadType == TIME_CODE ) {  // 7
      // timeCode( bitstream, sei );
    } else if ( payloadType == REGIONAL_NESTING ) {  // 8
      // regionalNesting( bitstream, sei );
    } else if ( payloadType == SEI_MANIFEST ) {  // 9
      seiManifest( bitstream, sei );
    } else if ( payloadType == SEI_PREFIX_INDICATION ) {  // 10
      seiPrefixIndication( bitstream, sei );
    } else if ( payloadType == ATTRIBUTE_TRANSFORMATION_PARAMS ) {  // 11
      attributeTransformationParams( bitstream, sei );
    } else if ( payloadType == ACTIVE_SUB_BITSTREAMS ) {  // 12
      activeSubBitstreams( bitstream, sei );
    } else if ( payloadType == COMPONENT_CODEC_MAPPING ) {  // 13
      componentCodecMapping( bitstream, sei );
    } else if ( payloadType == SCENE_OBJECT_INFORMATION ) {  // 14
      sceneObjectInformation( bitstream, sei );
    } else if ( payloadType == OBJECT_LABEL_INFORMATION ) {  // 15
      objectLabelInformation( bitstream, sei );
    } else if ( payloadType == PATCH_INFORMATION ) {  // 16
      patchInformation( bitstream, sei );
    } else if ( payloadType == VOLUMETRIC_RECTANGLE_INFORMATION ) {  // 17
      volumetricRectangleInformation( bitstream, sei );
    } else if ( payloadType == ATLAS_INFORMATION ) {  // 18
      atlasInformation( bitstream, sei );
    } else if ( payloadType == VIEWPORT_CAMERA_PARAMETERS ) {  // 19
      viewportCameraParameters( bitstream, sei );
    } else if ( payloadType == VIEWPORT_POSITION ) {  // 20
      viewportPosition( bitstream, sei );
    } else if ( payloadType == GEOMETRY_SMOOTHING ) {  // 64
      geometrySmoothing( bitstream, sei );
    } else if ( payloadType == ATTRIBUTE_SMOOTHING ) {  // 65
      attributeSmoothing( bitstream, sei );
    } else {
      reservedSeiMessage( bitstream, sei, payloadSize );
    }
  } else { /* psdUnitType  ==  NAL_SUFFIX_ESEI */
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
  if ( !bitstream.byteAligned() ) {  // this prevents from writing one more byte,
                                     // in case the payload is already byte
                                     // aligned (see xWriteByteAlign in HM)
    byteAlignment( bitstream );
  }
}

// F.2.2  Filler payload SEI message syntax
void PCCBitstreamReader::fillerPayload( PCCBitstream& bitstream, SEI& sei, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  for ( size_t k = 0; k < payloadSize; k++ ) {
    bitstream.read( 8 );  // f(8) equal to 0xFF
  }
}

// F.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
void PCCBitstreamReader::userDataRegisteredItuTT35( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIUserDataRegisteredItuTT35&>( seiAbstract );
  sei.setCountryCode( bitstream.read( 8 ) );  // b(8)
  payloadSize--;
  if ( sei.getCountryCode() == 0xFF ) {
    sei.setCountryCodeExtensionByte( bitstream.read( 8 ) );  // b(8)
    payloadSize--;
  }
  auto& payload = sei.getPayloadByte();
  payload.resize( payloadSize );
  for ( auto& element : payload ) {
    element = bitstream.read( 8 );  // b(8)
  }
}

// F.2.4  User data unregistered SEI message syntax
void PCCBitstreamReader::userDataUnregistered( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIUserDataUnregistered&>( seiAbstract );
  for ( size_t i = 0; i < 16; i++ ) {
    sei.setUuidIsoIec11578( i, bitstream.read( 8 ) );  // u(128) <=> 16 * u(8)
  }
  payloadSize -= 16;
  sei.getUserDataPayloadByte().resize( payloadSize );
  for ( size_t i = 0; i < payloadSize; i++ ) {
    sei.setUserDataPayloadByte( i, bitstream.read( 8 ) );  // b(8)
  }
}

// F.2.5  Recovery point SEI message syntax
void PCCBitstreamReader::recoveryPoint( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIRecoveryPoint&>( seiAbstract );
  sei.setRecoveryAfocCnt( bitstream.readSvlc() );  // se(v)
  sei.setExactMatchFlag( bitstream.read( 1 ) );    // u(1)
  sei.setBrokenLinkFlag( bitstream.read( 1 ) );    // u(1)
}

// F.2.6  No display SEI message syntax
void PCCBitstreamReader::noDisplay( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}

// F.2.7  Reserved SEI message syntax
void PCCBitstreamReader::reservedSeiMessage( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIReservedSeiMessage&>( seiAbstract );
  sei.getPayloadByte().resize( payloadSize );
  for ( size_t i = 0; i < payloadSize; i++ ) {
    sei.setPayloadByte( i, bitstream.read( 8 ) );  // b(8)
  }
}

// F.2.8  SEI manifest SEI message syntax
void PCCBitstreamReader::seiManifest( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIManifest&>( seiAbstract );
  sei.setNumSeiMsgTypes( bitstream.read( 16 ) );  // u(16)
  sei.allocate();
  for ( size_t i = 0; i < sei.getNumSeiMsgTypes(); i++ ) {
    sei.setSeiPayloadType( i, bitstream.read( 16 ) );  // u(16)
    sei.setSeiDescription( i, bitstream.read( 8 ) );   // u(8)
  }
}

// F.2.9  SEI prefix indication SEI message syntax
void PCCBitstreamReader::seiPrefixIndication( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIPrefixIndication&>( seiAbstract );
  sei.setPrefixSeiPayloadType( bitstream.read( 16 ) );          // u(16)
  sei.setNumSeiPrefixIndicationsMinus1( bitstream.read( 8 ) );  // u(8)
  sei.getNumBitsInPrefixIndicationMinus1().resize( sei.getNumSeiPrefixIndicationsMinus1() + 1, 0 );
  sei.getSeiPrefixDataBit().resize( sei.getNumSeiPrefixIndicationsMinus1() + 1 );
  for ( size_t i = 0; i <= sei.getNumSeiPrefixIndicationsMinus1(); i++ ) {
    sei.setNumBitsInPrefixIndicationMinus1( i, bitstream.read( 16 ) );  // u(16)
    sei.getSeiPrefixDataBit( i ).resize( sei.getNumBitsInPrefixIndicationMinus1( i ), false );
    for ( size_t j = 0; j <= sei.getNumBitsInPrefixIndicationMinus1( i ); j++ ) {
      sei.setSeiPrefixDataBit( i, j, bitstream.read( 1 ) != 0U );  // u(1)
    }
    while ( !bitstream.byteAligned() ) {
      bitstream.read( 1 );  // f(1): equal to 1
    }
  }
}

// // F.2.10  Geometry transformation parameters SEI message syntax
// void PCCBitstreamReader::geometryTransformationParams( PCCBitstream& bitstream, SEI& seiAbstract ) {
//   TRACE_BITSTREAM( "%s \n", __func__ );
//   auto& sei = static_cast<SEIGeometryTransformationParams&>( seiAbstract );
//   sei.setGtpCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
//   if ( !sei.getGtpCancelFlag() ) {
//     sei.setGtpScaleEnabledFlag( bitstream.read( 1 ) != 0U );     // u(1)
//     sei.setGtpOffsetEnabledFlag( bitstream.read( 1 ) != 0U );    // u(1)
//     sei.setGtpRotationEnabledFlag( bitstream.read( 1 ) != 0U );  // u(1)
//     sei.setGtpNumCameraInfoMinus1( bitstream.read( 3 ) );        // u(3)
//     if ( sei.getGtpScaleEnabledFlag() ) {
//       for ( size_t d = 0; d < 3; d++ ) {
//         sei.setGtpGeometryScaleOnAxis( d, bitstream.read( 32 ) );  // u(32)
//       }
//     }
//     if ( sei.getGtpOffsetEnabledFlag() ) {
//       for ( size_t d = 0; d < 3; d++ ) {
//         sei.setGtpGeometryOffsetOnAxis( d, bitstream.readS( 32 ) );  // i(32)
//       }
//     }
//     if ( sei.getGtpRotationEnabledFlag() ) {
//       sei.setGtpRotationQx( bitstream.readS( 16 ) );  // i(16)
//       sei.setGtpRotationQy( bitstream.readS( 16 ) );  // i(16)
//       sei.setGtpRotationQz( bitstream.readS( 16 ) );  // i(16)
//     }
//     if ( sei.getGtpNumCameraInfoMinus1() != 0 ) {
//       for ( uint8_t camId = 0; camId < sei.getGtpNumCameraInfoMinus1(); camId++ ) {
//         for ( size_t d = 0; d < 3; d++ ) {
//           sei.setGtpCameraOffsetOnAxis( camId, d, bitstream.readS( 16 ) );  // i(16)
//         }
//         for ( size_t d = 0; d < 3; d++ ) {
//           sei.setGtpCameraOrientationOnAxis( camId, d,
//                                              bitstream.readS( 16 ) );  // i(16)
//         }
//       }
//     }
//   }
// }

// F.2.10  Attribute transformation parameters SEI message syntax
void PCCBitstreamReader::attributeTransformationParams( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIAttributeTransformationParams&>( seiAbstract );
  sei.setCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !sei.getCancelFlag() ) {
    sei.setNumAttributeUpdates( bitstream.readUvlc() );  // ue(v)
    sei.allocate();
    for ( size_t j = 0; j < sei.getNumAttributeUpdates(); j++ ) {
      sei.setAttributeIdx( j, bitstream.read( 8 ) );  // u(8)
      size_t index = sei.getAttributeIdx( j );
      sei.setDimensionMinus1( index, bitstream.read( 8 ) );  // u(8)
      sei.allocate( index );
      for ( size_t i = 0; i < sei.getDimensionMinus1( index ); i++ ) {
        sei.setScaleParamsEnabledFlag( index, i,
                                       bitstream.read( 1 ) != 0U );  // u(1)
        sei.setOffsetParamsEnabledFlag( index, i,
                                        bitstream.read( 1 ) != 0U );  // u(1)
        if ( sei.getScaleParamsEnabledFlag( index, i ) ) {
          sei.setAttributeScale( index, i, bitstream.read( 32 ) );  // u(32)
        }
        if ( sei.getOffsetParamsEnabledFlag( index, i ) ) {
          sei.setAttributeOffset( index, i, bitstream.readS( 32 ) );  // i(32)
        }
      }
    }
    sei.setPersistenceFlag( bitstream.read( 1 ) );  // u(1)
  }
}

// F.2.11  Active substreams SEI message syntax
void PCCBitstreamReader::activeSubBitstreams( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIActiveSubBitstreams&>( seiAbstract );
  sei.setActiveAttributesChangesFlag( bitstream.read( 1 ) != 0U );          // u(1)
  sei.setActiveMapsChangesFlag( bitstream.read( 1 ) != 0U );                // u(1)
  sei.setAuxiliaryPointsSubstreamsActiveFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( sei.getActiveAttributesChangesFlag() ) {
    sei.setAllAttributesActiveFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( !sei.getAllAttributesActiveFlag() ) {
      sei.setActiveAttributeCountMinus1( bitstream.read( 7 ) );  // u(7)
      sei.getActiveAttributeIdx().resize( sei.getActiveAttributeCountMinus1() + 1, 0 );
      for ( size_t i = 0; i <= sei.getActiveAttributeCountMinus1(); i++ ) {
        sei.setActiveAttributeIdx( i, bitstream.read( 7 ) );  // u(7)
      }
    }
  }
  if ( sei.getActiveMapsChangesFlag() ) {
    sei.setAllMapsActiveFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( !sei.getAllMapsActiveFlag() ) {
      sei.setActiveMapCountMinus1( bitstream.read( 4 ) );  // u(4)
      sei.getActiveMapIdx().resize( sei.getActiveMapCountMinus1() + 1, 0 );
      for ( size_t i = 0; i <= sei.getActiveMapCountMinus1(); i++ ) {
        sei.setActiveMapIdx( i, bitstream.read( 4 ) );  // u(4)
      }
    }
  }
}

// F.2.12  Component codec mapping SEI message syntax
void PCCBitstreamReader::componentCodecMapping( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIComponentCodecMapping&>( seiAbstract );
  sei.setCodecMappingsCountMinus1( bitstream.read( 8 ) );  // u(8)
  sei.allocate();
  for ( size_t i = 0; i <= sei.getCodecMappingsCountMinus1(); i++ ) {
    sei.setCodecId( i, bitstream.read( 8 ) );                        // u(8)
    sei.setCodec4cc( sei.getCodecId( i ), bitstream.readString() );  // st(v)
  }
}

// m52705
// F.2.14  Volumetric Tiling SEI message syntax
// F.2.14.1  General
// F.2.14.2  Volumetric Tiling Info Labels
// F.2.14.3  Volumetric Tiling Info Objects

// F.2.13.1	Scene object information SEI message syntax
void PCCBitstreamReader::sceneObjectInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEISceneObjectInformation&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  sei.setCancelFlag( bool( bitstream.read( 1 ) ) );
  sei.setNumObjectUpdates( bitstream.readUvlc() );
  sei.allocateObjectIdx();
  if ( sei.getNumObjectUpdates() > 0 ) {
    sei.setSimpleObjectsFlag( bool( bitstream.read( 1 ) ) );
    if ( static_cast<int>( sei.getSimpleObjectsFlag() ) == 0 ) {
      sei.setObjectLabelPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setPriorityPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setObjectHiddenPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setObjectDependencyPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setVisibilityConesPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.set3dBoundingBoxPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setCollisionShapePresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setPointStylePresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setMaterialIdPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setExtensionPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
    }
    if ( sei.get3dBoundingBoxPresentFlag() ) {
      sei.set3dBoundingBoxScaleLog2( bitstream.read( 5 ) );
      sei.set3dBoundingBoxPrecisionMinus8( bitstream.read( 5 ) );
    }
    sei.setLog2MaxObjectIdxUpdated( bitstream.read( 5 ) );
    if ( sei.getObjectDependencyPresentFlag() ) { sei.setLog2MaxObjectDependencyIdx( bitstream.read( 5 ) ); }
    for ( size_t i = 0; i <= sei.getNumObjectUpdates(); i++ ) {
      assert( sei.getObjectIdx().size() >= sei.getNumObjectUpdates() );
      sei.setObjectIdx( i, bitstream.read( sei.getLog2MaxObjectIdxUpdated() ) );
      size_t k = sei.getObjectIdx( i );
      sei.allocate( k + 1 );
      sei.setObjectCancelFlag( k, bitstream.read( 1 ) != 0U );
      // ObjectTracked[k]=!piObjectCancelFlag[k];
      if ( sei.getObjectCancelFlag( k ) ) {
        if ( sei.getObjectLabelPresentFlag() ) {
          sei.setObjectLabelUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getObjectLabelUpdateFlag( k ) ) { sei.setObjectLabelIdx( k, bitstream.readUvlc() ); }
        }
        if ( sei.getPriorityPresentFlag() ) {
          sei.setPriorityUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getPriorityUpdateFlag( k ) ) { sei.setPriorityValue( k, bitstream.read( 4 ) ); }
        }
        if ( sei.getObjectHiddenPresentFlag() ) { sei.setObjectHiddenFlag( k, bitstream.read( 1 ) != 0U ); }

        if ( sei.getObjectDependencyPresentFlag() ) {
          sei.setObjectDependencyUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getObjectDependencyUpdateFlag( k ) ) {
            sei.setObjectNumDependencies( k, bitstream.read( 4 ) );
            sei.allocateObjectNumDependencies( k, sei.getObjectNumDependencies( k ) );
            for ( size_t j = 0; j < sei.getObjectNumDependencies( k ); j++ ) {
              sei.setObjectDependencyIdx( k, j, bitstream.read( 8 ) );  // size_t bitCount = ceil(log2(
              // sei.getObjectNumDependencies(k)
              // )+0.5);
            }
          }
        }
        if ( sei.getVisibilityConesPresentFlag() ) {
          sei.setVisibilityConesUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getVisibilityConesUpdateFlag( k ) ) {
            sei.setDirectionX( k, bitstream.read( 32 ) );
            sei.setDirectionY( k, bitstream.read( 32 ) );
            sei.setDirectionZ( k, bitstream.read( 32 ) );
            sei.setAngle( k, bitstream.read( 16 ) );
          }
        }  // cones

        if ( sei.get3dBoundingBoxPresentFlag() ) {
          sei.set3dBoundingBoxUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.get3dBoundingBoxUpdateFlag( k ) ) {
            sei.set3dBoundingBoxX( k, ( bitstream.read( fixedBitcount ) ) );
            sei.set3dBoundingBoxY( k, ( bitstream.read( fixedBitcount ) ) );
            sei.set3dBoundingBoxZ( k, ( bitstream.read( fixedBitcount ) ) );
            sei.set3dBoundingBoxDeltaX( k, ( bitstream.read( fixedBitcount ) ) );
            sei.set3dBoundingBoxDeltaY( k, ( bitstream.read( fixedBitcount ) ) );
            sei.set3dBoundingBoxDeltaZ( k, ( bitstream.read( fixedBitcount ) ) );
          }
        }  // 3dBB

        if ( sei.getCollisionShapePresentFlag() ) {
          sei.setCollisionShapeUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getCollisionShapeUpdateFlag( k ) ) { sei.setCollisionShapeId( k, bitstream.read( 16 ) ); }
        }  // collision
        if ( sei.getPointStylePresentFlag() ) {
          sei.setPointStyleUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getPointStyleUpdateFlag( k ) ) {
            sei.setPointShapeId( k, bitstream.read( 8 ) );  // only shape??
          }
          sei.setPointSize( k, bitstream.read( 16 ) );
        }  // pointstyle
        if ( sei.getMaterialIdPresentFlag() ) {
          sei.setMaterialIdUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getMaterialIdUpdateFlag( k ) ) { sei.setMaterialId( k, bitstream.read( 16 ) ); }
        }  // materialid
      }    // sei.getObjectCancelFlag(k)
    }      // for(size_t i=0; i<=sei.getNumObjectUpdates(); i++)
  }        // if( sei.getNumObjectUpdates() > 0 )
}

// F.2.13.2 Object label information SEI message syntax
void PCCBitstreamReader::objectLabelInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIObjectLabelInformation&>( seiAbstract );
  sei.setCancelFlag( bitstream.read( 1 ) != 0U );
  if ( !sei.getCancelFlag() ) {
    sei.setLabelLanguagePresentFlag( bitstream.read( 1 ) != 0U );
    if ( sei.getLabelLanguagePresentFlag() ) {
      while ( !bitstream.byteAligned() ) { bitstream.read( 1 ); }
      sei.setLabelLanguage( bitstream.readString() );
    }
    sei.setNumLabelUpdates( bitstream.readUvlc() );
    sei.allocate();
    for ( size_t i = 0; i < sei.getNumLabelUpdates(); i++ ) {
      sei.setLabelIdx( i, bitstream.readUvlc() );
      sei.setLabelCancelFlag( bitstream.read( 1 ) != 0U );
      if ( !sei.getLabelCancelFlag() ) {
        while ( !bitstream.byteAligned() ) { bitstream.read( 1 ); }
        sei.setLabel( sei.getLabelIdx( i ), bitstream.readString() );
      }
    }
    sei.setPersistenceFlag( bitstream.read( 1 ) != 0U );  // u(1)
  }
};

// F.2.13.3 Patch information SEI message syntax
void PCCBitstreamReader::patchInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIPatchInformation&>( seiAbstract );
  sei.setPersistenceFlag( bitstream.read( 1 ) != 0U );  // u(1)
  sei.setResetFlag( bitstream.read( 1 ) != 0U );        // u(1)
  sei.setNumTileUpdates( bitstream.readUvlc() );
  if ( sei.getNumTileUpdates() > 0 ) {
    sei.setLog2MaxObjectIdxTracked( bitstream.read( 5 ) );
    sei.setLog2MaxPatchIdxUpdated( bitstream.read( 4 ) );
  }
  for ( size_t i = 0; i < sei.getNumTileUpdates(); i++ ) {
    sei.setTileAddress( i, bitstream.readUvlc() );
    size_t j = sei.getTileAddress( i );
    sei.setTileCancelFlag( j, bitstream.read( 1 ) != 0U );
    sei.setNumPatchUpdates( j, bitstream.readUvlc() );
    for ( size_t k = 0; k < sei.getNumPatchUpdates( j ); k++ ) {
      sei.setPatchIdx( j, k, bitstream.read( sei.getLog2MaxPatchIdxUpdated() ) );
      auto p = sei.getPatchIdx( j, k );
      sei.setPatchCancelFlag( j, p, bitstream.read( 1 ) != 0U );
      if ( !sei.getPatchCancelFlag( j, p ) ) {
        sei.setPatchNumberOfObjectsMinus1( j, p, bitstream.readUvlc() );
        for ( size_t n = 0; n < sei.getPatchNumberOfObjectsMinus1( j, p ) + 1; n++ ) {
          sei.setPatchObjectIdx(
              j, p, n,
              bitstream.read( sei.getLog2MaxObjectIdxTracked() ) );  //?pi_log2_max_object_idx_updated?
        }
      }
    }
  }
};

// F.2.13.4 Volumetric rectangle information SEI message syntax
void PCCBitstreamReader::volumetricRectangleInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEIVolumetricRectangleInformation&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  sei.setPersistenceFlag( bitstream.read( 1 ) != 0U );
  sei.setResetFlag( bitstream.read( 1 ) != 0U );
  sei.setNumRectanglesUpdates( bitstream.readUvlc() );
  if ( sei.getNumRectanglesUpdates() > 0 ) {
    sei.setLog2MaxObjectIdxTracked( bitstream.read( 5 ) );
    sei.setLog2MaxRectangleIdxUpdated( bitstream.read( 4 ) );
  }
  for ( size_t k = 0; k < sei.getNumRectanglesUpdates(); k++ ) {
    sei.setRectangleIdx( k, bitstream.read( sei.getLog2MaxRectangleIdxUpdated() ) );
    auto p = sei.getRectangleIdx( k );
    sei.setRectangleCancelFlag( p, bitstream.read( 1 ) != 0U );
    if ( !sei.getRectangleCancelFlag( p ) ) {
      sei.allocate( p + 1 );
      sei.setBoundingBoxUpdateFlag( p, bitstream.read( 1 ) != 0U );
      if ( sei.getBoundingBoxUpdateFlag( p ) ) {
        sei.setBoundingBoxTop( p, ( bitstream.read( fixedBitcount ) ) );
        sei.setBoundingBoxLeft( p, ( bitstream.read( fixedBitcount ) ) );
        sei.setBoundingBoxWidth( p, ( bitstream.read( fixedBitcount ) ) );
        sei.setBoundingBoxHeight( p, ( bitstream.read( fixedBitcount ) ) );
      }
      sei.setRectangleNumberOfObjectsMinus1( p, bitstream.readUvlc() );
      sei.allocateRectangleObjectIdx( p, sei.getRectangleNumberOfObjectsMinus1( p ) + 1 );
      for ( size_t n = 0; n < sei.getRectangleNumberOfObjectsMinus1( p ) + 1; n++ ) {
        sei.setRectangleObjectIdx( p, n, bitstream.read( sei.getLog2MaxObjectIdxTracked() ) );
      }
    }
  }
};

// F.2.15  Buffering period SEI message syntax
void PCCBitstreamReader::bufferingPeriod( PCCBitstream&        bitstream,
                                          SEI&                 seiAbstract,
                                          bool                 NalHrdBpPresentFlag,
                                          bool                 AclHrdBpPresentFlag,
                                          std::vector<uint8_t> hrdCabCntMinus1 ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEIBufferingPeriod&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  sei.setAtlasSequenceParameterSetId( bitstream.readUvlc() );    // ue(v)
  sei.setIrapCabParamsPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( sei.getIrapCabParamsPresentFlag() ) {
    sei.setCabDelayOffset( bitstream.read( fixedBitcount ) );  // u(v)
    sei.setDabDelayOffset( bitstream.read( fixedBitcount ) );  // u(v)
  }
  sei.setConcatenationFlag( bitstream.read( 1 ) != 0U );                      // u(1)
  sei.setAtlasCabRemovalDelayDeltaMinus1( bitstream.read( fixedBitcount ) );  // u(v)
  sei.setMaxSubLayersMinus1( bitstream.read( 3 ) );                           // u(3)
  sei.allocate();
  for ( size_t i = 0; i <= sei.getMaxSubLayersMinus1(); i++ ) {
    if ( NalHrdBpPresentFlag ) {
      for ( size_t j = 0; j < hrdCabCntMinus1[i] + 1; j++ ) {
        sei.setNalInitialCabRemovalDelay( i, j, bitstream.read( fixedBitcount ) );   // u(v)
        sei.setNalInitialCabRemovalOffset( i, j, bitstream.read( fixedBitcount ) );  // u(v)
      }
      if ( sei.getIrapCabParamsPresentFlag() ) {
        sei.setNalInitialAltCabRemovalDelay( i, bitstream.read( fixedBitcount ) );   // u(v)
        sei.setNalInitialAltCabRemovalOffset( i, bitstream.read( fixedBitcount ) );  // u(v)
      }
    }
    if ( AclHrdBpPresentFlag ) {
      for ( size_t j = 0; j < hrdCabCntMinus1[i] + 1; j++ ) {
        sei.setAclInitialCabRemovalDelay( i, j, bitstream.read( fixedBitcount ) );   // u(v)
        sei.setAclInitialCabRemovalOffset( i, j, bitstream.read( fixedBitcount ) );  // u(v)
      }
      if ( sei.getIrapCabParamsPresentFlag() ) {
        sei.setAclInitialAltCabRemovalDelay( i, bitstream.read( fixedBitcount ) );   // u(v)
        sei.setAclInitialAltCabRemovalOffset( i, bitstream.read( fixedBitcount ) );  // u(v)
      }
    }
  }
}

// F.2.16  Atlas frame timing SEI message syntax
void PCCBitstreamReader::atlasFrameTiming( PCCBitstream& bitstream, SEI& seiAbstract, bool CabDabDelaysPresentFlag ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEIAtlasFrameTiming&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  if ( CabDabDelaysPresentFlag ) {
    sei.setAftCabRemovalDelayMinus1( bitstream.read( fixedBitcount ) );  // u(v)
    sei.setAftDabOutputDelay( bitstream.read( fixedBitcount ) );         // u(v)
  }
}

// F.2.13.5  Atlas information  SEI message syntax
void PCCBitstreamReader::atlasInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIAtlasInformation&>( seiAbstract );
  sei.setPersistenceFlag( bitstream.read( 1 ) );   //	u(1)
  sei.setResetFlag( bitstream.read( 1 ) );         // 	u(1)
  sei.setNumAtlasesMinus1( bitstream.read( 6 ) );  // 	u(6)
  sei.setNumUpdates( bitstream.readUvlc() );       // ue(v)
  sei.allocate();
  if ( sei.getNumUpdates() > 0 ) {
    sei.setLog2MaxObjectIdxTracked( bitstream.read( 5 ) );  //	u(5)
    for ( size_t i = 0; i < sei.getNumUpdates() + 1; i++ ) {
      sei.setObjectIdx( i, bitstream.read( sei.getLog2MaxObjectIdxTracked() ) );  // u(v)
      for ( size_t j = 0; j < sei.getNumAtlasesMinus1() + 1; j++ ) {
        sei.setObjectInAtlasPresentFlag( i, j, bitstream.read( 1 ) );  // u(1)
      }
    }
  }
}

// F.2.16.1	Viewport camera parameters SEI messages syntax
void PCCBitstreamReader::viewportCameraParameters( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIViewportCameraParameters&>( seiAbstract );
  sei.setCameraId( bitstream.read( 10 ) );   // u(10)
  sei.setCancelFlag( bitstream.read( 1 ) );  // u(1)
  if ( sei.getCameraId() > 0 && !sei.getCancelFlag() ) {
    sei.setPersistenceFlag( bitstream.read( 1 ) );              // u(1)
    sei.setCameraType( bitstream.read( 3 ) );                   // u(3)
    if ( sei.getCameraType() == 0 ) {                           // equirectangular
      sei.setErpHorizontalFov( bitstream.read( 32 ) );          // fl(32)
      sei.setErpVerticalFov( bitstream.read( 32 ) );            // fl(32)
    } else if ( sei.getCameraType() == 1 ) {                    // perspective
      sei.setPerspectiveAspectRatio( bitstream.read( 32 ) );    // fl(32)
      sei.setPerspectiveHorizontalFov( bitstream.read( 32 ) );  // fl(32)
    } else if ( sei.getCameraType() == 2 ) {                    /* orthographic */
      sei.setOrthoAspectRatio( bitstream.read( 32 ) );          // fl(32)
      sei.setOrthoHorizontalSize( bitstream.read( 32 ) );       // fl(32)
    }
    sei.setClippingNearPlane( bitstream.read( 32 ) );  // fl(32)
    sei.setClippingFarPlane( bitstream.read( 32 ) );   // fl(32)
  }
}

// F.2.16.1	Viewport camera parameters SEI messages syntax
void PCCBitstreamReader::viewportPosition( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIViewportPosition&>( seiAbstract );
  sei.setViewportId( bitstream.read( 8 ) );                   // u(8)
  sei.setCameraParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( sei.getCameraParametersPresentFlag() ) {
    sei.setViewportId( bitstream.read( 10 ) );  //	u(10)
  }
  sei.setCancelFlag( bitstream.read( 1 ) );  // u(1)
  if ( !sei.getCancelFlag() ) {
    sei.setPersistenceFlag( bitstream.read( 1 ) );  // u(1)
    for ( size_t d = 0; d < 3; d++ ) {
      sei.setPosition( d, bitstream.read( 32 ) );  //	fl(32)
    }
    sei.setQuaternionX( bitstream.read( 32 ) );    //	fl(32)
    sei.setQuaternionY( bitstream.read( 32 ) );    //	fl(32)
    sei.setQuaternionZ( bitstream.read( 32 ) );    //	fl(32)
    sei.setCenterViewFlag( bitstream.read( 1 ) );  // 	u(1)
    if ( !sei.getCenterViewFlag() ) {
      sei.setLeftViewFlag( bitstream.read( 1 ) );  // u(1)
    }
  }
}

// H.16.2.1 Geometry smoothing SEI message syntax
void PCCBitstreamReader::geometrySmoothing( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIGeometrySmoothing&>( seiAbstract );
  sei.setSmoothingPersistenceFlag( bitstream.read( 1 ) );   //	u(1)
  sei.setSmoothingResetFlag( bitstream.read( 1 ) );         //	u(1)
  sei.setSmoothingInstancesUpdated( bitstream.read( 8 ) );  //	u(8)
  sei.allocate();
  for ( size_t i = 0; i < sei.getSmoothingInstancesUpdated(); i++ ) {
    sei.setSmoothingInstanceIndex( i, bitstream.read( 8 ) );  // u(8)
    size_t k = sei.getSmoothingInstanceIndex( i );
    sei.setSmoothingInstanceCancelFlag( k, bitstream.read( 1 ) );  //	u(1)
    if ( !sei.getSmoothingInstanceCancelFlag( k ) ) {
      sei.setSmoothingMethodType( k, bitstream.read( 8 ) );  // u(8)
      if ( sei.getSmoothingMethodType( k ) == 1 ) {
        sei.setSmoothingGridSizeMinus2( k, bitstream.read( 7 ) );  // u(8)
        sei.setSmoothingThreshold( k, bitstream.read( 8 ) );       // u(8)
      } else if ( sei.getSmoothingMethodType( k ) == 2 ) {
        sei.setPbfLog2ThresholdMinus1( k, bitstream.read( 2 ) );  //	u(2)
        sei.setPbfPassesCountMinus1( k, bitstream.read( 2 ) );    //	u(2)
        sei.setPbfFilterSizeMinus1( k, bitstream.read( 3 ) );     //	u(3)
      }
    }
  }
}

// H.16.2.2	Attribute smoothing SEI message syntax
void PCCBitstreamReader::attributeSmoothing( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIAttributeSmoothing&>( seiAbstract );
  sei.setSmoothingPersistenceFlag( bitstream.read( 1 ) );  //	u(1)
  sei.setSmoothingResetFlag( bitstream.read( 1 ) );        //	u(1)
  sei.setNumAttributesUpdated( bitstream.readUvlc() );     //	ue(v)
  sei.allocate();
  for ( size_t j = 0; j < sei.getNumAttributesUpdated(); j++ ) {
    sei.setAttributeIdx( j, bitstream.read( 7 ) );  // u(7)
    size_t k = sei.getAttributeIdx( j );
    sei.setAttributeSmoothingCancelFlag( k, bitstream.read( 1 ) );  // u(1)
    sei.setSmoothingInstancesUpdated( k, bitstream.read( 8 ) );     //	u(8)
    for ( size_t i = 0; i < sei.getSmoothingInstancesUpdated( k ); i++ ) {
      size_t m = bitstream.read( 8 );  //	u(8)
      sei.allocate( k + 1, m + 1 );
      sei.setSmoothingInstanceIndex( k, i, m );
      sei.setSmoothingInstanceCancelFlag( k, m, bitstream.read( 1 ) );  // u(1)
      if ( sei.getSmoothingInstanceCancelFlag( k, m ) != 1 ) {
        sei.setSmoothingMethodType( k, m, bitstream.read( 8 ) );  // u(8)
        if ( sei.getSmoothingMethodType( k, m ) ) {
          sei.setSmoothingGridSizeMinus2( k, m, bitstream.read( 8 ) );       //	u(8)
          sei.setSmoothingThreshold( k, m, bitstream.read( 8 ) );            // u(8)
          sei.setSmoothingThresholdVariation( k, m, bitstream.read( 8 ) );   // u(8)
          sei.setSmoothingThresholdDifference( k, m, bitstream.read( 8 ) );  // u(8)
        }
      }
    }
  }
}

// G.2  VUI syntax
// G.2.1  VUI parameters syntax
void PCCBitstreamReader::vuiParameters( PCCBitstream& bitstream, VUIParameters& vp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  vp.setTimingInfoPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( vp.getTimingInfoPresentFlag() ) {
    vp.setNumUnitsInTick( bitstream.read( 32 ) );              // u(32)
    vp.setTimeScale( bitstream.read( 32 ) );                   // u(32)
    vp.setPocProportionalToTimingFlag( bitstream.read( 1 ) );  // u(1)
    if ( vp.getPocProportionalToTimingFlag() ) {
      vp.setNumTicksPocDiffOneMinus1( bitstream.readUvlc() );  // ue(v)
    }
    vp.setHrdParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( vp.getHrdParametersPresentFlag() ) { hrdParameters( bitstream, vp.getHrdParameters() ); }
  }
  vp.setBitstreamRestrictionPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( vp.getBitstreamRestrictionPresentFlag() ) {
    vp.setTilesRestrictedFlag( bitstream.read( 1 ) );                    // u(1)
    vp.setConsistentTilesForVideoComponentsFlag( bitstream.read( 1 ) );  // u(1)
    vp.setMaxNumTilesPerAtlas( bitstream.readUvlc() );                   // ue(v)
  }
  vp.setCoordinateSystemParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( vp.getCoordinateSystemParametersPresentFlag() ) {
    coordinateSystemParameters( bitstream, vp.getCoordinateSystemParameters() );
  }
  vp.setUnitInMetresFlag( bitstream.read( 1 ) );           // u(1)
  vp.setDisplayBoxInfoPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( vp.getDisplayBoxInfoPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      vp.setDisplayBoxOrigin( d, bitstream.readUvlc() );  // ue(v)
      vp.setDisplayBoxSize( d, bitstream.readUvlc() );    // ue(v)
    }
    vp.setAnchorPointPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( vp.getAnchorPointPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        vp.setAnchorPoint( d, bitstream.read( 32 ) );  // fl(32)
      }
    }
  }
}
// G.2.2  HRD parameters syntax
void PCCBitstreamReader::hrdParameters( PCCBitstream& bitstream, HrdParameters& hp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  hp.setNalParametersPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  hp.setAclParametersPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( hp.getNalParametersPresentFlag() || hp.getAclParametersPresentFlag() ) {
    hp.setBitRateScale( bitstream.read( 4 ) );                        // u(4)
    hp.setCabSizeScale( bitstream.read( 4 ) );                        // u(4)
    hp.setInitialCabRemovalDelayLengthMinus1( bitstream.read( 5 ) );  // u(5)
    hp.setAuCabRemovalDelayLengthMinus1( bitstream.read( 5 ) );       // u(5)
    hp.setDabOutputDelayLengthMinus1( bitstream.read( 5 ) );          // u(5)
  }
  for ( size_t i = 0; i <= hp.getMaxNumSubLayersMinus1(); i++ ) {
    hp.setFixedAtlasRateGeneralFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
    if ( !hp.getFixedAtlasRateGeneralFlag( i ) ) {
      hp.setFixedAtlasRateWithinCasFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
    }
    if ( hp.getFixedAtlasRateWithinCasFlag( i ) ) {
      hp.setElementalDurationInTcMinus1( i, bitstream.read( 1 ) );  // ue(v)
    } else {
      hp.setLowDelayFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
    }
    if ( !hp.getLowDelayFlag( i ) ) {
      hp.setCabCntMinus1( i, bitstream.read( 1 ) );  // ue(v)
    }
    if ( hp.getNalParametersPresentFlag() ) {
      hrdSubLayerParameters( bitstream, hp.getHdrSubLayerParameters( 0, i ),
                             static_cast<size_t>( hp.getCabCntMinus1( i ) ) );
    }
    if ( hp.getAclParametersPresentFlag() ) {
      hrdSubLayerParameters( bitstream, hp.getHdrSubLayerParameters( 1, i ),
                             static_cast<size_t>( hp.getCabCntMinus1( i ) ) );
    }
  }
}

// G.2.3  Sub-layer HRD parameters syntax
void PCCBitstreamReader::hrdSubLayerParameters( PCCBitstream& bitstream, HrdSubLayerParameters& hlsp, size_t cabCnt ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  hlsp.allocate( cabCnt + 1 );
  for ( size_t i = 0; i <= cabCnt; i++ ) {
    hlsp.setBitRateValueMinus1( i, bitstream.readUvlc() );  // ue(v)
    hlsp.setCabSizeValueMinus1( i, bitstream.readUvlc() );  // ue(v)
    hlsp.setCbrFlag( i, bitstream.read( 1 ) != 0U );        // u(1)
  }
}

// G.2.4	Coordinate system parameters syntax
void PCCBitstreamReader::coordinateSystemParameters( PCCBitstream& bitstream, CoordinateSystemParameters& csp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  csp.setForwardAxis( bitstream.read( 2 ) );    // u(2)
  csp.setDeltaLeftAxis( bitstream.read( 1 ) );  // u(1)
  csp.setForwardSign( bitstream.read( 1 ) );    // u(1)
  csp.setLeftSign( bitstream.read( 1 ) );       // u(1)
  csp.setUpSign( bitstream.read( 1 ) );         // u(1)
}

// H.7.3.4.1	VPS V-PCC extension syntax
void PCCBitstreamReader::vpsVpccExtension( PCCBitstream& bitstream, VpsVpccExtension& ext ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}

// H.7.3.4.2	VPS MIV extension syntax
void PCCBitstreamReader::vpsMivExtension( PCCBitstream& bitstream ) { TRACE_BITSTREAM( "%s \n", __func__ ); }

// H.7.3.6.1.1	ASPS V-PCC extension syntax
void PCCBitstreamReader::aspsVpccExtension( PCCBitstream&                  bitstream,
                                            AtlasSequenceParameterSetRbsp& asps,
                                            AspsVpccExtension&             ext ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ext.setRemoveDuplicatePointEnableFlag( bitstream.read( 1 ) );
  if ( asps.getPixelDeinterleavingFlag() || asps.getPointLocalReconstructionEnabledFlag() ) {
    ext.setSurfaceThicknessMinus1( bitstream.read( 7 ) );
  }
}
// H.7.3.6.1.2	ASPS MIV extension syntax
void PCCBitstreamReader::aspsMivExtension( PCCBitstream& bitstream ) { TRACE_BITSTREAM( "%s \n", __func__ ); }

// H.7.3.6.2.1	AFPS V-PCC extension syntax
void PCCBitstreamReader::afpsVpccExtension( PCCBitstream& bitstream, AfpsVpccExtension& ext ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}
// H.7.3.6.2.2	AFPS MIV extension syntax
void PCCBitstreamReader::afpsMivExtension( PCCBitstream& bitstream ) { TRACE_BITSTREAM( "%s \n", __func__ ); }

// H.7.3.6.3.1	AAPS V-PCC extension syntax
void PCCBitstreamReader::aapsVpccExtension( PCCBitstream& bitstream, AapsVpccExtension& ext ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ext.setCameraParametersPresentFlag( bitstream.read( 1 ) );  // u(1);
  if ( ext.getCameraParametersPresentFlag() ) { atlasCameraParameters( bitstream, ext.getAtlasCameraParameters() ); }
}
// H.7.3.6.3.2	AAPS MIV extension syntax
void PCCBitstreamReader::aapsMivExtension( PCCBitstream& bitstream ) { TRACE_BITSTREAM( "%s \n", __func__ ); }

// H.7.3.6.3.2	Atlas camera parameters syntax
void PCCBitstreamReader::atlasCameraParameters( PCCBitstream& bitstream, AtlasCameraParameters& acp ) {
  acp.setCameraModel( bitstream.read( 8 ) );  // u(8)
  if ( acp.getCameraModel() == 1 ) {
    acp.setScaleEnabledFlag( bitstream.read( 1 ) );     // u(1)
    acp.setOffsetEnabledFlag( bitstream.read( 1 ) );    // u(1)
    acp.setRotationEnabledFlag( bitstream.read( 1 ) );  // u(1)
    if ( acp.getScaleEnabledFlag() ) {
      for ( size_t i = 0; i < 3; i++ ) { acp.setScaleOnAxis( i, bitstream.read( 32 ) ); }  // u(32)
    }
    if ( acp.getOffsetEnabledFlag() ) {
      for ( size_t i = 0; i < 3; i++ ) { acp.setOffsetOnAxis( i, bitstream.readS( 32 ) ); }  // i(32)
    }
    if ( acp.getRotationEnabledFlag() ) {
      for ( size_t i = 0; i < 3; i++ ) { acp.setRotation( i, bitstream.readS( 16 ) ); }  // i(16)
    }
  }
}
