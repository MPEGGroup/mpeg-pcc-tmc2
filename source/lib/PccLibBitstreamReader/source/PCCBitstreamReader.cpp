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
    prevPatchSizeU_( 0 ), prevPatchSizeV_( 0 ), predPatchIndex_( 0 ), prevFrameIndex_( 0 ) {}
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
    TRACE_BITSTREAM( "V3C Unit Size(%zuth/%zu)  = %zu \n", unitCount, ssvu.getV3CUnitCount(),
                     v3cUnit.getV3CUnitSize() );
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
  v3cUnit.setV3CUnitSize( bitstream.read( 8 * ( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) ) );  // u(v)
  auto pos = bitstream.getPosition();
  v3cUnit.getV3CUnitDataBitstream().copyFrom( bitstream, pos.bytes, v3cUnit.getV3CUnitSize() );
  uint8_t v3cUnitType8 = v3cUnit.getV3CUnitDataBitstream().buffer()[0];
  auto    v3cUnitType  = static_cast<V3CUnitType>( v3cUnitType8 >>= 3 );
  v3cUnit.setV3CUnitType( v3cUnitType );
  TRACE_BITSTREAM( "V3CUnitType: %hhu V3CUnitSize: %zu\n", v3cUnitType8, v3cUnit.getV3CUnitSize() );
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
        int32_t v3cUnitSize = (int32_t)unit.getV3CUnitDataBitstream().capacity();
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
  PCCBitstream& bitstream = currV3CUnit.getV3CUnitDataBitstream();
#ifdef BITSTREAM_TRACE
  bitstream.setTrace( true );
  bitstream.setTraceFile( traceFile_ );
  TRACE_BITSTREAM( "PCCBitstream::(%s)\n", toString( currV3CUnit.getV3CUnitType() ).c_str() );
#endif
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto position = static_cast<int32_t>( bitstream.size() );
  v3cUnitHeader( syntax, bitstream, v3cUnitType );
  assert( v3cUnitType == currV3CUnit.getV3CUnitType() );
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
    sps.setAtlasId( j, bitstream.read( 16 ) );        // u(16)
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
    ai.setAttributeMapAbsoluteCodingPersistanceFlag( i, true );
    if ( sps.getMapCountMinus1( atlasIndex ) > 0 ) {
      ai.setAttributeMapAbsoluteCodingPersistanceFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
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
  for ( size_t i = 2; i < nalUnit.getNalUnitSize(); i++ ) {
    nalUnit.setNalUnitData( i, bitstream.read( 8 ) );  // b(8)
  }
}

// 7.3.5.2 NAL unit header syntax
void PCCBitstreamReader::nalUnitHeader( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.read( 1 );                                                        // f(1)
  nalUnit.setNalUnitType( static_cast<NalUnitType>( bitstream.read( 6 ) ) );  // u(6)
  nalUnit.setLayerId( bitstream.read( 6 ) );                                  // u(6)
  nalUnit.setTemporalyIdPlus1( bitstream.read( 3 ) );                         // u(3)
  TRACE_BITSTREAM( " NalUnitSize      = %zu \n", nalUnit.getNalUnitSize() );
  TRACE_BITSTREAM( " NalUnitType      = %hhu \n", nalUnit.getNalUnitType() );
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
  TRACE_BITSTREAM( "Log2PatchPackingBlockSize = %u \n", asps.getLog2PatchPackingBlockSize());
  asps.setPatchSizeQuantizerPresentFlag( bitstream.read( 1 ) );            // u(1)
  asps.setMapCountMinus1( bitstream.read( 4 ) );                           // u(4)
  asps.setPixelDeinterleavingFlag( bitstream.read( 1 ) );                  // u(1)
  if ( asps.getPixelDeinterleavingFlag() ) {
    for ( size_t i = 0; i < asps.getMapCountMinus1() + 1; i++ ) {
      asps.setPixeDeinterleavingMapFlag( i, bitstream.read( 1 ) );  // u(1)
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
      uint32_t NumTilesInPatchFrame =
          ( afti.getNumPartitionColumnsMinus1() + 1 ) * ( afti.getNumPartitionRowsMinus1() + 1 );
      afti.setNumTilesInAtlasFrameMinus1( bitstream.readUvlc() );  // ue(v)
      for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
        uint8_t bitCount = ceilLog2( NumTilesInPatchFrame );
        if ( i > 0 ) {
          afti.setTopLeftPartitionIdx( i, bitstream.read( bitCount ) );  // u(v)
        } else {
          afti.setTopLeftPartitionIdx( i, 0 );
        }
        bitCount = ceilLog2( NumTilesInPatchFrame - afti.getTopLeftPartitionIdx( i ) );
        afti.setBottomRightPartitionColumnOffset( i, bitstream.read( bitCount ) );  // u(v)
        afti.setBottomRightPartitionRowOffset( i, bitstream.read( bitCount ) );     // u(v)
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
  atlasTileHeader( atgl.getAtlasTileHeader(), syntax, bitstream );
  if ( atgl.getAtlasTileHeader().getType() != SKIP_TILE ) {
    atlasTileDataUnit( atgl.getAtlasTileDataUnit(), atgl.getAtlasTileHeader(), syntax, bitstream );
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.10	Frame order count RBSP syntax
uint32_t PCCBitstreamReader::frameOrderCountRbsp( PCCBitstream&                    bitstream,
                                                  V3CUnitPayloadHeader&            vuh,
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
  // auto   tileType = ath.getType();
  // TRACE_BITSTREAM( "ath.getType()        = %zu \n", ath.getType() );
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
  TRACE_BITSTREAM( " 2dSizeXY: %d,%d\n", int32_t( pdu.get2dSizeXMinus1() + 1 ),
                   int32_t( pdu.get2dSizeYMinus1() + 1 ) );
  uint8_t bitCount3DPos = syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() + 1;
  pdu.set3dPosX( bitstream.read( bitCount3DPos ) );  // u(v)
  pdu.set3dPosY( bitstream.read( bitCount3DPos ) );  // u(v)
  TRACE_BITSTREAM( " 3dPosXY: %zu,%zu\n", pdu.get3dPosX(), pdu.get3dPosY() );

  const uint8_t bitCountForMinDepth =
      syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
      ath.getPosMinZQuantizer() + 2;
  pdu.set3dPosMinZ( bitstream.read( bitCountForMinDepth ) );  // u(v)
  TRACE_BITSTREAM( " Pdu3dPosMinZ: %zu ( bitCountForMinDepth = %u = %u - %u + %u ) \n", pdu.get3dPosMinZ(),
                   bitCountForMinDepth,
                   syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1(),
                   ath.getPosMinZQuantizer(), 2 );

  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    uint8_t bitCountForMaxDepth = syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
                                  ath.getPosDeltaMaxZQuantizer() + 2;
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
      pdu.setLodScaleXminus1( uint8_t( bitstream.readUvlc() ) );    // ue(v)
      pdu.setLodScaleY( uint8_t( bitstream.readUvlc() ) );          // ue(v)
    }
  } else {
    pdu.setLodEnableFlag( false );
    pdu.setLodScaleXminus1( 0 );
    pdu.setLodScaleY( 0 );
  }
  TRACE_BITSTREAM( "PointLocalReconstructionEnabledFlag = %d \n", asps.getPointLocalReconstructionEnabledFlag() );
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plrd = pdu.getPointLocalReconstructionData();
    TRACE_BITSTREAM( "Size = %ld %ld\n", pdu.get2dSizeYMinus1() + 1, pdu.get2dSizeYMinus1() + 1 );
    plrd.allocate( pdu.get2dSizeXMinus1() + 1, pdu.get2dSizeYMinus1() + 1 );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
  }
#ifdef BITSTREAM_TRACE
  if ( asps.getMivExtensionFlag() ) {
    TRACE_BITSTREAM( "MivExtension \n" );
  } 
#endif
  TRACE_BITSTREAM(
      "Frame %zu, Patch(%zu) => 2Dpos = %4zu %4zu 2Dsize = %4ld %4ld 3Dpos = "
      "%ld %ld %ld DeltaMaxZ = %ld Projection = "
      "%zu "
      "Orientation = %zu lod=(%zu) %zu %zu\n ",
      pdu.getFrameIndex(), pdu.getPatchIndex(), pdu.get2dPosX(), pdu.get2dPosY(), pdu.get2dSizeXMinus1() + 1,
      pdu.get2dSizeYMinus1() + 1, pdu.get3dPosX(), pdu.get3dPosY(), pdu.get3dPosMinZ(),
      pdu.get3dPosDeltaMaxZ(), pdu.getProjectionId(), pdu.getOrientationIndex(), pdu.getLodEnableFlag(),
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
    mpdu.setOverride3dParamsFlag( bitstream.read( 1 ) != 0U ); // u(1) 
    if ( mpdu.getOverride3dParamsFlag() ) {
      mpdu.set3dPosX( bitstream.readSvlc() );     // se(v)
      mpdu.set3dPosY( bitstream.readSvlc() );     // se(v)
      mpdu.set3dPosMinZ( bitstream.readSvlc() );  // se(v)
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        mpdu.set3dPosDeltaMaxZ( bitstream.readSvlc() );  // se(v)
      }
      if ( asps.getPointLocalReconstructionEnabledFlag() ) {
        overridePlrFlag = ( bitstream.read( 1 ) != 0U ); // u(1) 
        mpdu.setOverridePlrFlag( static_cast<int64_t>( overridePlrFlag ) );
      }
    }
  }
  if ( overridePlrFlag && asps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plrd = mpdu.getPointLocalReconstructionData();
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n", prevPatchSizeU_, prevPatchSizeV_,
                     mpdu.get2dDeltaSizeX(), mpdu.get2dDeltaSizeY(),
                     prevPatchSizeU_ + mpdu.get2dDeltaSizeX(), prevPatchSizeV_ + mpdu.get2dDeltaSizeY() );
    plrd.allocate( prevPatchSizeU_ + mpdu.get2dDeltaSizeX(), prevPatchSizeV_ + mpdu.get2dDeltaSizeY() );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
    prevPatchSizeU_ += mpdu.get2dDeltaSizeX();
    prevPatchSizeV_ += mpdu.get2dDeltaSizeY();
  }
#ifdef BITSTREAM_TRACE
  if ( asps.getMivExtensionFlag() ) {
    TRACE_BITSTREAM( "MivExtension \n" );
  } 
#endif
  TRACE_BITSTREAM(
      "%zu frame %zu MergePatch => Ref Frame Idx = %d ShiftUV = %ld %ld "
      "DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      mpdu.getRefIndex(), mpdu.get2dPosX(), mpdu.get2dPosY(), mpdu.get2dDeltaSizeX(),
      mpdu.get2dDeltaSizeY(), mpdu.get3dPosX(), mpdu.get3dPosY(), mpdu.get3dPosMinZ(),
      mpdu.get3dPosDeltaMaxZ() );
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
      ipdu.get2dPosY(), ipdu.get2dDeltaSizeX(), ipdu.get2dDeltaSizeY(), ipdu.get3dPosX(),
      ipdu.get3dPosY(), ipdu.get3dPosMinZ(), ipdu.get3dPosDeltaMaxZ() );

  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    auto&   atglPrev      = syntax.getAtlasTileLayer( prevFrameIndex_ );
    auto&   atghPrev      = atglPrev.getAtlasTileHeader();
    auto&   atgdPrev      = atglPrev.getAtlasTileDataUnit();
    auto&   pidPrev       = atgdPrev.getPatchInformationData( ipdu.getRefPatchIndex() + predPatchIndex_ );
    auto    patchModePrev = pidPrev.getPatchMode();
    int32_t sizeU         = ipdu.get2dDeltaSizeX();
    int32_t sizeV         = ipdu.get2dDeltaSizeY();
    if ( atghPrev.getType() == P_TILE ) {
      if( patchModePrev == P_MERGE ) { 
        auto& plrdPrev = pidPrev.getMergePatchDataUnit().getPointLocalReconstructionData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      } else if( patchModePrev == P_INTER ) { 
        auto& plrdPrev = pidPrev.getInterPatchDataUnit().getPointLocalReconstructionData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      } else if( patchModePrev == P_INTRA ) { 
        auto& plrdPrev = pidPrev.getPatchDataUnit().getPointLocalReconstructionData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      }    
    } else if ( ath.getType() == I_TILE ) {
       if( patchModePrev == I_INTRA ) { 
        auto& plrdPrev = pidPrev.getPatchDataUnit().getPointLocalReconstructionData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      }
    }
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n", sizeU - ipdu.get2dDeltaSizeX(),
                     sizeV - ipdu.get2dDeltaSizeY(), ipdu.get2dDeltaSizeX(), ipdu.get2dDeltaSizeY(), sizeU,
                     sizeV );
    auto& plrd = ipdu.getPointLocalReconstructionData();
    plrd.allocate( sizeU, sizeV );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );    
    prevPatchSizeU_ = sizeU;
    prevPatchSizeV_ = sizeV;
    predPatchIndex_ += ipdu.getRefPatchIndex() + 1;
  }
#ifdef BITSTREAM_TRACE
  if ( asps.getMivExtensionFlag() ) {
    TRACE_BITSTREAM( "MivExtension \n" );
  } 
#endif
}

// 7.3.7.7  Raw patch data unit syntax
void PCCBitstreamReader::rawPatchDataUnit( RawPatchDataUnit&   rpdu,
                                           AtlasTileHeader&    ath,
                                           PCCHighLevelSyntax& syntax,
                                           PCCBitstream&       bitstream ) {
  auto& sps = syntax.getVps();
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  int32_t bitCount = ath.getRaw3dPosAxisBitCountMinus1() + 1;
  TRACE_BITSTREAM( " AtghRaw3dPosAxisBitCountMinus1 = %zu => bitcount = %d \n",
    ath.getRaw3dPosAxisBitCountMinus1(), bitCount );
  if ( 1 /* TODO: evaluate: AuxTileHeight[ TileIdToIndex[ ath_id ] ] > 0 */ ) {
    rpdu.setPatchInAuxiliaryVideoFlag( bitstream.read( 1 ) != 0U );  // u(1)
  }
  rpdu.set2dPosX( bitstream.readUvlc() );         // ue(v)
  rpdu.set2dPosY( bitstream.readUvlc() );         // ue(v)
  rpdu.set2dSizeXMinus1( bitstream.readUvlc() );  // ue(v)
  rpdu.set2dSizeYMinus1( bitstream.readUvlc() );  // ue(v)
  rpdu.set3dPosX( bitstream.read( bitCount ) );   // u(v)
  rpdu.set3dPosY( bitstream.read( bitCount ) );   // u(v)
  rpdu.set3dPosZ( bitstream.read( bitCount ) );   // u(v)
  rpdu.setRawPointsMinus1( bitstream.readUvlc() ); // ue(v)
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
  epdu.set2dPosX( bitstream.readUvlc() );         // ue(v)
  epdu.set2dPosY( bitstream.readUvlc() );         // ue(v)
  epdu.set2dSizeXMinus1( bitstream.readUvlc() );  // ue(v)
  epdu.set2dSizeYMinus1( bitstream.readUvlc() );  // ue(v)
  epdu.setPatchCountMinus1( bitstream.readUvlc() );// ue(v)
  for ( size_t i = 0; i < epdu.getPatchCountMinus1() + 1; i++ ) {
    epdu.setAssociatedPatchesIdx( i, bitstream.readUvlc( ) );// ue(v)
    epdu.setPoints( i, bitstream.readUvlc() ); // ue(v)
  }
#ifdef BITSTREAM_TRACE
  TRACE_BITSTREAM( "EOM Patch => UV %4zu %4zu  S=%4ld %4ld  N=%4ld\n", epdu.get2dPosX(), epdu.get2dPosY(),
                   epdu.get2dSizeXMinus1() + 1, epdu.get2dSizeYMinus1() + 1,
                   epdu.getPatchCountMinus1() + 1 );
  for ( size_t i = 0; i < epdu.getPatchCountMinus1() + 1; i++ ) {
    TRACE_BITSTREAM( "%4ld, %4ld\n", epdu.getAssociatedPatchesIdx(i), epdu.getPoints(i) );
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
        (int)nu.getNalUnitType(), toString( nu.getNalUnitType() ).c_str(),
        ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ), nu.getNalUnitSize(), bitstream.size() );
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
    plrd.setLevelFlag( bitstream.read( 1 ) != 0U );   // u(1)
  } else {
    plrd.setLevelFlag( true );
  }
  TRACE_BITSTREAM( "  LevelFlag = %u \n", plrd.getLevelFlag() );
  if ( plrd.getLevelFlag() ) {
    plrd.setPresentFlag( bitstream.read( 1 ) != 0U ); // u(1)
    if ( plrd.getPresentFlag() ) { 
      plrd.setModeMinus1( bitstream.read( bitCountMode ) ); // u(v)
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
  ssnu.setUnitSizePrecisionBytesMinus1( bitstream.read( 3 ) );  // u(3)
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 = %lu \n", ssnu.getUnitSizePrecisionBytesMinus1() );
  bitstream.read( 5 );                                          // u(5)
}

// C.2.2 Sample stream NAL unit syntax
void PCCBitstreamReader::sampleStreamNalUnit( PCCHighLevelSyntax&  syntax,
                                              PCCBitstream&        bitstream,
                                              SampleStreamNalUnit& ssnu,
                                              size_t               index ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& nu = ssnu.getNalUnit( index );
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 = %lu \n", ssnu.getUnitSizePrecisionBytesMinus1() );
  nu.setNalUnitSize( bitstream.read( 8 * ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ) ) );  // u(v)
  nu.allocate();
  nalUnitHeader( bitstream, nu );
  switch ( nu.getNalUnitType() ) {
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
    case NAL_SUFFIX_NSEI: seiRbsp( syntax, bitstream, nu.getNalUnitType() ); break;
    case NAL_PREFIX_ESEI: 
    case NAL_PREFIX_NSEI: seiRbsp( syntax, bitstream, nu.getNalUnitType() ); break;
    default:
      fprintf( stderr, "sampleStreamNalUnit type = %d not supported\n", static_cast<int32_t>( nu.getNalUnitType() ) );
  }
}

// E.2  SEI payload syntax
// E.2.1  General SEI message syntax
void PCCBitstreamReader::seiPayload( PCCBitstream&       bitstream,
                                     PCCHighLevelSyntax& syntax,
                                     NalUnitType         nalUnitType,
                                     SeiPayloadType      payloadType,
                                     size_t              payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SEI& sei = syntax.addSei( nalUnitType, payloadType );
  if ( nalUnitType == NAL_PREFIX_ESEI ) {
    if ( payloadType == BUFFERING_PERIOD ) {
      bool                 NalHrdBpPresentFlag = false;
      bool                 AclHrdBpPresentFlag = false;
      std::vector<uint8_t> hrdCabCntMinus1;
      bufferingPeriod( bitstream, sei, payloadSize, NalHrdBpPresentFlag, AclHrdBpPresentFlag, hrdCabCntMinus1 );
    } else if ( payloadType == ATLAS_FRAME_TIMING ) {
      atlasFrameTiming( bitstream, sei, payloadSize, false );
    } else if ( payloadType == FILLER_PAYLOAD ) {
      fillerPayload( bitstream, sei, payloadSize );
    } else if ( payloadType == USER_DATAREGISTERED_ITUTT35 ) {
      userDataRegisteredItuTT35( bitstream, sei, payloadSize );
    } else if ( payloadType == USER_DATA_UNREGISTERED ) {
      userDataUnregistered( bitstream, sei, payloadSize );
    } else if ( payloadType == RECOVERY_POINT ) {
      recoveryPoint( bitstream, sei, payloadSize );
    } else if ( payloadType == NO_DISPLAY ) {
      noDisplay( bitstream, sei, payloadSize );
    } else if ( payloadType == TIME_CODE ) {
      // timeCode( bitstream, sei, payloadSize );
    } else if ( payloadType == REGIONAL_NESTING ) {
      // regionalNesting( bitstream, sei, payloadSize );
    } else if ( payloadType == SEI_MANIFEST ) {
      seiManifest( bitstream, sei, payloadSize );
    } else if ( payloadType == SEI_PREFIX_INDICATION ) {
      seiPrefixIndication( bitstream, sei, payloadSize );
    } else if ( payloadType == GEOMETRY_TRANSFORMATION_PARAMS ) {
      geometryTransformationParams( bitstream, sei, payloadSize );
    } else if ( payloadType == ATTRIBUTE_TRANSFORMATION_PARAMS ) {
      attributeTransformationParams( bitstream, sei, payloadSize );
    } else if ( payloadType == ACTIVE_SUBSTREAMS ) {
      activeSubstreams( bitstream, sei, payloadSize );
    } else if ( payloadType == COMPONENT_CODEC_MAPPING ) {
      componentCodecMapping( bitstream, sei, payloadSize );
    } else if ( payloadType == PRESENTATION_INFORMATION ) {
      presentationInformation( bitstream, sei, payloadSize );
    } else if ( payloadType == SMOOTHING_PARAMETERS ) {
      smoothingParameters( bitstream, sei, payloadSize );
    } else if ( payloadType == SCENE_OBJECT_INFORMATION ) {
      sceneObjectInformation( bitstream, sei, payloadSize );
    } else if ( payloadType == OBJECT_LABEL_INFORMATION ) {
      objectLabelInformation( bitstream, sei, payloadSize );
    } else if ( payloadType == PATCH_INFORMATION ) {
      patchInformation( bitstream, sei, payloadSize );
    } else if ( payloadType == VOLUMETRIC_RECTANGLE_INFORMATION ) {
      volumetricRectangleInformation( bitstream, sei, payloadSize );
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
  auto& sei = static_cast<SEIUserDataRegisteredItuTT35&>( seiAbstract );
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

// E.2.5  Recovery point SEI message syntax
void PCCBitstreamReader::recoveryPoint( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIRecoveryPoint&>( seiAbstract );
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
  auto& sei = static_cast<SEIReservedSeiMessage&>( seiAbstract );
  sei.getReservedSeiMessagePayloadByte().resize( payloadSize );
  for ( size_t i = 0; i < payloadSize; i++ ) {
    sei.setReservedSeiMessagePayloadByte( i, bitstream.read( 8 ) );  // b(8)
  }
}

// E.2.8  SEI manifest SEI message syntax
void PCCBitstreamReader::seiManifest( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIManifest&>( seiAbstract );
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

// E.2.10  Geometry transformation parameters SEI message syntax
void PCCBitstreamReader::geometryTransformationParams( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIGeometryTransformationParams&>( seiAbstract );
  sei.setGtpCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !sei.getGtpCancelFlag() ) {
    sei.setGtpScaleEnabledFlag( bitstream.read( 1 ) != 0U );     // u(1)
    sei.setGtpOffsetEnabledFlag( bitstream.read( 1 ) != 0U );    // u(1)
    sei.setGtpRotationEnabledFlag( bitstream.read( 1 ) != 0U );  // u(1)
    sei.setGtpNumCameraInfoMinus1( bitstream.read( 3 ) );        // u(3)
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
      for ( uint8_t camId = 0; camId < sei.getGtpNumCameraInfoMinus1(); camId++ ) {
        for ( size_t d = 0; d < 3; d++ ) {
          sei.setGtpCameraOffsetOnAxis( camId, d, bitstream.readS( 16 ) );  // i(16)
        }
        for ( size_t d = 0; d < 3; d++ ) {
          sei.setGtpCameraOrientationOnAxis( camId, d,
                                             bitstream.readS( 16 ) );  // i(16)
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
  auto& sei = static_cast<SEIAttributeTransformationParams&>( seiAbstract );
  sei.setAtpCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !sei.getAtpCancelFlag() ) {
    sei.setAtpNumAttributeUpdates( bitstream.readUvlc() );  // ue(v)
    sei.allocate();
    for ( size_t j = 0; j < sei.getAtpNumAttributeUpdates(); j++ ) {
      sei.setAtpAttributeIdx( j, bitstream.read( 8 ) );  // u(8)
      size_t index = sei.getAtpAttributeIdx( j );
      sei.setAtpDimensionMinus1( index, bitstream.read( 8 ) );  // u(8)
      sei.allocate( index );
      for ( size_t i = 0; i < sei.getAtpDimensionMinus1( index ); i++ ) {
        sei.setAtpScaleParamsEnabledFlag( index, i,
                                          bitstream.read( 1 ) != 0U );  // u(1)
        sei.setAtpOffsetParamsEnabledFlag( index, i,
                                           bitstream.read( 1 ) != 0U );  // u(1)
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
  auto& sei = static_cast<SEIActiveSubstreams&>( seiAbstract );
  sei.setActiveAttributesChangesFlag( bitstream.read( 1 ) != 0U );    // u(1)
  sei.setActiveMapsChangesFlag( bitstream.read( 1 ) != 0U );          // u(1)
  sei.setRawPointsSubstreamsActiveFlag( bitstream.read( 1 ) != 0U );  // u(1)
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

// E.2.13  Component codec mapping SEI message syntax
void PCCBitstreamReader::componentCodecMapping( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIComponentCodecMapping&>( seiAbstract );
  sei.setCcmCodecMappingsCountMinus1( bitstream.read( 8 ) );  // u(8)
  sei.allocate();
  for ( size_t i = 0; i <= sei.getCcmCodecMappingsCountMinus1(); i++ ) {
    sei.setCcmCodecId( i, bitstream.read( 8 ) );                           // u(8)
    sei.setCcmCodec4cc( sei.getCcmCodecId( i ), bitstream.readString() );  // st(v)
  }
}

// m52705
// E.2.14  Volumetric Tiling SEI message syntax
// E.2.14.1  General
// E.2.14.2  Volumetric Tiling Info Labels
// E.2.14.3  Volumetric Tiling Info Objects

// E.2.15  Buffering period SEI message syntax
void PCCBitstreamReader::bufferingPeriod( PCCBitstream&        bitstream,
                                          SEI&                 seiAbstract,
                                          size_t               payloadSize,
                                          bool                 NalHrdBpPresentFlag,
                                          bool                 AclHrdBpPresentFlag,
                                          std::vector<uint8_t> hrdCabCntMinus1 ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEIBufferingPeriod&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  sei.setBpAtlasSequenceParameterSetId( bitstream.readUvlc() );    // ue(v)
  sei.setBpIrapCabParamsPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( sei.getBpIrapCabParamsPresentFlag() ) {
    sei.setBpCabDelayOffset( bitstream.read( fixedBitcount ) );  // u(v)
    sei.setBpDabDelayOffset( bitstream.read( fixedBitcount ) );  // u(v)
  }
  sei.setBpConcatenationFlag( bitstream.read( 1 ) != 0U );                      // u(1)
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
  auto&         sei           = static_cast<SEIAtlasFrameTiming&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  if ( CabDabDelaysPresentFlag ) {
    sei.setAftCabRemovalDelayMinus1( bitstream.read( fixedBitcount ) );  // u(v)
    sei.setAftDabOutputDelay( bitstream.read( fixedBitcount ) );         // u(v)
  }
}

// E.2.17  Presentation inforomation SEI message syntax
void PCCBitstreamReader::presentationInformation( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIPresentationInformation&>( seiAbstract );
  sei.setPiUnitOfLengthFlag( bitstream.read( 1 ) != 0U );        // u(1)
  sei.setPiOrientationPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  sei.setPiPivotPresentFlag( bitstream.read( 1 ) != 0U );        // u(1)
  sei.setPiDimensionPresentFlag( bitstream.read( 1 ) != 0U );    // u(1)
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
  auto& sei = static_cast<SEISmoothingParameters&>( seiAbstract );
  sei.setSpGeometryCancelFlag( bitstream.read( 1 ) != 0U );   // u(1)
  sei.setSpAttributeCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !sei.getSpGeometryCancelFlag() ) {
    sei.setSpGeometrySmoothingEnabledFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( static_cast<int>( sei.getSpGeometrySmoothingEnabledFlag() ) == 1 ) {
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
      size_t index           = sei.getSpAttributeIdx( j );
      size_t dimensionMinus1 = bitstream.read( 8 );  // u(8)
      sei.allocate( index + 1, dimensionMinus1 + 1 );
      sei.setSpDimensionMinus1( index, dimensionMinus1 );
      for ( size_t i = 0; i < sei.getSpDimensionMinus1( index ) + 1; i++ ) {
        sei.setSpAttrSmoothingParamsEnabledFlag( index, i, bitstream.read( 1 ) != 0U );  // u(1)
        if ( sei.getSpAttrSmoothingParamsEnabledFlag( index, i ) ) {
          sei.setSpAttrSmoothingGridSizeMinus2( index, i,
                                                bitstream.read( 8 ) );                   // u(8)
          sei.setSpAttrSmoothingThreshold( index, i, bitstream.read( 8 ) );              // u(8)
          sei.setSpAttrSmoothingLocalEntropyThreshold( index, i, bitstream.read( 8 ) );  // u(3)
          sei.setSpAttrSmoothingThresholdVariation( index, i,
                                                    bitstream.read( 8 ) );  // u(8)
          sei.setSpAttrSmoothingThresholdDifference( index, i,
                                                     bitstream.read( 8 ) );  // u(8)
        }
      }
    }
  }
}

// m52705
void PCCBitstreamReader::sceneObjectInformation( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEISceneObjectInformation&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  sei.setSoiCancelFlag( bool( bitstream.read( 1 ) ) );
  sei.setSoiNumObjectUpdates( bitstream.readUvlc() );
  sei.allocateObjectIdx();
  if ( sei.getSoiNumObjectUpdates() > 0 ) {
    sei.setSoiSimpleObjectsFlag( bool( bitstream.read( 1 ) ) );
    if ( static_cast<int>( sei.getSoiSimpleObjectsFlag() ) == 0 ) {
      sei.setSoiObjectLabelPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setSoiPriorityPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setSoiObjectHiddenPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setSoiObjectDependencyPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setSoiVisibilityConesPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setSoi3dBoundingBoxPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setSoiCollisionShapePresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setSoiPointStylePresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setSoiMaterialIdPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
      sei.setSoiExtensionPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );
    }
    if ( sei.getSoi3dBoundingBoxPresentFlag() ) {
      sei.setSoi3dBoundingBoxScaleLog2( bitstream.read( 5 ) );
      sei.setSoi3dBoundingBoxPrecisionMinus8( bitstream.read( 5 ) );
    }
    sei.setSoiLog2MaxObjectIdxUpdated( bitstream.read( 5 ) );
    if ( sei.getSoiObjectDependencyPresentFlag() ) { sei.setSoiLog2MaxObjectDependencyIdx( bitstream.read( 5 ) ); }
    for ( size_t i = 0; i <= sei.getSoiNumObjectUpdates(); i++ ) {
      assert( sei.getSoiObjectIdx().size() >= sei.getSoiNumObjectUpdates() );
      sei.setSoiObjectIdx( i, bitstream.read( sei.getSoiLog2MaxObjectIdxUpdated() ) );
      size_t k = sei.getSoiObjectIdx( i );
      sei.allocate( k + 1 );
      sei.setSoiObjectCancelFlag( k, bitstream.read( 1 ) != 0U );
      // ObjectTracked[k]=!piObjectCancelFlag[k];
      if ( sei.getSoiObjectCancelFlag( k ) ) {
        if ( sei.getSoiObjectLabelPresentFlag() ) {
          sei.setSoiObjectLabelUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getSoiObjectLabelUpdateFlag( k ) ) { sei.setSoiObjectLabelIdx( k, bitstream.readUvlc() ); }
        }
        if ( sei.getSoiPriorityPresentFlag() ) {
          sei.setSoiPriorityUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getSoiPriorityUpdateFlag( k ) ) { sei.setSoiPriorityValue( k, bitstream.read( 4 ) ); }
        }
        if ( sei.getSoiObjectHiddenPresentFlag() ) { sei.setSoiObjectHiddenFlag( k, bitstream.read( 1 ) != 0U ); }

        if ( sei.getSoiObjectDependencyPresentFlag() ) {
          sei.setSoiObjectDependencyUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getSoiObjectDependencyUpdateFlag( k ) ) {
            sei.setSoiObjectNumDependencies( k, bitstream.read( 4 ) );
            sei.allocateObjectNumDependencies( k, sei.getSoiObjectNumDependencies( k ) );
            for ( size_t j = 0; j < sei.getSoiObjectNumDependencies( k ); j++ ) {
              sei.setSoiObjectDependencyIdx( k, j, bitstream.read( 8 ) );  // size_t bitCount = ceil(log2(
              // sei.getSoiObjectNumDependencies(k)
              // )+0.5);
            }
          }
        }
        if ( sei.getSoiVisibilityConesPresentFlag() ) {
          sei.setSoiVisibilityConesUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getSoiVisibilityConesUpdateFlag( k ) ) {
            sei.setSoiDirectionX( k, bitstream.read( 32 ) );
            sei.setSoiDirectionY( k, bitstream.read( 32 ) );
            sei.setSoiDirectionZ( k, bitstream.read( 32 ) );
            sei.setSoiAngle( k, bitstream.read( 16 ) );
          }
        }  // cones

        if ( sei.getSoi3dBoundingBoxPresentFlag() ) {
          sei.setSoi3dBoundingBoxUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getSoi3dBoundingBoxUpdateFlag( k ) ) {
            sei.setSoi3dBoundingBoxX( k, ( bitstream.read( fixedBitcount ) ) );
            sei.setSoi3dBoundingBoxY( k, ( bitstream.read( fixedBitcount ) ) );
            sei.setSoi3dBoundingBoxZ( k, ( bitstream.read( fixedBitcount ) ) );
            sei.setSoi3dBoundingBoxDeltaX( k, ( bitstream.read( fixedBitcount ) ) );
            sei.setSoi3dBoundingBoxDeltaY( k, ( bitstream.read( fixedBitcount ) ) );
            sei.setSoi3dBoundingBoxDeltaZ( k, ( bitstream.read( fixedBitcount ) ) );
          }
        }  // 3dBB

        if ( sei.getSoiCollisionShapePresentFlag() ) {
          sei.setSoiCollisionShapeUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getSoiCollisionShapeUpdateFlag( k ) ) { sei.setSoiCollisionShapeId( k, bitstream.read( 16 ) ); }
        }  // collision
        if ( sei.getSoiPointStylePresentFlag() ) {
          sei.setSoiPointStyleUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getSoiPointStyleUpdateFlag( k ) ) {
            sei.setSoiPointShapeId( k, bitstream.read( 8 ) );  // only shape??
          }
          sei.setSoiPointSize( k, bitstream.read( 16 ) );
        }  // pointstyle
        if ( sei.getSoiMaterialIdPresentFlag() ) {
          sei.setSoiMaterialIdUpdateFlag( k, bitstream.read( 1 ) != 0U );
          if ( sei.getSoiMaterialIdUpdateFlag( k ) ) { sei.setSoiMaterialId( k, bitstream.read( 16 ) ); }
        }  // materialid
      }    // sei.getSoiObjectCancelFlag(k)
    }      // for(size_t i=0; i<=sei.getSoiNumObjectUpdates(); i++)
  }        // if( sei.getSoiNumObjectUpdates() > 0 )
}

void PCCBitstreamReader::objectLabelInformation( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIObjectLabelInformation&>( seiAbstract );
  sei.setOliCancelFlag( bitstream.read( 1 ) != 0U );
  if ( !sei.getOliCancelFlag() ) {
    sei.setOliLabelLanguagePresentFlag( bitstream.read( 1 ) != 0U );
    if ( sei.getOliLabelLanguagePresentFlag() ) {
      while ( !bitstream.byteAligned() ) { bitstream.read( 1 ); }
      sei.setOliLabelLanguage( bitstream.readString() );
    }
    sei.setOliNumLabelUpdates( bitstream.readUvlc() );
    sei.allocate();
    for ( size_t i = 0; i < sei.getOliNumLabelUpdates(); i++ ) {
      sei.setOliLabelIdx( i, bitstream.readUvlc() );
      sei.setOliLabelCancelFlag( bitstream.read( 1 ) != 0U );
      if ( !sei.getOliLabelCancelFlag() ) {
        while ( !bitstream.byteAligned() ) { bitstream.read( 1 ); }
        sei.setOliLabel( sei.getOliLabelIdx( i ), bitstream.readString() );
      }
    }
  }

};  // Object label information
void PCCBitstreamReader::patchInformation( PCCBitstream& bitstream, SEI& seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIPatchInformation&>( seiAbstract );
  sei.setPiCancelFlag( bitstream.read( 1 ) != 0U );
  sei.setPiNumTileUpdates( bitstream.readUvlc() );
  if ( sei.getPiNumTileUpdates() > 0 ) {
    sei.setPiLog2MaxObjectIdxTracked( bitstream.read( 5 ) );
    sei.setPiLog2MaxPatchIdxUpdated( bitstream.read( 4 ) );
  }
  for ( size_t i = 0; i < sei.getPiNumTileUpdates(); i++ ) {
    sei.setPiTileAddress( i, bitstream.readUvlc() );
    size_t j = sei.getPiTileAddress( i );
    sei.setPiTileCancelFlag( j, bitstream.read( 1 ) != 0U );
    sei.setPiNumPatchUpdates( j, bitstream.readUvlc() );
    for ( size_t k = 0; k < sei.getPiNumPatchUpdates( j ); k++ ) {
      sei.setPiPatchIdx( j, k, bitstream.read( sei.getPiLog2MaxPatchIdxUpdated() ) );
      auto p = sei.getPiPatchIdx( j, k );
      sei.setPiPatchCancelFlag( j, p, bitstream.read( 1 ) != 0U );
      if ( !sei.getPiPatchCancelFlag( j, p ) ) {
        sei.setPiPatchNumberOfObjectsMinus1( j, p, bitstream.readUvlc() );
        for ( size_t n = 0; n < sei.getPiPatchNumberOfObjectsMinus1( j, p ) + 1; n++ ) {
          sei.setPiPatchObjectIdx(
              j, p, n,
              bitstream.read( sei.getPiLog2MaxObjectIdxTracked() ) );  //?pi_log2_max_object_idx_updated?
        }
      }
    }
  }
};  // patch information
void PCCBitstreamReader::volumetricRectangleInformation( PCCBitstream& bitstream,
                                                         SEI&          seiAbstract,
                                                         size_t        payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEIVolumetricRectangleInformation&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  sei.setVriCancelFlag( bitstream.read( 1 ) != 0U );
  sei.setVriNumRectanglesUpdates( bitstream.readUvlc() );
  if ( sei.getVriNumRectanglesUpdates() > 0 ) {
    sei.setVriLog2MaxObjectIdxTracked( bitstream.read( 5 ) );
    sei.setVriLog2MaxRectangleIdxUpdated( bitstream.read( 4 ) );
  }
  for ( size_t k = 0; k < sei.getVriNumRectanglesUpdates(); k++ ) {
    sei.setVriRectangleIdx( k, bitstream.read( sei.getVriLog2MaxRectangleIdxUpdated() ) );
    auto p = sei.getVriRectangleIdx( k );
    sei.setVriRectangleCancelFlag( p, bitstream.read( 1 ) != 0U );
    if ( !sei.getVriRectangleCancelFlag( p ) ) {
      sei.allocate( p + 1 );
      sei.setVriBoundingBoxUpdateFlag( p, bitstream.read( 1 ) != 0U );
      if ( sei.getVriBoundingBoxUpdateFlag( p ) ) {
        sei.setVriBoundingBoxTop( p, ( bitstream.read( fixedBitcount ) ) );
        sei.setVriBoundingBoxLeft( p, ( bitstream.read( fixedBitcount ) ) );
        sei.setVriBoundingBoxWidth( p, ( bitstream.read( fixedBitcount ) ) );
        sei.setVriBoundingBoxHeight( p, ( bitstream.read( fixedBitcount ) ) );
      }
      sei.setVriRectangleNumberOfObjectsMinus1( p, bitstream.readUvlc() );
      sei.allocateRectangleObjectIdx( p, sei.getVriRectangleNumberOfObjectsMinus1( p ) + 1 );
      for ( size_t n = 0; n < sei.getVriRectangleNumberOfObjectsMinus1( p ) + 1; n++ ) {
        sei.setVriRectangleObjectIdx( p, n, bitstream.read( sei.getVriLog2MaxObjectIdxTracked() ) );
      }
    }
  }
};  // volumetric rectangle information
////////

// F.2  VUI syntax
// F.2.1  VUI parameters syntax
void PCCBitstreamReader::vuiParameters( PCCBitstream& bitstream, VUIParameters& vp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  vp.setVuiWorldCoordinatesInfoPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( vp.getVuiWorldCoordinatesInfoPresentFlag() ) {
    vp.setVuiUnitInMetresFlag( bitstream.read( 1 ) != 0U );  // u(1)
    vp.setVuiNumUnitsInBlock( bitstream.read( 32 ) );        // u(32)
    vp.setVuiBlockScale( bitstream.read( 32 ) );             // u(32)
  }
  vp.setVuiDefaultDispalyBoxSize( bitstream.read( 1 ) != 0U );  // u(1)
  if ( vp.getVuiDefaultDispalyBoxSize() ) {
    vp.setVuiDefDispBoxLeftOffset( bitstream.readUvlc() );    // ue(v)
    vp.setVuiDefDispBoxRightOffset( bitstream.readUvlc() );   // ue(v)
    vp.setVuiDefDispBoxTopOffset( bitstream.readUvlc() );     // ue(v)
    vp.setVuiDefDispBoxBottomOffset( bitstream.readUvlc() );  // ue(v)
    vp.setVuiDefDispBoxFrontOffset( bitstream.readUvlc() );   // ue(v)
    vp.setVuiDefDispBoxBackOffset( bitstream.readUvlc() );    // ue(v)
  }
  vp.setVuiTimingInfoPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( vp.getVuiTimingInfoPresentFlag() ) {
    vp.setVuiNumUnitsInTick( bitstream.read( 32 ) );                    // u(32)
    vp.setVuiTimeScale( bitstream.read( 32 ) );                         // u(32)
    vp.setVuiPocProportionalToTimingFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( vp.getVuiPocProportionalToTimingFlag() ) {
      vp.setVuiNumTicksPocDiffOneMinus1( bitstream.readUvlc() );  // ue(v)
    }
    vp.setVuiHrdParametersPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( vp.getVuiHrdParametersPresentFlag() ) { hrdParameters( bitstream, vp.getHrdParameters() ); }
  }
  vp.setVuiBitstreamRestrictionFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( vp.getVuiBitstreamRestrictionFlag() ) {
    vp.setVuiTilesRestrictedFlag( bitstream.read( 1 ) != 0U );                    // u(1)
    vp.setVuiConsistentTilesForVideoComponentsFlag( bitstream.read( 1 ) != 0U );  // u(1)
    vp.setVuiMaxNumTilePerAtlas( bitstream.readUvlc() );                          // ue(v)
  }
}

// F.2.2  HRD parameters syntax
void PCCBitstreamReader::hrdParameters( PCCBitstream& bitstream, HrdParameters& hp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  hp.setHrdNalParametersPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  hp.setHrdAclParametersPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( hp.getHrdNalParametersPresentFlag() || hp.getHrdAclParametersPresentFlag() ) {
    hp.setHrdBitRateScale( bitstream.read( 4 ) );                        // u(4)
    hp.setHrdCabSizeScale( bitstream.read( 4 ) );                        // u(4)
    hp.setHrdInitialCabRemovalDelayLengthMinus1( bitstream.read( 5 ) );  // u(5)
    hp.setHrdAuCabRemovalDelayLengthMinus1( bitstream.read( 5 ) );       // u(5)
    hp.setHrdDabOutputDelayLengthMinus1( bitstream.read( 5 ) );          // u(5)
  }
  for ( size_t i = 0; i <= hp.getMaxNumSubLayersMinus1(); i++ ) {
    hp.setHrdFixedAtlasRateGeneralFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
    if ( !hp.getHrdFixedAtlasRateGeneralFlag( i ) ) {
      hp.setHrdFixedAtlasRateWithinCasFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
    }
    if ( hp.getHrdFixedAtlasRateWithinCasFlag( i ) ) {
      hp.setHrdElementalDurationInTcMinus1( i, bitstream.read( 1 ) );  // ue(v)
    } else {
      hp.setHrdLowDelayFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
    }
    if ( !hp.getHrdLowDelayFlag( i ) ) {
      hp.setHrdCabCntMinus1( i, bitstream.read( 1 ) );  // ue(v)
    }
    if ( hp.getHrdNalParametersPresentFlag() ) {
      hrdSubLayerParameters( bitstream, hp.getHrdSubLayerParameters( 0, i ),
                             static_cast<size_t>( hp.getHrdCabCntMinus1( i ) ) );
    }
    if ( hp.getHrdAclParametersPresentFlag() ) {
      hrdSubLayerParameters( bitstream, hp.getHrdSubLayerParameters( 1, i ),
                             static_cast<size_t>( hp.getHrdCabCntMinus1( i ) ) );
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
    hlsp.setHrdCbrFlag( i, bitstream.read( 1 ) != 0U );        // u(1)
  }
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