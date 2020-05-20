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
#include "PCCBitstream.h"
#include "PCCHighLevelSyntax.h"

#include "PCCBitstreamWriter.h"

using namespace std;
using namespace pcc;

PCCBitstreamWriter::PCCBitstreamWriter()  = default;
PCCBitstreamWriter::~PCCBitstreamWriter() = default;

int32_t PCCBitstreamWriter::write( SampleStreamNalUnit& ssnu, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "PCCBitstreamXXcoder: SampleStream Nal Unit start \n" );
  ssnu.setUnitSizePrecisionBytesMinus1( 3 );
  sampleStreamNalHeader( bitstream, ssnu );
  // sampleStreamNalUnit( bitstream, ssnu, ssnu.getNalUnit() );
  //  for(auto& nalUnit : ssnu.getNalUnit() ){
  //  sampleStreamNalUnit( bitstream, ssnu, ssnu.getNalUnit() );
  //  }
  TRACE_BITSTREAM( "PCCBitstreamXXcoder: SampleStream Nal Unit start done \n" );
  return 0;
}

size_t PCCBitstreamWriter::write( SampleStreamVpccUnit& ssvu, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "PCCBitstreamXXcoder: SampleStream Vpcc Unit start \n" );
  size_t headerSize = 0;
  // Calculating the precision of the unit size
  uint32_t maxUnitSize = 0;
  for ( auto& vpccUnit : ssvu.getVpccUnit() ) {
    if ( maxUnitSize < vpccUnit.getVpccUnitSize() ) {
      maxUnitSize = static_cast<uint32_t>( vpccUnit.getVpccUnitSize() );
    }
  }
  TRACE_BITSTREAM("maxUnitSize = %u \n",maxUnitSize);
  uint32_t precision = static_cast<uint32_t>(
      min( max( static_cast<int>( ceil( static_cast<double>( ceilLog2( maxUnitSize ) ) / 8.0 ) ), 1 ), 8 ) - 1 );
  TRACE_BITSTREAM(" => SsvhUnitSizePrecisionBytesMinus1 = %u \n",precision);
  ssvu.setSsvhUnitSizePrecisionBytesMinus1( precision );
  sampleStreamVpccHeader( bitstream, ssvu );
  headerSize += 1;
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 %d <=> %d / 8 - 1\n", precision, ceilLog2( maxUnitSize ) );
  size_t unitCount = 0;
  for ( auto& vpccUnit : ssvu.getVpccUnit() ) {
    sampleStreamVpccUnit( bitstream, ssvu, vpccUnit );
    TRACE_BITSTREAM( "V-PCC Unit Size(unit type:%zu, %zuth/%zu)  = %zu \n", unitCount,
                     (size_t)vpccUnit.getVpccUnitType(), ssvu.getVpccUnitCount(), vpccUnit.getVpccUnitSize() );
    unitCount++;
    headerSize += ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1;
  }
  TRACE_BITSTREAM( "PCCBitstreamXXcoder: SampleStream Vpcc Unit start done \n" );
  return headerSize;
}

// B.2 Sample stream V-PCC unit syntax and semantics
// B.2.1 Sample stream V-PCC header syntax
void PCCBitstreamWriter::sampleStreamVpccHeader( PCCBitstream& bitstream, SampleStreamVpccUnit& ssvu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( ssvu.getSsvhUnitSizePrecisionBytesMinus1(), 3 ); // u(3)
  bitstream.write( 0, 5 );  // u(5)
}

// B.2.2 Sample stream V-PCC unit syntax
void PCCBitstreamWriter::sampleStreamVpccUnit( PCCBitstream& bitstream, SampleStreamVpccUnit& ssvu, VpccUnit& vpccu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( vpccu.getVpccUnitSize(), 8 * ( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) );  // u(v)
  TRACE_BITSTREAM( "vpccUnitType: %hhu VpccUnitSize: %zu\n", (uint8_t)vpccu.getVpccUnitType(),
                   vpccu.getVpccUnitSize() );
  bitstream.copyFrom( vpccu.getVpccUnitDataBitstream(), 0, vpccu.getVpccUnitSize() );
}

int PCCBitstreamWriter::encode( PCCHighLevelSyntax& syntax, SampleStreamVpccUnit& ssvu ) {
  auto& vuhGVD = syntax.getVpccUnitHeaderGVD();
  auto& vuhAD  = syntax.getVpccUnitHeaderAD();
  auto& vuhAVD = syntax.getVpccUnitHeaderAVD();
  auto& vuhOVD = syntax.getVpccUnitHeaderOVD();
  auto& sps    = syntax.getVps();
  vuhGVD.setVpccParameterSetId( sps.getVpccParameterSetId() );
  vuhAD.setVpccParameterSetId( sps.getVpccParameterSetId() );
  vuhAVD.setVpccParameterSetId( sps.getVpccParameterSetId() );
  vuhOVD.setVpccParameterSetId( sps.getVpccParameterSetId() );
  syntax.getBitstreamStat().newGOF();
  // encode the SPS
  PCCBitstream bitstreamVPS;
#ifdef BITSTREAM_TRACE
  bitstreamVPS.setTrace( true );
  bitstreamVPS.setTraceFile( traceFile_ );
  bitstreamVPS.trace( "PCCBitstream::(VPCC_VPS)\n" );
#endif
  vpccUnit( syntax, bitstreamVPS, VPCC_VPS );
  ssvu.addVpccUnit().setVpccUnitDataBitstream( std::move( bitstreamVPS ), VPCC_VPS );
  for ( uint8_t atlasIdx = 0; atlasIdx < sps.getAtlasCountMinus1() + 1; atlasIdx++ ) {
    vuhAD.setAtlasId( atlasIdx );
    vuhOVD.setAtlasId( atlasIdx );
    vuhGVD.setAtlasId( atlasIdx );
    vuhAVD.setAtlasId( atlasIdx );
    // encode the AD
    PCCBitstream bitstreamAD;
#ifdef BITSTREAM_TRACE
    bitstreamAD.setTrace( true );
    bitstreamAD.setTraceFile( traceFile_ );
    bitstreamAD.trace( "PCCBitstream::(VPCC_AD)\n" );
#endif
    vpccUnit( syntax, bitstreamAD, VPCC_AD );
    ssvu.addVpccUnit().setVpccUnitDataBitstream( std::move( bitstreamAD ), VPCC_AD );
    // encode OVD
    PCCBitstream bitstreamOVD;
#ifdef BITSTREAM_TRACE
    bitstreamOVD.setTrace( true );
    bitstreamOVD.setTraceFile( traceFile_ );
    bitstreamOVD.trace( "PCCBitstream::(VPCC_OVD)\n" );
#endif
    vpccUnit( syntax, bitstreamOVD, VPCC_OVD );
    ssvu.addVpccUnit().setVpccUnitDataBitstream( std::move( bitstreamOVD ), VPCC_OVD );
    // encode GVD
    if ( sps.getMapCountMinus1( atlasIdx ) > 0 && sps.getMultipleMapStreamsPresentFlag( atlasIdx ) ) {
      for ( uint32_t mapIdx = 0; mapIdx < sps.getMapCountMinus1( atlasIdx ) + 1; mapIdx++ ) {
        PCCBitstream bitstreamGVD;
#ifdef BITSTREAM_TRACE
        bitstreamGVD.setTrace( true );
        bitstreamGVD.setTraceFile( traceFile_ );
        bitstreamGVD.trace( "PCCBitstream::(VPCC_GVD)\n" );
#endif
        // encode D(mapIdx)
        vuhGVD.setMapIndex( mapIdx );
        vuhGVD.setRawVideoFlag( false );
        vpccUnit( syntax, bitstreamGVD, VPCC_GVD );
        ssvu.addVpccUnit().setVpccUnitDataBitstream( std::move( bitstreamGVD ), VPCC_GVD );
      }
    } else {
      PCCBitstream bitstreamGVD;
#ifdef BITSTREAM_TRACE
      bitstreamGVD.setTrace( true );
      bitstreamGVD.setTraceFile( traceFile_ );
      bitstreamGVD.trace( "PCCBitstream::(VPCC_GVD)\n" );
#endif
      // encode D=D(0)|D(1)|...|D(N)
      vuhGVD.setMapIndex( 0 );
      vuhGVD.setRawVideoFlag( false );
      vpccUnit( syntax, bitstreamGVD, VPCC_GVD );
      ssvu.addVpccUnit().setVpccUnitDataBitstream( std::move( bitstreamGVD ), VPCC_GVD );
    }
    if ( sps.getRawPatchEnabledFlag( atlasIdx ) && sps.getRawSeparateVideoPresentFlag( atlasIdx ) ) {
      PCCBitstream bitstreamGVD;
#ifdef BITSTREAM_TRACE
      bitstreamGVD.setTrace( true );
      bitstreamGVD.setTraceFile( traceFile_ );
      bitstreamGVD.trace( "PCCBitstream::(VPCC_GVD)\n" );
#endif
      // encode RAW
      vuhGVD.setMapIndex( 0 );
      vuhGVD.setRawVideoFlag( true );
      vpccUnit( syntax, bitstreamGVD, VPCC_GVD );
      ssvu.addVpccUnit().setVpccUnitDataBitstream( std::move( bitstreamGVD ), VPCC_GVD );
    }
    for ( int attIdx = 0; attIdx < sps.getAttributeInformation( atlasIdx ).getAttributeCount(); attIdx++ ) {
      vuhAVD.setAttributeIndex( attIdx );
      for ( int attDim = 0;
            attDim < sps.getAttributeInformation( atlasIdx ).getAttributeDimensionPartitionsMinus1( attIdx ) + 1;
            attDim++ ) {
        vuhAVD.setAttributeDimensionIndex( attDim );
        // encode AVD
        if ( sps.getMapCountMinus1( atlasIdx ) > 0 && sps.getMultipleMapStreamsPresentFlag( atlasIdx ) ) {
          for ( uint32_t mapIdx = 0; mapIdx < sps.getMapCountMinus1( atlasIdx ) + 1; mapIdx++ ) {
            PCCBitstream bitstreamAVD;
#ifdef BITSTREAM_TRACE
            bitstreamAVD.setTrace( true );
            bitstreamAVD.setTraceFile( traceFile_ );
            bitstreamAVD.trace( "PCCBitstream::(VPCC_AVD)\n" );
#endif
            // encode D(mapIdx)
            vuhAVD.setAttributeIndex( attIdx );
            vuhAVD.setAttributeDimensionIndex( attDim );
            vuhAVD.setMapIndex( mapIdx );
            vuhAVD.setRawVideoFlag( false );
            vpccUnit( syntax, bitstreamAVD, VPCC_AVD );
            ssvu.addVpccUnit().setVpccUnitDataBitstream( std::move( bitstreamAVD ), VPCC_AVD );
          }
        } else {
          PCCBitstream bitstreamAVD;
#ifdef BITSTREAM_TRACE
          bitstreamAVD.setTrace( true );
          bitstreamAVD.setTraceFile( traceFile_ );
          bitstreamAVD.trace( "PCCBitstream::(VPCC_AVD)\n" );
#endif
          // encode D=D(0)|D(1)|...|D(N)
          vuhAVD.setAttributeIndex( attIdx );
          vuhAVD.setAttributeDimensionIndex( attDim );
          vuhAVD.setMapIndex( 0 );
          vuhAVD.setRawVideoFlag( false );
          vpccUnit( syntax, bitstreamAVD, VPCC_AVD );
          ssvu.addVpccUnit().setVpccUnitDataBitstream( std::move( bitstreamAVD ), VPCC_AVD );
        }
        if ( sps.getRawPatchEnabledFlag( atlasIdx ) && sps.getRawSeparateVideoPresentFlag( atlasIdx ) ) {
          PCCBitstream bitstreamAVD;
#ifdef BITSTREAM_TRACE
          bitstreamAVD.setTrace( true );
          bitstreamAVD.setTraceFile( traceFile_ );
          bitstreamAVD.trace( "PCCBitstream::(VPCC_AVD)\n" );
#endif
          // encode RAW
          vuhAVD.setAttributeIndex( attIdx );
          vuhAVD.setAttributeDimensionIndex( attDim );
          vuhAVD.setMapIndex( 0 );
          vuhAVD.setRawVideoFlag( true );
          vpccUnit( syntax, bitstreamAVD, VPCC_AVD );
          ssvu.addVpccUnit().setVpccUnitDataBitstream( std::move( bitstreamAVD ), VPCC_AVD );
        }
      }
    }
  }
  return 0;
}

void PCCBitstreamWriter::videoSubStream( PCCHighLevelSyntax& syntax,
                                         PCCBitstream&       bitstream,
                                         VPCCUnitType        vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&  sps        = syntax.getVps();
  auto&  vuh        = syntax.getVpccUnitHeader( static_cast<int>( vpccUnitType ) - 1 );
  size_t atlasIndex = vuh.getAtlasId();
  if ( vpccUnitType == VPCC_OVD ) {
    TRACE_BITSTREAM( "OccupancyMap \n" );
    bitstream.write( syntax.getVideoBitstream( VIDEO_OCCUPANCY ) );
    syntax.getBitstreamStat().setVideoBinSize( VIDEO_OCCUPANCY, syntax.getVideoBitstream( VIDEO_OCCUPANCY ).size() );
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( vuh.getRawVideoFlag() ) {
      TRACE_BITSTREAM( "Geometry raw\n" );
      bitstream.write( syntax.getVideoBitstream( VIDEO_GEOMETRY_RAW ) );
      syntax.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY_RAW,
                                                 syntax.getVideoBitstream( VIDEO_GEOMETRY_RAW ).size() );
    } else {
      if ( sps.getMapCountMinus1( atlasIndex ) > 0 && sps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
        auto geometryIndex = static_cast<PCCVideoType>( VIDEO_GEOMETRY_D0 + vuh.getMapIndex() );
        TRACE_BITSTREAM( "Geometry MAP: %d\n", vuh.getMapIndex() );
        bitstream.write( syntax.getVideoBitstream( geometryIndex ) );
        syntax.getBitstreamStat().setVideoBinSize( geometryIndex, syntax.getVideoBitstream( geometryIndex ).size() );
      } else {
        TRACE_BITSTREAM( "Geometry \n" );
        bitstream.write( syntax.getVideoBitstream( VIDEO_GEOMETRY ) );
        syntax.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY, syntax.getVideoBitstream( VIDEO_GEOMETRY ).size() );
      }
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( sps.getAttributeInformation( atlasIndex ).getAttributeCount() > 0 ) {
      if ( vuh.getRawVideoFlag() ) {
        auto textureIndex = static_cast<PCCVideoType>( VIDEO_TEXTURE_RAW + vuh.getAttributeDimensionIndex() );
        TRACE_BITSTREAM( "Texture raw, PARTITION: %d\n", vuh.getAttributeDimensionIndex() );
        bitstream.write( syntax.getVideoBitstream( textureIndex ) );
        syntax.getBitstreamStat().setVideoBinSize( textureIndex, syntax.getVideoBitstream( textureIndex ).size() );
      } else {
        if ( sps.getMapCountMinus1( atlasIndex ) > 0 && sps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
          auto textureIndex = static_cast<PCCVideoType>(
              VIDEO_TEXTURE_T0 + vuh.getMapIndex() * MAX_NUM_ATTR_PARTITIONS + vuh.getAttributeDimensionIndex() );
          TRACE_BITSTREAM( "Texture MAP: %d, PARTITION: %d\n", vuh.getMapIndex(), vuh.getAttributeDimensionIndex() );
          bitstream.write( syntax.getVideoBitstream( textureIndex ) );
          syntax.getBitstreamStat().setVideoBinSize( textureIndex, syntax.getVideoBitstream( textureIndex ).size() );
        } else {
          auto textureIndex = static_cast<PCCVideoType>( VIDEO_TEXTURE + vuh.getAttributeDimensionIndex() );
          TRACE_BITSTREAM( "Texture PARTITION: %d\n", vuh.getAttributeDimensionIndex() );
          bitstream.write( syntax.getVideoBitstream( textureIndex ) );
          syntax.getBitstreamStat().setVideoBinSize( textureIndex, syntax.getVideoBitstream( textureIndex ).size() );
        }
      }
    }
  }
}

// 7.3.2.1 General V-PCC unit syntax
void PCCBitstreamWriter::vpccUnit( PCCHighLevelSyntax& syntax, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto position = static_cast<int32_t>( bitstream.size() );
  vpccUnitHeader( syntax, bitstream, vpccUnitType );
  vpccUnitPayload( syntax, bitstream, vpccUnitType );
  syntax.getBitstreamStat().setVpccUnitSize( vpccUnitType, static_cast<int32_t>( bitstream.size() ) - position );
  TRACE_BITSTREAM( "vpccUnit: vpccUnitType = %d(%s) \n", vpccUnitType, toString( vpccUnitType ).c_str() );
  TRACE_BITSTREAM( "vpccUnit: size [%d ~ %d] \n", position, bitstream.size() );
  TRACE_BITSTREAM( "%s done\n", __func__ );
}

// 7.3.2.2 V-PCC unit header syntax
void PCCBitstreamWriter::vpccUnitHeader( PCCHighLevelSyntax& syntax,
                                         PCCBitstream&       bitstream,
                                         VPCCUnitType        vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write(  vpccUnitType , 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_AD ) {
    auto& vpcc = syntax.getVpccUnitHeader( static_cast<int>( vpccUnitType ) - 1 );
    bitstream.write( vpcc.getVpccParameterSetId() ,  4 ); // u(4)
    bitstream.write( vpcc.getAtlasId() , 6 );  // u(6)
  }
  if ( vpccUnitType == VPCC_AVD ) {
    auto& vpcc = syntax.getVpccUnitHeaderAVD();
    bitstream.write( vpcc.getAttributeIndex(), 7 );  // u(7)
    bitstream.write( vpcc.getAttributeDimensionIndex(), 5 );    // u(5)
    bitstream.write( vpcc.getMapIndex(), 4 );      // u(4)
    bitstream.write( vpcc.getRawVideoFlag(), 1 );  // u(1)
  } else if ( vpccUnitType == VPCC_GVD ) {
    auto& vpcc = syntax.getVpccUnitHeaderGVD();
    bitstream.write( vpcc.getMapIndex(), 4 );      // u(4)
    bitstream.write( vpcc.getRawVideoFlag(), 1 );  // u(1)
    bitstream.write( 0, 12 );                      // u(12)
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_AD ) {
    bitstream.write( 0, 17 );  // u(17)
  } else {
    bitstream.write( 0, 27 );  // u(27)
  }
}

// 7.3.2.3 V-PCC unit payload syntax
void PCCBitstreamWriter::vpccUnitPayload( PCCHighLevelSyntax& syntax,
                                          PCCBitstream&       bitstream,
                                          VPCCUnitType        vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sps = syntax.getVps();
  TRACE_BITSTREAM( "vpccUnitType = %d \n", (int32_t)vpccUnitType );
  if ( vpccUnitType == VPCC_VPS ) {
    vpccParameterSet( sps, syntax, bitstream );
  } else if ( vpccUnitType == VPCC_AD ) {
    atlasSubStream( syntax, bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    videoSubStream( syntax, bitstream, vpccUnitType );
  }
}

// 7.3.3 Byte alignment syntax
void PCCBitstreamWriter::byteAlignment( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( 1, 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.write( 0, 1 );  // f(1): equal to 0
  }
}

// 7.3.4.1 General V-PCC parameter set syntax
void PCCBitstreamWriter::vpccParameterSet( VpccParameterSet&   vps,
                                           PCCHighLevelSyntax& syntax,
                                           PCCBitstream&       bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  profileTierLevel( vps.getProfileTierLevel(), bitstream );
  bitstream.write( vps.getVpccParameterSetId(), 4 );  // u(4)
  bitstream.write( vps.getAtlasCountMinus1(), 6 );    // u(6)
  for ( uint32_t j = 0; j < vps.getAtlasCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "Atlas = %zu \n", j );
    bitstream.write( vps.getFrameWidth( j ), 16 );   // u(16)
    bitstream.write( vps.getFrameHeight( j ), 16 );  // u(16)
    bitstream.write( vps.getMapCountMinus1( j ), 4 );                         // u(4)
    TRACE_BITSTREAM( " MapCountMinus1 = %zu \n", vps.getMapCountMinus1( j ) );
    if ( vps.getMapCountMinus1( j ) > 0 ) {
      bitstream.write( vps.getMultipleMapStreamsPresentFlag( j ), 1 );  // u(1)
      TRACE_BITSTREAM( "MultipleMapStreamsPresentFlag = %zu \n", vps.getMultipleMapStreamsPresentFlag( j ) );
    }
    for ( size_t i = 1; i <= vps.getMapCountMinus1( j ); i++ ) {
      if ( vps.getMultipleMapStreamsPresentFlag( j ) ) {
        bitstream.write( vps.getMapAbsoluteCodingEnableFlag( j, i ), 1 );  // u(1)
      }
      TRACE_BITSTREAM( " AbsoluteCoding Map%zu = %zu \n", i, vps.getMapAbsoluteCodingEnableFlag( j, i ) );
      if ( static_cast<int>( vps.getMapAbsoluteCodingEnableFlag( j, i ) ) == 0 ) {
        bitstream.writeUvlc( vps.getMapPredictorIndexDiff( j, i ) );
        TRACE_BITSTREAM( " PredictorIndex L%zu = %zu \n", i, vps.getMapPredictorIndexDiff( j, i ) );
      }
    }
    bitstream.write( vps.getRawPatchEnabledFlag( j ), 1 );  // u(1)
    TRACE_BITSTREAM( " RawPatchEnabledFlag = %zu \n", vps.getRawPatchEnabledFlag( j ) );
    if ( vps.getRawPatchEnabledFlag( j ) ) {
      bitstream.write( vps.getRawSeparateVideoPresentFlag( j ), 1 );  // u(1)
      TRACE_BITSTREAM( " RawSeparateVideoPresentFlag = %zu \n", vps.getRawSeparateVideoPresentFlag( j ) );
    }
#ifdef BITSTREAM_TRACE
    for ( size_t i = 0; i < vps.getMapCountMinus1( j ) + 1; i++ ) {
      TRACE_BITSTREAM( " AbsoluteCoding L%zu = %zu \n", i, vps.getMapAbsoluteCodingEnableFlag( j, i ) );
    }
#endif
    occupancyInformation( vps.getOccupancyInformation( j ), bitstream );
    geometryInformation( vps.getGeometryInformation( j ), vps, bitstream );
    attributeInformation( vps.getAttributeInformation( j ), vps, bitstream );
  }
  bitstream.write( vps.getExtensionPresentFlag(), 1 );  // u(1)
  if ( vps.getExtensionPresentFlag() ) {
    bitstream.writeUvlc( vps.getExtensionLength() );  // ue(v)
    for ( size_t i = 0; i < vps.getExtensionLength(); i++ ) {
      bitstream.write( vps.getExtensionDataByte( i ), 8 );  // u(8)
    }
  }
  byteAlignment( bitstream );
}

// 7.3.4.2 Profile, tier, and level syntax
void PCCBitstreamWriter::profileTierLevel( ProfileTierLevel& ptl, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( ptl.getTierFlag(), 1 );  // u(1)
  bitstream.write( ptl.getProfileCodecGroupIdc(),   7 );  // u(7)
  bitstream.write( ptl.getProfilePccToolsetIdc(), 8 );  // u(8)
  bitstream.write( ptl.getProfileReconctructionIdc(), 8 ); // u(8)
  bitstream.write( 0, 32 );                 // u(32)
  bitstream.write( ptl.getLevelIdc(), 8 );  // u(8)
}

// 7.3.4.3 Occupancy parameter set syntax
void PCCBitstreamWriter::occupancyInformation( OccupancyInformation& oi, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( oi.getOccupancyCodecId(), 8 );  // u(8)
  bitstream.write( oi.getLossyOccupancyMapCompressionThreshold(), 8 );  // u(8)
  bitstream.write( oi.getOccupancyNominal2DBitdepthMinus1(), 5 );  // u(5)
  bitstream.write( oi.getOccupancyMSBAlignFlag() , 1 );  // u(1)
  TRACE_BITSTREAM( "  OccupancyLossyThreshold = %d  \n", oi.getLossyOccupancyMapCompressionThreshold() );
}

// 7.3.4.4 Geometry parameter set syntax
void PCCBitstreamWriter::geometryInformation( GeometryInformation& gi,
                                              VpccParameterSet&    vps,
                                              PCCBitstream&        bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  bitstream.write( gi.getGeometryCodecId(), 8 );  // u(8)
  bitstream.write( gi.getGeometryNominal2dBitdepthMinus1(), 5 );  // u(5)
  bitstream.write( gi.getGeometryMSBAlignFlag(), 1 );  // u(1)
  bitstream.write( gi.getGeometry3dCoordinatesBitdepthMinus1(), 5 );  // u(5)
  if ( vps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
    bitstream.write( gi.getRawGeometryCodecId(), 8 );  // u(8)
  }
}

// 7.3.4.5 Attribute information
void PCCBitstreamWriter::attributeInformation( AttributeInformation& ai,
                                               VpccParameterSet&     sps,
                                               PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  bitstream.write( ai.getAttributeCount(), 7 );  // u(7)
  TRACE_BITSTREAM( "AttributeCount = %u  \n", ai.getAttributeCount() );
  for ( uint32_t i = 0; i < ai.getAttributeCount(); i++ ) {
    bitstream.write( ai.getAttributeTypeId( i ), 4 );  // u(4)
    bitstream.write( ai.getAttributeCodecId( i ), 8 );  // u(8)
    if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
      bitstream.write( ai.getRawAttributeCodecId( i ), 8 );  // u(8)
    }
    if ( sps.getMapCountMinus1( atlasIndex ) > 0 ) {
      bitstream.write( ai.getAttributeMapAbsoluteCodingPersistanceFlag( i ), 1 );  // u(1)
    }
    bitstream.write( ai.getAttributeDimensionMinus1( i ), 6 );  // u(6)
    if ( ai.getAttributeDimensionMinus1( i ) > 0 ) {
      bitstream.write( ai.getAttributeDimensionPartitionsMinus1( i ), 6 );  // u(6)
      int32_t remainingDimensions = ai.getAttributeDimensionMinus1( i );
      int32_t k                   = ai.getAttributeDimensionPartitionsMinus1( i );
      for ( int32_t j = 0; j < k; j++ ) {
        if ( k - j != remainingDimensions ) {
          bitstream.writeUvlc( static_cast<uint32_t>( ai.getAttributePartitionChannelsMinus1( i, j ) ) );  // ue(v)
        }
        remainingDimensions -= ai.getAttributePartitionChannelsMinus1( i, j ) + 1;
      }
    }
    bitstream.write( ai.getAttributeNominal2dBitdepthMinus1( i ), 5 );  // u(5)
    bitstream.write( ai.getAttributeMSBAlignFlag( i ), 1 );  // u(1)
  }
}

// 7.3.5 NAL unit syntax
// 7.3.5.1 General NAL unit syntax
void PCCBitstreamWriter::nalUnit( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  nalUnitHeader( bitstream, nalUnit );
  for ( size_t i = 0; i < nalUnit.getNalUnitSize() - 2; i++ ) {
    bitstream.write( nalUnit.getNalUnitData( i ), 8 );  // b(8)
  }
}

// 7.3.5.2 NAL unit header syntax
void PCCBitstreamWriter::nalUnitHeader( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( 0, 1 );                              // f(1)
  bitstream.write( nalUnit.getNalUnitType(), 6 );       // u(6)
  bitstream.write( nalUnit.getLayerId(), 6 );           // u(6)
  bitstream.write( nalUnit.getTemporalyIdPlus1(), 3 );  // u(3)
  TRACE_BITSTREAM( " NalUnitSize      = %zu \n", nalUnit.getNalUnitSize() );
  TRACE_BITSTREAM( " NalUnitType      = %hhu \n", nalUnit.getNalUnitType() );
  TRACE_BITSTREAM( " LayerId          = %hhu \n", nalUnit.getLayerId() );
  TRACE_BITSTREAM( " TemporalyIdPlus1 = %hhu \n", nalUnit.getTemporalyIdPlus1() );
}

// 7.3.6.1 Atlas sequence parameter set Rbsp
void PCCBitstreamWriter::atlasSequenceParameterSetRbsp( AtlasSequenceParameterSetRbsp& asps,
                                                        PCCHighLevelSyntax&            syntax,
                                                        PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( static_cast<uint32_t>( asps.getAltasSequenceParameterSetId() ) );  // ue(v)
  bitstream.write( asps.getFrameWidth(), 16 );                   // u(16)
  bitstream.write( asps.getFrameHeight(), 16 );                  // u(16)
  bitstream.write( asps.getLog2PatchPackingBlockSize(), 3 );   // u(3)
  bitstream.writeUvlc( static_cast<uint32_t>( asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() ) );  // ue(v)
  bitstream.writeUvlc( static_cast<uint32_t>( asps.getMaxDecAtlasFrameBufferingMinus1() ) );     // ue(v)
  bitstream.write( asps.getLongTermRefAtlasFramesFlag(), 1 );  // u(1)
  bitstream.writeUvlc( static_cast<uint32_t>( asps.getNumRefAtlasFrameListsInAsps() ) );  // ue(v)
  for ( size_t i = 0; i < asps.getNumRefAtlasFrameListsInAsps(); i++ ) {
    refListStruct( asps.getRefListStruct( i ), asps, bitstream );
  }
  bitstream.write( asps.getUseEightOrientationsFlag(), 1 );  // u(1)
  bitstream.write( asps.getExtendedProjectionEnabledFlag(), 1 );  // u(1)
  if (asps.getExtendedProjectionEnabledFlag())
    bitstream.writeUvlc(asps.getMaxNumberProjectionsMinus1());
  bitstream.write( asps.getNormalAxisLimitsQuantizationEnabledFlag(), 1 );  // u(1)
  bitstream.write( asps.getNormalAxisMaxDeltaValueEnabledFlag(), 1 );  // u(1)
  bitstream.write( asps.getRemoveDuplicatePointEnabledFlag(), 1 );  // u(1)
  bitstream.write( asps.getPixelDeinterleavingFlag(), 1 );  // u(1)
  bitstream.write( asps.getPatchPrecedenceOrderFlag(), 1 );  // u(1)
  bitstream.write( asps.getPatchSizeQuantizerPresentFlag(), 1 );  // u(1)
  bitstream.write( asps.getEnhancedOccupancyMapForDepthFlag(), 1 );  // u(1)
  bitstream.write( asps.getPointLocalReconstructionEnabledFlag(), 1 ); // u(1)
  bitstream.write( asps.getMapCountMinus1(), 4 );  // u(4)
  if ( asps.getEnhancedOccupancyMapForDepthFlag() && asps.getMapCountMinus1() == 0 ) {
    bitstream.write( asps.getEnhancedOccupancyMapFixBitCountMinus1(), 4 );  // u(4)
  }
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionInformation( asps, syntax, bitstream );
  }
  if ( asps.getPixelDeinterleavingFlag() || asps.getPointLocalReconstructionEnabledFlag() ) {
    bitstream.write( asps.getSurfaceThicknessMinus1(), 8 );  // u(8)
  }
  bitstream.write( asps.getVuiParametersPresentFlag(), 1 );  // u(1)
  if ( asps.getVuiParametersPresentFlag() ) { vuiParameters( bitstream, asps.getVuiParameters() ); }
  bitstream.write( asps.getExtensionPresentFlag(), 1 );  // u(1)
  if ( asps.getExtensionPresentFlag() ) {
    while ( moreRbspData( bitstream ) ) {
      bitstream.write( asps.getExtensionPresentFlag(), 1 );  // u(1)
    }
  }
  rbspTrailingBits( bitstream );
}

// 7.3.4.6 Point local reconstruction information syntax
void PCCBitstreamWriter::pointLocalReconstructionInformation( AtlasSequenceParameterSetRbsp& asps,
                                                              PCCHighLevelSyntax&            syntax,
                                                              PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "  MapCountMinus1() = %u \n", asps.getMapCountMinus1() );
  for ( size_t j = 0; j < asps.getMapCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "  PLR info map %zu \n", j );
    auto& plri = asps.getPointLocalReconstructionInformation( j );
    bitstream.write( plri.getMapEnabledFlag(), 1 );  // u(1)
    if ( plri.getMapEnabledFlag() ) {
      bitstream.write( plri.getNumberOfModesMinus1(), 4 );  // u(4)
      TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plri.getNumberOfModesMinus1() );
      for ( size_t i = 0; i < plri.getNumberOfModesMinus1(); i++ ) {
        bitstream.write( plri.getInterpolateFlag( i ), 1 );  // u(1)
        bitstream.write( plri.getFillingFlag( i ), 1 );  // u(1)
        bitstream.write( plri.getMinimumDepth( i ), 2 );  // u(2)
        bitstream.write( plri.getNeighbourMinus1( i ), 2 );  // u(2)
        TRACE_BITSTREAM( "  Mode[%zu] = I = %d F = %d D = %u N = %u \n", i, plri.getInterpolateFlag( i ),
                         plri.getFillingFlag( i ), plri.getMinimumDepth( i ), plri.getNeighbourMinus1( i ) );
      }
      bitstream.write( plri.getBlockThresholdPerPatchMinus1(), 6 );  // u(6)
      TRACE_BITSTREAM( "  BlockThresholdPerPatchMinus1 = %u \n", plri.getBlockThresholdPerPatchMinus1() );
    }
  }
}

// 7.2 Specification of syntax functions and descriptors
bool PCCBitstreamWriter::byteAligned( PCCBitstream& bitstream ) { return bitstream.byteAligned(); }
bool PCCBitstreamWriter::moreDataInPayload( PCCBitstream& bitstream ) { return false; }
bool PCCBitstreamWriter::moreRbspData( PCCBitstream& bitstream ) { return false; }
bool PCCBitstreamWriter::moreRbspTrailingData( PCCBitstream& bitstream ) { return false; }
bool PCCBitstreamWriter::moreDataInVpccUnit( PCCBitstream& bitstream ) { return false; }
void PCCBitstreamWriter::rbspTrailingBits( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( 1, 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.write( 0, 1 );  // f(1): equal to 0
  }
}
bool PCCBitstreamWriter::payloadExtensionPresent( PCCBitstream& bitstream ) { return false; }

// 7.3.6.3  Atlas frame parameter set Rbsp syntax
void PCCBitstreamWriter::atlasFrameParameterSetRbsp( AtlasFrameParameterSetRbsp& afps,
                                                     PCCHighLevelSyntax&         syntax,
                                                     PCCBitstream&               bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( afps.getAtlasFrameParameterSetId() );     // ue(v)
  bitstream.writeUvlc( afps.getAtlasSequenceParameterSetId() );  // ue(v)
  auto& asps = syntax.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  atlasFrameTileInformation( afps.getAtlasFrameTileInformation(), asps, bitstream );
  bitstream.write( afps.getAfpsOutputFlagPresentFlag(), 1 );
  bitstream.writeUvlc( afps.getAfpsNumRefIdxDefaultActiveMinus1() );  // ue(v)
  bitstream.writeUvlc( afps.getAfpsAdditionalLtAfocLsbLen() );        // ue(v)
  bitstream.write( afps.getAfpsOverrideEomForDepthFlag(), 1 );        // u(1)
  if ( afps.getAfpsOverrideEomForDepthFlag() ) {
    bitstream.write( afps.getAfpsEomNumberOfPatchBitCountMinus1(), 4 );
    bitstream.write( afps.getAfpsEomMaxBitCountMinus1(), 4 );
  }
  bitstream.write( afps.getAfpsRaw3dPosBitCountExplicitModeFlag(), 1 );
  bitstream.write( afps.getAfpsExtensionPresentFlag(), 1 );
  if ( afps.getAfpsExtensionPresentFlag() != 0u ) {
    while ( moreRbspData( bitstream ) ) { bitstream.write( afps.getAfpsExtensionDataFlag(), 1 ); }
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.4  Atlas frame tile information syntax
void PCCBitstreamWriter::atlasFrameTileInformation( AtlasFrameTileInformation& afti,
                                                    AtlasSequenceParameterSetRbsp& asps,
                                                    PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( afti.getSingleTileInAtlasFrameFlag(), 1 );  // u(1)
  if ( !afti.getSingleTileInAtlasFrameFlag() ) {
    bitstream.write( afti.getUniformTileSpacingFlag(), 1 );  // u(1)
    if ( afti.getUniformTileSpacingFlag() ) {
      bitstream.writeUvlc( afti.getTileColumnWidthMinus1( 0 ) );  //  ue(v)
      bitstream.writeUvlc( afti.getTileRowHeightMinus1( 0 ) );    //  ue(v)
    } else {
      bitstream.writeUvlc( afti.getNumTileColumnsMinus1() );  //  ue(v)
      bitstream.writeUvlc( afti.getNumTileRowsMinus1() );     //  ue(v)
      for ( size_t i = 0; i < afti.getNumTileColumnsMinus1(); i++ ) {
        bitstream.writeUvlc( afti.getTileColumnWidthMinus1( i ) );  //  ue(v)
      }
      for ( size_t i = 0; i < afti.getNumTileRowsMinus1(); i++ ) {
        bitstream.writeUvlc( afti.getTileRowHeightMinus1( i ) );  //  ue(v)
      }
    }
  
  bitstream.write( afti.getSingleTilePerTileGroupFlag(), 1 );  //  u(1)
  if ( afti.getSingleTilePerTileGroupFlag() == 0u ) {
    uint32_t NumTilesInPatchFrame = ( afti.getNumTileColumnsMinus1() + 1 ) * ( afti.getNumTileRowsMinus1() + 1 );
    bitstream.writeUvlc( afti.getNumTileGroupsInAtlasFrameMinus1() );  // ue(v)
    for ( size_t i = 0; i <= afti.getNumTileGroupsInAtlasFrameMinus1(); i++ ) {
      uint8_t bitCount = ceilLog2( NumTilesInPatchFrame );
      if ( i > 0 ) {
        bitstream.write( afti.getTopLeftTileIdx( i ), bitCount );  // u(v) :
      }
      bitCount = ceilLog2( NumTilesInPatchFrame - afti.getTopLeftTileIdx( i ) );
      bitstream.write( afti.getBottomRightTileIdxDelta( i ), bitCount );  // u(v)
    }
  }
  bitstream.write( afti.getSignalledTileGroupIdFlag(), 1 );  // u(1)
  if ( afti.getSignalledTileGroupIdFlag() ) {
    bitstream.writeUvlc( afti.getSignalledTileGroupIdLengthMinus1() );  // ue(v)
    for ( size_t i = 0; i <= afti.getSignalledTileGroupIdLengthMinus1(); i++ ) {
      uint8_t bitCount = afti.getSignalledTileGroupIdLengthMinus1() + 1;
      bitstream.write( afti.getTileGroupId( i ), bitCount );  // u(v)
    }
  }
 } //if ( !afti.getSingleTileInAtlasFrameFlag() )
}

// 7.3.6.5  Supplemental enhancement information Rbsp syntax
void PCCBitstreamWriter::seiRbsp( PCCHighLevelSyntax& syntax,
                                  PCCBitstream&       bitstream,
                                  SEI&                sei,
                                  NalUnitType         nalUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  // do { seiMessage( syntax, bitstream ); } while ( moreRbspData( bitstream )
  // );
  seiMessage( bitstream, syntax, sei, nalUnitType );
}

// 7.3.6.10  Atlas tile group layer Rbsp syntax = patchTileGroupLayerUnit
void PCCBitstreamWriter::atlasTileGroupLayerRbsp( AtlasTileGroupLayerRbsp& atgl,
                                                  PCCHighLevelSyntax&      syntax,
                                                  PCCBitstream&            bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  atlasTileGroupHeader( atgl.getAtlasTileGroupHeader(), syntax, bitstream );
  if ( atgl.getAtlasTileGroupHeader().getAtghType() != SKIP_TILE_GRP ) {
    atlasTileGroupDataUnit( atgl.getAtlasTileGroupDataUnit(), atgl.getAtlasTileGroupHeader(), syntax, bitstream );
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.11  Atlas tile group header syntax
void PCCBitstreamWriter::atlasTileGroupHeader( AtlasTileGroupHeader& atgh,
                                               PCCHighLevelSyntax&   syntax,
                                               PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps   = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps   = syntax.getAtlasSequenceParameterSet( aspsId );
  AtlasFrameTileInformation&     afti   = afps.getAtlasFrameTileInformation();
  bitstream.writeUvlc( atgh.getAtghAtlasFrameParameterSetId() );
  //v9.1
  if(afti.getSignalledTileGroupIdFlag())
    bitstream.write( uint32_t( atgh.getAtghAddress() ), afti.getSignalledTileGroupIdLengthMinus1() + 1 );
  else{
    if(afti.getNumTileGroupsInAtlasFrameMinus1()!=0)
      bitstream.write( uint32_t( atgh.getAtghAddress() ), ceilLog2( afti.getNumTileGroupsInAtlasFrameMinus1()+1) );
  }
  bitstream.writeUvlc( atgh.getAtghType() );
  if ( afps.getAfpsOutputFlagPresentFlag() ) { bitstream.write( atgh.getAtghAtlasOutputFlag(), 1 ); }
  bitstream.write( atgh.getAtghAtlasFrmOrderCntLsb(), asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4 );
  TRACE_BITSTREAM( " AtlasFrameParameterSetId: %zu\n", atgh.getAtghAtlasFrameParameterSetId() );
  TRACE_BITSTREAM( " AtlasSequenceParameterSetId: %zu\n", afps.getAtlasSequenceParameterSetId() );
  TRACE_BITSTREAM( " Address       %zu\n", (size_t)atgh.getAtghAddress() );
  TRACE_BITSTREAM( " Type          %zu\n", (size_t)atgh.getAtghType() );
  TRACE_BITSTREAM( " FrameOrderCnt %zu\n", (size_t)atgh.getAtghAtlasFrmOrderCntLsb() );
  if ( asps.getNumRefAtlasFrameListsInAsps() > 0 ) {
    bitstream.write( atgh.getAtghRefAtlasFrameListSpsFlag(), 1 );
  }
  if ( static_cast<int>( atgh.getAtghRefAtlasFrameListSpsFlag() ) == 0 ) {
    refListStruct( atgh.getRefListStruct(), asps, bitstream );
  } else if ( asps.getNumRefAtlasFrameListsInAsps() > 1 ) {
    size_t bitCount = ceilLog2( asps.getNumRefAtlasFrameListsInAsps() );
    bitstream.write( atgh.getAtghRefAtlasFrameListIdx(), bitCount );
  }
  uint8_t rlsIdx  = atgh.getAtghRefAtlasFrameListIdx();
  auto&   refList = atgh.getAtghRefAtlasFrameListSpsFlag() ? asps.getRefListStruct( rlsIdx ) : atgh.getRefListStruct();
  size_t  numLtrAtlasFrmEntries = 0;
  for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
    if ( !refList.getStRefAtalsFrameFlag( i ) ) { numLtrAtlasFrmEntries++; }
  }
  TRACE_BITSTREAM( " rlsIdx %u numLtrAtlasFrmEntries %zu \n", rlsIdx, (size_t)numLtrAtlasFrmEntries );
  for ( size_t j = 0; j < numLtrAtlasFrmEntries; j++ ) {
    bitstream.write( atgh.getAtghAdditionalAfocLsbPresentFlag( j ), 1 );
    if ( atgh.getAtghAdditionalAfocLsbPresentFlag( j ) ) {
      uint8_t bitCount = afps.getAfpsAdditionalLtAfocLsbLen();
      bitstream.write( atgh.getAtghAdditionalAfocLsbVal( j ), bitCount );
    }
  }
  if ( atgh.getAtghType() != SKIP_TILE_GRP ) {
    if ( asps.getNormalAxisLimitsQuantizationEnabledFlag() ) {
      bitstream.write( atgh.getAtghPosMinZQuantizer(), 5 );
      TRACE_BITSTREAM( " AtghPosMinZQuantizer = %zu \n", atgh.getAtghPosMinZQuantizer() );
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        bitstream.write( atgh.getAtghPosDeltaMaxZQuantizer(), 5 );
      }
    }
    if ( asps.getPatchSizeQuantizerPresentFlag() ) {
      bitstream.write( atgh.getAtghPatchSizeXinfoQuantizer(), 3 );
      bitstream.write( atgh.getAtghPatchSizeYinfoQuantizer(), 3 );
    }
    auto& gi = syntax.getVps().getGeometryInformation( 0 );
    if ( afps.getAfpsRaw3dPosBitCountExplicitModeFlag() ) {
      size_t bitCount = ceilLog2( gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
      bitstream.write(  atgh.getAtghRaw3dPosAxisBitCountMinus1(), bitCount );
    }
    if ( atgh.getAtghType() == P_TILE_GRP && refList.getNumRefEntries() > 1 ) {
      bitstream.write( atgh.getAtghNumRefIdxActiveOverrideFlag(), 1 );
      if ( atgh.getAtghNumRefIdxActiveOverrideFlag() ) {
        bitstream.writeUvlc( atgh.getAtghNumRefIdxActiveMinus1() );
      }
    }
    TRACE_BITSTREAM( "==> AtghRaw3dPosAxisBitCountMinus1 = %zu \n", atgh.getAtghRaw3dPosAxisBitCountMinus1() );
    TRACE_BITSTREAM( "==> AtghNumRefIdxActiveOverrideFlag = %zu \n", atgh.getAtghNumRefIdxActiveOverrideFlag() );
    TRACE_BITSTREAM( "==> AtghNumRefIdxActiveMinus1       = %zu \n", atgh.getAtghNumRefIdxActiveMinus1() );
  }
  byteAlignment( bitstream );
}

// 7.3.6.12  Reference list structure syntax
void PCCBitstreamWriter::refListStruct( RefListStruct&                 rls,
                                        AtlasSequenceParameterSetRbsp& asps,
                                        PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( rls.getNumRefEntries() );  // ue(v)
  TRACE_BITSTREAM( "NumRefEntries = %zu  \n", rls.getNumRefEntries() );
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( asps.getLongTermRefAtlasFramesFlag() ) {
      bitstream.write( rls.getStRefAtalsFrameFlag( i ), 1 );
    }  // u(1)

    if ( rls.getStRefAtalsFrameFlag( i ) ) {
      bitstream.writeUvlc( rls.getAbsDeltaAfocSt( i ) );  // ue(v)
      if ( rls.getAbsDeltaAfocSt( i ) > 0 ) {
        bitstream.write( rls.getStrpfEntrySignFlag( i ), 1 );  // u(1)
      }
    } else {
      uint8_t bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
      bitstream.write( rls.getAfocLsbLt( i ), bitCount );  // u(v) 
      // note: psps_log2_max_patch_frame_order_cnt_lsb_minus4 + 4 bits
    }
    TRACE_BITSTREAM( "stRefAtalsFrameFlag = %zu  \n", rls.getStRefAtalsFrameFlag( i ) );
    TRACE_BITSTREAM( "absDeltaAfocSt      = %zu  \n", rls.getAbsDeltaAfocSt( i ) );
    TRACE_BITSTREAM( "strpfEntrySignFlag  = %zu  \n", rls.getStrpfEntrySignFlag( i ) );
  }
  TRACE_BITSTREAM( "%s done \n", __func__ );
}

// 7.3.7.1  General atlas tile group data unit syntax =patchTileGroupDataUnit
void PCCBitstreamWriter::atlasTileGroupDataUnit( AtlasTileGroupDataUnit& atgdu,
                                                 AtlasTileGroupHeader&   atgh,
                                                 PCCHighLevelSyntax&     syntax,
                                                 PCCBitstream&           bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "atgh.getAtghType()        = %zu \n", atgh.getAtghType() );
  for ( size_t puCount = 0; puCount < atgdu.getPatchCount(); puCount++ ) {
    TRACE_BITSTREAM( "patch %zu : \n", puCount );
    bitstream.writeUvlc( uint32_t( atgdu.getPatchMode( puCount ) ) );
    auto& pid = atgdu.getPatchInformationData( puCount );
    TRACE_BITSTREAM( "patchMode = %zu \n", atgdu.getPatchMode( puCount ) );
    pid.setFrameIndex( atgdu.getFrameIndex() );
    pid.setPatchIndex( puCount );
    patchInformationData( pid, atgdu.getPatchMode( puCount ), atgh, syntax, bitstream );
  }
  TRACE_BITSTREAM( "atgdu.getPatchCount() including END = %zu \n", atgdu.getPatchCount() );
  byteAlignment( bitstream );
}

// 7.3.7.2  Patch information data syntax
void PCCBitstreamWriter::patchInformationData( PatchInformationData& pid,
                                               size_t                patchMode,
                                               AtlasTileGroupHeader& atgh,
                                               PCCHighLevelSyntax&   syntax,
                                               PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s: AtghType = %zu patchMode = %zu \n", __func__, atgh.getAtghType(), patchMode );
  if ( atgh.getAtghType() == static_cast<uint8_t>( P_TILE_GRP ) && patchMode == PATCH_MODE_P_SKIP ) {
    // skip mode: currently not supported but added it for convenience. Could
    // easily be removed
  } else if ( atgh.getAtghType() == static_cast<uint8_t>( P_TILE_GRP ) && patchMode == PATCH_MODE_P_MERGE ) {
    // skip mode: currently not supported but added it for convenience. Could
    // easily be removed
    auto& mpdu = pid.getMergePatchDataUnit();
    mpdu.setFrameIndex( pid.getFrameIndex() );
    mpdu.setPatchIndex( pid.getPatchIndex() );
    mergePatchDataUnit( mpdu, atgh, syntax, bitstream );
  } else if ( atgh.getAtghType() == static_cast<uint8_t>( P_TILE_GRP ) && patchMode == PATCH_MODE_P_INTER ) {
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

// 7.3.7.3  Patch data unit syntax : AtlasTileGroupHeader instead of
// PatchTileGroupHeader
void PCCBitstreamWriter::patchDataUnit( PatchDataUnit&        pdu,
                                        AtlasTileGroupHeader& atgh,
                                        PCCHighLevelSyntax&   syntax,
                                        PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps   = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps   = syntax.getAtlasSequenceParameterSet( aspsId );
  bitstream.writeUvlc( pdu.getPdu2dPosX() );  // ue(v)
  bitstream.writeUvlc( pdu.getPdu2dPosY() );  // ue(v)
  TRACE_BITSTREAM( " 2dPosXY: %zu,%zu\n", pdu.getPdu2dPosX(), pdu.getPdu2dPosX() );
  bitstream.writeUvlc( pdu.getPdu2dSizeXMinus1() );
  bitstream.writeUvlc( pdu.getPdu2dSizeYMinus1() );
  TRACE_BITSTREAM( " 2dSizeXY: %d,%d\n", int32_t( pdu.getPdu2dSizeXMinus1() + 1 ),
                   int32_t( pdu.getPdu2dSizeYMinus1() + 1 ) );
#if EXPAND_RANGE_CONDITIONAL
  uint8_t bitCount3DPos = syntax.getVps().getGeometryInformation(0).getGeometry3dCoordinatesBitdepthMinus1() + 1 + asps.getExtendedProjectionEnabledFlag();
  bitstream.write( pdu.getPdu3dPosX(), bitCount3DPos );  // u(v)
  bitstream.write( pdu.getPdu3dPosY(), bitCount3DPos );  // u(v)
#else
  uint8_t bitCount3DPos = syntax.getVps().getGeometryInformation(0).getGeometry3dCoordinatesBitdepthMinus1() + 2;
  bitstream.write( pdu.getPdu3dPosX(), bitCount3DPos );  // u(v)
  bitstream.write( pdu.getPdu3dPosY(), bitCount3DPos );  // u(v)
#endif
  TRACE_BITSTREAM( " 3dPosXY: %zu,%zu\n", pdu.getPdu3dPosX(), pdu.getPdu3dPosY() );
#if EXPAND_RANGE_CONDITIONAL
  const uint8_t bitCountForMinDepth =
      syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
      atgh.getAtghPosMinZQuantizer() + 1 + asps.getExtendedProjectionEnabledFlag();
#else
  const uint8_t bitCountForMinDepth =
      syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
      atgh.getAtghPosMinZQuantizer() + 2;
#endif

  bitstream.write( pdu.getPdu3dPosMinZ(), bitCountForMinDepth );  // u(v)
#if EXPAND_RANGE_CONDITIONAL
  TRACE_BITSTREAM( " Pdu3dPosMinZ: %zu ( bitCountForMinDepth = %u = %u - %u + %u ) \n", pdu.getPdu3dPosMinZ(),
                   bitCountForMinDepth,
                   syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1(),
                   atgh.getAtghPosMinZQuantizer(), 1 + asps.getExtendedProjectionEnabledFlag());
#else
  TRACE_BITSTREAM( " Pdu3dPosMinZ: %zu ( bitCountForMinDepth = %u = %u - %u + %u ) \n", pdu.getPdu3dPosMinZ(),
                   bitCountForMinDepth,
                   syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1(),
                   atgh.getAtghPosMinZQuantizer(), 2);
#endif
  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
#if EXPAND_RANGE_CONDITIONAL
    uint8_t bitCountForMaxDepth = syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
                                  atgh.getAtghPosDeltaMaxZQuantizer() + 1 + asps.getExtendedProjectionEnabledFlag();
#else
    uint8_t bitCountForMaxDepth = syntax.getVps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() -
                                  atgh.getAtghPosDeltaMaxZQuantizer() + 2;
#endif
    bitstream.write( pdu.getPdu3dPosDeltaMaxZ(), bitCountForMaxDepth );
    TRACE_BITSTREAM( " Pdu3dPosDeltaMaxZ: %zu ( bitCountForMaxDepth = %u) \n", pdu.getPdu3dPosDeltaMaxZ(),
                     bitCountForMaxDepth );
  }
  uint8_t bitCountForProjectionId = ceilLog2(asps.getMaxNumberProjectionsMinus1() + 1);
  bitstream.write( pdu.getPduProjectionId(),
                   bitCountForProjectionId );  // u(v)
  TRACE_BITSTREAM( "PduProjectionId = %zu (ExtendedProjectionEnabledFlag = %d ) \n", pdu.getPduProjectionId(),
                   asps.getExtendedProjectionEnabledFlag() );
  bitstream.write( pdu.getPduOrientationIndex(), asps.getUseEightOrientationsFlag() ? 3 : 1 );  // u(3 or 1)
  if ( afps.getLodModeEnableFlag() ) {
    bitstream.write( pdu.getLodEnableFlag(), 1 );  // u(1)
    if ( pdu.getLodEnableFlag() ) {
      bitstream.writeUvlc( pdu.getLodScaleXminus1() );
      bitstream.writeUvlc( pdu.getLodScaleY() );
    }
  }
  TRACE_BITSTREAM( "PointLocalReconstructionEnabledFlag = %d \n", asps.getPointLocalReconstructionEnabledFlag() );
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plrd = pdu.getPointLocalReconstructionData();
    TRACE_BITSTREAM( "Size = %ld %ld\n", pdu.getPdu2dSizeXMinus1(), pdu.getPdu2dSizeYMinus1() );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
  }
  TRACE_BITSTREAM(
      "Frame %zu, Patch(%zu) => 2Dpos = %4zu %4zu 2Dsize = %4ld %4ld 3Dpos = "
      "%ld %ld %ld DeltaMaxZ = %ld Projection = "
      "%zu "
      "Orientation = %zu lod= (%d) %d %d\n ",
      pdu.getFrameIndex(), pdu.getPatchIndex(), pdu.getPdu2dPosX(), pdu.getPdu2dPosY(), pdu.getPdu2dSizeXMinus1() + 1,
      pdu.getPdu2dSizeYMinus1() + 1, pdu.getPdu3dPosX(), pdu.getPdu3dPosY(), pdu.getPdu3dPosMinZ(),
      pdu.getPdu3dPosDeltaMaxZ(), pdu.getPduProjectionId(), pdu.getPduOrientationIndex(), pdu.getLodEnableFlag(),
      pdu.getLodEnableFlag() ? pdu.getLodScaleXminus1() : (uint8_t)0,
      pdu.getLodEnableFlag() ? pdu.getLodScaleY() : (uint8_t)0 );
}

// 7.3.7.4  Skip patch data unit syntax
void PCCBitstreamWriter::skipPatchDataUnit( SkipPatchDataUnit&    spdu,
                                            AtlasTileGroupHeader& atgh,
                                            PCCHighLevelSyntax&   syntax,
                                            PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}

// 7.3.7.5  Merge patch data unit syntax
void PCCBitstreamWriter::mergePatchDataUnit( MergePatchDataUnit&   mpdu,
                                             AtlasTileGroupHeader& atgh,
                                             PCCHighLevelSyntax&   syntax,
                                             PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId          = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps            = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId          = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps            = syntax.getAtlasSequenceParameterSet( aspsId );
  bool                           overridePlrFlag = false;
  size_t                         numRefIdxActive = syntax.getNumRefIdxActive( atgh );
  if ( numRefIdxActive > 1 ) { bitstream.writeUvlc( mpdu.getMpduRefIndex() ); }
  bitstream.write( mpdu.getMpduOverride2dParamsFlag(), 1 );
  if ( mpdu.getMpduOverride2dParamsFlag() ) {
    bitstream.writeSvlc(  mpdu.getMpdu2dPosX() );        // se(v)
    bitstream.writeSvlc(  mpdu.getMpdu2dPosY() );        // se(v)
    bitstream.writeSvlc(  mpdu.getMpdu2dDeltaSizeX() );  // se(v)
    bitstream.writeSvlc(  mpdu.getMpdu2dDeltaSizeY() );  // se(v)
    if ( asps.getPointLocalReconstructionEnabledFlag() ) { overridePlrFlag = true; }
  } else {
    bitstream.write( mpdu.getMpduOverride3dParamsFlag(), 1 );
    if ( mpdu.getMpduOverride3dParamsFlag() ) {
      bitstream.writeSvlc(  mpdu.getMpdu3dPosX() );
      bitstream.writeSvlc(  mpdu.getMpdu3dPosY() );
      bitstream.writeSvlc(  mpdu.getMpdu3dPosMinZ() );
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        bitstream.writeSvlc( mpdu.getMpdu3dPosDeltaMaxZ() );
      }
      if ( asps.getPointLocalReconstructionEnabledFlag() ) {
        bitstream.write( mpdu.getMpduOverridePlrFlag(), 1 );
      }
    }
  }
  if ( overridePlrFlag && asps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plrd = mpdu.getPointLocalReconstructionData();
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n",
                     plrd.getBlockToPatchMapWidth() - mpdu.getMpdu2dDeltaSizeX(),
                     plrd.getBlockToPatchMapHeight() - mpdu.getMpdu2dDeltaSizeY(), mpdu.getMpdu2dDeltaSizeX(),
                     mpdu.getMpdu2dDeltaSizeY(), plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight() );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
  }
  TRACE_BITSTREAM(
      "%zu frame %zu MergePatch => Ref Frame Idx = %d ShiftUV = %ld %ld "
      "DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      mpdu.getMpduRefIndex(), mpdu.getMpdu2dPosX(), mpdu.getMpdu2dPosY(), mpdu.getMpdu2dDeltaSizeX(),
      mpdu.getMpdu2dDeltaSizeY(), mpdu.getMpdu3dPosX(), mpdu.getMpdu3dPosY(), mpdu.getMpdu3dPosMinZ(),
      mpdu.getMpdu3dPosDeltaMaxZ() );
}

// 7.3.7.6  Inter patch data unit syntax
void PCCBitstreamWriter::interPatchDataUnit( InterPatchDataUnit&   ipdu,
                                             AtlasTileGroupHeader& atgh,
                                             PCCHighLevelSyntax&   syntax,
                                             PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId          = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps            = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId          = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp& asps            = syntax.getAtlasSequenceParameterSet( aspsId );
  size_t                         numRefIdxActive = syntax.getNumRefIdxActive( atgh );
  if ( numRefIdxActive > 1 ) { bitstream.writeUvlc( int32_t( ipdu.getIpduRefIndex() ) ); }
  bitstream.writeSvlc( int32_t( ipdu.getIpduRefPatchIndex() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dPosX() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dPosY() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dDeltaSizeX() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dDeltaSizeY() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosX() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosY() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosMinZ() ) );
  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosDeltaMaxZ() ) );
  }
  TRACE_BITSTREAM(
      "%zu frame: numRefIdxActive = %zu reference = frame%zu patch%d 2Dpos = "
      "%ld %ld 2DdeltaSize = %ld %ld 3Dpos = %ld "
      "%ld %ld DeltaMaxZ = %ld\n",
      ipdu.getFrameIndex(), numRefIdxActive, ipdu.getIpduRefIndex(), ipdu.getIpduRefPatchIndex(), ipdu.getIpdu2dPosX(),
      ipdu.getIpdu2dPosY(), ipdu.getIpdu2dDeltaSizeX(), ipdu.getIpdu2dDeltaSizeY(), ipdu.getIpdu3dPosX(),
      ipdu.getIpdu3dPosY(), ipdu.getIpdu3dPosMinZ(), ipdu.getIpdu3dPosDeltaMaxZ() );

  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    auto& plrd = ipdu.getPointLocalReconstructionData();
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n",
                     plrd.getBlockToPatchMapWidth() - ipdu.getIpdu2dDeltaSizeX(),
                     plrd.getBlockToPatchMapHeight() - ipdu.getIpdu2dDeltaSizeY(), ipdu.getIpdu2dDeltaSizeX(),
                     ipdu.getIpdu2dDeltaSizeY(), plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight() );
    pointLocalReconstructionData( plrd, syntax, asps, bitstream );
  }
}

// 7.3.7.7 raw patch data unit syntax
void PCCBitstreamWriter::rawPatchDataUnit( RawPatchDataUnit&     ppdu,
                                           AtlasTileGroupHeader& atgh,
                                           PCCHighLevelSyntax&   syntax,
                                           PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&  vps        = syntax.getVps();
  size_t atlasIndex = 0;
  if ( vps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
    bitstream.write( ppdu.getRpduPatchInRawVideoFlag(), 1 ); // u(1)
  }                                                               
  bitstream.writeUvlc( ppdu.getRpdu2dPosX() );         // ue(v)
  bitstream.writeUvlc( ppdu.getRpdu2dPosY() );         // ue(v)
  bitstream.writeUvlc( ppdu.getRpdu2dSizeXMinus1() );  // se(v)
  bitstream.writeUvlc(  ppdu.getRpdu2dSizeYMinus1() );  // se(v)
  TRACE_BITSTREAM( " AtghRaw3dPosAxisBitCountMinus1 = %zu \n", atgh.getAtghRaw3dPosAxisBitCountMinus1() );
  bitstream.write( ppdu.getRpdu3dPosX(), atgh.getAtghRaw3dPosAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.write( ppdu.getRpdu3dPosY(), atgh.getAtghRaw3dPosAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.write( ppdu.getRpdu3dPosZ(), atgh.getAtghRaw3dPosAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.writeUvlc( ppdu.getRpduRawPointsMinus1() );
  TRACE_BITSTREAM(
      "Raw Patch => UV %4zu %4zu  S=%4ld %4ld  UVD1=%4ld %4ld %4ld "
      "NumPcmPoints=%zu PatchInRawVideoFlag=%d \n",
      ppdu.getRpdu2dPosX(), ppdu.getRpdu2dPosY(), ppdu.getRpdu2dSizeXMinus1() + 1, ppdu.getRpdu2dSizeYMinus1() + 1,
      ppdu.getRpdu3dPosX(), ppdu.getRpdu3dPosY(), ppdu.getRpdu3dPosZ(), ppdu.getRpduRawPointsMinus1() + 1,
      ppdu.getRpduPatchInRawVideoFlag() );
}

// 7.3.7.8 EOM patch data unit syntax
void PCCBitstreamWriter::eomPatchDataUnit( EOMPatchDataUnit&     epdu,
                                           AtlasTileGroupHeader& atgh,
                                           PCCHighLevelSyntax&   syntax,
                                           PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( epdu.getEpdu2dPosX() );         // ue(v)
  bitstream.writeUvlc( epdu.getEpdu2dPosY() );         // ue(v)
  bitstream.writeUvlc( epdu.getEpdu2dSizeXMinus1() );  // se(v)
  bitstream.writeUvlc( epdu.getEpdu2dSizeYMinus1() );  // se(v)
  bitstream.write( epdu.getEpduAssociatedPatchesCountMinus1(), 8 );  // max255
  for ( size_t cnt = 0; cnt < epdu.getEpduAssociatedPatchesCountMinus1() + 1; cnt++ ) {
    bitstream.write( epdu.getEpduAssociatedPatches()[cnt], 8 );
    bitstream.writeUvlc( epdu.getEpduEomPointsPerPatch()[cnt]  );
  }
#ifdef BITSTREAM_TRACE
  TRACE_BITSTREAM( "EOM Patch => UV %4zu %4zu  S=%4ld %4ld  N=%4ld\n", epdu.getEpdu2dPosX(), epdu.getEpdu2dPosY(),
                   epdu.getEpdu2dSizeXMinus1() + 1, epdu.getEpdu2dSizeYMinus1() + 1,
                   epdu.getEpduAssociatedPatchesCountMinus1() + 1 );
  for ( size_t cnt = 0; cnt < epdu.getEpduAssociatedPatchesCountMinus1() + 1; cnt++ ) {
    TRACE_BITSTREAM( "%4ld, %4ld\n", epdu.getEpduAssociatedPatches()[cnt], epdu.getEpduEomPointsPerPatch()[cnt] );
  }
#endif
}

void PCCBitstreamWriter::atlasSubStream( PCCHighLevelSyntax& syntax, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  SampleStreamNalUnit   ssnu;
  uint32_t              maxUnitSize = 0;
  PCCBitstream          tempBitStream;
  std::vector<uint32_t> aspsSizeList;
  std::vector<uint32_t> afpsSizeList;
  std::vector<uint32_t> atglSizeList;
  std::vector<uint32_t> seiPrefixSizeList;
  std::vector<uint32_t> seiSuffixSizeList;
  aspsSizeList.resize( syntax.getAtlasSequenceParameterSetList().size() );
  afpsSizeList.resize( syntax.getAtlasFrameParameterSetList().size() );
  atglSizeList.resize( syntax.getGofSize() );
  seiPrefixSizeList.resize( syntax.getSeiPrefix().size() );
  seiSuffixSizeList.resize( syntax.getSeiSuffix().size() );
  uint32_t lastSize      = 0;
  size_t   nalHeaderSize = 2;
  for ( size_t aspsIdx = 0; aspsIdx < syntax.getAtlasSequenceParameterSetList().size(); aspsIdx++ ) {
    atlasSequenceParameterSetRbsp( syntax.getAtlasSequenceParameterSet( aspsIdx ), syntax, tempBitStream );
    aspsSizeList[aspsIdx] = tempBitStream.size() - lastSize + nalHeaderSize;
    lastSize              = tempBitStream.size();
    if ( maxUnitSize < aspsSizeList[aspsIdx] ) { maxUnitSize = aspsSizeList[aspsIdx]; }
  }
  for ( size_t afpsIdx = 0; afpsIdx < syntax.getAtlasFrameParameterSetList().size(); afpsIdx++ ) {
    atlasFrameParameterSetRbsp( syntax.getAtlasFrameParameterSet( afpsIdx ), syntax, tempBitStream );
    afpsSizeList[afpsIdx] = tempBitStream.size() - lastSize + nalHeaderSize;
    lastSize              = tempBitStream.size();
    if ( maxUnitSize < afpsSizeList[afpsIdx] ) { maxUnitSize = afpsSizeList[afpsIdx]; }
  }
  for ( size_t i = 0; i < syntax.getSeiPrefix().size(); i++ ) {
    seiRbsp( syntax, tempBitStream, syntax.getSeiPrefix( i ), NAL_PREFIX_SEI );
    seiPrefixSizeList[i] = tempBitStream.size() - lastSize + nalHeaderSize;
    lastSize             = tempBitStream.size();
    if ( maxUnitSize < seiPrefixSizeList[i] ) { maxUnitSize = seiPrefixSizeList[i]; }
  }
  for ( size_t atglIdx = 0; atglIdx < atglSizeList.size(); atglIdx++ ) {
    int numTilesPerFrame = 1;  // (afps.getAtlasFrameTileInformation().getNumTileRowsMinus1() + 1)
                               // *
    // (afps.getAtlasFrameTileInformation().getNumTileColumnsMinus1() + 1);
    for ( size_t tileGroupId = 0; tileGroupId < numTilesPerFrame; tileGroupId++ ) {  // TODO: make this more than just
                                                                                     // one tile groups per frame
      atlasTileGroupLayerRbsp( syntax.getAtlasTileGroupLayer( atglIdx ), syntax, tempBitStream );
      atglSizeList[atglIdx] = tempBitStream.size() - lastSize + nalHeaderSize;
      lastSize              = tempBitStream.size();
      if ( maxUnitSize < atglSizeList[atglIdx] ) { maxUnitSize = atglSizeList[atglIdx]; }
    }
  }
  for ( size_t i = 0; i < syntax.getSeiSuffix().size(); i++ ) {
    seiRbsp( syntax, tempBitStream, syntax.getSeiSuffix( i ), NAL_SUFFIX_SEI );
    seiSuffixSizeList[i] = tempBitStream.size() - lastSize + nalHeaderSize;
    lastSize             = tempBitStream.size();
    if ( maxUnitSize < seiSuffixSizeList[i] ) { maxUnitSize = seiSuffixSizeList[i]; }
  }
  // calculation of the max unit size done
    TRACE_BITSTREAM( "maxUnitSize                                                        = %u \n",maxUnitSize);
    TRACE_BITSTREAM( "ceilLog2( maxUnitSize + 1 )                                        = %d \n",ceilLog2( maxUnitSize + 1 ));
    TRACE_BITSTREAM( "ceil( static_cast<double>( ceilLog2( maxUnitSize + 1 ) ) / 8.0 ) ) = %f \n",
      ceil( static_cast<double>( ceilLog2( maxUnitSize + 1 ) ) / 8.0 ) );

  uint32_t precision = static_cast<uint32_t>(
      min( max( static_cast<int>( ceil( static_cast<double>( ceilLog2( maxUnitSize + 1 ) ) / 8.0 ) ), 1 ), 8 ) - 1 );
    TRACE_BITSTREAM( "precision                                                      = %u \n",precision);
  ssnu.setUnitSizePrecisionBytesMinus1( precision );
  sampleStreamNalHeader( bitstream, ssnu );
  for ( size_t aspsCount = 0; aspsCount < syntax.getAtlasSequenceParameterSetList().size(); aspsCount++ ) {
    NalUnit nu( NAL_ASPS, 0, 1 );
    nu.setNalUnitSize( aspsSizeList[aspsCount] );
    sampleStreamNalUnit( syntax, bitstream, ssnu, nu, aspsCount );
    TRACE_BITSTREAM(
        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
        "written: %llu\n",
        (int)nu.getNalUnitType(), toString( nu.getNalUnitType() ).c_str(),
        ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ), nu.getNalUnitSize(), bitstream.size() );
  }
  for ( size_t afpsCount = 0; afpsCount < syntax.getAtlasFrameParameterSetList().size(); afpsCount++ ) {
    NalUnit nu( NAL_AFPS, 0, 1 );
    nu.setNalUnitSize( afpsSizeList[afpsCount] );
    sampleStreamNalUnit( syntax, bitstream, ssnu, nu, afpsCount );
    TRACE_BITSTREAM(
        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
        "written: %llu\n",
        (int)nu.getNalUnitType(), toString( nu.getNalUnitType() ).c_str(),
        ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ), nu.getNalUnitSize(), bitstream.size() );
  }
  // NAL_PREFIX_SEI
  for ( size_t i = 0; i < syntax.getSeiPrefix().size(); i++ ) {
    NalUnit nu( NAL_PREFIX_SEI, 0, 1 );
    nu.setNalUnitSize( seiPrefixSizeList[i] );
    sampleStreamNalUnit( syntax, bitstream, ssnu, nu, i );
    TRACE_BITSTREAM(
        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
        "written: %llu\n",
        (int)nu.getNalUnitType(), toString( nu.getNalUnitType() ).c_str(),
        ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ), nu.getNalUnitSize(), bitstream.size() );
  }
  // NAL_TRAIL, NAL_TSA, NAL_STSA, NAL_RADL, NAL_RASL,NAL_SKIP
  for ( size_t frameIdx = 0; frameIdx < atglSizeList.size(); frameIdx++ ) {
    NalUnit nu( NAL_TSA, 0, 1 );
    nu.setNalUnitSize( atglSizeList[frameIdx] );  //+headsize
    auto& atgl = syntax.getAtlasTileGroupLayer( frameIdx );
    atgl.getAtlasTileGroupDataUnit().setFrameIndex( frameIdx );
    TRACE_BITSTREAM( " ATGL: frame %zu\n", frameIdx );
    sampleStreamNalUnit( syntax, bitstream, ssnu, nu, frameIdx );
    TRACE_BITSTREAM(
        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
        "written: %llu\n",
        (int)nu.getNalUnitType(), toString( nu.getNalUnitType() ).c_str(),
        ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ), nu.getNalUnitSize(), bitstream.size() );
  }
  // NAL_SUFFIX_SEI
  for ( size_t i = 0; i < syntax.getSeiSuffix().size(); i++ ) {
    NalUnit nu( NAL_SUFFIX_SEI, 0, 1 );
    nu.setNalUnitSize( seiSuffixSizeList[i] );
    sampleStreamNalUnit( syntax, bitstream, ssnu, nu, i );
    TRACE_BITSTREAM(
        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
        "written: %llu\n",
        (int)nu.getNalUnitType(), toString( nu.getNalUnitType() ).c_str(),
        ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ), nu.getNalUnitSize(), bitstream.size() );
  }
}

// 7.3.6.6 Point local reconstruction data syntax
void PCCBitstreamWriter::pointLocalReconstructionData( PointLocalReconstructionData&  plrd,
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
    bitstream.write( plrd.getLevelFlag(), 1 );  // u(1)
  }
  TRACE_BITSTREAM( "  LevelFlag = %u \n", plrd.getLevelFlag() );
  if ( plrd.getLevelFlag() ) {
    bitstream.write( plrd.getPresentFlag(), 1 );                                             // u(1)
    if ( plrd.getPresentFlag() ) { bitstream.write(  plrd.getModeMinus1(), bitCountMode ); }  // u(v)
    TRACE_BITSTREAM( "  ModePatch: Present = %d ModeMinus1 = %d \n", plrd.getPresentFlag(),
                     plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1 );
  } else {
    for ( size_t i = 0; i < blockCount; i++ ) {
      bitstream.write(  plrd.getBlockPresentFlag( i ), 1 );  // u(1)
      if ( plrd.getBlockPresentFlag( i ) ) {
        bitstream.write(  plrd.getBlockModeMinus1( i ),
                         bitCountMode );  // u(v)
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
void PCCBitstreamWriter::seiMessage( PCCBitstream&       bitstream,
                                     PCCHighLevelSyntax& syntax,
                                     SEI&                sei,
                                     NalUnitType         nalUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto payloadType = static_cast<int32_t>( sei.getPayloadType() );
  for ( ; payloadType >= 0xff; payloadType -= 0xff ) { bitstream.write( 0xff, 8 ); }
  bitstream.write( payloadType, 8 );

  // calculating the size of the sei message before writing it into the
  // bitstream, is there a better way to do this???
  PCCBitstream tempbitstream;
  seiPayload( tempbitstream, syntax, sei, nalUnitType );
  sei.setPayloadSize( tempbitstream.size() );

  auto payloadSize = static_cast<int32_t>( sei.getPayloadSize() );
  for ( ; payloadSize >= 0xff; payloadSize -= 0xff ) { bitstream.write( 0xff, 8 ); }
  bitstream.write( payloadSize, 8 );
  seiPayload( bitstream, syntax, sei, nalUnitType );
}

// C.2 Sample stream NAL unit syntax and semantics
// C.2.1 Sample stream NAL header syntax
void PCCBitstreamWriter::sampleStreamNalHeader( PCCBitstream& bitstream, SampleStreamNalUnit& ssnu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( ssnu.getUnitSizePrecisionBytesMinus1(), 3 );  // u(3)
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 = %lu \n",ssnu.getUnitSizePrecisionBytesMinus1() );
  bitstream.write( 0, 5 );                                       // u(5)
}

// C.2.2 Sample stream NAL unit syntax
void PCCBitstreamWriter::sampleStreamNalUnit( PCCHighLevelSyntax&  syntax,
                                              PCCBitstream&        bitstream,
                                              SampleStreamNalUnit& ssnu,
                                              NalUnit&             nalu,
                                              size_t               index ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 = %lu \n",ssnu.getUnitSizePrecisionBytesMinus1() );
  bitstream.write( nalu.getNalUnitSize(),
                   8 * ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ) );  // u(v)
  nalUnitHeader( bitstream, nalu );
  switch ( nalu.getNalUnitType() ) {
    case NAL_ASPS:
      atlasSequenceParameterSetRbsp( syntax.getAtlasSequenceParameterSet( index ), syntax, bitstream );
      break;
    case NAL_AFPS: atlasFrameParameterSetRbsp( syntax.getAtlasFrameParameterSet( index ), syntax, bitstream ); break;
    case NAL_TRAIL:
    case NAL_TSA:
    case NAL_STSA:
    case NAL_RADL:
    case NAL_RASL:
    case NAL_SKIP: atlasTileGroupLayerRbsp( syntax.getAtlasTileGroupLayer( index ), syntax, bitstream ); break;
    case NAL_SUFFIX_SEI: seiRbsp( syntax, bitstream, syntax.getSeiSuffix( index ), nalu.getNalUnitType() );
    case NAL_PREFIX_SEI: seiRbsp( syntax, bitstream, syntax.getSeiPrefix( index ), nalu.getNalUnitType() ); break;
    default:
      fprintf( stderr, "sampleStreamNalUnit type = %d not supported\n", static_cast<int32_t>( nalu.getNalUnitType() ) );
  }
}

// E.2  SEI payload syntax
// E.2.1  General SEI message syntax
void PCCBitstreamWriter::seiPayload( PCCBitstream&       bitstream,
                                     PCCHighLevelSyntax& syntax,
                                     SEI&                sei,
                                     NalUnitType         nalUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto payloadType = sei.getPayloadType();
  if ( nalUnitType == NAL_PREFIX_SEI ) {
    if ( payloadType == BUFFERING_PERIOD ) {
      bool                 NalHrdBpPresentFlag = false;
      bool                 AclHrdBpPresentFlag = false;
      std::vector<uint8_t> hrdCabCntMinus1;
      bufferingPeriod( bitstream, sei, NalHrdBpPresentFlag, AclHrdBpPresentFlag, hrdCabCntMinus1 );
    } else if ( payloadType == ATLAS_FRAME_TIMING ) {
      atlasFrameTiming( bitstream, sei, false );
    } else if ( payloadType == FILLER_PAYLOAD ) {
      fillerPayload( bitstream, sei );
    } else if ( payloadType == USER_DATAREGISTERED_ITUTT35 ) {
      userDataRegisteredItuTT35( bitstream, sei );
    } else if ( payloadType == USER_DATA_UNREGISTERED ) {
      userDataUnregistered( bitstream, sei );
    } else if ( payloadType == RECOVERY_POINT ) {
      recoveryPoint( bitstream, sei );
    } else if ( payloadType == NO_DISPLAY ) {
      noDisplay( bitstream, sei );
    } else if ( payloadType == TIME_CODE ) {
      // timeCode( bitstream, sei );
    } else if ( payloadType == REGIONAL_NESTING ) {
      // regionalNesting( bitstream, sei );
    } else if ( payloadType == SEI_MANIFEST ) {
      seiManifest( bitstream, sei );
    } else if ( payloadType == SEI_PREFIX_INDICATION ) {
      seiPrefixIndication( bitstream, sei );
    } else if ( payloadType == GEOMETRY_TRANSFORMATION_PARAMS ) {
      geometryTransformationParams( bitstream, sei );
    } else if ( payloadType == ATTRIBUTE_TRANSFORMATION_PARAMS ) {
      attributeTransformationParams( bitstream, sei );
    } else if ( payloadType == ACTIVE_SUBSTREAMS ) {
      activeSubstreams( bitstream, sei );
    } else if ( payloadType == COMPONENT_CODEC_MAPPING ) {
      componentCodecMapping( bitstream, sei );
    } else if ( payloadType == PRESENTATION_INFORMATION ) {
      presentationInformation( bitstream, sei );
    } else if ( payloadType == SMOOTHING_PARAMETERS ) {
      smoothingParameters( bitstream, sei );
    } else if ( payloadType == SCENE_OBJECT_INFORMATION ) {
      sceneObjectInformation( bitstream, sei );
    } else if ( payloadType == OBJECT_LABEL_INFORMATION ) {
      objectLabelInformation( bitstream, sei );
    } else if ( payloadType == PATCH_INFORMATION ) {
      patchInformation( bitstream, sei );
    } else if ( payloadType == VOLUMETRIC_RECTANGLE_INFORMATION ) {
      volumetricRectangleInformation( bitstream, sei );
    } else {
      reservedSeiMessage( bitstream, sei );
    }
  } else { /* psdUnitType  ==  NAL_SUFFIX_SEI */
    SEI& sei = syntax.addSeiSuffix( payloadType );
    if ( payloadType == 2 ) {
      fillerPayload( bitstream, sei );
    } else if ( payloadType == 3 ) {
      userDataRegisteredItuTT35( bitstream, sei );
    } else if ( payloadType == 4 ) {
      userDataUnregistered( bitstream, sei );
    } else {
      reservedSeiMessage( bitstream, sei );
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
void PCCBitstreamWriter::fillerPayload( PCCBitstream& bitstream, SEI& sei ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto payloadSize = sei.getPayloadSize();
  for ( size_t k = 0; k < payloadSize; k++ ) {
    bitstream.write( 0xFF, 8 );  // f(8) equal to 0xFF
  }
}

// E.2.3  User data registered by Recommendation ITU-T T.35 SEI message syntax
void PCCBitstreamWriter::userDataRegisteredItuTT35( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei         = static_cast<SEIUserDataRegisteredItuTT35&>( seiAbstract );
  auto  payloadSize = sei.getPayloadSize();
  bitstream.write( sei.getItuTT35CountryCode(), 8 );  // b(8)
  payloadSize--;
  if ( sei.getItuTT35CountryCode() == 0xFF ) {
    bitstream.write( sei.getItuTT35CountryCodeExtensionByte(), 8 );  // b(8)
    payloadSize--;
  }
  auto& payload = sei.getItuTT35PayloadByte();
  for ( auto& element : payload ) {
    bitstream.write( element, 8 );  // b(8)
  }
}

// E.2.4  User data unregistered SEI message syntax
void PCCBitstreamWriter::userDataUnregistered( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei         = static_cast<SEIUserDataUnregistered&>( seiAbstract );
  auto  payloadSize = sei.getPayloadSize();
  for ( size_t i = 0; i < 16; i++ ) {
    bitstream.write( sei.getUuidIsoIec11578( i ), 8 );  // u(128) <=> 16 * u(8)
  }
  payloadSize -= 16;
  for ( size_t i = 0; i < payloadSize; i++ ) {
    bitstream.write( sei.getUserDataPayloadByte( i ), 8 );  // b(8)
  }
}

// E.2.5  Recovery point SEI message syntax
void PCCBitstreamWriter::recoveryPoint( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIRecoveryPoint&>( seiAbstract );
  bitstream.writeSvlc( sei.getRecoveryAfocCnt() );  // se(v)
  bitstream.write( sei.getExactMatchFlag(), 1 );    // u(1)
  bitstream.write( sei.getBrokenLinkFlag(), 1 );    // u(1)
}

// E.2.6  No display SEI message syntax
void PCCBitstreamWriter::noDisplay( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}

// E.2.7  Reserved SEI message syntax
void PCCBitstreamWriter::reservedSeiMessage( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei         = static_cast<SEIReservedSeiMessage&>( seiAbstract );
  auto  payloadSize = sei.getPayloadSize();
  for ( size_t i = 0; i < payloadSize; i++ ) {
    bitstream.write( sei.getReservedSeiMessagePayloadByte( i ), 8 );  // b(8)
  }
}

// E.2.8  SEI manifest SEI message syntax
void PCCBitstreamWriter::seiManifest( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIManifest&>( seiAbstract );
  bitstream.write( sei.getManifestNumSeiMsgTypes(), 16 );  // u(16)
  for ( size_t i = 0; i < sei.getManifestNumSeiMsgTypes(); i++ ) {
    bitstream.write( sei.getManifestSeiPayloadType( i ), 16 );  // u(16)
    bitstream.write( sei.getManifestSeiDescription( i ), 8 );   // u(8)
  }
}

// E.2.9  SEI prefix indication SEI message syntax
void PCCBitstreamWriter::seiPrefixIndication( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIPrefixIndication&>( seiAbstract );
  bitstream.write( sei.getPrefixSeiPayloadType(), 16 );          // u(16)
  bitstream.write( sei.getNumSeiPrefixIndicationsMinus1(), 8 );  // u(8)
  for ( size_t i = 0; i <= sei.getNumSeiPrefixIndicationsMinus1(); i++ ) {
    bitstream.write( sei.getNumBitsInPrefixIndicationMinus1( i ), 16 );  // u(16)
    for ( size_t j = 0; j <= sei.getNumBitsInPrefixIndicationMinus1( i ); j++ ) {
      bitstream.write( sei.getSeiPrefixDataBit( i, j ),
                       1 );  // u(1)
    }
    while ( !bitstream.byteAligned() ) {
      bitstream.write( 1, 1 );  // f(1): equal to 1
    }
  }
}

// E.2.10  Geometry transformation parameters SEI message syntax
void PCCBitstreamWriter::geometryTransformationParams( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIGeometryTransformationParams&>( seiAbstract );
  bitstream.write( sei.getGtpCancelFlag(), 1 );  // u(1)
  if ( !sei.getGtpCancelFlag() ) {
    bitstream.write( sei.getGtpScaleEnabledFlag(),
                     1 );  // u(1)
    bitstream.write( sei.getGtpOffsetEnabledFlag(),
                     1 );  // u(1)
    bitstream.write( sei.getGtpRotationEnabledFlag(),
                     1 );                                   // u(1)
    bitstream.write( sei.getGtpNumCameraInfoMinus1(), 3 );  // u(3)
    if ( sei.getGtpScaleEnabledFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.write( sei.getGtpGeometryScaleOnAxis( d ), 32 );  // u(32)
      }
    }
    if ( sei.getGtpOffsetEnabledFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.writeS( sei.getGtpGeometryOffsetOnAxis( d ), 32 );  // i(32)
      }
    }
    if ( sei.getGtpRotationEnabledFlag() ) {
      bitstream.writeS( sei.getGtpRotationQx(), 16 );  // i(16)
      bitstream.writeS( sei.getGtpRotationQy(), 16 );  // i(16)
      bitstream.writeS( sei.getGtpRotationQz(), 16 );  // i(16)
    }
    if ( sei.getGtpNumCameraInfoMinus1() != 0 ) {
      for ( uint8_t camId = 0; camId < sei.getGtpNumCameraInfoMinus1(); camId++ ) {
        for ( size_t d = 0; d < 3; d++ ) {
          bitstream.writeS( sei.getGtpCameraOffsetOnAxis( camId, d ),
                            16 );  // i(16)
        }
        for ( size_t d = 0; d < 3; d++ ) {
          bitstream.writeS( sei.getGtpCameraOrientationOnAxis( camId, d ),
                            16 );  // i(16)
        }
      }
    }
  }
}

// E.2.11  Attribute transformation parameters SEI message syntax
void PCCBitstreamWriter::attributeTransformationParams( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIAttributeTransformationParams&>( seiAbstract );
  bitstream.write( sei.getAtpCancelFlag(), 1 );  // u(1)
  if ( !sei.getAtpCancelFlag() ) {
    bitstream.writeUvlc( sei.getAtpNumAttributeUpdates() );  // ue(v)
    for ( size_t j = 0; j < sei.getAtpNumAttributeUpdates(); j++ ) {
      bitstream.write( sei.getAtpAttributeIdx( j ), 8 );  // u(8)
      size_t index = sei.getAtpAttributeIdx( j );
      bitstream.write( sei.getAtpDimensionMinus1( index ), 8 );  // u(8)
      for ( size_t i = 0; i < sei.getAtpDimensionMinus1( index ); i++ ) {
        bitstream.write( sei.getAtpScaleParamsEnabledFlag( index, i ),
                         1 );  // u(1)
        bitstream.write( sei.getAtpOffsetParamsEnabledFlag( index, i ),
                         1 );  // u(1)
        if ( sei.getAtpScaleParamsEnabledFlag( index, i ) ) {
          bitstream.write( sei.getAtpAttributeScale( index, i ), 32 );  // u(32)
        }
        if ( sei.getAtpOffsetParamsEnabledFlag( index, i ) ) {
          bitstream.writeS( sei.getAtpAttributeOffset( index, i ), 32 );  // i(32)
        }
      }
    }
  }
}

// E.2.12  Active substreams SEI message syntax
void PCCBitstreamWriter::activeSubstreams( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIActiveSubstreams&>( seiAbstract );
  bitstream.write( sei.getActiveAttributesChangesFlag(),
                   1 );  // u(1)
  bitstream.write( sei.getActiveMapsChangesFlag(),
                   1 );  // u(1)
  bitstream.write( sei.getRawPointsSubstreamsActiveFlag(),
                   1 );  // u(1)
  if ( sei.getActiveAttributesChangesFlag() ) {
    bitstream.write( sei.getAllAttributesActiveFlag(),
                     1 );  // u(1)
    if ( !sei.getAllAttributesActiveFlag() ) {
      bitstream.write( sei.getActiveAttributeCountMinus1(), 7 );  // u(7)
      for ( size_t i = 0; i <= sei.getActiveAttributeCountMinus1(); i++ ) {
        bitstream.write( sei.getActiveAttributeIdx( i ), 7 );  // u(7)
      }
    }
  }
  if ( sei.getActiveMapsChangesFlag() ) {
    bitstream.write( sei.getAllMapsActiveFlag(),
                     1 );  // u(1)
    if ( !sei.getAllMapsActiveFlag() ) {
      bitstream.write( sei.getActiveMapCountMinus1(), 4 );  // u(4)
      // sei.getActiveMapIdx().resize( sei.getActiveMapCountMinus1() + 1, 0 );
      for ( size_t i = 0; i <= sei.getActiveMapCountMinus1(); i++ ) {
        bitstream.write( sei.getActiveMapIdx( i ), 4 );  // u(4)
      }
    }
  }
}

// E.2.13  Component codec mapping SEI message syntax
void PCCBitstreamWriter::componentCodecMapping( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIComponentCodecMapping&>( seiAbstract );
  bitstream.write( sei.getCcmCodecMappingsCountMinus1(), 8 );  // u(8)
  sei.allocate();
  for ( size_t i = 0; i <= sei.getCcmCodecMappingsCountMinus1(); i++ ) {
    bitstream.write( sei.getCcmCodecId( i ), 8 );                           // u(8)
    bitstream.writeString( sei.getCcmCodec4cc( sei.getCcmCodecId( i ) ) );  // st(v)
  }
}

// m52705
// E.2.14  Volumetric Tiling SEI message syntax
// E.2.14.1  General
// E.2.14.2  Volumetric Tiling Info Labels
// E.2.14.3  Volumetric Tiling Info Objects

// E.2.15  Buffering period SEI message syntax
void PCCBitstreamWriter::bufferingPeriod( PCCBitstream&        bitstream,
                                          SEI&                 seiAbstract,
                                          bool                 NalHrdBpPresentFlag,
                                          bool                 AclHrdBpPresentFlag,
                                          std::vector<uint8_t> hrdCabCntMinus1 ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEIBufferingPeriod&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  bitstream.writeUvlc( sei.getBpAtlasSequenceParameterSetId() );  // ue(v)
  bitstream.write( sei.getBpIrapCabParamsPresentFlag(),
                   1 );  // u(1)
  if ( sei.getBpIrapCabParamsPresentFlag() ) {
    bitstream.write( sei.getBpCabDelayOffset(), fixedBitcount );  // u(v)
    bitstream.write( sei.getBpDabDelayOffset(), fixedBitcount );  // u(v)
  }
  bitstream.write( sei.getBpConcatenationFlag(),
                   1 );  // u(1)
  bitstream.write( sei.getBpAtlasCabRemovalDelayDeltaMinus1(),
                   fixedBitcount );                     // u(v)
  bitstream.write( sei.getBpMaxSubLayersMinus1(), 3 );  // u(3)
  for ( size_t i = 0; i <= sei.getBpMaxSubLayersMinus1(); i++ ) {
    if ( NalHrdBpPresentFlag ) {
      for ( size_t j = 0; j < hrdCabCntMinus1[i] + 1; j++ ) {
        bitstream.write( sei.getBpNalInitialCabRemovalDelay( i, j ),
                         fixedBitcount );  // u(v)
        bitstream.write( sei.getBpNalInitialCabRemovalOffset( i, j ),
                         fixedBitcount );  // u(v)
      }
      if ( sei.getBpIrapCabParamsPresentFlag() ) {
        bitstream.write( sei.getBpNalInitialAltCabRemovalDelay( i ),
                         fixedBitcount );  // u(v)
        bitstream.write( sei.getBpNalInitialAltCabRemovalOffset( i ),
                         fixedBitcount );  // u(v)
      }
    }
    if ( AclHrdBpPresentFlag ) {
      for ( size_t j = 0; j < hrdCabCntMinus1[i] + 1; j++ ) {
        bitstream.write( sei.getBpAclInitialCabRemovalDelay( i, j ),
                         fixedBitcount );  // u(v)
        bitstream.write( sei.getBpAclInitialCabRemovalOffset( i, j ),
                         fixedBitcount );  // u(v)
      }
      if ( sei.getBpIrapCabParamsPresentFlag() ) {
        bitstream.write( sei.getBpAclInitialAltCabRemovalDelay( i ),
                         fixedBitcount );  // u(v)
        bitstream.write( sei.getBpAclInitialAltCabRemovalOffset( i ),
                         fixedBitcount );  // u(v)
      }
    }
  }
}

// E.2.16  Atlas frame timing SEI message syntax
void PCCBitstreamWriter::atlasFrameTiming( PCCBitstream& bitstream, SEI& seiAbstract, bool cabDabDelaysPresentFlag ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEIAtlasFrameTiming&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  if ( cabDabDelaysPresentFlag ) {
    bitstream.write( sei.getAftCabRemovalDelayMinus1(), fixedBitcount );  // u(v)
    bitstream.write( sei.getAftDabOutputDelay(), fixedBitcount );         // u(v)
  }
}

// E.2.17  Presentation inforomation SEI message syntax
void PCCBitstreamWriter::presentationInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIPresentationInformation&>( seiAbstract );
  bitstream.write( sei.getPiUnitOfLengthFlag(),
                   1 );  // u(1)
  bitstream.write( sei.getPiOrientationPresentFlag(),
                   1 );  // u(1)
  bitstream.write( sei.getPiPivotPresentFlag(),
                   1 );  // u(1)
  bitstream.write( sei.getPiDimensionPresentFlag(),
                   1 );  // u(1)
  if ( sei.getPiOrientationPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.writeS( sei.getPiUp( d ), 32 );     // i(32)
      bitstream.writeS( sei.getPiFront( d ), 32 );  // i(32)
    }
  }
  if ( sei.getPiPivotPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.writeS( sei.getPiPivot( d ) >> 32, 32 );
      bitstream.write( sei.getPiPivot( d ) & 0xFFFFFFFF, 32 );  // i(64)
    }
  }
  if ( sei.getPiDimensionPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.writeS( sei.getPiDimension( d ) >> 32, 32 );
      bitstream.write( sei.getPiDimension( d ) & 0xFFFFFFFF, 32 );  // i(64)
    }
  }
}

// E.2.18  Smoothing parameters SEI message syntax
void PCCBitstreamWriter::smoothingParameters( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEISmoothingParameters&>( seiAbstract );
  bitstream.write( sei.getSpGeometryCancelFlag(),
                   1 );  // u(1)
  bitstream.write( sei.getSpAttributeCancelFlag(),
                   1 );  // u(1)
  if ( !sei.getSpGeometryCancelFlag() ) {
    bitstream.write( sei.getSpGeometrySmoothingEnabledFlag(),
                     1 );  // u(1)
    if ( static_cast<int>( sei.getSpGeometrySmoothingEnabledFlag() ) == 1 ) {
      bitstream.write( sei.getSpGeometrySmoothingId(), 8 );  // u(8)
      TRACE_BITSTREAM( "SpGeometrySmoothingId = %u \n", sei.getSpGeometrySmoothingId() );
      if ( sei.getSpGeometrySmoothingId() == 0 ) {
        bitstream.write( sei.getSpGeometrySmoothingGridSizeMinus2(), 7 );  // u(7)
        bitstream.write( sei.getSpGeometrySmoothingThreshold(), 8 );       // u(8)
        TRACE_BITSTREAM( "  GridSizeMinus2 = %u \n", sei.getSpGeometrySmoothingGridSizeMinus2() );
        TRACE_BITSTREAM( "  Threshold = %u \n", sei.getSpGeometrySmoothingThreshold() );
      } else if ( sei.getSpGeometrySmoothingId() == 1 ) {
        bitstream.write( sei.getSpGeometryPatchBlockFilteringLog2ThresholdMinus1(),
                         2 );  // u(2)
        bitstream.write( sei.getSpGeometryPatchBlockFilteringPassesCountMinus1(),
                         2 );  // u(3)
        bitstream.write( sei.getSpGeometryPatchBlockFilteringFilterSizeMinus1(),
                         3 );  // u(3)
        TRACE_BITSTREAM( "  Log2ThresholdMinus1 = %u \n", sei.getSpGeometryPatchBlockFilteringLog2ThresholdMinus1() );
        TRACE_BITSTREAM( "  PassesCountMinus1 = %u \n", sei.getSpGeometryPatchBlockFilteringPassesCountMinus1() );
        TRACE_BITSTREAM( "  FilterSizeMinus1 = %u \n", sei.getSpGeometryPatchBlockFilteringFilterSizeMinus1() );
      }
    }
  }
  if ( !sei.getSpAttributeCancelFlag() ) {
    bitstream.writeUvlc( sei.getSpNumAttributeUpdates() );  // ue(v)
    for ( size_t j = 0; j < sei.getSpNumAttributeUpdates(); j++ ) {
      bitstream.write( sei.getSpAttributeIdx( j ), 8 );  // u(8)
      size_t index = sei.getSpAttributeIdx( j );
      bitstream.write( sei.getSpDimensionMinus1( index ), 8 );  // u(8)
      for ( size_t i = 0; i < sei.getSpDimensionMinus1( index ) + 1; i++ ) {
        bitstream.write( sei.getSpAttrSmoothingParamsEnabledFlag( index, i ),
                         1 );  // u(1)
        if ( sei.getSpAttrSmoothingParamsEnabledFlag( index, i ) ) {
          bitstream.write( sei.getSpAttrSmoothingGridSizeMinus2( index, i ),
                           8 );  // u(8)
          bitstream.write( sei.getSpAttrSmoothingThreshold( index, i ),
                           8 );  // u(8)
          bitstream.write( sei.getSpAttrSmoothingLocalEntropyThreshold( index, i ),
                           8 );  // u(3)
          bitstream.write( sei.getSpAttrSmoothingThresholdVariation( index, i ),
                           8 );  // u(8)
          bitstream.write( sei.getSpAttrSmoothingThresholdDifference( index, i ),
                           8 );  // u(8)
        }
      }
    }
  }
}
// m52705
void PCCBitstreamWriter::sceneObjectInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEISceneObjectInformation&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  bitstream.write( sei.getSoiCancelFlag(), 1 );
  bitstream.writeUvlc( sei.getSoiNumObjectUpdates() );
  if ( sei.getSoiNumObjectUpdates() > 0 ) {
    bitstream.write( sei.getSoiSimpleObjectsFlag(), 1 );
    if ( static_cast<int>( sei.getSoiSimpleObjectsFlag() ) == 0 ) {
      bitstream.write( sei.getSoiObjectLabelPresentFlag(), 1 );
      bitstream.write( sei.getSoiPriorityPresentFlag(), 1 );
      bitstream.write( sei.getSoiObjectHiddenPresentFlag(), 1 );
      bitstream.write( sei.getSoiObjectDependencyPresentFlag(), 1 );
      bitstream.write( sei.getSoiVisibilityConesPresentFlag(), 1 );
      bitstream.write( sei.getSoi3dBoundingBoxPresentFlag(), 1 );
      bitstream.write( sei.getSoiCollisionShapePresentFlag(), 1 );
      bitstream.write( sei.getSoiPointStylePresentFlag(), 1 );
      bitstream.write( sei.getSoiMaterialIdPresentFlag(), 1 );
      bitstream.write( sei.getSoiExtensionPresentFlag(), 1 );
    }
    if ( sei.getSoi3dBoundingBoxPresentFlag() ) {
      bitstream.write( sei.getSoi3dBoundingBoxScaleLog2(), 5 );
      bitstream.write( sei.getSoi3dBoundingBoxPrecisionMinus8(), 5 );
    }
    bitstream.write( sei.getSoiLog2MaxObjectIdxUpdated(), 5 );
    if ( sei.getSoiObjectDependencyPresentFlag() ) { bitstream.write( sei.getSoiLog2MaxObjectDependencyIdx(), 5 ); }
    for ( size_t i = 0; i <= sei.getSoiNumObjectUpdates(); i++ ) {
      assert( sei.getSoiObjectIdx( i ) >= sei.getSoiNumObjectUpdates() );
      bitstream.write( sei.getSoiObjectIdx( i ), sei.getSoiLog2MaxObjectIdxUpdated() );
      size_t k = sei.getSoiObjectIdx( i );
      bitstream.write( sei.getSoiObjectCancelFlag( k ), 1 );
      // ObjectTracked[k]=!piObjectCancelFlag[k];
      if ( sei.getSoiObjectCancelFlag( k ) ) {
        if ( sei.getSoiObjectLabelPresentFlag() ) {
          bitstream.write( sei.getSoiObjectLabelUpdateFlag( k ), 1 );
          if ( sei.getSoiObjectLabelUpdateFlag( k ) ) { bitstream.writeUvlc( sei.getSoiObjectLabelIdx( k ) ); }
        }
        if ( sei.getSoiPriorityPresentFlag() ) {
          bitstream.write( sei.getSoiPriorityUpdateFlag( k ), 1 );
          if ( sei.getSoiPriorityUpdateFlag( k ) ) { bitstream.write( sei.getSoiPriorityValue( k ), 4 ); }
        }
        if ( sei.getSoiObjectHiddenPresentFlag() ) {
          bitstream.write( sei.getSoiObjectHiddenFlag( k ), 1 );
        }

        if ( sei.getSoiObjectDependencyPresentFlag() ) {
          bitstream.write( sei.getSoiObjectDependencyUpdateFlag( k ), 1 );
          if ( sei.getSoiObjectDependencyUpdateFlag( k ) ) {
            bitstream.write( sei.getSoiObjectNumDependencies( k ), 4 );
            size_t bitCount = ceil( log2( sei.getSoiObjectNumDependencies( k ) ) + 0.5 );
            for ( size_t j = 0; j < sei.getSoiObjectNumDependencies( k ); j++ ) {
              bitstream.write( sei.getSoiObjectDependencyIdx( k, j ), bitCount );
            }
          }
        }
        if ( sei.getSoiVisibilityConesPresentFlag() ) {
          bitstream.write( sei.getSoiVisibilityConesUpdateFlag( k ), 1 );
          if ( sei.getSoiVisibilityConesUpdateFlag( k ) ) {
            bitstream.write( sei.getSoiDirectionX( k ), 32 );
            bitstream.write( sei.getSoiDirectionY( k ), 32 );
            bitstream.write( sei.getSoiDirectionZ( k ), 32 );
            bitstream.write( sei.getSoiAngle( k ), 16 );
          }
        }  // cones

        if ( sei.getSoi3dBoundingBoxPresentFlag() ) {
          bitstream.write( sei.getSoi3dBoundingBoxUpdateFlag( k ), 1 );
          if ( sei.getSoi3dBoundingBoxUpdateFlag( k ) ) {
            bitstream.write( sei.getSoi3dBoundingBoxX( k ), fixedBitcount );
            bitstream.write( sei.getSoi3dBoundingBoxY( k ), fixedBitcount );
            bitstream.write( sei.getSoi3dBoundingBoxZ( k ), fixedBitcount );
            bitstream.write( sei.getSoi3dBoundingBoxDeltaX( k ), fixedBitcount );
            bitstream.write( sei.getSoi3dBoundingBoxDeltaY( k ), fixedBitcount );
            bitstream.write( sei.getSoi3dBoundingBoxDeltaZ( k ), fixedBitcount );
          }
        }  // 3dBB

        if ( sei.getSoiCollisionShapePresentFlag() ) {
          bitstream.write( sei.getSoiCollisionShapeUpdateFlag( k ), 1 );
          if ( sei.getSoiCollisionShapeUpdateFlag( k ) ) { bitstream.write( sei.getSoiCollisionShapeId( k ), 16 ); }
        }  // collision
        if ( sei.getSoiPointStylePresentFlag() ) {
          bitstream.write( sei.getSoiPointStyleUpdateFlag( k ), 1 );
          if ( sei.getSoiPointStyleUpdateFlag( k ) ) {
            bitstream.write( sei.getSoiPointShapeId( k ), 8 );  // only shape??
          }
          bitstream.write( sei.getSoiPointSize( k ), 16 );
        }  // pointstyle
        if ( sei.getSoiMaterialIdPresentFlag() ) {
          bitstream.write( sei.getSoiMaterialIdUpdateFlag( k ), 1 );
          if ( sei.getSoiMaterialIdUpdateFlag( k ) ) { bitstream.write( sei.getSoiMaterialId( k ), 16 ); }
        }  // materialid
      }    // sei.getSoiObjectCancelFlag(k)
    }      // for(size_t i=0; i<=sei.getSoiNumObjectUpdates(); i++)
  }        // if( sei.getSoiNumObjectUpdates() > 0 )
}

void PCCBitstreamWriter::objectLabelInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIObjectLabelInformation&>( seiAbstract );
  bitstream.write( sei.getOliCancelFlag(), 1 );
  if ( !sei.getOliCancelFlag() ) {
    bitstream.write( sei.getOliLabelLanguagePresentFlag(), 1 );
    if ( sei.getOliLabelLanguagePresentFlag() ) {
      while ( !bitstream.byteAligned() ) { bitstream.write( uint32_t( 0 ), 1 ); }
      bitstream.writeString( sei.getOliLabelLanguage() );
    }
    bitstream.writeUvlc( sei.getOliNumLabelUpdates() );
    for ( size_t i = 0; i < sei.getOliNumLabelUpdates(); i++ ) {
      bitstream.writeUvlc( sei.getOliLabelIdx( i ) );
      bitstream.write( sei.getOliLabelCancelFlag(), 1 );
      if ( !sei.getOliLabelCancelFlag() ) {
        while ( !bitstream.byteAligned() ) { bitstream.write( uint32_t( 0 ), 1 ); }
        bitstream.writeString( sei.getOliLabel( sei.getOliLabelIdx( i ) ) );
      }
    }
  }
};  // Object label information
void PCCBitstreamWriter::patchInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sei = static_cast<SEIPatchInformation&>( seiAbstract );
  bitstream.write( sei.getPiCancelFlag(), 1 );
  bitstream.writeUvlc( sei.getPiNumTileGroupUpdates() );
  if ( sei.getPiNumTileGroupUpdates() > 0 ) {
    bitstream.write( sei.getPiLog2MaxObjectIdxTracked(), 5 );
    bitstream.write( sei.getPiLog2MaxPatchIdxUpdated(), 4 );
  }
  for ( size_t i = 0; i < sei.getPiNumTileGroupUpdates(); i++ ) {
    bitstream.writeUvlc( sei.getPiTileGroupAddress( i ) );
    size_t j = sei.getPiTileGroupAddress( i );
    bitstream.write( sei.getPiTileGroupCancelFlag( j ), 1 );
    bitstream.writeUvlc( sei.getPiNumPatchUpdates( j ) );
    for ( size_t k = 0; k < sei.getPiNumPatchUpdates( j ); k++ ) {
      bitstream.write( sei.getPiPatchIdx( j, k ), sei.getPiLog2MaxPatchIdxUpdated() );
      auto p = sei.getPiPatchIdx( j, k );
      bitstream.write( sei.getPiPatchCancelFlag( j, p ), 1 );
      if ( !sei.getPiPatchCancelFlag( j, p ) ) {
        bitstream.writeUvlc( sei.getPiPatchNumberOfObjectsMinus1( j, p ) );
        for ( size_t n = 0; n < sei.getPiPatchNumberOfObjectsMinus1( j, p ) + 1; n++ ) {
          bitstream.write( sei.getPiPatchObjectIdx( j, p, n ),
                           sei.getPiLog2MaxObjectIdxTracked() );  //?pi_log2_max_object_idx_updated?
        }
      }
    }
  }
};  // patch information
void PCCBitstreamWriter::volumetricRectangleInformation( PCCBitstream& bitstream, SEI& seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&         sei           = static_cast<SEIVolumetricRectangleInformation&>( seiAbstract );
  const int32_t fixedBitcount = 16;
  bitstream.write( sei.getVriCancelFlag(), 1 );
  bitstream.writeUvlc( sei.getVriNumRectanglesUpdates() );
  if ( sei.getVriNumRectanglesUpdates() > 0 ) {
    bitstream.write( sei.getVriLog2MaxObjectIdxTracked(), 5 );
    bitstream.write( sei.getVriLog2MaxRectangleIdxUpdated(), 4 );
  }

  for ( size_t k = 0; k < sei.getVriNumRectanglesUpdates(); k++ ) {
    bitstream.write( sei.getVriRectangleIdx( k ), sei.getVriLog2MaxRectangleIdxUpdated() );
    auto p = sei.getVriRectangleIdx( k );
    bitstream.write( sei.getVriRectangleCancelFlag( p ), 1 );

    if ( !sei.getVriRectangleCancelFlag( p ) ) {
      bitstream.write( sei.getVriBoundingBoxUpdateFlag( p ), 1 );
      if ( sei.getVriBoundingBoxUpdateFlag( p ) ) {
        bitstream.write( sei.getVriBoundingBoxTop( p ), fixedBitcount );
        bitstream.write( sei.getVriBoundingBoxLeft( p ), fixedBitcount );
        bitstream.write( sei.getVriBoundingBoxWidth( p ), fixedBitcount );
        bitstream.write( sei.getVriBoundingBoxHeight( p ), fixedBitcount );
      }
      bitstream.writeUvlc( sei.getVriRectangleNumberOfObjectsMinus1( p ) );
      for ( size_t n = 0; n < sei.getVriRectangleNumberOfObjectsMinus1( p ) + 1; n++ ) {
        bitstream.write( sei.getVriRectangleObjectIdx( p, n ), sei.getVriLog2MaxObjectIdxTracked() );
      }
    }
  }
};  // volumetric rectangle information
////////

// F.2  VUI syntax
// F.2.1  VUI parameters syntax
void PCCBitstreamWriter::vuiParameters( PCCBitstream& bitstream, VUIParameters& vp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( vp.getVuiWorldCoordinatesInfoPresentFlag(),
                   1 );  // u(1)
  if ( vp.getVuiWorldCoordinatesInfoPresentFlag() ) {
    bitstream.write( vp.getVuiUnitInMetresFlag(),
                     1 );                               // u(1)
    bitstream.write( vp.getVuiNumUnitsInBlock(), 32 );  // u(32)
    bitstream.write( vp.getVuiBlockScale(), 32 );       // u(32)
  }
  bitstream.write( vp.getVuiDefaultDispalyBoxSize(),
                   1 );  // u(1)
  if ( vp.getVuiDefaultDispalyBoxSize() ) {
    bitstream.writeUvlc( vp.getVuiDefDispBoxLeftOffset() );    // ue(v)
    bitstream.writeUvlc( vp.getVuiDefDispBoxRightOffset() );   // ue(v)
    bitstream.writeUvlc( vp.getVuiDefDispBoxTopOffset() );     // ue(v)
    bitstream.writeUvlc( vp.getVuiDefDispBoxBottomOffset() );  // ue(v)
    bitstream.writeUvlc( vp.getVuiDefDispBoxFrontOffset() );   // ue(v)
    bitstream.writeUvlc( vp.getVuiDefDispBoxBackOffset() );    // ue(v)
  }
  bitstream.write( vp.getVuiTimingInfoPresentFlag(),
                   1 );  // u(1)
  if ( vp.getVuiTimingInfoPresentFlag() ) {
    bitstream.write( vp.getVuiNumUnitsInTick(), 32 );  // u(32)
    bitstream.write( vp.getVuiTimeScale(), 32 );       // u(32)
    bitstream.write( vp.getVuiPocProportionalToTimingFlag(),
                     1 );  // u(1)
    if ( vp.getVuiPocProportionalToTimingFlag() ) {
      bitstream.writeUvlc( vp.getVuiNumTicksPocDiffOneMinus1() );  // ue(v)
    }
    bitstream.write( vp.getVuiHrdParametersPresentFlag(),
                     1 );  // u(1)
    if ( vp.getVuiHrdParametersPresentFlag() ) { hrdParameters( bitstream, vp.getHrdParameters() ); }
  }
  bitstream.write( vp.getVuiBitstreamRestrictionFlag(),
                   1 );  // u(1)
  if ( vp.getVuiBitstreamRestrictionFlag() ) {
    bitstream.write( vp.getVuiTileGroupsRestrictedFlag(),
                     1 );  // u(1)
    bitstream.write( vp.getVuiConsistentTilesForVideoComponentsFlag(),
                     1 );                                       // u(1)
    bitstream.writeUvlc( vp.getVuiMaxNumTileGroupPerAtlas() );  // ue(v)
  }
}

// F.2.2  HRD parameters syntax
void PCCBitstreamWriter::hrdParameters( PCCBitstream& bitstream, HrdParameters& hp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( hp.getHrdNalParametersPresentFlag(),
                   1 );  // u(1)
  bitstream.write( hp.getHrdAclParametersPresentFlag(),
                   1 );  // u(1)
  if ( hp.getHrdNalParametersPresentFlag() || hp.getHrdAclParametersPresentFlag() ) {
    bitstream.write( hp.getHrdBitRateScale(), 4 );                        // u(4)
    bitstream.write( hp.getHrdCabSizeScale(), 4 );                        // u(4)
    bitstream.write( hp.getHrdInitialCabRemovalDelayLengthMinus1(), 5 );  // u(5)
    bitstream.write( hp.getHrdAuCabRemovalDelayLengthMinus1(), 5 );       // u(5)
    bitstream.write( hp.getHrdDabOutputDelayLengthMinus1(), 5 );          // u(5)
  }
  for ( size_t i = 0; i <= hp.getMaxNumSubLayersMinus1(); i++ ) {
    bitstream.write( hp.getHrdFixedAtlasRateGeneralFlag( i ), 1 );  // u(1)
    if ( !hp.getHrdFixedAtlasRateGeneralFlag( i ) ) {
      bitstream.write( hp.getHrdFixedAtlasRateWithinCasFlag( i ), 1 );  // u(1)
    }
    if ( hp.getHrdFixedAtlasRateWithinCasFlag( i ) ) {
      bitstream.write( hp.getHrdElementalDurationInTcMinus1( i ), 1 );  // ue(v)
    } else {
      bitstream.write( hp.getHrdLowDelayFlag( i ), 1 );  // u(1)
    }
    if ( !hp.getHrdLowDelayFlag( i ) ) {
      bitstream.write( hp.getHrdCabCntMinus1( i ) , 1 );  // ue(v)
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
void PCCBitstreamWriter::hrdSubLayerParameters( PCCBitstream& bitstream, HrdSubLayerParameters& hlsp, size_t cabCnt ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  for ( size_t i = 0; i <= cabCnt; i++ ) {
    bitstream.writeUvlc( hlsp.getHrdBitRateValueMinus1( i ) );               // ue(v)
    bitstream.writeUvlc( hlsp.getHrdCabSizeValueMinus1( i ) );               // ue(v)
    bitstream.write( hlsp.getHrdCbrFlag( i ), 1 );  // u(1)
  }
}
