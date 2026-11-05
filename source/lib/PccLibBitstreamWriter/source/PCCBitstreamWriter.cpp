/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2025, ISO/IEC
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
#include "PCCAtlasAdaptationParameterSetRbsp.h"
#include "PCCAccessUnitDelimiterRbsp.h"

#include "PCCBitstreamWriter.h"
#include <cmath>

#if defined( BITSTREAM_TRACE )
static std::string format( const std::string &var ) {
  std::string substr = var;
  auto        npos   = substr.find( "get" );
  if ( npos != std::string::npos ) substr = substr.substr( npos + 3 );
  npos = substr.find( "(" );
  if ( npos != std::string::npos ) substr = substr.substr( 0, npos );
  return substr;
}

#define WRITE_CODE_40bits( VAR, LEN )                                              \
  {                                                                                \
    bitstream.write40bits( VAR, LEN );                                             \
    bitstream.traceBit( format( #VAR ), VAR, "u(" + std::to_string( LEN ) + ")" ); \
  }

#define WRITE_CODE( VAR, LEN )                                                     \
  {                                                                                \
    bitstream.write( VAR, LEN );                                                   \
    bitstream.traceBit( format( #VAR ), VAR, "u(" + std::to_string( LEN ) + ")" ); \
  }

#define WRITE_CODES( VAR, LEN )                                                    \
  {                                                                                \
    bitstream.writeS( VAR, LEN );                                                  \
    bitstream.traceBit( format( #VAR ), VAR, "s(" + std::to_string( LEN ) + ")" ); \
  }

#define WRITE_UVLC( VAR )                               \
  {                                                     \
    bitstream.writeUvlc( VAR );                         \
    bitstream.traceBit( format( #VAR ), VAR, "ue(v)" ); \
  }

#define WRITE_VU( VAR )                                 \
  {                                                     \
    bitstream.writeVu( VAR );                           \
    bitstream.traceBit( format( #VAR ), VAR, "vu(v)" ); \
  }

#define WRITE_SVLC( VAR )                               \
  {                                                     \
    bitstream.writeSvlc( VAR );                         \
    bitstream.traceBit( format( #VAR ), VAR, "se(v)" ); \
  }

#define WRITE_FLOAT( VAR )                                   \
  {                                                          \
    bitstream.writeFloat( VAR );                             \
    bitstream.traceBitFloat( format( #VAR ), VAR, "f(32)" ); \
  }
#define WRITE_DOUBLE( VAR )                                   \
  {                                                           \
    bitstream.writeDouble( VAR );                             \
    bitstream.traceBitDouble( format( #VAR ), VAR, "f(64)" ); \
  }
#define WRITE_STRING( VAR )                                \
  {                                                        \
    bitstream.writeString( VAR );                          \
    bitstream.traceBitStr( format( #VAR ), VAR, "f(32)" ); \
  }

#define WRITE_VECTOR( VAR )                                                                 \
  {                                                                                         \
    bitstream.write( VAR );                                                                 \
    bitstream.trace( "Data stream: %s size = %zu \n", format( #VAR ).c_str(), VAR.size() ); \
  }

#define WRITE_VIDEO( VAR )                                                                                  \
  {                                                                                                         \
    bitstream.write( VAR );                                                                                 \
    bitstream.trace( "Video stream: type = %s size = %zu \n", toString( VAR.type() ).c_str(), VAR.size() ); \
  }
#else
#define WRITE_CODE( VAR, LEN ) bitstream.write( VAR, LEN );
#define WRITE_CODES( VAR, LEN ) bitstream.writeS( VAR, LEN );
#define WRITE_UVLC( VAR ) bitstream.writeUvlc( VAR );
#define WRITE_SVLC( VAR ) bitstream.writeSvlc( VAR );
#define WRITE_FLOAT( VAR ) bitstream.writeFloat( VAR );
#define WRITE_DOUBLE( VAR ) bitstream.writeDouble( VAR );
#define WRITE_STRING( VAR ) bitstream.writeString( VAR );
#define WRITE_VECTOR( VAR ) bitstream.write( VAR );
#define WRITE_VIDEO( VAR ) bitstream.write( VAR );
#define WRITE_VU( VAR ) bitstream.writeVu( VAR );
#define WRITE_CODE_40bits( VAR, LEN ) bitstream.write40bits( VAR, LEN );
#endif

using namespace std;
using namespace pcc;

PCCBitstreamWriter::PCCBitstreamWriter()  = default;
PCCBitstreamWriter::~PCCBitstreamWriter() = default;

int32_t PCCBitstreamWriter::write( SampleStreamNalUnit &ssnu,
                                   PCCBitstream        &bitstream,
                                   uint32_t             forcedSsvhUnitSizePrecisionBytes ) {
  TRACE_BITSTREAM_IN( "%s", "SampleStreamNalUnits" );
  uint32_t precision = 4;
  precision          = (std::max)( precision, forcedSsvhUnitSizePrecisionBytes );
  ssnu.setSizePrecisionBytesMinus1( precision - 1 );
  sampleStreamNalHeader( bitstream, ssnu );
  TRACE_BITSTREAM_OUT( "%s", "SampleStreamNalUnits" );
  return 0;
}

size_t PCCBitstreamWriter::write( SampleStreamV3CUnit &ssvu,
                                  PCCBitstream        &bitstream,
                                  uint32_t             forcedSsvhUnitSizePrecisionBytes ) {
  TRACE_BITSTREAM_IN( "%s", "SampleStreamV3CUnits" );
  size_t headerSize = 0;
  // Calculating the precision of the unit size
  uint32_t maxUnitSize = 0;
  for ( auto &v3cUnit : ssvu.getV3CUnit() ) {
    if ( maxUnitSize < v3cUnit.getSize() ) { maxUnitSize = static_cast<uint32_t>( v3cUnit.getSize() ); }
  }
  // TRACE_BITSTREAM( "maxUnitSize = %u \n", maxUnitSize );
  uint32_t precision = static_cast<uint32_t>(
      min( max( static_cast<int>( ceil( static_cast<double>( ceilLog2( maxUnitSize ) ) / 8.0 ) ), 1 ), 8 ) );
  precision = (std::max)( precision, forcedSsvhUnitSizePrecisionBytes );
  ssvu.setSsvhUnitSizePrecisionBytesMinus1( precision - 1 );
  // TRACE_BITSTREAM( " => SsvhUnitSizePrecisionBytesMinus1 = %u \n", ssvu.getSsvhUnitSizePrecisionBytesMinus1() );

  sampleStreamV3CHeader( bitstream, ssvu );
  headerSize += 1;
  // TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 %d <=> bytesToRead %d\n", precision,
  //                  ( 8 * ( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) ) );
  size_t unitCount = 0;
  for ( auto &v3cUnit : ssvu.getV3CUnit() ) {
    sampleStreamV3CUnit( bitstream, ssvu, v3cUnit );
    // TRACE_BITSTREAM( "V3C Unit Size(unit type:%zu, %zuth/%zu)  = %zu \n", unitCount, (size_t)v3cUnit.getType(),
    //                  ssvu.getV3CUnitCount(), v3cUnit.getSize() );
    unitCount++;
    headerSize += ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1;
  }
  // TRACE_BITSTREAM( "%s \n", "PCCBitstreamXXcoder: SampleStream Vpcc Unit start done" );
  TRACE_BITSTREAM_OUT( "%s", "SampleStreamV3CUnits" );
  return headerSize;
}

int PCCBitstreamWriter::encode( PCCHighLevelSyntax &syntax, SampleStreamV3CUnit &ssvu ) {
  auto &vuhGVD = syntax.getV3CUnitHeaderGVD();
  auto &vuhAD  = syntax.getV3CUnitHeaderAD();
  auto &vuhAVD = syntax.getV3CUnitHeaderAVD();
  auto &vuhOVD = syntax.getV3CUnitHeaderOVD();
  auto &vuhPVD = syntax.getV3CUnitHeaderPVD();
  auto &sps    = syntax.getVps();
  auto &asps   = syntax.getAtlasSequenceParameterSet( 0 );  // TODO: get correct value
  vuhGVD.setV3CParameterSetId( sps.getV3CParameterSetId() );
  vuhAD.setV3CParameterSetId( sps.getV3CParameterSetId() );
  vuhAVD.setV3CParameterSetId( sps.getV3CParameterSetId() );
  vuhOVD.setV3CParameterSetId( sps.getV3CParameterSetId() );
  vuhPVD.setV3CParameterSetId( sps.getV3CParameterSetId() );
  syntax.getBitstreamStat().newGOF();

  // encode the SPS
  PCCBitstream bitstreamVPS;
#ifdef BITSTREAM_TRACE
  bitstreamVPS.setTrace( true );
  bitstreamVPS.setLogger( *logger_ );
  // bitstreamVPS.trace( "%s \n", "PCCBitstream::(V3C_VPS)" );
#endif
  v3cUnit( syntax, bitstreamVPS, V3C_VPS );
  ssvu.addV3CUnit().setBitstream( std::move( bitstreamVPS ), V3C_VPS );
  for ( uint8_t atlasIdx = 0; atlasIdx < sps.getAtlasCountMinus1() + 1; atlasIdx++ ) {
    vuhAD.setAtlasId( atlasIdx );
    vuhOVD.setAtlasId( atlasIdx );
    vuhGVD.setAtlasId( atlasIdx );
    vuhAVD.setAtlasId( atlasIdx );
    vuhPVD.setAtlasId( atlasIdx );
    // encode the AD
    PCCBitstream bitstreamAD;
#ifdef BITSTREAM_TRACE
    bitstreamAD.setTrace( true );
    bitstreamAD.setLogger( *logger_ );
    // bitstreamAD.trace( "%s \n", "PCCBitstream::(V3C_AD)" );
#endif
    v3cUnit( syntax, bitstreamAD, V3C_AD );
    ssvu.addV3CUnit().setBitstream( std::move( bitstreamAD ), V3C_AD );

    if ( sps.getOccupancyVideoPresentFlag( atlasIdx ) ) {
      // encode OVD
      PCCBitstream bitstreamOVD;
#ifdef BITSTREAM_TRACE
      bitstreamOVD.setTrace( true );
      bitstreamOVD.setLogger( *logger_ );
      // bitstreamOVD.trace( "%s \n", "PCCBitstream::(V3C_OVD)" );
#endif
      v3cUnit( syntax, bitstreamOVD, V3C_OVD );
      ssvu.addV3CUnit().setBitstream( std::move( bitstreamOVD ), V3C_OVD );
    }
    // encode GVD
    if ( sps.getGeometryVideoPresentFlag( atlasIdx ) ) {
      if ( sps.getMapCountMinus1( atlasIdx ) > 0 && sps.getMultipleMapStreamsPresentFlag( atlasIdx ) ) {
        for ( uint32_t mapIdx = 0; mapIdx < sps.getMapCountMinus1( atlasIdx ) + 1; mapIdx++ ) {
          PCCBitstream bitstreamGVD;
#ifdef BITSTREAM_TRACE
          bitstreamGVD.setTrace( true );
          bitstreamGVD.setLogger( *logger_ );
          // bitstreamGVD.trace( "%s \n", "PCCBitstream::(V3C_GVD)" );
#endif
          // encode D(mapIdx)
          vuhGVD.setMapIndex( mapIdx );
          vuhGVD.setAuxiliaryVideoFlag( false );
          v3cUnit( syntax, bitstreamGVD, V3C_GVD );
          ssvu.addV3CUnit().setBitstream( std::move( bitstreamGVD ), V3C_GVD );
        }
      } else {
        PCCBitstream bitstreamGVD;
#ifdef BITSTREAM_TRACE
        bitstreamGVD.setTrace( true );
        bitstreamGVD.setLogger( *logger_ );
        // bitstreamGVD.trace( "%s \n", "PCCBitstream::(V3C_GVD)" );
#endif
        // encode D=D(0)|D(1)|...|D(N)
        vuhGVD.setMapIndex( 0 );
        vuhGVD.setAuxiliaryVideoFlag( false );
        v3cUnit( syntax, bitstreamGVD, V3C_GVD );
        ssvu.addV3CUnit().setBitstream( std::move( bitstreamGVD ), V3C_GVD );
      }
      if ( asps.getRawPatchEnabledFlag() && sps.getAuxiliaryVideoPresentFlag( atlasIdx ) ) {
        PCCBitstream bitstreamGVD;
#ifdef BITSTREAM_TRACE
        bitstreamGVD.setTrace( true );
        bitstreamGVD.setLogger( *logger_ );
        // bitstreamGVD.trace( "%s \n", "PCCBitstream::(V3C_GVD)" );
#endif
        // encode RAW
        vuhGVD.setMapIndex( 0 );
        vuhGVD.setAuxiliaryVideoFlag( true );
        v3cUnit( syntax, bitstreamGVD, V3C_GVD );
        ssvu.addV3CUnit().setBitstream( std::move( bitstreamGVD ), V3C_GVD );
      }
    }
    if ( sps.getAttributeVideoPresentFlag( atlasIdx ) ) {
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
              bitstreamAVD.setLogger( *logger_ );
              // bitstreamAVD.trace( "%s \n", "PCCBitstream::(V3C_AVD)" );
#endif
              // encode D(mapIdx)
              vuhAVD.setAttributeIndex( attIdx );
              vuhAVD.setAttributeDimensionIndex( attDim );
              vuhAVD.setMapIndex( mapIdx );
              vuhAVD.setAuxiliaryVideoFlag( false );
              v3cUnit( syntax, bitstreamAVD, V3C_AVD );
              ssvu.addV3CUnit().setBitstream( std::move( bitstreamAVD ), V3C_AVD );
            }
          } else {
            PCCBitstream bitstreamAVD;
#ifdef BITSTREAM_TRACE
            bitstreamAVD.setTrace( true );
            bitstreamAVD.setLogger( *logger_ );
            // bitstreamAVD.trace( "%s \n", "PCCBitstream::(V3C_AVD)" );
#endif
            // encode D=D(0)|D(1)|...|D(N)
            vuhAVD.setAttributeIndex( attIdx );
            vuhAVD.setAttributeDimensionIndex( attDim );
            vuhAVD.setMapIndex( 0 );
            vuhAVD.setAuxiliaryVideoFlag( false );
            v3cUnit( syntax, bitstreamAVD, V3C_AVD );
            ssvu.addV3CUnit().setBitstream( std::move( bitstreamAVD ), V3C_AVD );
          }
          if ( asps.getRawPatchEnabledFlag() && sps.getAuxiliaryVideoPresentFlag( atlasIdx ) ) {
            PCCBitstream bitstreamAVD;
#ifdef BITSTREAM_TRACE
            bitstreamAVD.setTrace( true );
            bitstreamAVD.setLogger( *logger_ );
            // bitstreamAVD.trace( "%s \n", "PCCBitstream::(V3C_AVD)" );
#endif
            // encode RAW
            vuhAVD.setAttributeIndex( attIdx );
            vuhAVD.setAttributeDimensionIndex( attDim );
            vuhAVD.setMapIndex( 0 );
            vuhAVD.setAuxiliaryVideoFlag( true );
            v3cUnit( syntax, bitstreamAVD, V3C_AVD );
            ssvu.addV3CUnit().setBitstream( std::move( bitstreamAVD ), V3C_AVD );
          }
        }
      }
    }

    if ( sps.getVpsPackedVideoExtension().getPackedVideoPresentFlag( atlasIdx ) ) {
      auto &pin = sps.getVpsPackedVideoExtension().getPackingInformation( atlasIdx );

      for ( uint8_t packedMapIndex = 0; packedMapIndex <= pin.getPackedMapCountMinus1(); ++packedMapIndex ) {
        PCCBitstream bitstreamPVD;
#ifdef BITSTREAM_TRACE
        bitstreamPVD.setTrace( true );
        bitstreamPVD.setLogger( *logger_ );
#endif
        vuhPVD.setPackedMapIndex( packedMapIndex );
        v3cUnit( syntax, bitstreamPVD, V3C_PVD );
        ssvu.addV3CUnit().setBitstream( std::move( bitstreamPVD ), V3C_PVD );
      }
    }
  }
  return 0;
}

void PCCBitstreamWriter::videoSubStream( PCCHighLevelSyntax &syntax,
                                         PCCBitstream       &bitstream,
                                         V3CUnitType         V3CUnitType ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto  &sps          = syntax.getVps();
  auto  &vuh          = syntax.getV3CUnitHeader( static_cast<int>( V3CUnitType ) - 1 );
  size_t atlasIndex   = vuh.getAtlasId();
  auto  &bistreamStat = syntax.getBitstreamStat();
  if ( V3CUnitType == V3C_OVD ) {
    // TRACE_BITSTREAM( "%s \n", "OccupancyMap" );
    bitstream.writeVideoStream( syntax.getVideoBitstream( VIDEO_OCCUPANCY ) );
    bistreamStat.setVideoBinSize( VIDEO_OCCUPANCY, syntax.getVideoBitstream( VIDEO_OCCUPANCY ).size() );
  } else if ( V3CUnitType == V3C_GVD ) {
    if ( vuh.getAuxiliaryVideoFlag() ) {
      // TRACE_BITSTREAM( "%s \n", "Geometry RAW" );
      bitstream.writeVideoStream( syntax.getVideoBitstream( VIDEO_GEOMETRY_RAW ) );
      bistreamStat.setVideoBinSize( VIDEO_GEOMETRY_RAW, syntax.getVideoBitstream( VIDEO_GEOMETRY_RAW ).size() );
    } else {
      if ( sps.getMapCountMinus1( atlasIndex ) > 0 && sps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
        auto geometryIndex = static_cast<PCCVideoType>( VIDEO_GEOMETRY_D0 + vuh.getMapIndex() );
        // TRACE_BITSTREAM( "Geometry MAP: %d\n", vuh.getMapIndex() );
        bitstream.writeVideoStream( syntax.getVideoBitstream( geometryIndex ) );
        bistreamStat.setVideoBinSize( geometryIndex, syntax.getVideoBitstream( geometryIndex ).size() );
      } else {
        // TRACE_BITSTREAM( "%s \n", "Geometry" );
        bitstream.writeVideoStream( syntax.getVideoBitstream( VIDEO_GEOMETRY ) );
        bistreamStat.setVideoBinSize( VIDEO_GEOMETRY, syntax.getVideoBitstream( VIDEO_GEOMETRY ).size() );
      }
    }
  } else if ( V3CUnitType == V3C_AVD ) {
    if ( sps.getAttributeVideoPresentFlag( atlasIndex ) &&
         sps.getAttributeInformation( atlasIndex ).getAttributeCount() > 0 ) {
      if ( vuh.getAuxiliaryVideoFlag() ) {
        int          attrIndex          = vuh.getAttributeIndex();
        int          attrPartitionIndex = vuh.getAttributeDimensionIndex();
        auto         attrTypeId         = sps.getAttributeInformation( atlasIndex ).getAttributeTypeId( attrIndex );
        PCCVideoType attributeIndex     = getAttributeTypeRaw( attrTypeId, attrPartitionIndex );
        std::string  attributeName      = getAttributeName( attrTypeId );
        // TRACE_BITSTREAM( "Attribute %s RAW, PARTITION: %d\n", attributeName.c_str(), vuh.getAttributeDimensionIndex()
        // );
        bitstream.writeVideoStream( syntax.getVideoBitstream( attributeIndex ) );
        bistreamStat.setVideoBinSize( attributeIndex, syntax.getVideoBitstream( attributeIndex ).size() );
      } else {
        if ( sps.getMapCountMinus1( atlasIndex ) > 0 && sps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
          auto attributeIndex = static_cast<PCCVideoType>(
              VIDEO_ATTRIBUTE_T0 + vuh.getMapIndex() * MAX_NUM_ATTR_PARTITIONS + vuh.getAttributeDimensionIndex() );
          // TRACE_BITSTREAM( "Attribute MAP: %d, PARTITION: %d\n", vuh.getMapIndex(), vuh.getAttributeDimensionIndex()
          // );
          bitstream.writeVideoStream( syntax.getVideoBitstream( attributeIndex ) );
          bistreamStat.setVideoBinSize( attributeIndex, syntax.getVideoBitstream( attributeIndex ).size() );
        } else {
          int          attrIndex          = vuh.getAttributeIndex();
          int          attrPartitionIndex = vuh.getAttributeDimensionIndex();
          auto         attrTypeId         = sps.getAttributeInformation( atlasIndex ).getAttributeTypeId( attrIndex );
          PCCVideoType attributeIndex     = getAttributeType( attrTypeId, attrPartitionIndex );
          std::string  attributeName      = getAttributeName( attrTypeId );
          TRACE_BITSTREAM( "attribute %s \n", attributeName.c_str() );
          bitstream.writeVideoStream( syntax.getVideoBitstream( attributeIndex ) );
          bistreamStat.setVideoBinSize( attributeIndex, syntax.getVideoBitstream( attributeIndex ).size() );
        }
      }
    }
  } else if ( V3CUnitType == V3C_PVD ) {
    if ( sps.getVpsPackedVideoExtension().getPackedVideoPresentFlag( atlasIndex ) ) {
      int        packedMapIndex = vuh.getPackedMapIndex();
      const auto videoType      = PCCVideoType( VIDEO_PACKED + packedMapIndex );
      bitstream.writeVideoStream( syntax.getVideoBitstream( videoType ) );
      bistreamStat.setVideoBinSize( videoType, syntax.getVideoBitstream( videoType ).size() );
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.2 Specification of syntax functions and descriptors
bool PCCBitstreamWriter::byteAligned( PCCBitstream &bitstream ) { return bitstream.byteAligned(); }
bool PCCBitstreamWriter::moreDataInPayload( PCCBitstream &bitstream ) {
  // TRACE_BITSTREAM( "%s \n", __func__ );
  return !bitstream.byteAligned();
}
bool PCCBitstreamWriter::moreRbspData( PCCBitstream &bitstream ) {
  // TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}
bool PCCBitstreamWriter::moreRbspTrailingData( PCCBitstream &bitstream ) { return false; }
bool PCCBitstreamWriter::moreDataInV3CUnit( PCCBitstream &bitstream ) { return false; }
bool PCCBitstreamWriter::payloadExtensionPresent( PCCBitstream &bitstream ) {
  // TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}
// 8.3.2 V3C unit syntax
// 8.3.2.1 General V3C unit syntax
void PCCBitstreamWriter::v3cUnit( PCCHighLevelSyntax &syntax, PCCBitstream &bitstream, V3CUnitType V3CUnitType ) {
  TRACE_BITSTREAM_IN( "%s(%s)", __func__, toString( V3CUnitType ).c_str() );
  auto position = static_cast<int32_t>( bitstream.size() );
  v3cUnitHeader( syntax, bitstream, V3CUnitType );
  v3cUnitPayload( syntax, bitstream, V3CUnitType );
  syntax.getBitstreamStat().setV3CUnitSize( V3CUnitType, static_cast<int32_t>( bitstream.size() ) - position );
  TRACE_BITSTREAM_OUT( "%s(%s)", __func__, toString( V3CUnitType ).c_str() );
}

// 8.3.2.2 V3C unit header syntax
void PCCBitstreamWriter::v3cUnitHeader( PCCHighLevelSyntax &syntax, PCCBitstream &bitstream, V3CUnitType v3cUnitType ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( v3cUnitType, 5 )
  auto &vpcc = syntax.getV3CUnitHeader( static_cast<int>( v3cUnitType ) - 1 );

  if ( v3cUnitType == V3C_AVD || v3cUnitType == V3C_GVD || v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD ||
       v3cUnitType == V3C_CAD || v3cUnitType == V3C_PVD || v3cUnitType == V3C_BMD || v3cUnitType == V3C_ADD ) {
    WRITE_CODE( vpcc.getV3CParameterSetId(), 4 );  // u(4)
  }
  if ( v3cUnitType == V3C_AVD || v3cUnitType == V3C_GVD || v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD ||
       v3cUnitType == V3C_PVD || v3cUnitType == V3C_BMD || v3cUnitType == V3C_ADD ) {
    WRITE_CODE( vpcc.getAtlasId(), 6 );  // u(6)
  }
  if ( v3cUnitType == V3C_AVD ) {
    WRITE_CODE( vpcc.getAttributeIndex(), 7 );           // u(7)
    WRITE_CODE( vpcc.getAttributeDimensionIndex(), 5 );  // u(5)
    WRITE_CODE( vpcc.getMapIndex(), 4 );                 // u(4)
    WRITE_CODE( vpcc.getAuxiliaryVideoFlag(), 1 );       // u(1)
  } else if ( v3cUnitType == V3C_GVD ) {
    WRITE_CODE( vpcc.getMapIndex(), 4 );            // u(4)
    WRITE_CODE( vpcc.getAuxiliaryVideoFlag(), 1 );  // u(1)
    auto vuh_reserved_zero_12bits = 0;
    WRITE_CODE( vuh_reserved_zero_12bits, 12 );  // u(12)
  } else if ( v3cUnitType == V3C_PVD ) {
    WRITE_CODE( vpcc.getPackedMapIndex(), 8 );  // u(8)
    auto vuh_reserved_zero_9bits = 0;
    WRITE_CODE( vuh_reserved_zero_9bits, 9 );  // u(9)
  } else if ( v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD || v3cUnitType == V3C_BMD || v3cUnitType == V3C_ADD ) {
    auto vuh_reserved_zero_17bits = 0;
    WRITE_CODE( vuh_reserved_zero_17bits, 17 );  // u(17)
  } else if ( v3cUnitType == V3C_CAD ) {
    auto vuh_reserved_zero_23bits = 0;
    WRITE_CODE( vuh_reserved_zero_23bits, 23 );  // u(23)
  } else {
    auto vuh_reserved_zero_27bits = 0;
    WRITE_CODE( vuh_reserved_zero_27bits, 27 );  // u(27)
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.2.3 V3C unit payload syntax
void PCCBitstreamWriter::v3cUnitPayload( PCCHighLevelSyntax &syntax,
                                         PCCBitstream       &bitstream,
                                         V3CUnitType         v3cUnitType ) {
  TRACE_BITSTREAM_IN( "%s(%s)", __func__, toString( v3cUnitType ).c_str() );
  auto &sps = syntax.getVps();
  if ( v3cUnitType == V3C_VPS ) {
    v3cParameterSet( sps, syntax, bitstream );
  } else if ( v3cUnitType == V3C_AD ) {
    atlasSubStream( syntax, bitstream );
  } else if ( v3cUnitType == V3C_OVD || v3cUnitType == V3C_GVD || v3cUnitType == V3C_AVD || v3cUnitType == V3C_PVD ) {
    videoSubStream( syntax, bitstream, v3cUnitType );
  }
  TRACE_BITSTREAM_OUT( "%s(%s)", __func__, toString( v3cUnitType ).c_str() );
}

// 8.3.2.4 Atlas sub-bitstream syntax
void PCCBitstreamWriter::atlasSubStream( PCCHighLevelSyntax &syntax, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  SampleStreamNalUnit                ssnu;
  uint32_t                           maxUnitSize = 0;
  PCCBitstream                       tempBitStream;
  std::vector<uint32_t>              aspsSizeList;
  std::vector<uint32_t>              afpsSizeList;
  std::vector<uint32_t>              aapsSizeList;
  std::vector<std::vector<uint32_t>> seiPrefixSizeList;
  std::vector<std::vector<uint32_t>> seiSuffixSizeList;
  std::vector<uint32_t>              atglSizeList;
  const size_t                       atglSize = syntax.getAtlasTileLayerList().size();
  aspsSizeList.resize( syntax.getAtlasSequenceParameterSetList().size() );
  afpsSizeList.resize( syntax.getAtlasFrameParameterSetList().size() );
  aapsSizeList.resize( syntax.getAtlasAdaptationParameterSetList().size() );
  atglSizeList.resize( atglSize );
  seiPrefixSizeList.resize( atglSize );
  seiSuffixSizeList.resize( atglSize );

  uint32_t lastSize      = 0;
  size_t   nalHeaderSize = 2;
  int8_t   aspsIdx       = syntax.getAtlasHighLevelSyntax( 0 ).getActiveASPS();  // TODO: get correct value of atlas id
  atlasSequenceParameterSetRbsp( syntax.getAtlasSequenceParameterSet( aspsIdx ), syntax, tempBitStream );
  aspsSizeList[aspsIdx] = tempBitStream.size() - lastSize + nalHeaderSize;
  lastSize              = tempBitStream.size();
  if ( maxUnitSize < aspsSizeList[aspsIdx] ) { maxUnitSize = aspsSizeList[aspsIdx]; }

  int8_t afpsIdx = syntax.getAtlasHighLevelSyntax( 0 ).getActiveAFPS();  // TODO: get correct value of atlas id
  atlasFrameParameterSetRbsp( syntax.getAtlasFrameParameterSet( afpsIdx ), syntax, tempBitStream );
  afpsSizeList[afpsIdx] = tempBitStream.size() - lastSize + nalHeaderSize;
  lastSize              = tempBitStream.size();
  if ( maxUnitSize < afpsSizeList[afpsIdx] ) { maxUnitSize = afpsSizeList[afpsIdx]; }

  int8_t aapsIdx = syntax.getAtlasHighLevelSyntax( 0 ).getActiveAAPS();  // TODO: get correct value of atlas id
  if ( syntax.getAtlasAdaptationParameterSetList().size() > 0 ) {
    atlasAdaptationParameterSetRbsp( syntax.getAtlasAdaptationParameterSet( aapsIdx ), tempBitStream );
    aapsSizeList[aapsIdx] = tempBitStream.size() - lastSize + nalHeaderSize;
    lastSize              = tempBitStream.size();
    if ( maxUnitSize < aapsSizeList[aapsIdx] ) { maxUnitSize = aapsSizeList[aapsIdx]; }
  }

  for ( size_t atglIndex = 0; atglIndex < atglSize; atglIndex++ ) {
    auto &atgl = syntax.getAtlasTileLayer( atglIndex );
    seiPrefixSizeList[atglIndex].resize( atgl.getSEI().getSeiPrefix().size() );
    seiSuffixSizeList[atglIndex].resize( atgl.getSEI().getSeiSuffix().size() );
    // PREFIX SEI MESSAGE
    for ( size_t i = 0; i < atgl.getSEI().getSeiPrefix().size(); i++ ) {
      seiRbsp( syntax, tempBitStream, atgl.getSEI().getSeiPrefix( i ), NAL_PREFIX_ESEI, atglIndex );
      seiPrefixSizeList[atglIndex][i] = tempBitStream.size() - lastSize + nalHeaderSize;
      lastSize                        = tempBitStream.size();
      if ( maxUnitSize < seiPrefixSizeList[atglIndex][i] ) { maxUnitSize = seiPrefixSizeList[atglIndex][i]; }
    }

    // ATLAS TILE LAYER DATA
    atlasTileLayerRbsp( syntax.getAtlasTileLayer( atglIndex ), syntax, NAL_SKIP_R, tempBitStream );
    atglSizeList[atglIndex] = tempBitStream.size() - lastSize + nalHeaderSize;
    lastSize                = tempBitStream.size();
    if ( maxUnitSize < atglSizeList[atglIndex] ) { maxUnitSize = atglSizeList[atglIndex]; }

    // SUFFIX SEI MESSAGE
    for ( size_t i = 0; i < atgl.getSEI().getSeiSuffix().size(); i++ ) {
      seiRbsp( syntax, tempBitStream, atgl.getSEI().getSeiSuffix( i ), NAL_SUFFIX_ESEI, atglIndex );
      seiSuffixSizeList[atglIndex][i] = tempBitStream.size() - lastSize + nalHeaderSize;
      lastSize                        = tempBitStream.size();
      if ( maxUnitSize < seiSuffixSizeList[atglIndex][i] ) { maxUnitSize = seiSuffixSizeList[atglIndex][i]; }
    }
  }
  // calculation of the max unit size done
  // TRACE_BITSTREAM( "maxUnitSize                 = %u\n", maxUnitSize );
  // TRACE_BITSTREAM( "ceilLog2( maxUnitSize + 1 ) = %d\n", ceilLog2( maxUnitSize + 1 ) );
  // TRACE_BITSTREAM( "ceil( static_cast<double>( ceilLog2( maxUnitSize + 1 ) ) / 8.0 ) ) = %f\n",
  //                  ceil( static_cast<double>( ceilLog2( maxUnitSize + 1 ) ) / 8.0 ) );
  uint32_t precision = static_cast<uint32_t>(
      min( max( static_cast<int>( ceil( static_cast<double>( ceilLog2( maxUnitSize + 1 ) ) / 8.0 ) ), 1 ), 8 ) - 1 );
  ssnu.setSizePrecisionBytesMinus1( precision );
  sampleStreamNalHeader( bitstream, ssnu );
  // for ( size_t aspsCount = 0; aspsCount < syntax.getAtlasSequenceParameterSetList().size(); aspsCount++ )
  {
    int8_t  aspsIdx = syntax.getAtlasHighLevelSyntax( 0 ).getActiveASPS();
    NalUnit nu( NAL_ASPS, 0, 1 );
    nu.setSize( aspsSizeList[aspsIdx] );
    sampleStreamNalUnit( syntax, bitstream, ssnu, nu, aspsIdx );
    //    TRACE_BITSTREAM(
    //        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
    //        "written: %llu\n",
    //        (int)nu.getType(), toString( nu.getType() ).c_str(), ( ssnu.getSizePrecisionBytesMinus1() + 1 ),
    //        nu.getSize(), bitstream.size() );
  }
  // for ( size_t afpsCount = 0; afpsCount < syntax.getAtlasFrameParameterSetList().size(); afpsCount++ )
  {
    int8_t  afpsIdx = syntax.getAtlasHighLevelSyntax( 0 ).getActiveAFPS();
    NalUnit nu( NAL_AFPS, 0, 1 );
    nu.setSize( afpsSizeList[afpsIdx] );
    sampleStreamNalUnit( syntax, bitstream, ssnu, nu, afpsIdx );
    //    TRACE_BITSTREAM(
    //        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
    //        "written: %llu\n",
    //        (int)nu.getType(), toString( nu.getType() ).c_str(), ( ssnu.getSizePrecisionBytesMinus1() + 1 ),
    //        nu.getSize(), bitstream.size() );
  }

  // for ( size_t aapsCount = 0; aapsCount < syntax.getAtlasAdaptationParameterSetList().size(); aapsCount++ )
  if ( aapsSizeList.size() > 0 ) {
    int8_t  aapsIdx = syntax.getAtlasHighLevelSyntax( 0 ).getActiveAAPS();
    NalUnit nu( NAL_AAPS, 0, 1 );
    nu.setSize( aapsSizeList[aapsIdx] );
    sampleStreamNalUnit( syntax, bitstream, ssnu, nu, aapsIdx );
    //    TRACE_BITSTREAM(
    //        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
    //        "written: %llu\n",
    //        (int)nu.getType(), toString( nu.getType() ).c_str(), ( ssnu.getSizePrecisionBytesMinus1() + 1 ),
    //        nu.getSize(), bitstream.size() );
  }

  // NAL_TRAIL, NAL_TSA, NAL_STSA, NAL_RADL, NAL_RASL,NAL_SKIP
  for ( size_t atglIndex = 0; atglIndex < atglSize; atglIndex++ ) {
    auto &atgl = syntax.getAtlasTileLayer( atglIndex );

    // NAL_PREFIX_SEI
    for ( size_t i = 0; i < atgl.getSEI().getSeiPrefix().size(); i++ ) {
      // NOKIA: it should be check if it is N or E SEI
      NalUnit nu( NAL_PREFIX_NSEI, 0, 1 );
      nu.setSize( seiPrefixSizeList[atglIndex][i] );
      sampleStreamNalUnit( syntax, bitstream, ssnu, nu, i, atglIndex );
      //      TRACE_BITSTREAM(
      //          "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream "
      //          "written: %llu\n",
      //          (int)nu.getType(), toString( nu.getType() ).c_str(), ( ssnu.getSizePrecisionBytesMinus1() + 1 ),
      //          nu.getSize(), bitstream.size() );
    }

    // ATLAS_TILE_LAYER_RBSP
    auto       &atgh     = atgl.getHeader();
    NalUnitType naluType = NAL_IDR_N_LP;
    if ( atgh.getTileNaluTypeInfo() == 1 ) {
      naluType = NAL_TRAIL_R;
    } else if ( atgh.getTileNaluTypeInfo() == 2 ) {
      naluType = NAL_TRAIL_N;
    }
    NalUnit nu( naluType, 0, 1 );
    nu.setSize( atglSizeList[atglIndex] );  //+headsize
    auto &atl = syntax.getAtlasTileLayer( atglIndex );
    atl.getDataUnit().setTileOrder( atglIndex );
    sampleStreamNalUnit( syntax, bitstream, ssnu, nu, atglIndex );
    //    TRACE_BITSTREAM(
    //        "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream written: %llu\n",
    //        (int)nu.getType(), toString( nu.getType() ).c_str(), ( ssnu.getSizePrecisionBytesMinus1() + 1 ),
    //        nu.getSize(), bitstream.size() );

    // NAL_SUFFIX_SEI
    for ( size_t i = 0; i < atgl.getSEI().getSeiSuffix().size(); i++ ) {
      NalUnit nu( NAL_SUFFIX_ESEI, 0, 1 );
      nu.setSize( seiSuffixSizeList[atglIndex][i] );
      sampleStreamNalUnit( syntax, bitstream, ssnu, nu, i, atglIndex );
      //      TRACE_BITSTREAM(
      //          "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream written: %llu\n",
      //          (int)nu.getType(), toString( nu.getType() ).c_str(), ( ssnu.getSizePrecisionBytesMinus1() + 1 ),
      //          nu.getSize(), bitstream.size() );
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.3 Byte alignment syntax
void PCCBitstreamWriter::byteAlignment( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  uint8_t alignment_bit_equal_to_one  = 1;
  uint8_t alignment_bit_equal_to_zero = 0;
  WRITE_CODE( alignment_bit_equal_to_one, 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    WRITE_CODE( alignment_bit_equal_to_zero, 1 );  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4 V3C parameter set syntax
// 8.3.4.1 General V3C parameter set syntax
void PCCBitstreamWriter::v3cParameterSet( V3CParameterSet &vps, PCCHighLevelSyntax &syntax, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  profileTierLevel( vps.getProfileTierLevel(), bitstream );
  uint8_t vps_reserved_zero_8bits = 0;
  WRITE_CODE( vps.getV3CParameterSetId(), 4 );  // u(4)
  WRITE_CODE( vps_reserved_zero_8bits, 8 );     // u(8)
  WRITE_CODE( vps.getAtlasCountMinus1(), 6 );   // u(6)
  for ( uint32_t j = 0; j < vps.getAtlasCountMinus1() + 1; j++ ) {
    // TRACE_BITSTREAM( "Atlas = %zu \n", j );
    WRITE_CODE( vps.getAtlasId( j ), 6 );         // u(6)
    WRITE_UVLC( vps.getFrameWidth( j ) );         // ue(v)
    WRITE_UVLC( vps.getFrameHeight( j ) );        // ue(v)
    WRITE_CODE( vps.getMapCountMinus1( j ), 4 );  // u(4)
    // TRACE_BITSTREAM( " MapCountMinus1 = %zu \n", vps.getMapCountMinus1( j ) );
    if ( vps.getMapCountMinus1( j ) > 0 ) {
      WRITE_CODE( vps.getMultipleMapStreamsPresentFlag( j ), 1 );  // u(1)
      // TRACE_BITSTREAM( "MultipleMapStreamsPresentFlag = %zu \n", vps.getMultipleMapStreamsPresentFlag( j ) );
    }
    for ( size_t i = 1; i <= vps.getMapCountMinus1( j ); i++ ) {
      if ( vps.getMultipleMapStreamsPresentFlag( j ) ) {
        WRITE_CODE( vps.getMapAbsoluteCodingEnableFlag( j, i ), 1 );  // u(1)
      }
      // TRACE_BITSTREAM( " AbsoluteCoding Map%zu = %zu \n", i, vps.getMapAbsoluteCodingEnableFlag( j, i ) );
      if ( static_cast<int>( vps.getMapAbsoluteCodingEnableFlag( j, i ) ) == 0 ) {
        WRITE_UVLC( vps.getMapPredictorIndexDiff( j, i ) );
        // TRACE_BITSTREAM( " PredictorIndex L%zu = %zu \n", i, vps.getMapPredictorIndexDiff( j, i ) );
      }
    }
    WRITE_CODE( vps.getAuxiliaryVideoPresentFlag( j ), 1 );  // u(1)
    WRITE_CODE( vps.getOccupancyVideoPresentFlag( j ), 1 );  // u(1)
    WRITE_CODE( vps.getGeometryVideoPresentFlag( j ), 1 );   // u(1)
    WRITE_CODE( vps.getAttributeVideoPresentFlag( j ), 1 );  // u(1)
#ifdef BITSTREAM_TRACE
    for ( size_t i = 0; i < vps.getMapCountMinus1( j ) + 1; i++ ) {
      // TRACE_BITSTREAM( " AbsoluteCoding L%zu = %zu \n", i, vps.getMapAbsoluteCodingEnableFlag( j, i ) );
    }
#endif
    if ( vps.getOccupancyVideoPresentFlag( j ) ) {
      occupancyInformation( vps.getOccupancyInformation( j ), bitstream );
    }
    if ( vps.getGeometryVideoPresentFlag( j ) ) {
      geometryInformation( vps.getGeometryInformation( j ), vps, bitstream );
    }
    if ( vps.getAttributeVideoPresentFlag( j ) ) {
      attributeInformation( vps.getAttributeInformation( j ), vps, bitstream );
    }
  }
  WRITE_CODE( vps.getExtensionPresentFlag(), 1 );  // u(1)

  if ( vps.getExtensionPresentFlag() ) {
    WRITE_CODE( vps.getExtensionCount(), 8 );  // u(8)
  }
  if ( vps.getExtensionCount() ) {
    auto extensionsLengthMinus1 = uint32_t{ 3 } * vps.getExtensionCount() - 1;
    auto extensionLength        = std::vector<uint16_t>( vps.getExtensionCount() );

    // Write the VPS extensions only to measure their length
    for ( uint8_t i = 0; i < vps.getExtensionCount(); ++i ) {
      auto counter = PCCBitstream{};
      vpsExtension( vps.getExtensionType( i ), vps, counter );
      extensionLength[i] = static_cast<uint16_t>( counter.size() );
      extensionsLengthMinus1 += extensionLength[i];
    }
    WRITE_UVLC( extensionsLengthMinus1 );  // ue(v)

    // Write the VPS extensions for real, including headers
    for ( uint8_t i = 0; i < vps.getExtensionCount(); ++i ) {
      WRITE_CODE( vps.getExtensionType( i ), 8 );  // u(8)
      WRITE_CODE( extensionLength[i], 16 );        // u(16)
      vpsExtension( vps.getExtensionType( i ), vps, bitstream );
    }
  }
  byteAlignment( bitstream );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4.2 Profile, tier, and level syntax
void PCCBitstreamWriter::profileTierLevel( ProfileTierLevel &ptl, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  constexpr uint8_t  reservedZero6Bits         = 0;
  constexpr uint16_t ptl_reserved_0xfff_12bits = 0xFFF;
  WRITE_CODE( ptl.getTierFlag(), 1 );                  // u(1)
  WRITE_CODE( ptl.getProfileCodecGroupIdc(), 7 );      // u(7)
  WRITE_CODE( ptl.getProfileToolsetIdc(), 8 );         // u(8)
  WRITE_CODE( ptl.getProfileReconstructionIdc(), 8 );  // u(8)
  WRITE_CODE( ptl.getAttributeTierFlag(), 1 );         // u(1)
  WRITE_CODE( ptl.getAttributeLevelIdc(), 8 );         // u(8)
  WRITE_CODE( reservedZero6Bits, 6 );                  // u(6)
  WRITE_CODE( ptl.getPriorityOrderEnableFlag(), 1 );   // u(1)
  WRITE_CODE( ptl.getMaxDecodesIdc(), 4 );             // u(4)
  WRITE_CODE( ptl_reserved_0xfff_12bits, 12 );         // u(12)
  WRITE_CODE( ptl.getLevelIdc(), 8 );                  // u(8)
  WRITE_CODE( ptl.getNumSubProfiles(), 6 );            // u(6)
  WRITE_CODE( ptl.getExtendedSubProfileFlag(), 1 );    // u(1)
  for ( size_t i = 0; i < ptl.getNumSubProfiles(); i++ ) {
    size_t v = ptl.getExtendedSubProfileFlag() == 0 ? 32 : 64;
    WRITE_CODE( ptl.getSubProfileIdc( i ), v );  // u(v)
  }
  WRITE_CODE( ptl.getToolConstraintsPresentFlag(), 1 );  // u(1)
  if ( ptl.getToolConstraintsPresentFlag() ) {
    profileToolsetConstraintsInformation( ptl.getProfileToolsetConstraintsInformation(), bitstream );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4.3 Occupancy parameter set syntax
void PCCBitstreamWriter::occupancyInformation( OccupancyInformation &oi, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( oi.getOccupancyCodecId(), 8 );                    // u(8)
  WRITE_CODE( oi.getLossyOccupancyCompressionThreshold(), 8 );  // u(8)
  WRITE_CODE( oi.getOccupancy2DBitdepthMinus1(), 5 );           // u(5)
  WRITE_CODE( oi.getOccupancyMSBAlignFlag(), 1 );               // u(1)
  // TRACE_BITSTREAM( "  OccupancyLossyThreshold = %d  \n", oi.getLossyOccupancyCompressionThreshold() );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4.4 Geometry parameter set syntax
void PCCBitstreamWriter::geometryInformation( GeometryInformation &gi, V3CParameterSet &vps, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  size_t atlasIndex = 0;
  WRITE_CODE( gi.getGeometryCodecId(), 8 );                      // u(8)
  WRITE_CODE( gi.getGeometry2dBitdepthMinus1(), 5 );             // u(5)
  WRITE_CODE( gi.getGeometryMSBAlignFlag(), 1 );                 // u(1)
  WRITE_CODE( gi.getGeometry3dCoordinatesBitdepthMinus1(), 5 );  // u(5)
  if ( vps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
    WRITE_CODE( gi.getAuxiliaryGeometryCodecId(), 8 );  // u(8)
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4.5 Attribute information
void PCCBitstreamWriter::attributeInformation( AttributeInformation &ai,
                                               V3CParameterSet      &sps,
                                               PCCBitstream         &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  size_t atlasIndex = 0;
  WRITE_CODE( ai.getAttributeCount(), 7 );  // u(7)
  // TRACE_BITSTREAM( "AttributeCount = %u  \n", ai.getAttributeCount() );
  for ( uint32_t i = 0; i < ai.getAttributeCount(); i++ ) {
    WRITE_CODE( ai.getAttributeTypeId( i ), 4 );   // u(4)
    WRITE_CODE( ai.getAttributeCodecId( i ), 8 );  // u(8)
    if ( sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
      WRITE_CODE( ai.getAuxiliaryAttributeCodecId( i ), 8 );  // u(8)
    }
    if ( sps.getMapCountMinus1( atlasIndex ) > 0 ) {
      WRITE_CODE( ai.getAttributeMapAbsoluteCodingPersistenceFlag( i ), 1 );  // u(1)
    }
    WRITE_CODE( ai.getAttributeDimensionMinus1( i ), 6 );  // u(6)
    if ( ai.getAttributeDimensionMinus1( i ) > 0 ) {
      WRITE_CODE( ai.getAttributeDimensionPartitionsMinus1( i ), 6 );  // u(6)
      int32_t remainingDimensions = ai.getAttributeDimensionMinus1( i );
      int32_t k                   = ai.getAttributeDimensionPartitionsMinus1( i );
      for ( int32_t j = 0; j < k; j++ ) {
        if ( k - j != remainingDimensions ) {
          WRITE_UVLC( static_cast<uint32_t>( ai.getAttributePartitionChannelsMinus1( i, j ) ) );  // ue(v)
        }
        remainingDimensions -= ai.getAttributePartitionChannelsMinus1( i, j ) + 1;
      }
    }
    WRITE_CODE( ai.getAttribute2dBitdepthMinus1( i ), 5 );  // u(5)
    WRITE_CODE( ai.getAttributeMSBAlignFlag( i ), 1 );      // u(1)
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4.6	Profile toolset constraints information syntax
void PCCBitstreamWriter::profileToolsetConstraintsInformation( ProfileToolsetConstraintsInformation &ptci,
                                                               PCCBitstream                         &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( ptci.getOneFrameOnlyFlag(), 1 );                         // u(1)
  WRITE_CODE( ptci.getEOMContraintFlag(), 1 );                         // u(1)
  WRITE_CODE( ptci.getMaxMapCountMinus1(), 4 );                        // u(4)
  WRITE_CODE( ptci.getMaxAtlasCountMinus1(), 4 );                      // u(4)
  WRITE_CODE( ptci.getMultipleMapStreamsConstraintFlag(), 1 );         // u(1)
  WRITE_CODE( ptci.getPLRConstraintFlag(), 1 );                        // u(1)
  WRITE_CODE( ptci.getAttributeMaxDimensionMinus1(), 6 );              // u(6)
  WRITE_CODE( ptci.getAttributeMaxDimensionPartitionsMinus1(), 6 );    // u(6)
  WRITE_CODE( ptci.getNoEightOrientationsConstraintFlag(), 1 );        // u(1)
  WRITE_CODE( ptci.getNo45DegreeProjectionPatchConstraintFlag(), 1 );  // u(1)
  WRITE_CODE( 0, 6 );                                                  // u(6)
  WRITE_CODE( ptci.getNumReservedConstraintBytes(), 8 );               // u(8)
  for ( size_t i = 0; i < ptci.getNumReservedConstraintBytes(); i++ ) {
    WRITE_CODE( ptci.getReservedConstraintByte( i ), 8 );  // u(8)
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4.7 Packing information syntax
void PCCBitstreamWriter::packingInformation( PackingInformation &pin,
                                             uint8_t             j,
                                             uint8_t             version,
                                             V3CParameterSet    &vps,
                                             PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );

  const auto &pve = vps.getVpsPackedVideoExtension();

  WRITE_CODE( pin.getCodecId(), 8 );               // u(8)
  WRITE_CODE( pin.getOccupancyPresentFlag(), 1 );  // u(1)
  WRITE_CODE( pin.getGeometryPresentFlag(), 1 );   // u(1)
  WRITE_CODE( pin.getAttributePresentFlag(), 1 );  // u(1)

  if ( pin.getOccupancyPresentFlag() && vps.getOccupancyVideoPresentFlag( j ) ) {
    std::cerr << "It is a requirement of bitstream conformance that if pin_occupancy_present_flag[ j ] is equal to 1 "
                 "for an atlas with atlas ID j, vps_occupancy_video_present_flag[ j ] shall be equal to 0 for an atlas "
                 "with the same atlas ID j.\n";
    exit( -1 );
  }
  if ( pin.getGeometryPresentFlag() && vps.getGeometryVideoPresentFlag( j ) ) {
    std::cerr << "It is a requirement of bitstream conformance that if vps_geometry_video_present_flag[ j ] is equal "
                 "to 1 for an atlas with atlas ID j, pin_geometry_present_flag[ j ] shall be equal to 0 for an atlas "
                 "with the same atlas ID j.\n";
    exit( -1 );
  }
  if ( pin.getAttributePresentFlag() && vps.getAttributeVideoPresentFlag( j ) ) {
    std::cerr << "It is a requirement of bitstream conformance that if vps_attribute_video_present_flag[ j ] is equal "
                 "to 1 for an atlas with atlas ID j, pin_attribute_present_flag[ j ] shall be equal to 0 for an atlas "
                 "with the same atlas ID j.\n";
    exit( -1 );
  }

  if ( pin.getOccupancyPresentFlag() ) {
    auto &oi = vps.getOccupancyInformation( j );
    WRITE_CODE( oi.getOccupancy2DBitdepthMinus1(), 5 );           // u(5)
    WRITE_CODE( oi.getOccupancyMSBAlignFlag(), 1 );               // u(1)
    WRITE_CODE( oi.getLossyOccupancyCompressionThreshold(), 8 );  // u(8)
  }
  if ( pin.getGeometryPresentFlag() ) {
    auto &gi = vps.getGeometryInformation( j );
    WRITE_CODE( gi.getGeometry2dBitdepthMinus1(), 5 );             // u(5)
    WRITE_CODE( gi.getGeometryMSBAlignFlag(), 1 );                 // u(1)
    WRITE_CODE( gi.getGeometry3dCoordinatesBitdepthMinus1(), 5 );  // u(5)
  }
  if ( pin.getAttributePresentFlag() ) {
    auto &ai = vps.getAttributeInformation( j );
    WRITE_CODE( ai.getAttributeCount(), 7 );  // u(7)
    for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
      WRITE_CODE( ai.getAttributeTypeId( i ), 4 );                            // u(4)
      WRITE_CODE( ai.getAttribute2dBitdepthMinus1( i ), 5 );                  // u(5)
      WRITE_CODE( ai.getAttributeMSBAlignFlag( i ), 1 );                      // u(1)
      WRITE_CODE( ai.getAttributeMapAbsoluteCodingPersistenceFlag( i ), 1 );  // u(1)
      uint8_t d = ai.getAttributeDimensionMinus1( i );
      WRITE_CODE( d, 6 );  // u(6)
      uint8_t m;
      if ( d == 0 ) {
        // pin_attribute_dimension_partitions_minus1[ j ][ i ] = 0
        m = 0;
      } else {
        m = ai.getAttributeDimensionPartitionsMinus1( i );
        WRITE_CODE( m, 6 );  // u(6)
      }
      for ( uint8_t k = 0; k < m; k++ ) {
        uint8_t n;
        if ( k + d == m ) {
          // pin_attribute_partition_channels_minus1[ j ][ i ][ k ] = 0
          n = 0;
        } else {
          n = ai.getAttributePartitionChannelsMinus1( i, k );
          WRITE_UVLC( n );  // ue(v)
        }
        d -= n + 1;
      }
      // pin_attribute_partition_channels_minus1[ j ][ i ][ m ] = d
    }
  }
  WRITE_UVLC( pin.getRegionsCountMinus1() );  // ue(v)

  if ( pve.getMultiplePackedMapsEnabledFlag() ) {
    WRITE_CODE( pin.getPackedMapCountMinus1(), ceilLog2( pin.getRegionsCountMinus1() + 1 ) );  // u(v)
  }
  if ( pve.getTemporalPackingEnabledFlag() ) {
    for ( uint8_t k = 0; k <= pin.getPackedMapCountMinus1(); ++k ) {
      WRITE_CODE( pin.getConstituentFrameCountMinus1( k ), ceilLog2( pin.getRegionsCountMinus1() + 1 ) );  // u(v)
    }
  }
  if ( pve.getImplicitPackingMode() == ImplicitPackingMode::implicit_mode ) {
    for ( uint8_t k = 0; k <= pin.getPackedMapCountMinus1(); ++k ) {
      WRITE_UVLC( pin.getPackedMapGridColumnsMinus1( k ) );  // ue(v)
      WRITE_UVLC( pin.getPackedMapGridRowsMinus1( k ) );     // ue(v)
    }
  }
  if ( version >= 2 ) {
    WRITE_CODE( pin.getLodEnabledFlag(), 1 );  // u(1)
    if ( pin.getLodEnabledFlag() ) {
      WRITE_CODE( pin.getLodLevelCountMinus2(), 2 );  // u(2)
    }
  }
  for ( size_t i = 0; i <= pin.getRegionsCountMinus1(); i++ ) {
    packedRegion( pin.getRegion( i ), pin, j, vps, bitstream );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

void PCCBitstreamWriter::packedRegion( PackedRegion             &region,
                                       const PackingInformation &pin,
                                       uint8_t                   j,
                                       V3CParameterSet          &vps,
                                       PCCBitstream             &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );

  const auto &pve = vps.getVpsPackedVideoExtension();

  if ( pin.getLodEnabledFlag() ) {
    uint8_t maskBits = pin.getLodLevelCountMinus2() + 2;
    WRITE_CODE( region.getRegionLodMask(), maskBits );  // u(v)
  }
  if ( pin.getPackedMapCountMinus1() > 0 ) {
    WRITE_CODE( region.getPackedMapIndex(), ceilLog2( pin.getPackedMapCountMinus1() + 1 ) );  // u(v)
  }
  const auto k = region.getPackedMapIndex();
  assert( k <= pin.getPackedMapCountMinus1() );
  if ( pin.getConstituentFrameCountMinus1( k ) > 0 ) {
    WRITE_CODE( region.getConstituentFrameIndex(), ceilLog2( pin.getConstituentFrameCountMinus1( k ) + 1 ) );  // u(v)
  }
  WRITE_CODE( region.getTileId(), 8 );        // u(8)
  WRITE_CODE( region.getTypeIdMinus2(), 2 );  // u(2)

  if ( pve.getImplicitPackingMode() == ImplicitPackingMode::explicit_mode ) {
    WRITE_CODE( region.getTopLeftX(), 16 );        // u(16)
    WRITE_CODE( region.getTopLeftY(), 16 );        // u(16)
    WRITE_CODE( region.getWidthMinus1(), 16 );     // u(16)
    WRITE_CODE( region.getHeightMinus1(), 16 );    // u(16)
    WRITE_CODE( region.getUnpackTopLeftX(), 16 );  // u(16)
    WRITE_CODE( region.getUnpackTopLeftY(), 16 );  // u(16)
    WRITE_CODE( region.getRotationFlag(), 1 );     // u(1)
  }

  const auto typeId = region.getTypeIdMinus2() + 2;

  if ( typeId == V3C_AVD || typeId == V3C_GVD ) {
    WRITE_CODE( region.getMapIndex(), 4 );           // u(4)
    WRITE_CODE( region.getAuxiliaryDataFlag(), 1 );  // u(1)
  }
  if ( typeId == V3C_AVD ) {
    WRITE_CODE( region.getAttrIndex(), 7 );  // u(7)
    size_t k  = region.getAttrIndex();
    auto  &ai = vps.getAttributeInformation( j );
    if ( ai.getAttributeDimensionMinus1( k ) > 0 ) {
      if ( pve.getImplicitPackingMode() == ImplicitPackingMode::explicit_mode )
        WRITE_CODE( region.getAttrPartitionIndex(), 5 );  // u(5)
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4.8 VPS extension syntax
void PCCBitstreamWriter::vpsExtension( uint8_t extensionType, V3CParameterSet &vps, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  const auto bitOffset = bitstream.getPosition().bits_;

  if ( extensionType == VPS_EXT_PACKED ) {
    vpsPackedVideoExtension( vps.getVpsPackedVideoExtension(), 1, vps, bitstream );
  } else if ( extensionType == VPS_EXT_PACKED_2 ) {
    vpsPackedVideoExtension( vps.getVpsPackedVideoExtension(), 2, vps, bitstream );
  } else {
    std::cerr << "Error: Unsupported VPS extension type " << static_cast<int>( extensionType ) << std::endl;
    exit( -1 );
  }

  lengthAlignment( bitOffset, bitstream );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4.9 VPS packed video extension syntax
void PCCBitstreamWriter::vpsPackedVideoExtension( VpsPackedVideoExtension &pve,
                                                  uint8_t                  version,
                                                  V3CParameterSet         &vps,
                                                  PCCBitstream            &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );

  if ( version >= 2 ) {
    WRITE_CODE( pve.getTemporalPackingEnabledFlag(), 1 );  // u(1)
    if ( pve.getTemporalPackingEnabledFlag() ) {
      WRITE_UVLC( static_cast<uint32_t>( pve.getTemporalPackingMode() ) );  // ue(v)
    }
    WRITE_CODE( pve.getMultiplePackedMapsEnabledFlag(), 1 );              // u(1)
    WRITE_UVLC( static_cast<uint32_t>( pve.getImplicitPackingMode() ) );  // ue(v)
  }

  for ( uint8_t k = 0; k <= vps.getAtlasCountMinus1(); ++k ) {
    const auto j = vps.getAtlasId( k );
    WRITE_CODE( pve.getPackedVideoPresentFlag( j ), 1 );  // u(1)

    if ( pve.getPackedVideoPresentFlag( j ) ) {
      packingInformation( pve.getPackingInformation( k ), j, version, vps, bitstream );
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.4.10 Length alignment syntax
void PCCBitstreamWriter::lengthAlignment( uint8_t bitOffset, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );

  while ( bitstream.getPosition().bits_ != bitOffset ) {
    static constexpr auto length_alignment_bit_equal_to_zero = 0;
    WRITE_CODE( length_alignment_bit_equal_to_zero, 1 );  // f(1)
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.5 NAL unit syntax
// 8.3.5.1 General NAL unit syntax
void PCCBitstreamWriter::nalUnit( PCCBitstream &bitstream, NalUnit &nalUnit ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  nalUnitHeader( bitstream, nalUnit );
  for ( size_t i = 2; i < nalUnit.getSize(); i++ ) {
    WRITE_CODE( nalUnit.getData( i ), 8 );  // b(8)
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.5.2 NAL unit header syntax
void PCCBitstreamWriter::nalUnitHeader( PCCBitstream &bitstream, NalUnit &nalUnit ) {
  TRACE_BITSTREAM_IN( "%s(%s)", __func__, toString( nalUnit.getType() ).c_str() );
  uint8_t nal_forbidden_zero_bit = 0;
  WRITE_CODE( nal_forbidden_zero_bit, 1 );         // f(1)
  WRITE_CODE( nalUnit.getType(), 6 );              // u(6)
  WRITE_CODE( nalUnit.getLayerId(), 6 );           // u(6)
  WRITE_CODE( nalUnit.getTemporalyIdPlus1(), 3 );  // u(3)
  //  TRACE_BITSTREAM( " NalUnitSize      = %zu \n", nalUnit.getSize() );
  //  TRACE_BITSTREAM( " NalUnitType      = %hhu \n", nalUnit.getType() );
  //  TRACE_BITSTREAM( " LayerId          = %hhu \n", nalUnit.getLayerId() );
  //  TRACE_BITSTREAM( " TemporalyIdPlus1 = %hhu \n", nalUnit.getTemporalyIdPlus1() );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.6 Raw byte sequence payloads, trailing bits, and byte alignment syntax
// 8.3.6.1 Atlas sequence parameter set Rbsp
// 8.3.6.1.1 General Atlas sequence parameter set Rbsp
void PCCBitstreamWriter::atlasSequenceParameterSetRbsp( AtlasSequenceParameterSetRbsp &asps,
                                                        PCCHighLevelSyntax            &syntax,
                                                        PCCBitstream                  &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_UVLC( static_cast<uint32_t>( asps.getAtlasSequenceParameterSetId() ) );         // ue(v)
  WRITE_UVLC( asps.getFrameWidth() );                                                   // ue(v)
  WRITE_UVLC( asps.getFrameHeight() );                                                  // ue(v)
  WRITE_CODE( asps.getGeometry3dBitdepthMinus1(), 5 );                                  // u(5)
  WRITE_CODE( asps.getGeometry2dBitdepthMinus1(), 5 );                                  // u(5)
  WRITE_UVLC( static_cast<uint32_t>( asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() ) );  // ue(v)
  WRITE_UVLC( static_cast<uint32_t>( asps.getMaxDecAtlasFrameBufferingMinus1() ) );     // ue(v)
  WRITE_CODE( asps.getLongTermRefAtlasFramesFlag(), 1 );                                // u(1)
  WRITE_UVLC( static_cast<uint32_t>( asps.getNumRefAtlasFrameListsInAsps() ) );         // ue(v)
  // TRACE_BITSTREAM( "ASPS: NumRefListStruct = %u \n", asps.getNumRefAtlasFrameListsInAsps() );
  for ( size_t i = 0; i < asps.getNumRefAtlasFrameListsInAsps(); i++ ) {
    refListStruct( asps.getRefListStruct( i ), asps, bitstream );
  }
  WRITE_CODE( asps.getUseEightOrientationsFlag(), 1 );       // u(1)
  WRITE_CODE( asps.getExtendedProjectionEnabledFlag(), 1 );  // u(1)
  if ( asps.getExtendedProjectionEnabledFlag() ) {
    WRITE_UVLC( static_cast<uint32_t>( asps.getMaxNumberProjectionsMinus1() ) );  // ue(v)
  }
  WRITE_CODE( asps.getNormalAxisLimitsQuantizationEnabledFlag(),
              1 );                                                // u(1)
  WRITE_CODE( asps.getNormalAxisMaxDeltaValueEnabledFlag(), 1 );  // u(1)
  WRITE_CODE( asps.getPatchPrecedenceOrderFlag(), 1 );            // u(1)
  WRITE_CODE( asps.getLog2PatchPackingBlockSize(), 3 );           // u(3)
  // TRACE_BITSTREAM( "Log2PatchPackingBlockSize = %u \n", asps.getLog2PatchPackingBlockSize() );
  WRITE_CODE( asps.getPatchSizeQuantizerPresentFlag(), 1 );  // u(1)
  WRITE_CODE( asps.getMapCountMinus1(), 4 );                 // u(4)
  WRITE_CODE( asps.getPixelDeinterleavingFlag(), 1 );        // u(1)
  if ( asps.getPixelDeinterleavingFlag() ) {
    for ( size_t i = 0; i < asps.getMapCountMinus1() + 1; i++ ) {
      WRITE_CODE( asps.getPixelDeinterleavingMapFlag( i ), 1 );  // u(1)
    }
  }
  WRITE_CODE( asps.getRawPatchEnabledFlag(), 1 );  // u(1)
  WRITE_CODE( asps.getEomPatchEnabledFlag(), 1 );  // u(1)
  if ( asps.getEomPatchEnabledFlag() && asps.getMapCountMinus1() == 0 ) {
    WRITE_CODE( asps.getEomFixBitCountMinus1(), 4 );  // u(4)
  }
  if ( asps.getRawPatchEnabledFlag() || asps.getEomPatchEnabledFlag() ) {
    WRITE_CODE( asps.getAuxiliaryVideoEnabledFlag(), 1 );  // u(1)
    // TRACE_BITSTREAM( "getAuxiliaryVideoEnabledFlag = %u \n", asps.getAuxiliaryVideoEnabledFlag() );
  }
  WRITE_CODE( asps.getPLREnabledFlag(), 1 );  // u(1)
  if ( asps.getPLREnabledFlag() ) { plrInformation( asps, syntax, bitstream ); }
  WRITE_CODE( asps.getVuiParametersPresentFlag(), 1 );  // u(1)
  if ( asps.getVuiParametersPresentFlag() ) {
    vuiParameters( bitstream, asps.getVuiParameters(), asps.getGeometry3dBitdepthMinus1() );
  }

  WRITE_CODE( asps.getExtensionFlag(), 1 );  // u(1)
  if ( asps.getExtensionFlag() ) {
    WRITE_CODE( asps.getVpccExtensionFlag(), 1 );  // u(1)
    WRITE_CODE( asps.getExtension7Bits(), 7 );     // u(7)
  }
  if ( asps.getVpccExtensionFlag() ) { aspsVpccExtension( bitstream, asps, asps.getAspsVpccExtension() ); }
  if ( asps.getExtension7Bits() ) {
    uint8_t asps_extension_data_flag = 0;
    while ( moreRbspData( bitstream ) ) {
      WRITE_CODE( asps_extension_data_flag, 1 );  // u(1)
    }
  }
  rbspTrailingBits( bitstream );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.6.1.2 Point local reconstruction information syntax
void PCCBitstreamWriter::plrInformation( AtlasSequenceParameterSetRbsp &asps,
                                         PCCHighLevelSyntax            &syntax,
                                         PCCBitstream                  &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "  MapCountMinus1() = %u \n", asps.getMapCountMinus1() );
  for ( size_t j = 0; j < asps.getMapCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "  PLR info map %zu \n", j );
    auto &plri = asps.getPLRInformation( j );
    WRITE_CODE( plri.getMapEnabledFlag(), 1 );  // u(1)
    if ( plri.getMapEnabledFlag() ) {
      WRITE_CODE( plri.getNumberOfModesMinus1(), 4 );  // u(4)
      TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plri.getNumberOfModesMinus1() );
      for ( size_t i = 0; i < plri.getNumberOfModesMinus1() + 1; i++ ) {
        WRITE_CODE( plri.getInterpolateFlag( i ), 1 );  // u(1)
        WRITE_CODE( plri.getFillingFlag( i ), 1 );      // u(1)
        WRITE_CODE( plri.getMinimumDepth( i ), 2 );     // u(2)
        WRITE_CODE( plri.getNeighbourMinus1( i ), 2 );  // u(2)
        TRACE_BITSTREAM( "  Mode[%zu] = I = %d F = %d D = %u N = %u \n", i, plri.getInterpolateFlag( i ),
                         plri.getFillingFlag( i ), plri.getMinimumDepth( i ), plri.getNeighbourMinus1( i ) );
      }
      WRITE_CODE( plri.getBlockThresholdPerPatchMinus1(), 6 );  // u(6)
      TRACE_BITSTREAM( "  BlockThresholdPerPatchMinus1 = %u \n", plri.getBlockThresholdPerPatchMinus1() );
    }
  }
}

// 8.3.6.2 Atlas frame parameter set Rbsp syntax
// 8.3.6.2.1 General atlas frame parameter set Rbsp syntax
void PCCBitstreamWriter::atlasFrameParameterSetRbsp( AtlasFrameParameterSetRbsp &afps,
                                                     PCCHighLevelSyntax         &syntax,
                                                     PCCBitstream               &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  WRITE_UVLC( afps.getAtlasFrameParameterSetId() );     // ue(v)
  WRITE_UVLC( afps.getAtlasSequenceParameterSetId() );  // ue(v)
  auto &asps = syntax.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  atlasFrameTileInformationRbsp( afps.getAtlasFrameTileInformationRbsp(), asps, bitstream );
  WRITE_CODE( afps.getOutputFlagPresentFlag(), 1 );                // u(1)
  WRITE_UVLC( afps.getNumRefIdxDefaultActiveMinus1() );            // ue(v)
  WRITE_UVLC( afps.getAdditionalLtAfocLsbLen() );                  // ue(v)
  WRITE_CODE( afps.getLodModeEnableFlag(), 1 );                    // u(1)
  WRITE_CODE( afps.getRaw3dOffsetBitCountExplicitModeFlag(), 1 );  // u(1)
  WRITE_CODE( afps.getExtensionFlag(), 1 );                        // u(1)
  if ( afps.getExtensionFlag() ) {
    WRITE_CODE( afps.getExtension8Bits(), 8 );  // u(8)
  }
  if ( afps.getExtension8Bits() ) {
    uint8_t afps_extension_data_flag = 0;
    while ( moreRbspData( bitstream ) ) { WRITE_CODE( afps_extension_data_flag, 1 ); }  // u(1)
  }
  rbspTrailingBits( bitstream );
}

// 8.3.6.2.2 Atlas frame tile information RBSP syntax
void PCCBitstreamWriter::atlasFrameTileInformationRbsp( AtlasFrameTileInformationRbsp &afti,
                                                        AtlasSequenceParameterSetRbsp &asps,
                                                        PCCBitstream                  &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( afti.getSingleTileInAtlasFrameFlag(), 1 );  // u(1)
  if ( !afti.getSingleTileInAtlasFrameFlag() ) {
    WRITE_CODE( afti.getUniformPartitionSpacingFlag(), 1 );  // u(1)
    // TRACE_BITSTREAM( "afti: uniformPartition :%zu !singleTile\n", afti.getUniformPartitionSpacingFlag() );
    if ( afti.getUniformPartitionSpacingFlag() ) {
      WRITE_UVLC( afti.getPartitionColumnWidthMinus1( 0 ) );  //  ue(v)
      WRITE_UVLC( afti.getPartitionRowHeightMinus1( 0 ) );    //  ue(v)
      //      TRACE_BITSTREAM( "afti: aspsWidth :%zu, partitionWidth: %zu, Number of Partitions Hor: %zu\n",
      //                       asps.getFrameWidth(), afti.getPartitionColumnWidthMinus1( 0 ) + 1,
      //                       afti.getNumPartitionColumnsMinus1() + 1 );
      //      TRACE_BITSTREAM( "afti: aspsHeight :%zu, partitionHeight: %zu, Number of Partitions Ver: %zu\n",
      //                       asps.getFrameHeight(), afti.getPartitionRowHeightMinus1( 0 ) + 1,
      //                       afti.getNumPartitionRowsMinus1() + 1 );

    } else {
      WRITE_UVLC( afti.getNumPartitionColumnsMinus1() );  //  ue(v)
      WRITE_UVLC( afti.getNumPartitionRowsMinus1() );     //  ue(v)
      for ( size_t i = 0; i < afti.getNumPartitionColumnsMinus1(); i++ ) {
        WRITE_UVLC( afti.getPartitionColumnWidthMinus1( i ) );  //  ue(v)
      }
      for ( size_t i = 0; i < afti.getNumPartitionRowsMinus1(); i++ ) {
        WRITE_UVLC( afti.getPartitionRowHeightMinus1( i ) );  //  ue(v)
      }
      //      TRACE_BITSTREAM( "afti: aspsWidth :%zu, partitionWidth: ", asps.getFrameWidth() );
      //      for ( size_t i = 0; i < afti.getNumPartitionColumnsMinus1(); i++ ) {
      //        TRACE_BITSTREAM( "%d, ", afti.getPartitionColumnWidthMinus1( i ) );
      //      }
      //      TRACE_BITSTREAM( "Number of Partitions Hor: %zu\n", afti.getNumPartitionColumnsMinus1() + 1 );
      //
      //      TRACE_BITSTREAM( "afti: aspsHeight :%zu, partitionWidth: ", asps.getFrameHeight() );
      //      for ( size_t i = 0; i < afti.getNumPartitionRowsMinus1(); i++ ) {
      //        TRACE_BITSTREAM( "%d, ", afti.getPartitionRowHeightMinus1( i ) );
      //      }
      //      TRACE_BITSTREAM( "Number of Partitions Ver: %zu\n", afti.getNumPartitionRowsMinus1() + 1 );
    }
    WRITE_CODE( afti.getSinglePartitionPerTileFlag(), 1 );  //  u(1)
    if ( afti.getSinglePartitionPerTileFlag() == 0u ) {
      uint32_t NumPartitionsInAtlasFrame =
          ( afti.getNumPartitionColumnsMinus1() + 1 ) * ( afti.getNumPartitionRowsMinus1() + 1 );
      WRITE_UVLC( afti.getNumTilesInAtlasFrameMinus1() );  // ue(v)
      for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
        uint8_t bitCount = ceilLog2( NumPartitionsInAtlasFrame );
        WRITE_CODE( afti.getTopLeftPartitionIdx( i ), bitCount );     // u(v)
        WRITE_UVLC( afti.getBottomRightPartitionColumnOffset( i ) );  // ue(v)
        WRITE_UVLC( afti.getBottomRightPartitionRowOffset( i ) );     // ue(v)
      }
    }
    //    lse {
    //      TRACE_BITSTREAM( "afti: singlePartitionPerTileFlag = true : %zu\n", afti.getNumTilesInAtlasFrameMinus1() );
    //      for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
    //        TRACE_BITSTREAM( "afti: blocks:%zu,%zu,%zu\n", afti.getTopLeftPartitionIdx( i ),
    //                         afti.getBottomRightPartitionColumnOffset( i ), afti.getBottomRightPartitionRowOffset( i )
    //                         );
    //      }
    //    }
  }
  //  TRACE_BITSTREAM( "afti: singleTile :%zu\n", afti.getSingleTileInAtlasFrameFlag() );
  //  TRACE_BITSTREAM( "afti: uniformPartition :%zu\n", afti.getUniformPartitionSpacingFlag() );
  //  TRACE_BITSTREAM( "afti: numTilesInAtlasFrameMinus1 :%zu\n", afti.getNumTilesInAtlasFrameMinus1() );
  //  TRACE_BITSTREAM( "asps: getAuxiliaryVideoEnabledFlag :%zu\n", asps.getAuxiliaryVideoEnabledFlag() );
  if ( asps.getAuxiliaryVideoEnabledFlag() ) {
    WRITE_UVLC( afti.getAuxiliaryVideoTileRowWidthMinus1() );  // ue(v)
    for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
      WRITE_UVLC( afti.getAuxiliaryVideoTileRowHeight( i ) );  // ue(v)
    }
  }
  WRITE_CODE( afti.getSignalledTileIdFlag(), 1 );  // u(1)
  if ( afti.getSignalledTileIdFlag() ) {
    WRITE_UVLC( afti.getSignalledTileIdLengthMinus1() );  // ue(v)
    for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
      uint8_t bitCount = afti.getSignalledTileIdLengthMinus1() + 1;
      WRITE_CODE( afti.getTileId( i ), bitCount );  // u(v)
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.6.3	Atlas adaptation parameter set RBSP syntax
void PCCBitstreamWriter::atlasAdaptationParameterSetRbsp( AtlasAdaptationParameterSetRbsp &aaps,
                                                          PCCBitstream                    &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  WRITE_UVLC( aaps.getAtlasAdaptationParameterSetId() );  // ue(v)
  WRITE_CODE( aaps.getExtensionFlag(), 1 );               // u(1)
  if ( aaps.getExtensionFlag() ) {
    WRITE_CODE( aaps.getVpccExtensionFlag(), 1 );    // u(1)
    WRITE_CODE( aaps.getVpccGsExtensionFlag(), 1 );  // u(1)
    WRITE_CODE( aaps.getExtension6Bits(), 6 );       // u(7)
  }
  if ( aaps.getVpccExtensionFlag() ) { aapsVpccExtension( bitstream, aaps.getAapsVpccExtension() ); }
  if ( aaps.getVpccGsExtensionFlag() ) { aapsVpccGsExtension( bitstream, aaps.getAapsVpccGsExtension() ); }
  if ( aaps.getExtension6Bits() ) {
    while ( moreRbspData( bitstream ) ) { WRITE_CODE( 0, 1 ); }  // u(1)
  }
  rbspTrailingBits( bitstream );
}

// 8.3.6.4  Supplemental enhancement information Rbsp
void PCCBitstreamWriter::seiRbsp( PCCHighLevelSyntax &syntax,
                                  PCCBitstream       &bitstream,
                                  SEI                &sei,
                                  NalUnitType         nalUnitType,
                                  size_t              atglIndex ) {
  // TRACE_BITSTREAM( "%s \n", __func__ );
  do { seiMessage( bitstream, syntax, sei, nalUnitType, atglIndex ); } while ( moreRbspData( bitstream ) );
  rbspTrailingBits( bitstream );
}

// 8.3.6.5 Access unit delimiter RBSP syntax
void PCCBitstreamWriter::accessUnitDelimiterRbsp( AccessUnitDelimiterRbsp &aud,
                                                  PCCHighLevelSyntax      &syntax,
                                                  PCCBitstream            &bitstream ) {
  WRITE_CODE( aud.getAframeType(), 3 );  //	u(3)
  rbspTrailingBits( bitstream );
}
// 8.3.6.6 End of sequence RBSP syntax
void PCCBitstreamWriter::endOfSequenceRbsp( EndOfSequenceRbsp  &eos,
                                            PCCHighLevelSyntax &syntax,
                                            PCCBitstream       &bitstream ) {}

// 8.3.6.7 End of bitstream RBSP syntax
void PCCBitstreamWriter::endOfAtlasSubBitstreamRbsp( EndOfAtlasSubBitstreamRbsp &eoasb,
                                                     PCCHighLevelSyntax         &syntax,
                                                     PCCBitstream               &bitstream ) {}

// 8.3.6.8 Filler data RBSP syntax
void PCCBitstreamWriter::fillerDataRbsp( FillerDataRbsp &filler, PCCHighLevelSyntax &syntax, PCCBitstream &bitstream ) {
  // while ( next_bits( 8 ) == 0xFF ) bitstream.read( 8 );  // f(8)  // JR TODO
  rbspTrailingBits( bitstream );
}

// 8.3.6.9 Atlas tile group layer Rbsp syntax = patchTileLayerUnit
void PCCBitstreamWriter::atlasTileLayerRbsp( AtlasTileLayerRbsp &atgl,
                                             PCCHighLevelSyntax &syntax,
                                             NalUnitType         nalUnitType,
                                             PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  atlasTileHeader( atgl.getHeader(), syntax, nalUnitType, bitstream );
  atlasTileDataUnit( atgl.getDataUnit(), atgl.getHeader(), syntax, bitstream );
  rbspTrailingBits( bitstream );
}

// 8.3.6.10 RBSP trailing bit syntax
void PCCBitstreamWriter::rbspTrailingBits( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto rbsp_stop_one_bit       = 1;
  auto rbsp_alignment_zero_bit = 0;

  WRITE_CODE( rbsp_stop_one_bit, 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    WRITE_CODE( rbsp_alignment_zero_bit, 1 );  // f(1): equal to 0
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.6.11  Atlas tile group header syntax
void PCCBitstreamWriter::atlasTileHeader( AtlasTileHeader    &ath,
                                          PCCHighLevelSyntax &syntax,
                                          NalUnitType         nalUnitType,
                                          PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  size_t                         afpsId = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp    &afps   = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp &asps   = syntax.getAtlasSequenceParameterSet( aspsId );
  AtlasFrameTileInformationRbsp &afti   = afps.getAtlasFrameTileInformationRbsp();
  if ( nalUnitType >= NAL_BLA_W_LP && nalUnitType <= NAL_RSV_IRAP_ACL_29 ) {
    WRITE_CODE( ath.getNoOutputOfPriorAtlasFramesFlag(), 1 );  // u(1)
  }
  WRITE_UVLC( ath.getAtlasFrameParameterSetId() );       // ue(v)
  WRITE_UVLC( ath.getAtlasAdaptationParameterSetId() );  // ue(v)
  if ( afti.getSignalledTileIdFlag() )
    WRITE_CODE( uint32_t( ath.getId() ), afti.getSignalledTileIdLengthMinus1() + 1 )  // u(v)
  else {
    // TRACE_BITSTREAM( " getNumTilesInAtlasFrameMinus1: %zu\n", afti.getNumTilesInAtlasFrameMinus1() );
    if ( afti.getNumTilesInAtlasFrameMinus1() != 0 ) {
      WRITE_CODE( uint32_t( ath.getId() ),
                  ceilLog2( afti.getNumTilesInAtlasFrameMinus1() + 1 ) );  // u(v)
    }
  }
  WRITE_UVLC( ath.getType() );  // ue(v)
  if ( afps.getOutputFlagPresentFlag() ) { WRITE_CODE( ath.getAtlasOutputFlag(), 1 ); }
  WRITE_CODE( ath.getAtlasFrmOrderCntLsb(),
              asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4 );  // u(v)
  //  TRACE_BITSTREAM( " AtlasFrameParameterSetId: %zu\n", ath.getAtlasFrameParameterSetId() );
  //  TRACE_BITSTREAM( " AtlasSequenceParameterSetId: %zu\n", afps.getAtlasSequenceParameterSetId() );
  //  TRACE_BITSTREAM( " Address       %zu\n", (size_t)ath.getId() );
  //  TRACE_BITSTREAM( " Type          %zu\n", (size_t)ath.getType() );
  //  TRACE_BITSTREAM( " FrameOrderCnt %zu\n", (size_t)ath.getAtlasFrmOrderCntLsb() );
  if ( asps.getNumRefAtlasFrameListsInAsps() > 0 ) {
    WRITE_CODE( ath.getRefAtlasFrameListSpsFlag(), 1 );  // u(1)
  }
  if ( static_cast<int>( ath.getRefAtlasFrameListSpsFlag() ) == 0 ) {
    refListStruct( ath.getRefListStruct(), asps, bitstream );
  } else if ( asps.getNumRefAtlasFrameListsInAsps() > 1 ) {
    size_t bitCount = ceilLog2( asps.getNumRefAtlasFrameListsInAsps() );
    WRITE_CODE( ath.getRefAtlasFrameListIdx(), bitCount );  // u(v)
  }
  uint8_t rlsIdx  = ath.getRefAtlasFrameListIdx();
  auto   &refList = ath.getRefAtlasFrameListSpsFlag() ? asps.getRefListStruct( rlsIdx ) : ath.getRefListStruct();
  size_t  numLtrAtlasFrmEntries = 0;
  for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
    if ( !refList.getStRefAtalsFrameFlag( i ) ) { numLtrAtlasFrmEntries++; }
  }
  // TRACE_BITSTREAM( " rlsIdx %u numLtrAtlasFrmEntries %zu \n", rlsIdx, (size_t)numLtrAtlasFrmEntries );
  for ( size_t j = 0; j < numLtrAtlasFrmEntries; j++ ) {
    WRITE_CODE( ath.getAdditionalAfocLsbPresentFlag( j ), 1 );  // u(1)
    if ( ath.getAdditionalAfocLsbPresentFlag( j ) ) {
      uint8_t bitCount = afps.getAdditionalLtAfocLsbLen();
      WRITE_CODE( ath.getAdditionalAfocLsbVal( j ), bitCount );  // u(v)
    }
  }
  if ( ath.getType() != SKIP_TILE ) {
    if ( asps.getNormalAxisLimitsQuantizationEnabledFlag() ) {
      WRITE_CODE( ath.getPosMinDQuantizer(), 5 );  // u(5)
      // TRACE_BITSTREAM( " AtghPosMinDQuantizer = %zu \n", ath.getPosMinDQuantizer() );
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        WRITE_CODE( ath.getPosDeltaMaxDQuantizer(), 5 );  // u(5)
      }
    }
    if ( asps.getPatchSizeQuantizerPresentFlag() ) {
      WRITE_CODE( ath.getPatchSizeXinfoQuantizer(), 3 );  // u(3)
      WRITE_CODE( ath.getPatchSizeYinfoQuantizer(), 3 );  // u(3)
    }

    // TRACE_BITSTREAM( "Raw3dOffsetBitCountExplicitModeFlag    = %d \n", afps.getRaw3dOffsetBitCountExplicitModeFlag()
    // );
    if ( afps.getRaw3dOffsetBitCountExplicitModeFlag() ) {
      size_t bitCount = floorLog2( asps.getGeometry3dBitdepthMinus1() + 1 );
      // TRACE_BITSTREAM( "bitCount(floorLog2( asps.getGeometry3dBitdepthMinus1() + 1 ) = %zu \n", bitCount );
      WRITE_CODE( ath.getRaw3dOffsetAxisBitCountMinus1(), bitCount );  // u(v)
    } else {
      //      TRACE_BITSTREAM( "Geometry3dBitdepthMinus1 = %zu \n", asps.getGeometry3dBitdepthMinus1() );
      //      TRACE_BITSTREAM( "Geometry2dBitdepthMinus1 = %zu \n", asps.getGeometry2dBitdepthMinus1() );
    }
    if ( ath.getType() == P_TILE && refList.getNumRefEntries() > 1 ) {
      WRITE_CODE( ath.getNumRefIdxActiveOverrideFlag(), 1 );  // u(1)
      if ( ath.getNumRefIdxActiveOverrideFlag() ) {
        WRITE_UVLC( ath.getNumRefIdxActiveMinus1() );  // ue(v)
      }
    }
    //    TRACE_BITSTREAM( "==> Raw3dOffsetAxisBitCountMinus1  = %zu \n", ath.getRaw3dOffsetAxisBitCountMinus1() );
    //    TRACE_BITSTREAM( "==> NumRefIdxActiveOverrideFlag = %zu \n", ath.getNumRefIdxActiveOverrideFlag() );
    //    TRACE_BITSTREAM( "==> NumRefIdxActiveMinus1       = %zu \n", ath.getNumRefIdxActiveMinus1() );
  }
  byteAlignment( bitstream );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.6.12  Reference list structure syntax
void PCCBitstreamWriter::refListStruct( RefListStruct                 &rls,
                                        AtlasSequenceParameterSetRbsp &asps,
                                        PCCBitstream                  &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_UVLC( rls.getNumRefEntries() );  // ue(v)
  // TRACE_BITSTREAM( "NumRefEntries = %zu  \n", rls.getNumRefEntries() );
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( asps.getLongTermRefAtlasFramesFlag() ) { WRITE_CODE( rls.getStRefAtalsFrameFlag( i ), 1 ); }  // u(1)

    if ( rls.getStRefAtalsFrameFlag( i ) ) {
      WRITE_UVLC( rls.getAbsDeltaAfocSt( i ) );  // ue(v)
      if ( rls.getAbsDeltaAfocSt( i ) > 0 ) {
        WRITE_CODE( rls.getStrafEntrySignFlag( i ), 1 );  // u(1)
      }
    } else {
      uint8_t bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
      WRITE_CODE( rls.getAfocLsbLt( i ), bitCount );  // u(v)
    }
    //    TRACE_BITSTREAM( "stRefAtalsFrameFlag = %zu  \n", rls.getStRefAtalsFrameFlag( i ) );
    //    TRACE_BITSTREAM( "absDeltaAfocSt      = %zu  \n", rls.getAbsDeltaAfocSt( i ) );
    //    TRACE_BITSTREAM( "strpfEntrySignFlag  = %zu  \n", rls.getStrafEntrySignFlag( i ) );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.7.1  General atlas tile group data unit syntax =patchTileDataUnit
void PCCBitstreamWriter::atlasTileDataUnit( AtlasTileDataUnit  &atdu,
                                            AtlasTileHeader    &ath,
                                            PCCHighLevelSyntax &syntax,
                                            PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  // TRACE_BITSTREAM( "Type = %zu \n", ath.getType() );
  if ( ath.getType() == SKIP_TILE ) {
    skipPatchDataUnit( bitstream );
  } else {
    for ( size_t puCount = 0; puCount < atdu.getPatchCount(); puCount++ ) {
      // TRACE_BITSTREAM( "patch %zu : \n", puCount );
      WRITE_UVLC( uint32_t( atdu.getPatchMode( puCount ) ) );  // ue(v)
      auto &pid = atdu.getPatchInformationData( puCount );
      // TRACE_BITSTREAM( "patchMode = %zu \n", atdu.getPatchMode( puCount ) );
      pid.setTileOrder( atdu.getTileOrder() );
      pid.setPatchIndex( puCount );
      patchInformationData( pid, atdu.getPatchMode( puCount ), ath, syntax, bitstream );
    }
    // TRACE_BITSTREAM( "atdu.getPatchCount() including END = %zu \n", atdu.getPatchCount() );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.7.2  Patch information data syntax
void PCCBitstreamWriter::patchInformationData( PatchInformationData &pid,
                                               size_t                patchMode,
                                               AtlasTileHeader      &ath,
                                               PCCHighLevelSyntax   &syntax,
                                               PCCBitstream         &bitstream ) {
  // TRACE_BITSTREAM_IN( "%s: AtghType = %zu patchMode = %zu \n", __func__, ath.getType(), patchMode );
  TRACE_BITSTREAM_IN( "%s", __func__ );
  if ( ath.getType() == SKIP_TILE ) {
    // skip mode: currently not supported but added it for convenience. Could
    // easily be removed
  } else if ( ath.getType() == P_TILE ) {
    if ( patchMode == P_SKIP ) {
      // skip mode: currently not supported but added it for convenience. Could
      // easily be removed
      skipPatchDataUnit( bitstream );
    } else if ( patchMode == P_MERGE ) {
      auto &mpdu = pid.getMergePatchDataUnit();
      mpdu.setTileOrder( pid.getTileOrder() );
      mpdu.setPatchIndex( pid.getPatchIndex() );
      mergePatchDataUnit( mpdu, ath, syntax, bitstream );
    } else if ( patchMode == P_INTRA ) {
      auto &pdu = pid.getPatchDataUnit();
      pdu.setTileOrder( pid.getTileOrder() );
      pdu.setPatchIndex( pid.getPatchIndex() );
      patchDataUnit( pdu, ath, syntax, bitstream );
    } else if ( patchMode == P_INTER ) {
      auto &ipdu = pid.getInterPatchDataUnit();
      ipdu.setTileOrder( pid.getTileOrder() );
      ipdu.setPatchIndex( pid.getPatchIndex() );
      interPatchDataUnit( ipdu, ath, syntax, bitstream );
    } else if ( patchMode == P_RAW ) {
      auto &rpdu = pid.getRawPatchDataUnit();
      rpdu.setTileOrder( pid.getTileOrder() );
      rpdu.setPatchIndex( pid.getPatchIndex() );
      rawPatchDataUnit( rpdu, ath, syntax, bitstream );
    } else if ( patchMode == P_EOM ) {
      auto &epdu = pid.getEomPatchDataUnit();
      epdu.setTileOrder( pid.getTileOrder() );
      epdu.setPatchIndex( pid.getPatchIndex() );
      eomPatchDataUnit( epdu, ath, syntax, bitstream );
    }
  } else if ( ath.getType() == I_TILE ) {
    if ( patchMode == I_INTRA ) {
      auto &pdu = pid.getPatchDataUnit();
      pdu.setTileOrder( pid.getTileOrder() );
      pdu.setPatchIndex( pid.getPatchIndex() );
      patchDataUnit( pdu, ath, syntax, bitstream );
    } else if ( patchMode == I_RAW ) {
      auto &rpdu = pid.getRawPatchDataUnit();
      rpdu.setTileOrder( pid.getTileOrder() );
      rpdu.setPatchIndex( pid.getPatchIndex() );
      rawPatchDataUnit( rpdu, ath, syntax, bitstream );
    } else if ( patchMode == I_EOM ) {
      auto &epdu = pid.getEomPatchDataUnit();
      epdu.setTileOrder( pid.getTileOrder() );
      epdu.setPatchIndex( pid.getPatchIndex() );
      eomPatchDataUnit( epdu, ath, syntax, bitstream );
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.7.3  Patch data unit syntax : AtlasTileHeader instead of
void PCCBitstreamWriter::patchDataUnit( PatchDataUnit      &pdu,
                                        AtlasTileHeader    &ath,
                                        PCCHighLevelSyntax &syntax,
                                        PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t  afpsId     = ath.getAtlasFrameParameterSetId();
  auto   &afps       = syntax.getAtlasFrameParameterSet( afpsId );
  size_t  aspsId     = afps.getAtlasSequenceParameterSetId();
  auto   &asps       = syntax.getAtlasSequenceParameterSet( aspsId );
  uint8_t bitCountUV = asps.getGeometry3dBitdepthMinus1() + 1;
  uint8_t bitCountD  = asps.getGeometry3dBitdepthMinus1() - ath.getPosMinDQuantizer() + 1;
  WRITE_UVLC( pdu.get2dPosX() );                 // ue(v)
  WRITE_UVLC( pdu.get2dPosY() );                 // ue(v)
  WRITE_UVLC( pdu.get2dSizeXMinus1() );          // ue(v)
  WRITE_UVLC( pdu.get2dSizeYMinus1() );          // ue(v)
  WRITE_CODE( pdu.get3dOffsetU(), bitCountUV );  // u(v)
  WRITE_CODE( pdu.get3dOffsetV(), bitCountUV );  // u(v)
  WRITE_CODE( pdu.get3dOffsetD(), bitCountD );   // u(v)
  TRACE_BITSTREAM( " 2dPosXY: %zu,%zu\n", pdu.get2dPosX(), pdu.get2dPosX() );
  TRACE_BITSTREAM( " 2dSizeXY: %d,%d\n", int32_t( pdu.get2dSizeXMinus1() + 1 ), int32_t( pdu.get2dSizeYMinus1() + 1 ) );
  TRACE_BITSTREAM( " 3dPosXY: %zu,%zu\n", pdu.get3dOffsetU(), pdu.get3dOffsetV() );
  TRACE_BITSTREAM( " Pdu3dPosMinZ: %zu ( bitCountD = %u = %u - %u + %u ) \n", pdu.get3dOffsetD(), bitCountD,
                   asps.getGeometry3dBitdepthMinus1(), ath.getPosMinDQuantizer(), 2 );

  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    uint8_t bitCountForMaxDepth = min( asps.getGeometry2dBitdepthMinus1(), asps.getGeometry3dBitdepthMinus1() ) + 1 -
                                  ath.getPosDeltaMaxDQuantizer();
    WRITE_CODE( pdu.get3dRangeD(), bitCountForMaxDepth );  // u(v)
    TRACE_BITSTREAM( " Pdu3dPosDeltaMaxZ: %zu ( bitCountForMaxDepth = %u) \n", pdu.get3dRangeD(), bitCountForMaxDepth );
  }
  WRITE_CODE( pdu.getProjectionId(),
              ceilLog2( asps.getMaxNumberProjectionsMinus1() + 1 ) );  // u(5 or 3)
  TRACE_BITSTREAM( "PduProjectionId = %zu ( MaxNumberProjectionsMinus1 = %d ) \n", pdu.getProjectionId(),
                   asps.getMaxNumberProjectionsMinus1() );
  WRITE_CODE( pdu.getOrientationIndex(),
              asps.getUseEightOrientationsFlag() ? 3 : 1 );  // u(3 or 1)
  if ( afps.getLodModeEnableFlag() ) {
    WRITE_CODE( pdu.getLodEnableFlag(), 1 );  // u(1)
    if ( pdu.getLodEnableFlag() ) {
      WRITE_UVLC( pdu.getLodScaleXMinus1() );  // ue(v)
      WRITE_UVLC( pdu.getLodScaleYIdc() );     // ue(v)
    }
  }
  TRACE_BITSTREAM( "PLREnabledFlag = %d \n", asps.getPLREnabledFlag() );
  if ( asps.getPLREnabledFlag() ) {
    auto &plrd = pdu.getPLRData();
    TRACE_BITSTREAM( "Size = %ld %ld\n", pdu.get2dSizeXMinus1() + 1, pdu.get2dSizeYMinus1() + 1 );
    plrData( plrd, syntax, asps, bitstream );
  }
  TRACE_BITSTREAM(
      "Frame %zu, Patch(%zu) => 2Dpos = %4zu %4zu 2Dsize = %4ld %4ld 3Dpos = %ld %ld %ld "
      "3dRangeD = %ld Projection = %zu Orientation = %zu lod=(%d) %u %u\n",
      pdu.getFrameIndex(), pdu.getPatchIndex(), pdu.get2dPosX(), pdu.get2dPosY(), pdu.get2dSizeXMinus1() + 1,
      pdu.get2dSizeYMinus1() + 1, pdu.get3dOffsetU(), pdu.get3dOffsetV(), pdu.get3dOffsetD(), pdu.get3dRangeD(),
      pdu.getProjectionId(), pdu.getOrientationIndex(), pdu.getLodEnableFlag(),
      pdu.getLodEnableFlag() ? pdu.getLodScaleXMinus1() : (uint8_t)0,
      pdu.getLodEnableFlag() ? pdu.getLodScaleYIdc() : (uint8_t)0 );
}

// 8.3.7.4  Skip patch data unit syntax
void PCCBitstreamWriter::skipPatchDataUnit( PCCBitstream &bitstream ) { TRACE_BITSTREAM( "%s \n", __func__ ); }

// 8.3.7.5  Merge patch data unit syntax
void PCCBitstreamWriter::mergePatchDataUnit( MergePatchDataUnit &mpdu,
                                             AtlasTileHeader    &ath,
                                             PCCHighLevelSyntax &syntax,
                                             PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId          = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp    &afps            = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId          = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp &asps            = syntax.getAtlasSequenceParameterSet( aspsId );
  bool                           overridePlrFlag = false;
  size_t                         numRefIdxActive = syntax.getNumRefIdxActive( ath );
  if ( numRefIdxActive > 1 ) { WRITE_UVLC( mpdu.getRefIndex() ); }  // ue(v)
  WRITE_CODE( mpdu.getOverride2dParamsFlag(), 1 );                  // u(1)
  if ( mpdu.getOverride2dParamsFlag() ) {
    WRITE_SVLC( mpdu.get2dPosX() );        // se(v)
    WRITE_SVLC( mpdu.get2dPosY() );        // se(v)
    WRITE_SVLC( mpdu.get2dDeltaSizeX() );  // se(v)
    WRITE_SVLC( mpdu.get2dDeltaSizeY() );  // se(v)
    if ( asps.getPLREnabledFlag() ) { overridePlrFlag = true; }
  } else {
    WRITE_CODE( mpdu.getOverride3dParamsFlag(), 1 );  // u(1)
    if ( mpdu.getOverride3dParamsFlag() ) {
      WRITE_SVLC( mpdu.get3dOffsetU() );  // se(v)
      WRITE_SVLC( mpdu.get3dOffsetV() );  // se(v)
      WRITE_SVLC( mpdu.get3dOffsetD() );  // se(v)
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        WRITE_SVLC( mpdu.get3dRangeD() );  // se(v)
      }
      if ( asps.getPLREnabledFlag() ) {
        WRITE_CODE( mpdu.getOverridePlrFlag(), 1 );  // u(1)
      }
    }
  }
  if ( overridePlrFlag && asps.getPLREnabledFlag() ) {
    auto &plrd = mpdu.getPLRData();
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n",
                     plrd.getBlockToPatchMapWidth() - mpdu.get2dDeltaSizeX(),
                     plrd.getBlockToPatchMapHeight() - mpdu.get2dDeltaSizeY(), mpdu.get2dDeltaSizeX(),
                     mpdu.get2dDeltaSizeY(), plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight() );
    plrData( plrd, syntax, asps, bitstream );
  }
  TRACE_BITSTREAM(
      "%zu frame %zu MergePatch => Ref Frame Idx = %d ShiftUV = %ld %ld "
      "DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      mpdu.getRefIndex(), mpdu.get2dPosX(), mpdu.get2dPosY(), mpdu.get2dDeltaSizeX(), mpdu.get2dDeltaSizeY(),
      mpdu.get3dOffsetU(), mpdu.get3dOffsetV(), mpdu.get3dOffsetD(), mpdu.get3dRangeD() );
}

// 8.3.7.6  Inter patch data unit syntax
void PCCBitstreamWriter::interPatchDataUnit( InterPatchDataUnit &ipdu,
                                             AtlasTileHeader    &ath,
                                             PCCHighLevelSyntax &syntax,
                                             PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t afpsId          = ath.getAtlasFrameParameterSetId();
  auto  &afps            = syntax.getAtlasFrameParameterSet( afpsId );
  size_t aspsId          = afps.getAtlasSequenceParameterSetId();
  auto  &asps            = syntax.getAtlasSequenceParameterSet( aspsId );
  size_t numRefIdxActive = syntax.getNumRefIdxActive( ath );
  if ( numRefIdxActive > 1 ) {
    WRITE_UVLC( int32_t( ipdu.getRefIndex() ) );  // ue(v)
  }
  TRACE_BITSTREAM( "%zu frame: numRefIdxActive = %zu reference = frame%zu\n", ipdu.getFrameIndex(), numRefIdxActive,
                   ipdu.getRefIndex() );
  WRITE_SVLC( int32_t( ipdu.getRefPatchIndex() ) );  // se(v)
  WRITE_SVLC( int32_t( ipdu.get2dPosX() ) );         // se(v)
  WRITE_SVLC( int32_t( ipdu.get2dPosY() ) );         // se(v)
  WRITE_SVLC( int32_t( ipdu.get2dDeltaSizeX() ) );   // se(v)
  WRITE_SVLC( int32_t( ipdu.get2dDeltaSizeY() ) );   // se(v)
  WRITE_SVLC( int32_t( ipdu.get3dOffsetU() ) );      // se(v)
  WRITE_SVLC( int32_t( ipdu.get3dOffsetV() ) );      // se(v)
  WRITE_SVLC( int32_t( ipdu.get3dOffsetD() ) );      // se(v)
  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    WRITE_SVLC( int32_t( ipdu.get3dRangeD() ) );  // se(v)
  }
  TRACE_BITSTREAM(
      "%zu frame: numRefIdxActive = %zu reference = frame%zu patch%d 2Dpos = "
      "%ld %ld 2DdeltaSize = %ld %ld 3Dpos = %ld "
      "%ld %ld DeltaMaxZ = %ld\n",
      ipdu.getFrameIndex(), numRefIdxActive, ipdu.getRefIndex(), ipdu.getRefPatchIndex(), ipdu.get2dPosX(),
      ipdu.get2dPosY(), ipdu.get2dDeltaSizeX(), ipdu.get2dDeltaSizeY(), ipdu.get3dOffsetU(), ipdu.get3dOffsetV(),
      ipdu.get3dOffsetD(), ipdu.get3dRangeD() );
  if ( asps.getPLREnabledFlag() ) {
    auto &plrd = ipdu.getPLRData();
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n",
                     plrd.getBlockToPatchMapWidth() - ipdu.get2dDeltaSizeX(),
                     plrd.getBlockToPatchMapHeight() - ipdu.get2dDeltaSizeY(), ipdu.get2dDeltaSizeX(),
                     ipdu.get2dDeltaSizeY(), plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight() );
    plrData( plrd, syntax, asps, bitstream );
  }
}

// 8.3.7.7 raw patch data unit syntax
void PCCBitstreamWriter::rawPatchDataUnit( RawPatchDataUnit   &rpdu,
                                           AtlasTileHeader    &ath,
                                           PCCHighLevelSyntax &syntax,
                                           PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  int32_t bitCount = ath.getRaw3dOffsetAxisBitCountMinus1() + 1;
  // TRACE_BITSTREAM( " AtghRaw3dOffsetAxisBitCountMinus1 = %zu => bitcount = %d \n",
  //                  ath.getRaw3dOffsetAxisBitCountMinus1(), bitCount );
  auto &afti = syntax.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() ).getAtlasFrameTileInformationRbsp();
  auto  ath_id        = ath.getId();
  auto  tileIdToIndex = afti.getTileId( ath_id );
  if ( afti.getAuxiliaryVideoTileRowHeight( tileIdToIndex ) ) {
    WRITE_CODE( rpdu.getPatchInAuxiliaryVideoFlag(), 1 );  // u(1)
  }
  WRITE_UVLC( rpdu.get2dPosX() );               // ue(v)
  WRITE_UVLC( rpdu.get2dPosY() );               // ue(v)
  WRITE_UVLC( rpdu.get2dSizeXMinus1() );        // ue(v)
  WRITE_UVLC( rpdu.get2dSizeYMinus1() );        // ue(v)
  WRITE_CODE( rpdu.get3dOffsetU(), bitCount );  // u(v)
  WRITE_CODE( rpdu.get3dOffsetV(), bitCount );  // u(v)
  WRITE_CODE( rpdu.get3dOffsetD(), bitCount );  // u(v)
  WRITE_UVLC( rpdu.getRawPointsMinus1() );      // ue(v)
                                                //  TRACE_BITSTREAM(
                                                //      "Raw Patch => UV %4zu %4zu  S=%4ld %4ld  UVD1=%4ld %4ld %4ld "
                                                //      "NumPcmPoints=%zu PatchInRawVideoFlag=%d \n",
  //      rpdu.get2dPosX(), rpdu.get2dPosY(), rpdu.get2dSizeXMinus1() + 1, rpdu.get2dSizeYMinus1() + 1,
  //      rpdu.get3dOffsetU(), rpdu.get3dOffsetV(), rpdu.get3dOffsetD(), rpdu.getRawPointsMinus1() + 1,
  //      rpdu.getPatchInAuxiliaryVideoFlag() );
  auto &aaps = syntax.getAtlasAdaptationParameterSet( ath.getAtlasAdaptationParameterSetId() );
  if ( aaps.getVpccGsExtensionFlag() ) {
    auto &aapsGsExt = aaps.getAapsVpccGsExtension();
    rpduVpccGsExtension( bitstream, aapsGsExt, rpdu );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// 8.3.7.8 EOM patch data unit syntax
void PCCBitstreamWriter::eomPatchDataUnit( EOMPatchDataUnit   &epdu,
                                           AtlasTileHeader    &ath,
                                           PCCHighLevelSyntax &syntax,
                                           PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &afti = syntax.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() ).getAtlasFrameTileInformationRbsp();
  auto  ath_id        = ath.getId();
  auto  tileIdToIndex = afti.getTileId( ath_id );
  if ( afti.getAuxiliaryVideoTileRowHeight( tileIdToIndex ) ) {
    WRITE_CODE( epdu.getPatchInAuxiliaryVideoFlag(), 1 );  // u(1)
  }
  WRITE_UVLC( epdu.get2dPosX() );            // ue(v)
  WRITE_UVLC( epdu.get2dPosY() );            // ue(v)
  WRITE_UVLC( epdu.get2dSizeXMinus1() );     // ue(v)
  WRITE_UVLC( epdu.get2dSizeYMinus1() );     // ue(v)
  WRITE_UVLC( epdu.getPatchCountMinus1() );  //  ue(v)
  for ( size_t i = 0; i < epdu.getPatchCountMinus1() + 1; i++ ) {
    WRITE_UVLC( epdu.getAssociatedPatchesIdx( i ) );  //  ue(v)
    WRITE_UVLC( epdu.getPoints( i ) );                //  ue(v)
  }
#ifdef BITSTREAM_TRACE
  TRACE_BITSTREAM( "EOM Patch => UV %4zu %4zu  S=%4ld %4ld  N=%4ld\n", epdu.get2dPosX(), epdu.get2dPosY(),
                   epdu.get2dSizeXMinus1() + 1, epdu.get2dSizeYMinus1() + 1, epdu.getPatchCountMinus1() + 1 );
  for ( size_t i = 0; i < epdu.getPatchCountMinus1() + 1; i++ ) {
    TRACE_BITSTREAM( "%4ld, %4ld\n", epdu.getAssociatedPatchesIdx( i ), epdu.getPoints( i ) );
  }
#endif
}

// 8.3.7.9 Point local reconstruction data syntax
void PCCBitstreamWriter::plrData( PLRData                       &plrd,
                                  PCCHighLevelSyntax            &syntax,
                                  AtlasSequenceParameterSetRbsp &asps,
                                  PCCBitstream                  &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  for ( size_t i = 0; i < asps.getMapCountMinus1() + 1; i++ ) {
    auto &plri = asps.getPLRInformation( i );
    if ( plri.getMapEnabledFlag() ) {
      const size_t blockCount   = plrd.getBlockToPatchMapWidth() * plrd.getBlockToPatchMapHeight();
      const auto   bitCountMode = uint8_t( ceilLog2( uint32_t( plri.getNumberOfModesMinus1() ) ) );
      TRACE_BITSTREAM( "WxH= %zu x %zu => blockCount = %zu \n", plrd.getBlockToPatchMapWidth(),
                       plrd.getBlockToPatchMapHeight(), blockCount );
      TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u  => bitCountMode = %u  \n", plri.getNumberOfModesMinus1(),
                       bitCountMode );
      if ( blockCount > plri.getBlockThresholdPerPatchMinus1() + 1 ) {
        WRITE_CODE( plrd.getLevelFlag(), 1 );  // u(1)
      }
      TRACE_BITSTREAM( "  LevelFlag = %u \n", plrd.getLevelFlag() );
      if ( !plrd.getLevelFlag() ) {
        for ( size_t i = 0; i < blockCount; i++ ) {
          WRITE_CODE( plrd.getBlockPresentFlag( i ), 1 );  // u(1)
          if ( plrd.getBlockPresentFlag( i ) ) {
            WRITE_CODE( plrd.getBlockModeMinus1( i ), bitCountMode );  // u(v)
          }
          TRACE_BITSTREAM( "  Mode[ %4zu / %4zu ]: Present = %d ModeMinus1 = %d \n", i, blockCount,
                           plrd.getBlockPresentFlag( i ),
                           plrd.getBlockPresentFlag( i ) ? plrd.getBlockModeMinus1( i ) : -1 );
        }
      } else {
        WRITE_CODE( plrd.getPresentFlag(), 1 );  // u(1)
        if ( plrd.getPresentFlag() ) {
          WRITE_CODE( plrd.getModeMinus1(), bitCountMode );  // u(v)
        }
        TRACE_BITSTREAM( "  ModePatch: Present = %d ModeMinus1 = %d \n", plrd.getPresentFlag(),
                         plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1 );
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
              plrd.getLevelFlag()             ? plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1
              : plrd.getBlockPresentFlag( i ) ? plrd.getBlockModeMinus1( i )
                                              : -1 );
        }
      }
#endif
    }
  }
}

// 8.3.8 Supplemental enhancement information message syntax
void PCCBitstreamWriter::seiMessage( PCCBitstream       &bitstream,
                                     PCCHighLevelSyntax &syntax,
                                     SEI                &sei,
                                     NalUnitType         nalUnitType,
                                     size_t              atglIndex ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto payloadType = static_cast<int32_t>( sei.getPayloadType() );
  for ( ; payloadType >= 0xff; payloadType -= 0xff ) {
    WRITE_CODE( 0xff, 8 );  // u(8)
  }
  WRITE_CODE( payloadType, 8 );  // u(8)

  // Note: calculating the size of the sei message before writing it into the bitstream
  auto                startRealPos = bitstream.getPosition();
  PCCBitstream        tempbitstream;
  PCCBistreamPosition tempStartPos{ 0, startRealPos.bits_ };
  seiPayload( tempbitstream, syntax, sei, nalUnitType, atglIndex );
  auto    endPos       = tempbitstream.getPosition();
  int64_t payloadBits  = int64_t( endPos.bytes_ ) * 8 + int64_t( endPos.bits_ ) - int64_t( startRealPos.bits_ );
  size_t  payloadBytes = size_t( ( payloadBits + 7 ) >> 3 );

  sei.setPayloadSize( payloadBytes );

  auto payloadSize = static_cast<int32_t>( sei.getPayloadSize() );
  for ( ; payloadSize >= 0xff; payloadSize -= 0xff ) {
    WRITE_CODE( 0xff, 8 );  // u(8)
  }
  WRITE_CODE( payloadSize, 8 );  // u(8)
  seiPayload( bitstream, syntax, sei, nalUnitType, atglIndex );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// C.2 Sample stream V3C unit syntax and semantics
// C.2.1 Sample stream V3C header syntax
void PCCBitstreamWriter::sampleStreamV3CHeader( PCCBitstream &bitstream, SampleStreamV3CUnit &ssvu ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( ssvu.getSsvhUnitSizePrecisionBytesMinus1(), 3 );  // u(3)
  WRITE_CODE( 0, 5 );                                           // u(5)
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// C.2.2 Sample stream V3C unit syntax
void PCCBitstreamWriter::sampleStreamV3CUnit( PCCBitstream &bitstream, SampleStreamV3CUnit &ssvu, V3CUnit &v3cUnit ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( v3cUnit.getSize(),
              8 * ( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) );  // u(v)
  // TRACE_BITSTREAM( "V3CUnitType: %hhu V3CUnitSize: %zu\n", (uint8_t)v3cUnit.getType(), v3cUnit.getSize() );
  bitstream.copyFrom( v3cUnit.getBitstream(), 0, v3cUnit.getSize() );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// D.2 Sample stream NAL unit syntax and semantics
// D.2.1 Sample stream NAL header syntax
void PCCBitstreamWriter::sampleStreamNalHeader( PCCBitstream &bitstream, SampleStreamNalUnit &ssnu ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto ssnh_unit_size_precision_bytes_minus1 = ssnu.getSizePrecisionBytesMinus1();
  auto ssnh_reserved_zero_5bits              = 0;
  WRITE_CODE( ssnh_unit_size_precision_bytes_minus1, 3 );  // u(3)
  WRITE_CODE( ssnh_reserved_zero_5bits, 5 );               // u(5)
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// D.2.2 Sample stream NAL unit syntax
void PCCBitstreamWriter::sampleStreamNalUnit( PCCHighLevelSyntax  &syntax,
                                              PCCBitstream        &bitstream,
                                              SampleStreamNalUnit &ssnu,
                                              NalUnit             &nalu,
                                              size_t               index,
                                              size_t               atglIndex ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto ssnu_nal_unit_size = nalu.getSize();
  WRITE_CODE( ssnu_nal_unit_size, 8 * ( ssnu.getSizePrecisionBytesMinus1() + 1 ) );  // u(v)
  PCCBitstream ssnuBitstream;
#if defined( CONFORMANCE_TRACE ) || defined( BITSTREAM_TRACE )
  ssnuBitstream.setTrace( true );
  ssnuBitstream.setLogger( *logger_ );
#endif
  nalUnitHeader( ssnuBitstream, nalu );
  switch ( nalu.getType() ) {
    case NAL_ASPS:
      atlasSequenceParameterSetRbsp( syntax.getAtlasSequenceParameterSet( index ), syntax, ssnuBitstream );
      break;
    case NAL_AFPS:
      atlasFrameParameterSetRbsp( syntax.getAtlasFrameParameterSet( index ), syntax, ssnuBitstream );
      break;
    case NAL_AAPS:
      atlasAdaptationParameterSetRbsp( syntax.getAtlasAdaptationParameterSet( index ), ssnuBitstream );
      break;
      break;
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
    case NAL_SKIP_R:
    case NAL_IDR_N_LP:
      atlasTileLayerRbsp( syntax.getAtlasTileLayer( index ), syntax, nalu.getType(), ssnuBitstream );
      break;
    case NAL_SUFFIX_ESEI:
    case NAL_SUFFIX_NSEI:
      seiRbsp( syntax, ssnuBitstream, syntax.getAtlasTileLayer( atglIndex ).getSEI().getSeiSuffix( index ),
               nalu.getType(), atglIndex );
      break;
    case NAL_PREFIX_ESEI:
    case NAL_PREFIX_NSEI:
      seiRbsp( syntax, ssnuBitstream, syntax.getAtlasTileLayer( atglIndex ).getSEI().getSeiPrefix( index ),
               nalu.getType(), atglIndex );
      break;
    default: fprintf( stderr, "sampleStreamNalUnit type = %d not supported\n", static_cast<int32_t>( nalu.getType() ) );
  }
  bitstream.copyFrom( ssnuBitstream, 0, ssnuBitstream.size() );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// F.2  SEI payload syntax
// F.2.1  General SEI message syntax
void PCCBitstreamWriter::seiPayload( PCCBitstream       &bitstream,
                                     PCCHighLevelSyntax &syntax,
                                     SEI                &sei,
                                     NalUnitType         nalUnitType,
                                     size_t              atglIndex ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto payloadType = sei.getPayloadType();
  if ( nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ) {
    if ( payloadType == BUFFERING_PERIOD ) {  // 0
      bufferingPeriod( bitstream, sei );
    } else if ( payloadType == ATLAS_FRAME_TIMING ) {  // 1
      assert( syntax.getAtlasTileLayer( atglIndex ).getSEI().seiIsPresent( NAL_PREFIX_NSEI, BUFFERING_PERIOD ) );
      auto &bpsei = *syntax.getAtlasTileLayer( atglIndex ).getSEI().getLastSei( NAL_PREFIX_NSEI, BUFFERING_PERIOD );
      atlasFrameTiming( bitstream, sei, bpsei, false );
    } else if ( payloadType == FILLER_PAYLOAD ) {  // 2
      fillerPayload( bitstream, sei );
    } else if ( payloadType == USER_DATAREGISTERED_ITUTT35 ) {  // 3
      userDataRegisteredItuTT35( bitstream, sei );
    } else if ( payloadType == USER_DATA_UNREGISTERED ) {  // 4
      userDataUnregistered( bitstream, sei );
    } else if ( payloadType == RECOVERY_POINT ) {  // 5
      recoveryPoint( bitstream, sei );
    } else if ( payloadType == NO_RECONSTRUCTION ) {  // 6
      noReconstruction( bitstream, sei );
    } else if ( payloadType == TIME_CODE ) {  // 7
      timeCode( bitstream, sei );
    } else if ( payloadType == SEI_MANIFEST ) {  // 8
      seiManifest( bitstream, sei );
    } else if ( payloadType == SEI_PREFIX_INDICATION ) {  // 9
      seiPrefixIndication( bitstream, sei );
    } else if ( payloadType == ACTIVE_SUB_BITSTREAMS ) {  // 10
      activeSubBitstreams( bitstream, sei );
    } else if ( payloadType == COMPONENT_CODEC_MAPPING ) {  // 11
      componentCodecMapping( bitstream, sei );
    } else if ( payloadType == SCENE_OBJECT_INFORMATION ) {  // 12
      sceneObjectInformation( bitstream, sei );
    } else if ( payloadType == OBJECT_LABEL_INFORMATION ) {  // 13
      objectLabelInformation( bitstream, sei );
    } else if ( payloadType == PATCH_INFORMATION ) {  // 14
      patchInformation( bitstream, sei );
    } else if ( payloadType == VOLUMETRIC_RECTANGLE_INFORMATION ) {  // 15
      volumetricRectangleInformation( bitstream, sei );
    } else if ( payloadType == ATLAS_OBJECT_INFORMATION ) {  // 16
      atlasObjectInformation( bitstream, sei );
    } else if ( payloadType == VIEWPORT_CAMERA_PARAMETERS ) {  // 17
      viewportCameraParameters( bitstream, sei );
    } else if ( payloadType == VIEWPORT_POSITION ) {  // 18
      viewportPosition( bitstream, sei );
    } else if ( payloadType == ATTRIBUTE_TRANSFORMATION_PARAMS ) {  // 64
      attributeTransformationParams( bitstream, sei );
    } else if ( payloadType == OCCUPANCY_SYNTHESIS ) {  // 65
      occupancySynthesis( bitstream, sei );
    } else if ( payloadType == GEOMETRY_SMOOTHING ) {  // 66
      geometrySmoothing( bitstream, sei );
    } else if ( payloadType == ATTRIBUTE_SMOOTHING ) {  // 67
      attributeSmoothing( bitstream, sei );
    } else if ( payloadType == VPCC_REGISTERED ) {  // 68
      vpccRegisteredSEI( bitstream, sei );
    } else {
      reservedSeiMessage( bitstream, sei );
    }
  } else { /* nalUnitType  ==  NAL_SUFFIX_SEI  || nalUnitType  ==
              NAL_SUFFIX_NSEI */
    /*SEI& sei = syntax.addSeiSuffix( payloadType, nalUnitType == NAL_SUFFIX_ESEI );*/
    if ( payloadType == FILLER_PAYLOAD ) {  // 2
      fillerPayload( bitstream, sei );
    } else if ( payloadType == USER_DATAREGISTERED_ITUTT35 ) {  // 3
      userDataRegisteredItuTT35( bitstream, sei );
    } else if ( payloadType == USER_DATA_UNREGISTERED ) {  // 4
      userDataUnregistered( bitstream, sei );
    } else if ( payloadType == DECODED_ATLAS_INFORMATION_HASH ) {  // 21
      decodedAtlasInformationHash( bitstream, sei );
    } else {
      reservedSeiMessage( bitstream, sei );
    }
  }
  if ( moreDataInPayload( bitstream ) ) {
    if ( payloadExtensionPresent( bitstream ) ) {
      WRITE_CODE( 1, 1 );  // u(v)
    }
    byteAlignment( bitstream );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// Specified in ISO/IEC 23002-7 -> Filler payload SEI message syntax
void PCCBitstreamWriter::fillerPayload( PCCBitstream &bitstream, SEI &sei ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto payloadSize = sei.getPayloadSize();
  for ( size_t k = 0; k < payloadSize; k++ ) {
    WRITE_CODE( 0xFF, 8 );  // f(8) equal to 0xFF
  }
}

// Specified in ISO/IEC 23002-7 -> User data registered by Recommendation ITU-T T.35 SEI message syntax
void PCCBitstreamWriter::userDataRegisteredItuTT35( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei         = static_cast<SEIUserDataRegisteredItuTT35 &>( seiAbstract );
  auto  payloadSize = sei.getPayloadSize();
  WRITE_CODE( sei.getCountryCode(), 8 );  // b(8)
  payloadSize--;
  if ( sei.getCountryCode() == 0xFF ) {
    WRITE_CODE( sei.getCountryCodeExtensionByte(), 8 );  // b(8)
    payloadSize--;
  }
  auto &payload = sei.getPayloadByte();
  for ( auto &element : payload ) {
    WRITE_CODE( element, 8 );  // b(8)
  }
}

// Specified in ISO/IEC 23002-7 -> User data unregistered SEI message syntax
void PCCBitstreamWriter::userDataUnregistered( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei         = static_cast<SEIUserDataUnregistered &>( seiAbstract );
  auto  payloadSize = sei.getPayloadSize();
  for ( size_t i = 0; i < 16; i++ ) {
    WRITE_CODE( sei.getUuidIsoIec11578( i ), 8 );  // u(128) <=> 16 * u(8)
  }
  payloadSize -= 16;
  for ( size_t i = 0; i < payloadSize; i++ ) {
    WRITE_CODE( sei.getUserDataPayloadByte( i ), 8 );  // b(8)
  }
}

// F.2.2  Recovery point SEI message syntax
void PCCBitstreamWriter::recoveryPoint( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIRecoveryPoint &>( seiAbstract );
  WRITE_SVLC( sei.getRecoveryAfocCnt() );    // se(v)
  WRITE_CODE( sei.getExactMatchFlag(), 1 );  // u(1)
  WRITE_CODE( sei.getBrokenLinkFlag(), 1 );  // u(1)
}

// F.2.3  No reconstruction SEI message syntax
void PCCBitstreamWriter::noReconstruction( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}

// Specified in ISO/IEC 23002-7 -> Reserved SEI message syntax
void PCCBitstreamWriter::reservedSeiMessage( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei         = static_cast<SEIReservedSeiMessage &>( seiAbstract );
  auto  payloadSize = sei.getPayloadSize();
  for ( size_t i = 0; i < payloadSize; i++ ) {
    WRITE_CODE( sei.getPayloadByte( i ), 8 );  // b(8)
  }
}

// F.2.4  SEI manifest SEI message syntax
void PCCBitstreamWriter::seiManifest( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIManifest &>( seiAbstract );
  WRITE_CODE( sei.getNumSeiMsgTypes(), 16 );  // u(16)
  for ( size_t i = 0; i < sei.getNumSeiMsgTypes(); i++ ) {
    WRITE_CODE( sei.getSeiPayloadType( i ), 16 );  // u(16)
    WRITE_CODE( sei.getSeiDescription( i ), 8 );   // u(8)
  }
}

// F.2.5  SEI prefix indication SEI message syntax
void PCCBitstreamWriter::seiPrefixIndication( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIPrefixIndication &>( seiAbstract );
  WRITE_CODE( sei.getPrefixSeiPayloadType(), 16 );          // u(16)
  WRITE_CODE( sei.getNumSeiPrefixIndicationsMinus1(), 8 );  // u(8)
  for ( size_t i = 0; i <= sei.getNumSeiPrefixIndicationsMinus1(); i++ ) {
    WRITE_CODE( sei.getNumBitsInPrefixIndicationMinus1( i ), 16 );  // u(16)
    for ( size_t j = 0; j <= sei.getNumBitsInPrefixIndicationMinus1( i ); j++ ) {
      WRITE_CODE( sei.getSeiPrefixDataBit( i, j ), 1 );  // u(1)
    }
    while ( !bitstream.byteAligned() ) {
      WRITE_CODE( 1, 1 );  // f(1): equal to 1
    }
  }
}

// F.2.6  Active substreams SEI message syntax
void PCCBitstreamWriter::activeSubBitstreams( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIActiveSubBitstreams &>( seiAbstract );
  WRITE_CODE( sei.getActiveSubBitstreamsCancelFlag(), 1 );  // u(1)
  if ( !sei.getActiveSubBitstreamsCancelFlag() ) {
    WRITE_CODE( sei.getActiveAttributesChangesFlag(), 1 );    // u(1)
    WRITE_CODE( sei.getActiveMapsChangesFlag(), 1 );          // u(1)
    WRITE_CODE( sei.getAuxiliarySubstreamsActiveFlag(), 1 );  // u(1)
    if ( sei.getActiveAttributesChangesFlag() ) {
      WRITE_CODE( sei.getAllAttributesActiveFlag(), 1 );  // u(1)
      if ( !sei.getAllAttributesActiveFlag() ) {
        WRITE_CODE( sei.getActiveAttributeCountMinus1(), 7 );  // u(7)
        for ( size_t i = 0; i <= sei.getActiveAttributeCountMinus1(); i++ ) {
          WRITE_CODE( sei.getActiveAttributeIdx( i ), 7 );  // u(7)
        }
      }
    }
    if ( sei.getActiveMapsChangesFlag() ) {
      WRITE_CODE( sei.getAllMapsActiveFlag(), 1 );  // u(1)
      if ( !sei.getAllMapsActiveFlag() ) {
        WRITE_CODE( sei.getActiveMapCountMinus1(), 4 );  // u(4)
        for ( size_t i = 0; i <= sei.getActiveMapCountMinus1(); i++ ) {
          WRITE_CODE( sei.getActiveMapIdx( i ), 4 );  // u(4)
        }
      }
    }
  }
}

// F.2.7  Component codec mapping SEI message syntax
void PCCBitstreamWriter::componentCodecMapping( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIComponentCodecMapping &>( seiAbstract );
  WRITE_CODE( sei.getComponentCodecCancelFlag(), 1 );  // u(1)
  if ( !sei.getComponentCodecCancelFlag() ) {
    WRITE_CODE( sei.getCodecMappingsCountMinus1(), 8 );  // u(8)
    sei.allocate();
    uint8_t ccm_bit_equal_to_zero = 0;
    for ( size_t i = 0; i <= sei.getCodecMappingsCountMinus1(); i++ ) {
      WRITE_CODE( sei.getCodecId( i ), 8 );  // u(8)
      while ( !bitstream.byteAligned() ) {
        WRITE_CODE( ccm_bit_equal_to_zero, 1 );  // u(1): equal to 0
      }
      WRITE_STRING( sei.getCodec4cc( sei.getCodecId( i ) ) );  // st(v)
    }
  }
}

// F.2.8  Volumetric Tiling SEI message syntax
// F.2.8.1	Scene object information SEI message syntax
void PCCBitstreamWriter::sceneObjectInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEISceneObjectInformation &>( seiAbstract );
  WRITE_CODE( sei.getPersistenceFlag(), 1 );  // u(1)
  WRITE_CODE( sei.getResetFlag(), 1 );        // u(1)
  WRITE_UVLC( sei.getNumObjectUpdates() );    // ue(v)
  if ( sei.getNumObjectUpdates() > 0 ) {
    WRITE_CODE( sei.getSimpleObjectsFlag(), 1 );  // u(1)
    if ( static_cast<int>( sei.getSimpleObjectsFlag() ) == 0 ) {
      WRITE_CODE( sei.getObjectLabelPresentFlag(), 1 );       // u(1)
      WRITE_CODE( sei.getPriorityPresentFlag(), 1 );          // u(1)
      WRITE_CODE( sei.getObjectHiddenPresentFlag(), 1 );      // u(1)
      WRITE_CODE( sei.getObjectDependencyPresentFlag(), 1 );  // u(1)
      WRITE_CODE( sei.getVisibilityConesPresentFlag(), 1 );   // u(1)
      WRITE_CODE( sei.get3dBoundingBoxPresentFlag(), 1 );     // u(1)
      WRITE_CODE( sei.getCollisionShapePresentFlag(), 1 );    // u(1)
      WRITE_CODE( sei.getPointStylePresentFlag(), 1 );        // u(1)
      WRITE_CODE( sei.getMaterialIdPresentFlag(), 1 );        // u(1)
      WRITE_CODE( sei.getExtensionPresentFlag(), 1 );         // u(1)
    }
    if ( sei.get3dBoundingBoxPresentFlag() ) {
      WRITE_CODE( sei.get3dBoundingBoxScaleLog2(), 5 );        // u(5)
      WRITE_CODE( sei.get3dBoundingBoxPrecisionMinus8(), 5 );  // u(5)
    }
    WRITE_CODE( sei.getLog2MaxObjectIdxUpdated(), 5 );  // u(5)
    if ( sei.getObjectDependencyPresentFlag() ) {
      WRITE_CODE( sei.getLog2MaxObjectDependencyIdx(), 5 );  // u(5)
    }
    for ( size_t i = 0; i <= sei.getNumObjectUpdates(); i++ ) {
      assert( sei.getObjectIdx( i ) >= sei.getNumObjectUpdates() );
      WRITE_CODE( sei.getObjectIdx( i ), sei.getLog2MaxObjectIdxUpdated() );  // u(v)
      size_t k = sei.getObjectIdx( i );
      WRITE_CODE( sei.getObjectCancelFlag( k ), 1 );  // u(1)
      if ( sei.getObjectCancelFlag( k ) ) {
        if ( sei.getObjectLabelPresentFlag() ) {
          WRITE_CODE( sei.getObjectLabelUpdateFlag( k ), 1 );  // u(1)
          if ( sei.getObjectLabelUpdateFlag( k ) ) {
            WRITE_UVLC( sei.getObjectLabelIdx( k ) );  // ue(v)
          }
        }
        if ( sei.getPriorityPresentFlag() ) {
          WRITE_CODE( sei.getPriorityUpdateFlag( k ), 1 );  // u(1)
          if ( sei.getPriorityUpdateFlag( k ) ) {
            WRITE_CODE( sei.getPriorityValue( k ), 4 );  // u(4)
          }
        }
        if ( sei.getObjectHiddenPresentFlag() ) {
          WRITE_CODE( sei.getObjectHiddenFlag( k ), 1 );  // u(1)
        }
        if ( sei.getObjectDependencyPresentFlag() ) {
          WRITE_CODE( sei.getObjectDependencyUpdateFlag( k ), 1 );  // u(1)
          if ( sei.getObjectDependencyUpdateFlag( k ) ) {
            WRITE_CODE( sei.getObjectNumDependencies( k ), 4 );  // u(4)
            size_t bitCount = ceil( log2( sei.getObjectNumDependencies( k ) ) + 0.5 );
            for ( size_t j = 0; j < sei.getObjectNumDependencies( k ); j++ ) {
              WRITE_CODE( sei.getObjectDependencyIdx( k, j ), bitCount );  // u(v)
            }
          }
        }
        if ( sei.getVisibilityConesPresentFlag() ) {
          WRITE_CODE( sei.getVisibilityConesUpdateFlag( k ), 1 );  // u(1)
          if ( sei.getVisibilityConesUpdateFlag( k ) ) {
            WRITE_CODE( sei.getDirectionX( k ), 16 );  // u(16)
            WRITE_CODE( sei.getDirectionY( k ), 16 );  // u(16)
            WRITE_CODE( sei.getDirectionZ( k ), 16 );  // u(16)
            WRITE_CODE( sei.getAngle( k ), 16 );       // u(16)
          }
        }  // cones

        if ( sei.get3dBoundingBoxPresentFlag() ) {
          WRITE_CODE( sei.get3dBoundingBoxUpdateFlag( k ), 1 );  // u(1)
          if ( sei.get3dBoundingBoxUpdateFlag( k ) ) {
            WRITE_UVLC( sei.get3dBoundingBoxX( k ) );       // ue(v)
            WRITE_UVLC( sei.get3dBoundingBoxY( k ) );       // ue(v)
            WRITE_UVLC( sei.get3dBoundingBoxZ( k ) );       // ue(v)
            WRITE_UVLC( sei.get3dBoundingBoxDeltaX( k ) );  // ue(v)
            WRITE_UVLC( sei.get3dBoundingBoxDeltaY( k ) );  // ue(v)
            WRITE_UVLC( sei.get3dBoundingBoxDeltaZ( k ) );  // ue(v)
          }
        }  // 3dBB

        if ( sei.getCollisionShapePresentFlag() ) {
          WRITE_CODE( sei.getCollisionShapeUpdateFlag( k ), 1 );  // u(1)
          if ( sei.getCollisionShapeUpdateFlag( k ) ) {
            WRITE_CODE( sei.getCollisionShapeId( k ), 16 );  // u(16)
          }
        }  // collision
        if ( sei.getPointStylePresentFlag() ) {
          WRITE_CODE( sei.getPointStyleUpdateFlag( k ), 1 );  // u(1)
          if ( sei.getPointStyleUpdateFlag( k ) ) {
            WRITE_CODE( sei.getPointShapeId( k ), 8 );  // u(8)
            WRITE_CODE( sei.getPointSize( k ), 16 );    // u(16)
          }
        }  // pointstyle
        if ( sei.getMaterialIdPresentFlag() ) {
          WRITE_CODE( sei.getMaterialIdUpdateFlag( k ), 1 );  // u(1)
          if ( sei.getMaterialIdUpdateFlag( k ) ) {
            WRITE_CODE( sei.getMaterialId( k ), 16 );  // u(16)
          }
        }  // materialid
      }  // sei.getObjectCancelFlag(k)
    }  // for(size_t i=0; i<=sei.getNumObjectUpdates(); i++)
  }  // if( sei.getNumObjectUpdates() > 0 )
}

// F.2.8.2 Object label information SEI message syntax
void PCCBitstreamWriter::objectLabelInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIObjectLabelInformation &>( seiAbstract );
  WRITE_CODE( sei.getCancelFlag(), 1 );  // u(1)
  if ( !sei.getCancelFlag() ) {
    WRITE_CODE( sei.getLabelLanguagePresentFlag(), 1 );  // u(1)
    if ( sei.getLabelLanguagePresentFlag() ) {
      while ( !bitstream.byteAligned() ) {
        WRITE_CODE( uint32_t( 0 ), 1 );  // u(1)
      }
      WRITE_STRING( sei.getLabelLanguage() );  // st(v)
    }
    WRITE_UVLC( sei.getNumLabelUpdates() );  // ue(v)
    for ( size_t i = 0; i < sei.getNumLabelUpdates(); i++ ) {
      WRITE_UVLC( sei.getLabelIdx( i ) );         // ue(v)
      WRITE_CODE( sei.getLabelCancelFlag(), 1 );  // u(1)
      if ( !sei.getLabelCancelFlag() ) {
        while ( !bitstream.byteAligned() ) {
          WRITE_CODE( uint32_t( 0 ), 1 );  // u(1)
        }
        WRITE_STRING( sei.getLabel( sei.getLabelIdx( i ) ) );  // st(v)
      }
    }
    WRITE_CODE( sei.getPersistenceFlag(), 1 );  // u(1)
  }
};

// F.2.8.3 Patch information SEI message syntax
void PCCBitstreamWriter::patchInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIPatchInformation &>( seiAbstract );
  WRITE_CODE( sei.getPersistenceFlag(), 1 );  // u(1)
  WRITE_CODE( sei.getResetFlag(), 1 );        // u(1)
  WRITE_UVLC( sei.getNumTileUpdates() );      // ue(v)
  if ( sei.getNumTileUpdates() > 0 ) {
    WRITE_CODE( sei.getLog2MaxObjectIdxTracked(), 5 );  // u(5)
    WRITE_CODE( sei.getLog2MaxPatchIdxUpdated(), 4 );   // u(4)
  }
  for ( size_t i = 0; i < sei.getNumTileUpdates(); i++ ) {
    WRITE_UVLC( sei.getTileId( i ) );  // ue(v)
    size_t j = sei.getTileId( i );
    WRITE_CODE( sei.getTileCancelFlag( j ), 1 );  // u(1)
    WRITE_UVLC( sei.getNumPatchUpdates( j ) );    // ue(v)
    for ( size_t k = 0; k < sei.getNumPatchUpdates( j ); k++ ) {
      WRITE_CODE( sei.getPatchIdx( j, k ),
                  sei.getLog2MaxPatchIdxUpdated() );  // u(v)
      auto p = sei.getPatchIdx( j, k );
      WRITE_CODE( sei.getPatchCancelFlag( j, p ), 1 );  // u(1)
      if ( !sei.getPatchCancelFlag( j, p ) ) {
        WRITE_UVLC( sei.getPatchNumberOfObjectsMinus1( j, p ) );  // ue(v)
        for ( size_t n = 0; n < sei.getPatchNumberOfObjectsMinus1( j, p ) + 1; n++ ) {
          WRITE_CODE( sei.getPatchObjectIdx( j, p, n ),
                      sei.getLog2MaxObjectIdxTracked() );  // u(v)
        }
      }
    }
  }
};

// F.2.8.4 Volumetric rectangle information SEI message syntax
void PCCBitstreamWriter::volumetricRectangleInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIVolumetricRectangleInformation &>( seiAbstract );
  WRITE_CODE( sei.getPersistenceFlag(), 1 );    // u(1)
  WRITE_CODE( sei.getResetFlag(), 1 );          // u(1)
  WRITE_UVLC( sei.getNumRectanglesUpdates() );  // ue(v)
  if ( sei.getNumRectanglesUpdates() > 0 ) {
    WRITE_CODE( sei.getLog2MaxObjectIdxTracked(), 5 );     // u(5)
    WRITE_CODE( sei.getLog2MaxRectangleIdxUpdated(), 4 );  // u(4)
  }
  for ( size_t k = 0; k < sei.getNumRectanglesUpdates(); k++ ) {
    WRITE_CODE( sei.getRectangleIdx( k ),
                sei.getLog2MaxRectangleIdxUpdated() );  // u(v)
    auto p = sei.getRectangleIdx( k );
    WRITE_CODE( sei.getRectangleCancelFlag( p ), 1 );  // u(1)
    if ( !sei.getRectangleCancelFlag( p ) ) {
      WRITE_CODE( sei.getBoundingBoxUpdateFlag( p ), 1 );  // u(1)
      if ( sei.getBoundingBoxUpdateFlag( p ) ) {
        WRITE_UVLC( sei.getBoundingBoxTop( p ) );     // ue(v)
        WRITE_UVLC( sei.getBoundingBoxLeft( p ) );    // ue(v)
        WRITE_UVLC( sei.getBoundingBoxWidth( p ) );   // ue(v)
        WRITE_UVLC( sei.getBoundingBoxHeight( p ) );  // ue(v)
      }
      WRITE_UVLC( sei.getRectangleNumberOfObjectsMinus1( p ) );  // ue(v)
      for ( size_t n = 0; n < sei.getRectangleNumberOfObjectsMinus1( p ) + 1; n++ ) {
        WRITE_CODE( sei.getRectangleObjectIdx( p, n ),
                    sei.getLog2MaxObjectIdxTracked() );  // u(v)
      }
    }
  }
};

// F.2.8.5  Atlas object information  SEI message syntax
void PCCBitstreamWriter::atlasObjectInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIAtlasInformation &>( seiAbstract );
  WRITE_CODE( sei.getPersistenceFlag(), 1 );   //	u(1)
  WRITE_CODE( sei.getResetFlag(), 1 );         // 	u(1)
  WRITE_CODE( sei.getNumAtlasesMinus1(), 6 );  // 	u(6)
  WRITE_UVLC( sei.getNumUpdates() );           // ue(v)
  if ( sei.getNumUpdates() > 0 ) {
    WRITE_CODE( sei.getLog2MaxObjectIdxTracked(), 5 );  //	u(5)
    for ( size_t i = 0; i < sei.getNumAtlasesMinus1() + 1; i++ ) {
      WRITE_CODE( sei.getAtlasId( i ), 5 );  // 	u(6)
    }
    for ( size_t i = 0; i < sei.getNumUpdates() + 1; i++ ) {
      WRITE_CODE( sei.getObjectIdx( i ), sei.getLog2MaxObjectIdxTracked() );  // u(v)
      for ( size_t j = 0; j < sei.getNumAtlasesMinus1() + 1; j++ ) {
        WRITE_CODE( sei.getObjectInAtlasPresentFlag( i, j ), 1 );  // u(1)
      }
    }
  }
}

// F.2.9  Buffering period SEI message syntax
void PCCBitstreamWriter::bufferingPeriod( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIBufferingPeriod &>( seiAbstract );
  WRITE_CODE( sei.getNalHrdParamsPresentFlag(), 1 );             // u(1)
  WRITE_CODE( sei.getAclHrdParamsPresentFlag(), 1 );             // u(1)
  WRITE_CODE( sei.getInitialCabRemovalDelayLengthMinus1(), 5 );  // u(5)
  WRITE_CODE( sei.getAuCabRemovalDelayLengthMinus1(), 5 );       // u(5)
  WRITE_CODE( sei.getDabOutputDelayLengthMinus1(), 5 );          // u(5)
  WRITE_CODE( sei.getIrapCabParamsPresentFlag(), 1 );            // u(1)
  if ( sei.getIrapCabParamsPresentFlag() ) {
    WRITE_CODE( sei.getCabDelayOffset(), sei.getAuCabRemovalDelayLengthMinus1() + 1 );  // u(v)
    WRITE_CODE( sei.getDabDelayOffset(), sei.getDabOutputDelayLengthMinus1() + 1 );     // u(v)
  }
  WRITE_CODE( sei.getConcatenationFlag(), 1 );                                                         // u(1)
  WRITE_CODE( sei.getAtlasCabRemovalDelayDeltaMinus1(), sei.getAuCabRemovalDelayLengthMinus1() + 1 );  // u(v)
  WRITE_CODE( sei.getMaxSubLayersMinus1(), 3 );                                                        // u(3)
  int32_t bitCount = sei.getInitialCabRemovalDelayLengthMinus1() + 1;
  for ( size_t i = 0; i <= sei.getMaxSubLayersMinus1(); i++ ) {
    WRITE_CODE( sei.getHrdCabCntMinus1( i ), 3 );  // u(3)
    if ( sei.getNalHrdParamsPresentFlag() ) {
      for ( size_t j = 0; j < sei.getHrdCabCntMinus1( i ) + 1; j++ ) {
        WRITE_CODE( sei.getNalInitialCabRemovalDelay( i, j ), bitCount );   // u(v)
        WRITE_CODE( sei.getNalInitialCabRemovalOffset( i, j ), bitCount );  // u(v)
        if ( sei.getIrapCabParamsPresentFlag() ) {
          WRITE_CODE( sei.getNalInitialAltCabRemovalDelay( i, j ), bitCount );   // u(v)
          WRITE_CODE( sei.getNalInitialAltCabRemovalOffset( i, j ), bitCount );  // u(v)
        }
      }
    }
    if ( sei.getAclHrdParamsPresentFlag() ) {
      for ( size_t j = 0; j < sei.getHrdCabCntMinus1( i ) + 1; j++ ) {
        WRITE_CODE( sei.getAclInitialCabRemovalDelay( i, j ), bitCount );   // u(v)
        WRITE_CODE( sei.getAclInitialCabRemovalOffset( i, j ), bitCount );  // u(v)
        if ( sei.getIrapCabParamsPresentFlag() ) {
          WRITE_CODE( sei.getAclInitialAltCabRemovalDelay( i, j ), bitCount );   // u(v)
          WRITE_CODE( sei.getAclInitialAltCabRemovalOffset( i, j ), bitCount );  // u(v)
        }
      }
    }
  }
}

// F.2.10  Atlas frame timing SEI message syntax
void PCCBitstreamWriter::atlasFrameTiming( PCCBitstream &bitstream,
                                           SEI          &seiAbstract,
                                           SEI          &seiBufferingPeriodAbstract,
                                           bool          cabDabDelaysPresentFlag ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei   = static_cast<SEIAtlasFrameTiming &>( seiAbstract );
  auto &bpsei = static_cast<SEIBufferingPeriod &>( seiBufferingPeriodAbstract );
  if ( cabDabDelaysPresentFlag ) {
    for ( uint32_t i = 0; i <= bpsei.getMaxSubLayersMinus1(); i++ ) {
      WRITE_CODE( sei.getAftCabRemovalDelayMinus1( i ), bpsei.getAuCabRemovalDelayLengthMinus1() + 1 );  // u(v)
      WRITE_CODE( sei.getAftDabOutputDelay( i ), bpsei.getDabOutputDelayLengthMinus1() + 1 );            // u(v)
    }
  }
}

// F.2.11	Viewport SEI messages family syntax
// F.2.11.1	Viewport camera parameters SEI messages syntax
void PCCBitstreamWriter::viewportCameraParameters( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIViewportCameraParameters &>( seiAbstract );
  WRITE_CODE( sei.getCameraId(), 10 );   // u(10)
  WRITE_CODE( sei.getCancelFlag(), 1 );  // u(1)
  if ( sei.getCameraId() > 0 && !sei.getCancelFlag() ) {
    WRITE_CODE( sei.getPersistenceFlag(), 1 );              // u(1)
    WRITE_CODE( sei.getCameraType(), 3 );                   // u(3)
    if ( sei.getCameraType() == 0 ) {                       // equirectangular
      WRITE_CODE( sei.getErpHorizontalFov(), 32 );          // u(32)
      WRITE_CODE( sei.getErpVerticalFov(), 32 );            // u(32)
    } else if ( sei.getCameraType() == 1 ) {                // perspective
      WRITE_FLOAT( sei.getPerspectiveAspectRatio() );       // fl(32)
      WRITE_CODE( sei.getPerspectiveHorizontalFov(), 32 );  // u(32)
    } else if ( sei.getCameraType() == 2 ) {                // orthographic
      WRITE_FLOAT( sei.getOrthoAspectRatio() );             // fl(32)
      WRITE_FLOAT( sei.getOrthoHorizontalSize() );          // fl(32)
    }
    WRITE_FLOAT( sei.getClippingNearPlane() );  // fl(32)
    WRITE_FLOAT( sei.getClippingFarPlane() );   // fl(32)
  }
}

// F.2.11.2	Viewport position SEI messages syntax
void PCCBitstreamWriter::viewportPosition( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto &sei = static_cast<SEIViewportPosition &>( seiAbstract );
  WRITE_UVLC( sei.getViewportId() );                      // ue(v
  WRITE_CODE( sei.getCameraParametersPresentFlag(), 1 );  // u(1)
  if ( sei.getCameraParametersPresentFlag() ) {
    WRITE_CODE( sei.getVcpCameraId(), 10 );  //	u(10)
  }
  WRITE_CODE( sei.getCancelFlag(), 1 );  // u(1)
  if ( !sei.getCancelFlag() ) {
    WRITE_CODE( sei.getPersistenceFlag(), 1 );  // u(1)
    for ( size_t d = 0; d < 3; d++ ) {
      WRITE_FLOAT( sei.getPosition( d ) );  //	fl(32)
    }
    WRITE_CODE( sei.getRotationQX(), 16 );     //	i(16)
    WRITE_CODE( sei.getRotationQY(), 16 );     //	i(16)
    WRITE_CODE( sei.getRotationQZ(), 16 );     //	i(16)
    WRITE_CODE( sei.getCenterViewFlag(), 1 );  // 	u(1)
    if ( !sei.getCenterViewFlag() ) {
      WRITE_CODE( sei.getLeftViewFlag(), 1 );  // u(1)
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// F.2.12	Decoded Atlas Information Hash SEI message syntax
// F.2.12.1 Decoded Atlas Information Hash SEI syntax
void PCCBitstreamWriter::decodedAtlasInformationHash( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIDecodedAtlasInformationHash &>( seiAbstract );
  WRITE_CODE( sei.getCancelFlag(), 1 );  // u(1)
  if ( !sei.getCancelFlag() ) {
    WRITE_CODE( sei.getPersistenceFlag(), 1 );                      // u(1)
    WRITE_CODE( sei.getHashType(), 8 );                             // u(8)
    WRITE_CODE( sei.getDecodedHighLevelHashPresentFlag(), 1 );      // u(1)
    WRITE_CODE( sei.getDecodedAtlasHashPresentFlag(), 1 );          // u(1)
    WRITE_CODE( sei.getDecodedAtlasB2pHashPresentFlag(), 1 );       // u(1)
    WRITE_CODE( sei.getDecodedAtlasTilesHashPresentFlag(), 1 );     // u(1)
    WRITE_CODE( sei.getDecodedAtlasTilesB2pHashPresentFlag(), 1 );  // u(1)
    WRITE_CODE( 0, 1 );                                             // u(1)
    if ( sei.getDecodedHighLevelHashPresentFlag() ) { decodedHighLevelHash( bitstream, sei ); }
    if ( sei.getDecodedAtlasHashPresentFlag() ) { decodedAtlasHash( bitstream, sei ); }
    if ( sei.getDecodedAtlasB2pHashPresentFlag() ) { decodedAtlasB2pHash( bitstream, sei ); }
    if ( sei.getDecodedAtlasTilesHashPresentFlag() || sei.getDecodedAtlasTilesB2pHashPresentFlag() ) {
      WRITE_UVLC( sei.getNumTilesMinus1() );   // ue(v)
      WRITE_UVLC( sei.getTileIdLenMinus1() );  // ue(v)
      for ( size_t t = 0; t <= sei.getNumTilesMinus1(); t++ ) {
        WRITE_CODE( sei.getTileId( t ), sei.getTileIdLenMinus1() + 1 );  // u(v)
      }
      byteAlignment( bitstream );
      for ( size_t t = 0; t <= sei.getNumTilesMinus1(); t++ ) {
        size_t j = sei.getTileId( t );
        if ( sei.getDecodedAtlasTilesHashPresentFlag() ) { decodedAtlasTilesHash( bitstream, sei, j ); }
        if ( sei.getDecodedAtlasTilesB2pHashPresentFlag() ) { decodedAtlasTilesB2pHash( bitstream, sei, j ); }
      }
    }
  }
}

// F.2.12.2 Decoded high level hash unit syntax
void PCCBitstreamWriter::decodedHighLevelHash( PCCBitstream &bitstream, SEI &seiAbs ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      WRITE_CODE( sei.getHighLevelMd5( i ), 8 );  // b(8)
    }
  } else if ( hType == 1 ) {
    WRITE_CODE( sei.getHighLevelCrc(), 16 );  // u(16)
  } else if ( hType == 2 ) {
    WRITE_CODE( sei.getHighLevelCheckSum(), 32 );  // u(32)
  }
}

// F.2.12.3 Decoded atlas hash unit syntax
void PCCBitstreamWriter::decodedAtlasHash( PCCBitstream &bitstream, SEI &seiAbs ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      WRITE_CODE( sei.getAtlasMd5( i ), 8 );  // b(8)
    }
  } else if ( hType == 1 ) {
    WRITE_CODE( sei.getAtlasCrc(), 16 );  // u(16)
  } else if ( hType == 2 ) {
    WRITE_CODE( sei.getAtlasCheckSum(), 32 );  // u(32)
  }
}

// F.2.12.4 Decoded atlas b2p hash unit syntax
void PCCBitstreamWriter::decodedAtlasB2pHash( PCCBitstream &bitstream, SEI &seiAbs ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      WRITE_CODE( sei.getAtlasB2pMd5( i ), 8 );  // b(8)
    }
  } else if ( hType == 1 ) {
    WRITE_CODE( sei.getAtlasB2pCrc(), 16 );  // u(16)
  } else if ( hType == 2 ) {
    WRITE_CODE( sei.getAtlasB2pCheckSum(), 32 );  // u(32)
  }
}

// F.2.12.5 Decoded atlas tile hash unit syntax
void PCCBitstreamWriter::decodedAtlasTilesHash( PCCBitstream &bitstream, SEI &seiAbs, size_t id ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      WRITE_CODE( sei.getAtlasTilesMd5( id, i ), 8 );  // b(8)
    }
  } else if ( hType == 1 ) {
    WRITE_CODE( sei.getAtlasTilesCrc( id ), 16 );  // u(16)
  } else if ( hType == 2 ) {
    WRITE_CODE( sei.getAtlasTilesCheckSum( id ), 32 );  // u(32)
  }
}

// F.2.12.6 Decoded atlas tile b2p hash unit
void PCCBitstreamWriter::decodedAtlasTilesB2pHash( PCCBitstream &bitstream, SEI &seiAbs, size_t id ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      WRITE_CODE( sei.getAtlasTilesB2pMd5( id, i ), 8 );  // b(8)
    }
  } else if ( hType == 1 ) {
    WRITE_CODE( sei.getAtlasTilesB2pCrc( id ), 16 );  // u(16)
  } else if ( hType == 2 ) {
    WRITE_CODE( sei.getAtlasTilesB2pCheckSum( id ), 32 );  // u(32)
  }
}

// F.2.13 Time code SEI message syntax
void PCCBitstreamWriter::timeCode( PCCBitstream &bitstream, SEI &seiAbstract ) {
  auto &sei = static_cast<SEITimeCode &>( seiAbstract );
  WRITE_CODE( sei.getNumUnitsInTick(), 32 );    // u(32)
  WRITE_CODE( sei.getTimeScale(), 32 );         // u(32)
  WRITE_CODE( sei.getCountingType(), 5 );       // u(5)
  WRITE_CODE( sei.getFullTimestampFlag(), 1 );  // u(1)
  WRITE_CODE( sei.getDiscontinuityFlag(), 1 );  // u(1)
  WRITE_CODE( sei.getCntDroppedFlag(), 1 );     // u(1)
  WRITE_CODE( sei.getNFrames(), 9 );            // u(9)
  if ( sei.getFullTimestampFlag() ) {
    WRITE_CODE( sei.getSecondsValue(), 6 );  // u(6)
    WRITE_CODE( sei.getMinutesValue(), 6 );  // u(6)
    WRITE_CODE( sei.getHoursValue(), 5 );    // u(5)
  } else {
    WRITE_CODE( sei.getSecondFlag(), 1 );  // u(1)
    if ( sei.getSecondFlag() ) {
      WRITE_CODE( sei.getSecondsValue(), 6 );  // u(6)
      WRITE_CODE( sei.getMinutesFlag(), 1 );   // u(1)
      if ( sei.getMinutesFlag() ) {
        WRITE_CODE( sei.getMinutesValue(), 6 );  // u(6)
        WRITE_CODE( sei.getHoursFlag(), 1 );     // u(1)
        if ( sei.getHoursFlag() ) {
          WRITE_CODE( sei.getHoursValue(), 5 );  // u(5)
        }
      }
    }
  }
  WRITE_CODE( sei.getTimeOffsetLength(), 5 );  // u(5)
  if ( sei.getTimeOffsetLength() > 0 ) {
    WRITE_CODES( sei.getTimeOffsetValue(), sei.getTimeOffsetLength() );  // i(v)
  }
}

// F.2.14	Packed independent regions SEI payload syntax

// F.2.15	Custom attribute information SEI payload syntax

// G.2 VUI syntax
// G.2.1 VUI parameters syntax
void PCCBitstreamWriter::vuiParameters( PCCBitstream  &bitstream,
                                        VUIParameters &vp,
                                        uint32_t       geometry3dBitdepthMinus1 ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  const uint32_t n = geometry3dBitdepthMinus1 + 1;
  WRITE_CODE( vp.getTimingInfoPresentFlag(), 1 );  // u(1)
  if ( vp.getTimingInfoPresentFlag() ) {
    WRITE_CODE( vp.getNumUnitsInTick(), 32 );              // u(32)
    WRITE_CODE( vp.getTimeScale(), 32 );                   // u(32)
    WRITE_CODE( vp.getPocProportionalToTimingFlag(), 1 );  // u(1)
    if ( vp.getPocProportionalToTimingFlag() ) {
      WRITE_UVLC( vp.getNumTicksPocDiffOneMinus1() );  // ue(v)
    }
    WRITE_CODE( vp.getHrdParametersPresentFlag(), 1 );  // u(1)
    if ( vp.getHrdParametersPresentFlag() ) { hrdParameters( bitstream, vp.getHrdParameters() ); }
  }
  WRITE_CODE( vp.getTileRestrictionsPresentFlag(), 1 );  // u(1)
  if ( vp.getTileRestrictionsPresentFlag() ) {
    WRITE_CODE( vp.getFixedAtlasTileStructureFlag(), 1 );          // u(1)
    WRITE_CODE( vp.getFixedVideoTileStructureFlag(), 1 );          //	u(1)
    WRITE_UVLC( vp.getConstrainedTilesAcrossV3cComponentsIdc() );  // ue(v)
    WRITE_UVLC( vp.getMaxNumTilesPerAtlasMinus1() );               // 	ue(v)
  }
  WRITE_CODE( vp.getMaxCodedVideoResolutionPresentFlag(), 1 );  // u(1)
  if ( vp.getMaxCodedVideoResolutionPresentFlag() ) {
    maxCodedVideoResolution( bitstream, vp.getMaxCodedVideoResolution() );
  }
  WRITE_CODE( vp.getCoordinateSystemParametersPresentFlag(), 1 );  // u(1)
  if ( vp.getCoordinateSystemParametersPresentFlag() ) {
    coordinateSystemParameters( bitstream, vp.getCoordinateSystemParameters() );
  }
  WRITE_CODE( vp.getUnitInMetresFlag(), 1 );           // u(1)
  WRITE_CODE( vp.getDisplayBoxInfoPresentFlag(), 1 );  // u(1)
  if ( vp.getDisplayBoxInfoPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      WRITE_CODE( vp.getDisplayBoxOrigin( d ), n );  // u(v)
      WRITE_CODE( vp.getDisplayBoxSize( d ), n );    // u(v)
    }
    WRITE_CODE( vp.getAnchorPointPresentFlag(), 1 );  // u(1)
    if ( vp.getAnchorPointPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        WRITE_CODE( vp.getAnchorPoint( d ), n );  // u(v)
      }
    }
  }
}

// G.2.2  HRD parameters syntax
void PCCBitstreamWriter::hrdParameters( PCCBitstream &bitstream, HrdParameters &hp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  WRITE_CODE( hp.getNalParametersPresentFlag(), 1 );  // u(1)
  WRITE_CODE( hp.getAclParametersPresentFlag(), 1 );  // u(1)
  if ( hp.getNalParametersPresentFlag() || hp.getAclParametersPresentFlag() ) {
    WRITE_CODE( hp.getBitRateScale(), 4 );  // u(4)
    WRITE_CODE( hp.getCabSizeScale(), 4 );  // u(4)
  }
  for ( size_t i = 0; i <= hp.getMaxNumSubLayersMinus1(); i++ ) {
    WRITE_CODE( hp.getFixedAtlasRateGeneralFlag( i ), 1 );  // u(1)
    if ( !hp.getFixedAtlasRateGeneralFlag( i ) ) {
      WRITE_CODE( hp.getFixedAtlasRateWithinCasFlag( i ), 1 );  // u(1)
    }
    if ( hp.getFixedAtlasRateWithinCasFlag( i ) ) {
      WRITE_CODE( hp.getElementalDurationInTcMinus1( i ), 1 );  // ue(v)
    } else {
      WRITE_CODE( hp.getLowDelayFlag( i ), 1 );  // u(1)
    }
    if ( !hp.getLowDelayFlag( i ) ) {
      WRITE_CODE( hp.getCabCntMinus1( i ), 1 );  // ue(v)
    }
    if ( hp.getNalParametersPresentFlag() ) {
      hrdSubLayerParameters( bitstream, hp.getHdrSubLayerParameters( 0, i ), hp.getCabCntMinus1( i ) );
    }
    if ( hp.getAclParametersPresentFlag() ) {
      hrdSubLayerParameters( bitstream, hp.getHdrSubLayerParameters( 1, i ), hp.getCabCntMinus1( i ) );
    }
  }
}

// G.2.3  Sub-layer HRD parameters syntax
void PCCBitstreamWriter::hrdSubLayerParameters( PCCBitstream &bitstream, HrdSubLayerParameters &hlsp, size_t cabCnt ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  for ( size_t i = 0; i <= cabCnt; i++ ) {
    WRITE_UVLC( hlsp.getBitRateValueMinus1( i ) );  // ue(v)
    WRITE_UVLC( hlsp.getCabSizeValueMinus1( i ) );  // ue(v)
    WRITE_CODE( hlsp.getCbrFlag( i ), 1 );          // u(1)
  }
}

// G.2.4 Maximum coded video resolution syntax
void PCCBitstreamWriter::maxCodedVideoResolution( PCCBitstream &bitstream, MaxCodedVideoResolution &mcvr ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  WRITE_CODE( mcvr.getOccupancyResolutionPresentFlag(), 1 );  // u(1)
  WRITE_CODE( mcvr.getGeometryResolutionPresentFlag(), 1 );   // u(1)
  WRITE_CODE( mcvr.getAttributeResolutionPresentFlag(), 1 );  // u(1)
  if ( mcvr.getOccupancyResolutionPresentFlag() ) {
    WRITE_UVLC( mcvr.getOccupancyWidth() );   // ue(v)
    WRITE_UVLC( mcvr.getOccupancyHeight() );  // ue(v)
  }
  if ( mcvr.getGeometryResolutionPresentFlag() ) {
    WRITE_UVLC( mcvr.getGeometryWidth() );   // ue(v)
    WRITE_UVLC( mcvr.getGeometryHeight() );  // ue(v)
  }
  if ( mcvr.getAttributeResolutionPresentFlag() ) {
    WRITE_UVLC( mcvr.getAttributeWidth() );   // ue(v)
    WRITE_UVLC( mcvr.getAttributeHeight() );  // ue(v)
  }
}

// G.2.5 Coordinate system parameters syntax
void PCCBitstreamWriter::coordinateSystemParameters( PCCBitstream &bitstream, CoordinateSystemParameters &csp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  WRITE_CODE( csp.getForwardAxis(), 2 );    // u(2)
  WRITE_CODE( csp.getDeltaLeftAxis(), 1 );  // u(1)
  WRITE_CODE( csp.getForwardSign(), 1 );    // u(1)
  WRITE_CODE( csp.getLeftSign(), 1 );       // u(1)
  WRITE_CODE( csp.getUpSign(), 1 );         // u(1)
}

// H.8	Syntax and Semantics
// H.8.3.6.1.3	ASPS V-PCC extension syntax
void PCCBitstreamWriter::aspsVpccExtension( PCCBitstream                  &bitstream,
                                            AtlasSequenceParameterSetRbsp &asps,
                                            AspsVpccExtension             &ext ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( ext.getRemoveDuplicatePointEnableFlag(), 1 );  // u(1)
  if ( asps.getPixelDeinterleavingFlag() || asps.getPLREnabledFlag() ) {
    WRITE_UVLC( ext.getSurfaceThicknessMinus1() );  // ue(v)
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.2	AAPS V-PCC extension syntax
void PCCBitstreamWriter::aapsVpccExtension( PCCBitstream &bitstream, AapsVpccExtension &ext ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( ext.getCameraParametersPresentFlag(), 1 );  // u(1);
  if ( ext.getCameraParametersPresentFlag() ) { atlasCameraParameters( bitstream, ext.getAtlasCameraParameters() ); }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.3	Atlas camera parameters syntax
void PCCBitstreamWriter::atlasCameraParameters( PCCBitstream &bitstream, AtlasCameraParameters &acp ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( acp.getCameraModel(), 8 );  // u(8)
  if ( acp.getCameraModel() == 1 ) {
    WRITE_CODE( acp.getScaleEnabledFlag(), 1 );     // u(1)
    WRITE_CODE( acp.getOffsetEnabledFlag(), 1 );    // u(1)
    WRITE_CODE( acp.getRotationEnabledFlag(), 1 );  // u(1)
    if ( acp.getScaleEnabledFlag() ) {
      for ( size_t i = 0; i < 3; i++ ) { WRITE_CODE( acp.getScaleOnAxis( i ), 32 ); }  // u(32)
    }
    if ( acp.getOffsetEnabledFlag() ) {
      for ( size_t i = 0; i < 3; i++ ) { WRITE_CODES( acp.getOffsetOnAxis( i ), 32 ); }  // i(32)
    }
    if ( acp.getRotationEnabledFlag() ) {
      for ( size_t i = 0; i < 3; i++ ) { WRITE_CODES( acp.getRotation( i ), 16 ); }  // i(16)
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.4	AAPS V-PCC GS extension syntax
void PCCBitstreamWriter::aapsVpccGsExtension( PCCBitstream &bitstream, AapsVpccGsExtension &ext ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  uint32_t numShCoeffs( 0 ), numShComps( 0 );
  if ( ext.getDequantizationPresentFlag() ) {
    auto &dequant   = ext.getGsDequantization();
    int   numLevels = dequant.getShNumLvl();
    WRITE_UVLC( numLevels );
    for ( int i = 0; i < numLevels; i++ ) {
      WRITE_UVLC( dequant.getShNumChan( i ) );
      numShComps += dequant.getShNumChan( i );
    }
  } else {
    WRITE_UVLC( 0 );
  }
  numShCoeffs = numShComps / 3;

  WRITE_CODE( ext.getDequantizationPresentFlag(), 1 );       // u(1)
  WRITE_CODE( ext.getAdaptiveQuantizationEnableFlag(), 1 );  // u(1)
  WRITE_CODE( ext.getShPropertyPresentFlag(), 1 );           // u(1)
  WRITE_CODE( ext.getPredEnableFlag() ? 1 : 0, 1 );          // u(1)
  WRITE_CODE( ext.getTransform2DPresentFlag(), 1 );          // u(1)
  WRITE_CODE( ext.getScaleTransformPresentFlag(), 1 );       // u(1)
  WRITE_CODE( ext.getGeometryPredictionPresentFlag(), 1 );   // u(1)
  WRITE_UVLC( ext.getRawPatchDataArrangementMethod() );      // uvlc
  WRITE_CODE( ext.getLinearOpacityFlag(), 1 );               // u(1)
  WRITE_CODE( ext.getReservedZero8bits(), 8 );               // u(8)

  if ( ext.getDequantizationPresentFlag() ) { gsDequantization( bitstream, ext.getGsDequantization() ); }
  if ( ext.getShPropertyPresentFlag() ) { gsShProperty( bitstream, ext.getGsShProperty(), numShComps ); }
  if ( ext.getPredEnableFlag() ) { gsPrediction( bitstream, ext.getGsPrediction(), numShComps ); }
  if ( ext.getTransform2DPresentFlag() ) {
    gs2DTransform( bitstream, ext.getGs2DTransform(), ext.getGsShProperty(), numShCoeffs );
  }
  if ( ext.getScaleTransformPresentFlag() ) { gsScaleTransform( bitstream, ext.getGsScaleTransform() ); }
  if ( ext.getGeometryPredictionPresentFlag() ) { gsGeometryPrediction( bitstream, ext.getGsGeometryPrediction() ); }

  byteAlignment( bitstream );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.5	Gaussian splat dequantization syntax
void PCCBitstreamWriter::gsDequantization( PCCBitstream &bitstream, GsDequantization &gsd ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_UVLC( gsd.getDequantizationType() );
  if ( gsd.getDequantizationType() == 0 ) {
    for ( int i = 0; i < 3; i++ ) {
      WRITE_FLOAT( gsd.getMinPosition( i ) );
      WRITE_FLOAT( gsd.getMaxPosition( i ) );
    }
    WRITE_CODE( gsd.getRotationPresentFlag(), 1 );
    if ( gsd.getRotationPresentFlag() ) {
      WRITE_CODE( gsd.getRotSameMinMaxFlag(), 1 );
      auto gsd_rot_quaternion_representation_type_flag = gsd.getRotQuaternionRepresentationTypeFlag() ? 1 : 0;
      WRITE_CODE( gsd_rot_quaternion_representation_type_flag, 1 );
      auto GsdRotationDim = ( gsd_rot_quaternion_representation_type_flag ) ? 4 : 3;
      for ( int i = 0; i < GsdRotationDim; i++ ) {
        if ( !gsd.getRotSameMinMaxFlag() || i == 0 ) {
          WRITE_FLOAT( gsd.getMinRotation( i ) );
          WRITE_FLOAT( gsd.getMaxRotation( i ) );
        }
      }
    }
    WRITE_CODE( gsd.getScalePresentFlag(), 1 );
    if ( gsd.getScalePresentFlag() ) {
      WRITE_CODE( gsd.getScaleMultipleRegionsEnableFlag(), 1 );
      if ( gsd.getScaleMultipleRegionsEnableFlag() ) {
        WRITE_CODE( gsd.getScaleSameNumQuantRegionMinus1_(), 1 );
        if ( gsd.getScaleSameNumQuantRegionMinus1_() ) {
          WRITE_CODE( gsd.getScaleSameQuantRegionTransitionPoint_(), 1 );
          WRITE_CODE( gsd.getScaleSameRegionStepSize_(), 1 );
        }
      }
      WRITE_CODE( gsd.getScaleSameMinMaxFlag(), 1 );
      for ( int i = 0; i < 3; i++ ) {
        if ( !gsd.getScaleSameMinMaxFlag() || i == 0 ) {
          WRITE_FLOAT( gsd.getMinScale( i ) );
          WRITE_FLOAT( gsd.getMaxScale( i ) );
        }
        if ( gsd.getScaleMultipleRegionsEnableFlag() ) {
          if ( ( i == 0 ) || !gsd.getScaleSameNumQuantRegionMinus1_() )
            WRITE_UVLC( gsd.getScaleNumQuantRegionsMinus1( i ) );
          if ( gsd.getScaleNumQuantRegionsMinus1( i ) ) {
            if ( ( i == 0 ) || !gsd.getScaleSameQuantRegionTransitionPoint_() )
              for ( int j = 0; j < gsd.getScaleNumQuantRegionsMinus1( i ); j++ ) {
                WRITE_UVLC( gsd.getScaleQuantRegionTransitionPointDelta( i, j ) );
              }
            if ( ( i == 0 ) || !gsd.getScaleSameRegionStepSize_() )
              for ( int j = 0; j < gsd.getScaleNumQuantRegionsMinus1( i ); j++ ) {
                WRITE_FLOAT( gsd.getScaleRegionStepSize( i, j ) );
              }
          }
        }
      }
    }
    WRITE_CODE( gsd.getOpacityPresentFlag(), 1 );
    if ( gsd.getOpacityPresentFlag() ) {
      WRITE_FLOAT( gsd.getMinOpacity() );
      WRITE_FLOAT( gsd.getMaxOpacity() );
      WRITE_CODE( gsd.getOpacityMultipleRegionsEnableFlag(), 1 );
      if ( gsd.getOpacityMultipleRegionsEnableFlag() ) {
        WRITE_UVLC( static_cast<uint32_t>( gsd.getOpacityNumQuantRegionsMinus1() ) );
        if ( gsd.getOpacityNumQuantRegionsMinus1() > 0 ) {
          for ( int m = 0; m < gsd.getOpacityNumQuantRegionsMinus1(); m++ ) {
            WRITE_UVLC( gsd.getOpacityQuantRegionTransitionPointDelta( m ) );
          }
          for ( int m = 0; m < gsd.getOpacityNumQuantRegionsMinus1(); m++ ) {
            WRITE_FLOAT( gsd.getOpacityRegionStepSize( m ) );
          }
        }
      }
    }
    WRITE_CODE( gsd.getShPresentFlag(), 1 );
    if ( gsd.getShPresentFlag() ) {
      WRITE_CODE( gsd.getShMultipleRegionsEnableFlag(), 1 );
      if ( gsd.getShMultipleRegionsEnableFlag() ) {
        WRITE_CODE( gsd.getShSameNumRegionMius1Flag(), 1 );
        if ( gsd.getShSameNumRegionMius1Flag() ) {
          WRITE_CODE( gsd.getShSameQuantRegionTransitionPointFlag(), 1 );
          WRITE_CODE( gsd.getShSameRegionStepSizeFlag(), 1 );
        }
      }
      WRITE_CODE( gsd.getShSameMinMaxFlag(), 1 );
      for ( int i = 0; i < gsd.getShNumLvl(); i++ ) {
        for ( int c = 0; c < gsd.getShNumChan( i ); c++ ) {
          if ( !gsd.getShSameMinMaxFlag() || ( ( i == 0 ) && ( c == 0 ) ) ) {
            WRITE_FLOAT( gsd.getMinSh( i, c ) );
            WRITE_FLOAT( gsd.getMaxSh( i, c ) );
          }
          if ( gsd.getShMultipleRegionsEnableFlag() ) {
            if ( !gsd.getShSameNumRegionMius1Flag() || ( ( i == 0 ) && ( c == 0 ) ) ) {
              WRITE_UVLC( static_cast<uint32_t>( gsd.getNumQuantRegionsMinus1( i, c ) ) );
            }
            if ( gsd.getNumQuantRegionsMinus1( i, c ) > 0 ) {
              if ( !gsd.getShSameQuantRegionTransitionPointFlag() || ( ( i == 0 ) && ( c == 0 ) ) ) {
                for ( int m = 0; m < gsd.getNumQuantRegionsMinus1( i, c ); m++ ) {
                  WRITE_UVLC( gsd.getQuantRegionTransitionPointDelta( i, c, m ) );
                }
              }
              if ( !gsd.getShSameRegionStepSizeFlag() || ( ( i == 0 ) && ( c == 0 ) ) ) {
                for ( int m = 0; m < gsd.getNumQuantRegionsMinus1( i, c ); m++ ) {
                  WRITE_FLOAT( gsd.getRegionStepSize( i, c, m ) );
                }
              }
            }
          }
        }
      }
    }
  } else {
    TRACE_BITSTREAM( "Error: Values of DequantizationType greater than 0 are reserved.\n" );
    exit( EXIT_FAILURE );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.6	Gaussian splat spherical harmonics property syntax
void PCCBitstreamWriter::gsShProperty( PCCBitstream                 &bitstream,
                                       GsSphericalHarmonicsProperty &gshp,
                                       uint32_t                      numShComps ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_UVLC( gshp.getShMappingMethod() );
  if ( gshp.getShMappingMethod() == 2 ) {
    uint32_t numShComps = GsSphericalHarmonicsProperty::NUM_SH_COMPONENTS;
    {
      uint32_t bitWidth = (uint32_t)std::ceil( std::log2( (double)numShComps ) );
      if ( bitWidth < 1 ) bitWidth = 1;
      WRITE_CODE( gshp.getTotalCodedComponentsMinus1(), bitWidth );
    }
    uint32_t       tableSize = gshp.getTotalCodedComponentsMinus1() + 1;
    const uint32_t kBitWidth =
        (uint32_t)std::ceil( std::log2( (double)GsSphericalHarmonicsProperty::NUM_SH_COMPONENTS ) );
    for ( uint32_t m = 0; m < tableSize; m++ ) { WRITE_CODE( gshp.getShMappingTable( m ), kBitWidth ); }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.7	Gaussian splat spherical harmonics transform syntax
void PCCBitstreamWriter::gs2DTransform( PCCBitstream                       &bitstream,
                                        Gs2DTransform                      &gs2DTransform,
                                        const GsSphericalHarmonicsProperty &gshp,
                                        uint32_t                            numShCoeffs ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  uint32_t numBits = static_cast<uint32_t>( ceilLog2( numShCoeffs + 1 ) );
  WRITE_CODE( gs2DTransform.getTrRowNumGroups(), numBits );
  WRITE_CODE( gs2DTransform.getDimensionReductionEnableFlag(), 1 );
  for ( int i = 0; i < gs2DTransform.getTrRowNumGroups(); i++ ) {
    WRITE_CODE( gs2DTransform.getRowTrSizeMinus1( i ), 4 );
    for ( int j = 0; j < ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); j++ ) {
      WRITE_CODE( gs2DTransform.getTrRowGroupIdx( i, j ), 4 );
    }
    WRITE_CODE( gs2DTransform.getKltFlag( i ), 1 );
    if ( gs2DTransform.getKltFlag( i ) ) {
      for ( int m = 0; m < 3 * ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); m++ ) {
        for ( int n = 0; n < 3 * ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); n++ ) {
          WRITE_FLOAT( gs2DTransform.getKltCoeff( i, m, n ) );
        }
      }
      bool isDcGroup = ( gs2DTransform.getRowTrSizeMinus1( i ) == 0 && gs2DTransform.getTrRowGroupIdx( i, 0 ) == 0 );
      for ( int m = 0; m < 3 * ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); m++ ) {
        if ( !isDcGroup ) { WRITE_FLOAT( gs2DTransform.getKltStdDev( i, m ) ); }
        WRITE_FLOAT( gs2DTransform.getKltMean( i, m ) );
      }
      if ( gs2DTransform.getDimensionReductionEnableFlag() && gshp.getShMappingMethod() != 2 ) {
        uint32_t groupSize = gs2DTransform.getRowTrSizeMinus1( i ) + 1;
        uint32_t bitWidth  = (uint32_t)std::ceil( std::log2( (double)( groupSize * 3 ) ) );
        if ( bitWidth < 1 ) bitWidth = 1;
        WRITE_CODE( gs2DTransform.getNumOfRetainComponentsMinus1( i ), bitWidth );
      }
    } else {
      WRITE_CODE( gs2DTransform.getTrRowType( i ), 3 );
    }
  }
  std::vector<uint8_t> colTr;
  colTr.resize( numShCoeffs, 1 );
  for ( int i = 0; i < gs2DTransform.getTrRowNumGroups(); i++ ) {
    if ( gs2DTransform.getKltFlag( i ) ) {
      for ( int j = 0; j < ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); j++ ) {
        colTr[gs2DTransform.getTrRowGroupIdx( i, j )] = 0;
      }
    }
  }
  for ( int i = 0; i < numShCoeffs; i++ ) {
    if ( colTr[i] == 1 ) { WRITE_CODE( gs2DTransform.getTrColType( i ), 3 ); }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}
// H.8.3.6.3.10 Gaussian splat geometry prediction syntax
void PCCBitstreamWriter::gsGeometryPrediction( PCCBitstream &bitstream, GsGeometryPrediction &gsgp ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( gsgp.getGeometryPredictionEnableFlag(), 1 );
  WRITE_UVLC( gsgp.getGeometryPredictionSegmentVal() );
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.7  Gaussian splat spherical harmonics prediction syntax
void PCCBitstreamWriter::gsPrediction( PCCBitstream &bitstream, GsPrediction &pred, uint32_t numShComps ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  uint32_t numBits = static_cast<uint32_t>( ceilLog2( numShComps ) );
  for ( int i = 0; i < pred.getShNumComponents(); i++ ) {
    WRITE_CODE( pred.getUsePredictionFlag( i ) ? 1 : 0, 1 );
    if ( pred.getUsePredictionFlag( i ) ) {
      WRITE_CODE( pred.getNumPredSrcMinus1( i ), 4 );
      for ( int j = 0; j <= pred.getNumPredSrcMinus1( i ); j++ ) {
        WRITE_CODE( pred.getPredSrcIdx( i, j ), numBits );
        WRITE_FLOAT( pred.getPredCoeff( i, j ) );
      }
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.9  Gaussian splat scale transform syntax
void PCCBitstreamWriter::gsScaleTransform( PCCBitstream &bitstream, GsScaleTransform &gsst ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  WRITE_CODE( gsst.getSameScalingFlag(), 1 );
  WRITE_CODE( gsst.getSamePowerTransformFlag(), 1 );
  for ( int i = 0; i < 3; i++ ) {
    if ( i == 0 || !gsst.getSameScalingFlag() ) { WRITE_FLOAT( gsst.getScaleScaling( i ) ); }
    if ( i == 0 || !gsst.getSamePowerTransformFlag() ) { WRITE_FLOAT( gsst.getScalePower( i ) ); }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.9	Raw patch data unit V-PCC Gaussian Splat extension syntax
void PCCBitstreamWriter::rpduVpccGsExtension( PCCBitstream        &bitstream,
                                              AapsVpccGsExtension &ext,
                                              RawPatchDataUnit    &rpdu ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  if ( ext.getAdaptiveQuantizationEnableFlag() ) {
    WRITE_UVLC( rpdu.getShBitdepthDelta() );  // ue(v)
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.20.2	SEI payload syntax
// H.20.2.13 Attribute transformation parameters SEI message syntax
void PCCBitstreamWriter::attributeTransformationParams( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto &sei = static_cast<SEIAttributeTransformationParams &>( seiAbstract );
  WRITE_CODE( sei.getCancelFlag(), 1 );  // u(1)
  if ( !sei.getCancelFlag() ) {
    WRITE_UVLC( sei.getNumAttributeUpdates() );  // ue(v)
    for ( size_t j = 0; j < sei.getNumAttributeUpdates(); j++ ) {
      WRITE_CODE( sei.getAttributeIdx( j ), 8 );  // u(8)
      size_t index = sei.getAttributeIdx( j );
      WRITE_CODE( sei.getDimensionMinus1( index ), 8 );  // u(8)
      for ( size_t i = 0; i < sei.getDimensionMinus1( index ) + 1; i++ ) {
        WRITE_CODE( sei.getScaleParamsEnabledFlag( index, i ), 1 );   // u(1)
        WRITE_CODE( sei.getOffsetParamsEnabledFlag( index, i ), 1 );  // u(1)
        if ( sei.getScaleParamsEnabledFlag( index, i ) ) {
          WRITE_CODE( sei.getAttributeScale( index, i ), 32 );  // u(32)
        }
        if ( sei.getOffsetParamsEnabledFlag( index, i ) ) {
          WRITE_CODES( sei.getAttributeOffset( index, i ), 32 );  // i(32)
        }
      }
    }
    WRITE_CODE( sei.getPersistenceFlag(), 1 );  // u(1)
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.20.2.14 Occupancy synthesis SEI message syntax
void PCCBitstreamWriter::occupancySynthesis( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIOccupancySynthesis &>( seiAbstract );
  WRITE_CODE( sei.getPersistenceFlag(), 1 );   //	u(1)
  WRITE_CODE( sei.getResetFlag(), 1 );         //	u(1)
  WRITE_CODE( sei.getInstancesUpdated(), 8 );  //	u(8)
  for ( size_t i = 0; i < sei.getInstancesUpdated(); i++ ) {
    WRITE_CODE( sei.getInstanceIndex( i ), 8 );  // u(8)
    size_t k = sei.getInstanceIndex( i );
    WRITE_CODE( sei.getInstanceCancelFlag( k ), 1 );  //	u(1)
    if ( !sei.getInstanceCancelFlag( k ) ) {
      WRITE_UVLC( sei.getMethodType( k ) );  // ue(v)
      if ( sei.getMethodType( k ) == 1 ) {
        WRITE_CODE( sei.getPbfLog2ThresholdMinus1( k ), 2 );  //	u(2)
        WRITE_CODE( sei.getPbfPassesCountMinus1( k ), 2 );    //	u(2)
        WRITE_CODE( sei.getPbfFilterSizeMinus1( k ), 3 );     //	u(3)
      }
    }
  }
}

// H.20.2.15 Geometry smoothing SEI message syntax
void PCCBitstreamWriter::geometrySmoothing( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIGeometrySmoothing &>( seiAbstract );
  WRITE_CODE( sei.getPersistenceFlag(), 1 );   //	u(1)
  WRITE_CODE( sei.getResetFlag(), 1 );         //	u(1)
  WRITE_CODE( sei.getInstancesUpdated(), 8 );  //	u(8)
  for ( size_t i = 0; i < sei.getInstancesUpdated(); i++ ) {
    WRITE_CODE( sei.getInstanceIndex( i ), 8 );  // u(8)
    size_t k = sei.getInstanceIndex( i );
    WRITE_CODE( sei.getInstanceCancelFlag( k ), 1 );  //	u(1)
    if ( !sei.getInstanceCancelFlag( k ) ) {
      WRITE_UVLC( sei.getMethodType( k ) );  // ue(v)
      if ( sei.getMethodType( k ) == 1 ) {
        WRITE_CODE( sei.getFilterEomPointsFlag( k ), 1 );  // u(1)
        WRITE_CODE( sei.getGridSizeMinus2( k ), 5 );       // u(5)
        WRITE_CODE( sei.getThreshold( k ), 8 );            // u(8)
      }
    }
  }
}

// H.20.2.16 Attribute smoothing SEI message syntax
void PCCBitstreamWriter::attributeSmoothing( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIAttributeSmoothing &>( seiAbstract );
  WRITE_CODE( sei.getPersistenceFlag(), 1 );    //	u(1)
  WRITE_CODE( sei.getResetFlag(), 1 );          //	u(1)
  WRITE_UVLC( sei.getNumAttributesUpdated() );  //	ue(v)
  for ( size_t j = 0; j < sei.getNumAttributesUpdated(); j++ ) {
    WRITE_CODE( sei.getAttributeIdx( j ), 7 );  // u(7)
    size_t k = sei.getAttributeIdx( j );
    WRITE_CODE( sei.getAttributeSmoothingCancelFlag( k ), 1 );  // u(1)
    WRITE_CODE( sei.getInstancesUpdated( k ), 8 );              //	u(8)
    for ( size_t i = 0; i < sei.getInstancesUpdated( k ); i++ ) {
      WRITE_CODE( sei.getInstanceIndex( k, i ), 8 );  //	u(8)
      size_t m = sei.getInstanceIndex( k, i );
      WRITE_CODE( sei.getInstanceCancelFlag( k, m ), 1 );  // u(1)
      if ( sei.getInstanceCancelFlag( k, m ) != 1 ) {
        WRITE_UVLC( sei.getMethodType( k, m ) );  // ue(v)
        if ( sei.getMethodType( k, m ) ) {
          WRITE_CODE( sei.getFilterEomPointsFlag( k, m ), 1 );  // u(1)
          WRITE_CODE( sei.getGridSizeMinus2( k, m ), 5 );       //	u(5)
          WRITE_CODE( sei.getThreshold( k, m ), 8 );            // u(8)
          WRITE_CODE( sei.getThresholdVariation( k, m ), 8 );   // u(8)
          WRITE_CODE( sei.getThresholdDifference( k, m ), 8 );  // u(8)
        }
      }
    }
  }
}

// H.20.2.17	V-PCC registered SEI message syntax
void PCCBitstreamWriter::vpccRegisteredSEI( PCCBitstream &bitstream, SEI &seiAbstract ) {
  auto &sei         = static_cast<SEIVPCCRegistered &>( seiAbstract );
  auto  payloadType = static_cast<int32_t>( sei.getVpccPayloadType() );
  for ( ; payloadType >= 0xff; payloadType -= 0xff ) {
    WRITE_CODE( 0xff, 8 );  // u(8)
  }
  WRITE_CODE( payloadType, 8 );  // u(8)
  vpccRegisteredSeiPayload( bitstream, static_cast<VpccSeiPayloadType>( payloadType ), seiAbstract );
}

// H.20.2.18	V-PCC registered SEI payload syntax
void PCCBitstreamWriter::vpccRegisteredSeiPayload( PCCBitstream      &bitstream,
                                                   VpccSeiPayloadType vpccPayloadType,
                                                   SEI               &seiAbstract ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  if ( vpccPayloadType == GS_CAMERA_MODEL_INFO ) {  // 0
    gsCameraModelInfo( bitstream, seiAbstract );
  } else {
    reservedSeiMessage( bitstream, seiAbstract );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.20.2.19 GS Camera Model Info SEI message syntax
void PCCBitstreamWriter::gsCameraModelInfo( PCCBitstream &bitstream, SEI &seiAbstract ) {
  static int cnt = 0;
  std::cerr << "Number of Entries: " << cnt++ << std::endl;
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto &sei = static_cast<SEIGsCameraModelInfo &>( seiAbstract );
  WRITE_UVLC( sei.getCameraModelInfoId() );      // ue(v)
  WRITE_UVLC( sei.getCameraNum() );              // ue(v)
  WRITE_UVLC( sei.getImageNum() );               // ue(v)
  WRITE_CODE( sei.getSameIntrinsicsFlag(), 1 );  // u(1)
  for ( size_t i = 0; i < sei.getCameraNum(); i++ ) {
    WRITE_UVLC( sei.getCameraId( i ) );       // ue(v)
    WRITE_CODE( sei.getModelId( i ), 4 );     // u(4)
    WRITE_CODE( sei.getCancelFlag( i ), 1 );  // u(1)
    if ( !sei.getCancelFlag( i ) ) {
      WRITE_CODE( sei.getPersistenceFlag( i ), 1 );  // u(1)
      if ( i == 0 || ( !sei.getSameIntrinsicsFlag() ) ) {
        WRITE_CODE( sei.getImageWidth( i ), 16 );       // u(16)
        WRITE_CODE( sei.getImageHeight( i ), 16 );      // u(16)
        WRITE_CODE( sei.getIntrCamParamsNum( i ), 8 );  // u(8)
        for ( size_t j = 0; j < sei.getIntrCamParamsNum( i ); j++ )
          WRITE_DOUBLE( sei.getIntrCamParams( i, j ) );  // fl(64)
      }
    }
  }
  for ( size_t i = 0; i < sei.getImageNum(); i++ ) {
    WRITE_UVLC( sei.getImageId( i ) );            // ue(v)
    WRITE_UVLC( sei.getImageIdToCameraId( i ) );  // ue(v)
    size_t idx = sei.getCamIdToIdx( sei.getImageIdToCameraId( i ) );
    if ( !sei.getCancelFlag( idx ) ) {
      for ( size_t j = 0; j < 4; j++ ) WRITE_DOUBLE( sei.getQuaternion( i, j ) )   // fl(64)
      for ( size_t j = 0; j < 3; j++ ) WRITE_DOUBLE( sei.getTranslation( i, j ) )  // fl(64)
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}
