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
#include "PCCVideoBitstream.h"
#include "PCCBitstream.h"
#include "PCCHighLevelSyntax.h"
#include "PCCAtlasAdaptationParameterSetRbsp.h"
#include "PCCAccessUnitDelimiterRbsp.h"

#include "PCCBitstreamReader.h"

using namespace pcc;

#if defined( BITSTREAM_TRACE )
static std::string format( const std::string &var ) {
  std::string substr = var;
  auto        npos   = substr.find( "get" );
  if ( npos != std::string::npos ) substr = substr.substr( npos + 3 );
  npos = substr.find( "(" );
  if ( npos != std::string::npos ) substr = substr.substr( 0, npos );
  return substr;
}

#define READ_CODE_40bits( VAR, LEN )                                               \
  {                                                                                \
    VAR = bitstream.read40bits( LEN );                                             \
    bitstream.traceBit( format( #VAR ), VAR, "u(" + std::to_string( LEN ) + ")" ); \
  }

#define READ_CODE( VAR, LEN )                                                      \
  {                                                                                \
    VAR = bitstream.read( LEN );                                                   \
    bitstream.traceBit( format( #VAR ), VAR, "u(" + std::to_string( LEN ) + ")" ); \
  }
#define READ_CODE_DESC( VAR, LEN, DESC )                                           \
  {                                                                                \
    VAR = bitstream.read( LEN );                                                   \
    bitstream.traceBit( format( DESC ), VAR, "u(" + std::to_string( LEN ) + ")" ); \
  }
#define READ_UVLC_DESC( VAR, DESC )                     \
  {                                                     \
    VAR = bitstream.readUvlc();                         \
    bitstream.traceBit( format( DESC ), VAR, "ue(v)" ); \
  }
#define READ_VU( VAR )                                  \
  {                                                     \
    VAR = bitstream.readVu();                           \
    bitstream.traceBit( format( #VAR ), VAR, "vu(v)" ); \
  }
#define READ_SVLC_DESC( VAR, DESC )                     \
  {                                                     \
    VAR = bitstream.readSvlc();                         \
    bitstream.traceBit( format( DESC ), VAR, "se(v)" ); \
  }
#define READ_CODE_CAST( VAR, LEN, CAST )                                           \
  {                                                                                \
    VAR = static_cast<CAST>( bitstream.read( LEN ) );                              \
    bitstream.traceBit( format( #VAR ), VAR, "u(" + std::to_string( LEN ) + ")" ); \
  }

#define READ_CODES( VAR, LEN )                                                     \
  {                                                                                \
    VAR = bitstream.readS( LEN );                                                  \
    bitstream.traceBit( format( #VAR ), VAR, "s(" + std::to_string( LEN ) + ")" ); \
  }

#define READ_UVLC( VAR )                                \
  {                                                     \
    VAR = bitstream.readUvlc();                         \
    bitstream.traceBit( format( #VAR ), VAR, "ue(v)" ); \
  }
#define READ_UVLC_CAST( VAR, CAST )                     \
  {                                                     \
    VAR = static_cast<CAST>( bitstream.readUvlc() );    \
    bitstream.traceBit( format( #VAR ), VAR, "ue(v)" ); \
  }

#define READ_SVLC( VAR )                                \
  {                                                     \
    VAR = bitstream.readSvlc();                         \
    bitstream.traceBit( format( #VAR ), VAR, "se(v)" ); \
  }

#define READ_FLOAT( VAR )                                    \
  {                                                          \
    VAR = bitstream.readFloat();                             \
    bitstream.traceBitFloat( format( #VAR ), VAR, "f(32)" ); \
  }
#define READ_FLOAT_DESC( VAR, DESC )                         \
  {                                                          \
    VAR = bitstream.readFloat();                             \
    bitstream.traceBitFloat( format( DESC ), VAR, "f(32)" ); \
  }

#define READ_DOUBLE( VAR )                                    \
  {                                                           \
    VAR = bitstream.readDouble();                             \
    bitstream.traceBitDouble( format( #VAR ), VAR, "f(64)" ); \
  }

#define READ_STRING( VAR )                                 \
  {                                                        \
    VAR = bitstream.readString();                          \
    bitstream.traceBitStr( format( #VAR ), VAR, "str()" ); \
  }

#define READ_VECTOR( VAR, SIZE )                                                            \
  {                                                                                         \
    bitstream.read( VAR, SIZE );                                                            \
    bitstream.trace( "Data stream: %s size = %zu \n", format( #VAR ).c_str(), VAR.size() ); \
  }
#define READ_VIDEO( VAR, SIZE )                                                                       \
  {                                                                                                   \
    bitstream.read( VAR, SIZE );                                                                      \
    bitstream.trace( "Video stream: type = %s size = %zu \n", toString( VAR.type() ).c_str(), SIZE ); \
  }
#else
#define READ_CODE_DESC( VAR, LEN, DESC ) VAR = bitstream.read( LEN );
#define READ_UVLC_DESC( VAR, DESC ) VAR = bitstream.readUvlc();
#define READ_SVLC_DESC( VAR, DESC ) VAR = bitstream.readSvlc();
#define READ_CODE( VAR, LEN ) VAR = bitstream.read( LEN );
#define READ_CODE_CAST( VAR, LEN, CAST ) VAR = static_cast<CAST>( bitstream.read( LEN ) );
#define READ_CODES( VAR, LEN ) VAR = bitstream.readS( LEN );
#define READ_UVLC( VAR ) VAR = bitstream.readUvlc();
#define READ_UVLC_CAST( VAR, CAST ) VAR = static_cast<CAST>( bitstream.readUvlc() );
#define READ_SVLC( VAR ) VAR = bitstream.readSvlc();
#define READ_FLOAT( VAR ) VAR = bitstream.readFloat();
#define READ_DOUBLE( VAR ) VAR = bitstream.readDouble();
#define READ_STRING( VAR ) VAR = bitstream.readString();
#define READ_VECTOR( VAR, SIZE ) bitstream.read( VAR, SIZE );
#define READ_VIDEO( VAR, SIZE ) bitstream.read( VAR, SIZE );
#define READ_VU( VAR ) VAR = bitstream.readVu();
#define READ_CODE_40bits( VAR, LEN ) VAR = bitstream.read40bits( LEN );
#endif

PCCBitstreamReader::PCCBitstreamReader() :
    prevPatchSizeU_( 0 ), prevPatchSizeV_( 0 ), predPatchIndex_( 0 ), prevFrameIndex_( 0 ) {}
PCCBitstreamReader::~PCCBitstreamReader() = default;

// B.2  Sample stream V3C unit syntax
size_t PCCBitstreamReader::read( PCCBitstream &bitstream, SampleStreamV3CUnit &ssvu ) {
  size_t headerSize = 0;
  TRACE_BITSTREAM( "%s \n", "PCCBitstreamXXcoder: SampleStream Vpcc Unit start" );
  sampleStreamV3CHeader( bitstream, ssvu );
  headerSize++;
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 %d <=> bytesToRead %d\n", ssvu.getSsvhUnitSizePrecisionBytesMinus1(),
                   ( 8 * ( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) ) );
  size_t unitCount = 0;
  printf( "PCCBitstreamReader read: \n" );
  while ( bitstream.moreData() ) {
    auto &v3cUnit = ssvu.addV3CUnit();
    sampleStreamV3CUnit( bitstream, ssvu, v3cUnit );
    TRACE_BITSTREAM( "V3C Unit Size(%zuth/%zu)  = %zu \n", unitCount, ssvu.getV3CUnitCount(), v3cUnit.getSize() );
    unitCount++;
    headerSize += ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1;
  }
  TRACE_BITSTREAM( "%s \n", "PCCBitstreamXXcoder: SampleStream Vpcc Unit start done" );
  return headerSize;
}

int32_t PCCBitstreamReader::decode( SampleStreamV3CUnit &ssvu, PCCHighLevelSyntax &syntax ) {
  printf( "PCCBitstreamReader decode: \n" );
  bool  endOfGop     = false;
  int   numVPS       = 0;  // counter for the atlas information
  auto &bistreamStat = syntax.getBitstreamStat();
  bistreamStat.newGOF();
  while ( !endOfGop && ssvu.getV3CUnitCount() > 0 ) {
    auto       &unit        = ssvu.front();
    V3CUnitType v3cUnitType = V3C_VPS;
    v3cUnit( syntax, unit, v3cUnitType );
    if ( v3cUnitType == V3C_VPS ) {
      numVPS++;
      if ( numVPS > 1 ) {
        // remove the bits counted for the last VPS
        int32_t v3cUnitSize = (int32_t)unit.getBitstream().capacity();
        int32_t statSize    = bistreamStat.getV3CUnitSize( V3C_VPS ) - v3cUnitSize;
        bistreamStat.overwriteV3CUnitSize( V3C_VPS, statSize );
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

// 8.2 Specification of syntax functions and descriptors
bool PCCBitstreamReader::byteAligned( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return bitstream.byteAligned();
}
bool PCCBitstreamReader::moreDataInPayload( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return !bitstream.byteAligned();
}
bool PCCBitstreamReader::moreRbspData( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  // Return false if there is no more data.
  if ( !bitstream.moreData() ) { return false; }

  // Store bitstream state.
  auto position = bitstream.getPosition();

#if defined( CONFORMANCE_TRACE ) || defined( BITSTREAM_TRACE )
  bitstream.setTrace( false );
#endif

  // Skip first bit. It may be part of a RBSP or a rbsp_one_stop_bit.
  bitstream.read( 1 );

  while ( bitstream.moreData() ) {
    if ( bitstream.read( 1 ) ) {
      // We found a one bit beyond the first bit. Restore bitstream state and return true.
      bitstream.setPosition( position );
#if defined( CONFORMANCE_TRACE ) || defined( BITSTREAM_TRACE )
      bitstream.setTrace( true );
#endif
      return true;
    }
  }

  // We did not found a one bit beyond the first bit. Restore bitstream state and return false.
  bitstream.setPosition( position );
#if defined( CONFORMANCE_TRACE ) || defined( BITSTREAM_TRACE )
  bitstream.setTrace( true );
#endif
  return false;
}
bool PCCBitstreamReader::moreRbspTrailingData( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}
bool PCCBitstreamReader::moreDataInV3CUnit( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}
bool PCCBitstreamReader::payloadExtensionPresent( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  return false;
}

// 8.3.2 V3C unit syntax
// 8.3.2.1 General V3C unit syntax
void PCCBitstreamReader::v3cUnit( PCCHighLevelSyntax &syntax, V3CUnit &currV3CUnit, V3CUnitType &v3cUnitType ) {
  PCCBitstream &bitstream = currV3CUnit.getBitstream();
#ifdef BITSTREAM_TRACE
  bitstream.setTrace( true );
  bitstream.setLogger( *logger_ );
  TRACE_BITSTREAM( "PCCBitstream::(%s)\n", toString( currV3CUnit.getType() ).c_str() );
#endif
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto position = static_cast<int32_t>( bitstream.size() );
  v3cUnitHeader( syntax, bitstream, v3cUnitType );
  assert( v3cUnitType == currV3CUnit.getType() );
  printf( "  v3cUnit: size = %8zu type = %2hhu %s \n", currV3CUnit.getSize(), v3cUnitType,
          toString( V3CUnitType( v3cUnitType ) ).c_str() );
  fflush( stdout );
  v3cUnitPayload( syntax, bitstream, v3cUnitType, currV3CUnit.getSize() );
  syntax.getBitstreamStat().setV3CUnitSize( v3cUnitType, static_cast<int32_t>( bitstream.size() ) - position );
  TRACE_BITSTREAM( "v3cUnit: V3CUnitType = %d(%s) \n", v3cUnitType, toString( v3cUnitType ).c_str() );
  TRACE_BITSTREAM( "v3cUnit: size [%d ~ %d] \n", position, bitstream.size() );
  TRACE_BITSTREAM( "%s done\n", __func__ );
}

// 8.3.2.2 V3C unit header syntax
void PCCBitstreamReader::v3cUnitHeader( PCCHighLevelSyntax &syntax,
                                        PCCBitstream       &bitstream,
                                        V3CUnitType        &v3cUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  uint32_t value;
  READ_CODE_DESC( value, 5, "v3cUnitType" )
  v3cUnitType = static_cast<V3CUnitType>( value );  // u(5)

  if ( v3cUnitType == V3C_AVD || v3cUnitType == V3C_GVD || v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD ||
       v3cUnitType == V3C_CAD || v3cUnitType == V3C_PVD || v3cUnitType == V3C_BMD || v3cUnitType == V3C_ADD ) {
    auto &v3cUnitHeader = syntax.getV3CUnitHeader( static_cast<int>( v3cUnitType ) - 1 );
    READ_CODE_DESC( value, 4, "V3CParameterSetId" )
    v3cUnitHeader.setV3CParameterSetId( value );  // u(4)
    syntax.setActiveVpsId( value );
  }
  if ( v3cUnitType == V3C_AVD || v3cUnitType == V3C_GVD || v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD ||
       v3cUnitType == V3C_PVD || v3cUnitType == V3C_BMD || v3cUnitType == V3C_ADD ) {
    auto &v3cUnitHeader = syntax.getV3CUnitHeader( static_cast<int>( v3cUnitType ) - 1 );
    READ_CODE_DESC( value, 6, "atlasID" )
    v3cUnitHeader.setAtlasId( value );  // u(6)
    syntax.setAtlasIndex( v3cUnitHeader.getAtlasId() );
  }
  if ( v3cUnitType == V3C_AVD ) {
    auto &v3cUnitHeader = syntax.getV3CUnitHeader( static_cast<int>( v3cUnitType ) - 1 );
    READ_CODE_DESC( value, 7, "attributeIndex" )
    v3cUnitHeader.setAttributeIndex( value );  // u(7)
    READ_CODE_DESC( value, 5, "attributeDimensionIndex" )
    v3cUnitHeader.setAttributeDimensionIndex( value );  // u(5)
    READ_CODE_DESC( value, 4, "mapIndex" )
    v3cUnitHeader.setMapIndex( value );  // u(4)
    READ_CODE_DESC( value, 1, "auxiliaryVideoFlag" )
    v3cUnitHeader.setAuxiliaryVideoFlag( value != 0U );  // u(1)
  } else if ( v3cUnitType == V3C_GVD ) {
    auto &vpcc = syntax.getV3CUnitHeader( static_cast<int>( v3cUnitType ) - 1 );
    READ_CODE_DESC( value, 4, "mapIndex" )
    vpcc.setMapIndex( value );  // u(4)
    READ_CODE_DESC( value, 1, "auxiliaryVideoFlag" )
    vpcc.setAuxiliaryVideoFlag( value != 0U );     // u(1)
    READ_CODE_DESC( value, 12, "reserved12bits" )  // u(12)
  } else if ( v3cUnitType == V3C_PVD ) {
    auto &vpcc = syntax.getV3CUnitHeader( static_cast<int>( v3cUnitType ) - 1 );
    READ_CODE_DESC( value, 8, "packedMapIndex" )
    vpcc.setPackedMapIndex( value );             // u(8)
    READ_CODE_DESC( value, 9, "reserved9bits" )  // u(9)
  } else if ( v3cUnitType == V3C_OVD || v3cUnitType == V3C_AD || v3cUnitType == V3C_BMD || v3cUnitType == V3C_ADD ) {
    READ_CODE_DESC( value, 17, "reserved17bits" )  // u(17)
  } else if ( v3cUnitType == V3C_CAD ) {
    READ_CODE_DESC( value, 23, "reserved23bits" )  // u(23)
  } else {
    READ_CODE_DESC( value, 27, "reserved27bits" )  // u(27)
  }
}

// 8.3.2.3 V3C unit payload syntax
void PCCBitstreamReader::v3cUnitPayload( PCCHighLevelSyntax &syntax,
                                         PCCBitstream       &bitstream,
                                         V3CUnitType        &v3cUnitType,
                                         size_t              v3cUnitSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "v3cUnitType = %d \n", (int32_t)v3cUnitType );
  if ( v3cUnitType == V3C_VPS ) {
    auto &vps = syntax.addV3CParameterSet();
    v3cParameterSet( vps, syntax, bitstream );
  } else if ( v3cUnitType == V3C_AD ) {
    atlasSubStream( syntax, bitstream );
    auto vps = syntax.getVps();
  } else if ( v3cUnitType == V3C_OVD || v3cUnitType == V3C_GVD || v3cUnitType == V3C_AVD || v3cUnitType == V3C_PVD ) {
    videoSubStream( syntax, bitstream, v3cUnitType, v3cUnitSize - 4 );
  }
}

// 8.3.2.4 Atlas sub-bitstream syntax
void PCCBitstreamReader::atlasSubStream( PCCHighLevelSyntax &syntax, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  printf( "    atlasSubStream \n" );
  size_t              sizeBitstream = bitstream.capacity();
  SampleStreamNalUnit ssnu;
  sampleStreamNalHeader( bitstream, ssnu );
  PCCSEI prefixSEI;  // list to store the prefix sei message, this list will be copy in the next ATGL.
  while ( bitstream.size() < sizeBitstream ) {
    ssnu.addNalUnit();
    sampleStreamNalUnit( syntax, bitstream, ssnu, ssnu.getNalUnit().size() - 1, prefixSEI );
#ifdef BITSTREAM_TRACE
    auto &nu = ssnu.getNalUnit( ssnu.getNalUnit().size() - 1 );
    TRACE_BITSTREAM( "nalu[%d]:%s, nalSizePrecision:%d, naluSize:%zu, sizeBitstream written: %llu\n", (int)nu.getType(),
                     toString( nu.getType() ).c_str(), ( ssnu.getSizePrecisionBytesMinus1() + 1 ), nu.getSize(),
                     bitstream.size() );
#endif
  }
}

void PCCBitstreamReader::videoSubStream( PCCHighLevelSyntax &syntax,
                                         PCCBitstream       &bitstream,
                                         V3CUnitType        &V3CUnitType,
                                         size_t              V3CPayloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  printf( "    videoSubStream \n" );
  size_t atlasIndex   = 0;
  auto  &bistreamStat = syntax.getBitstreamStat();
  if ( V3CUnitType == V3C_OVD ) {
    TRACE_BITSTREAM( "%s \n", "OccupancyMap" );
    bitstream.readVideoStream( syntax.createVideoBitstream( VIDEO_OCCUPANCY ), V3CPayloadSize );
    bistreamStat.setVideoBinSize( VIDEO_OCCUPANCY, syntax.getVideoBitstream( VIDEO_OCCUPANCY ).size() );
  } else if ( V3CUnitType == V3C_GVD ) {
    auto &vuh = syntax.getV3CUnitHeader( static_cast<size_t>( V3C_GVD ) - 1 );
    if ( vuh.getAuxiliaryVideoFlag() ) {
      TRACE_BITSTREAM( "%s \n", "Geometry RAW" );
      bitstream.readVideoStream( syntax.createVideoBitstream( VIDEO_GEOMETRY_RAW ), V3CPayloadSize );
      bistreamStat.setVideoBinSize( VIDEO_GEOMETRY_RAW, syntax.getVideoBitstream( VIDEO_GEOMETRY_RAW ).size() );
    } else {
      auto &vps = syntax.getVps();
      if ( vps.getMapCountMinus1( atlasIndex ) > 0 && vps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
        auto geometryIndex = static_cast<PCCVideoType>( VIDEO_GEOMETRY_D0 + vuh.getMapIndex() );
        TRACE_BITSTREAM( "Geometry MAP: %d\n", vuh.getMapIndex() );
        bitstream.readVideoStream( syntax.createVideoBitstream( geometryIndex ), V3CPayloadSize );
        bistreamStat.setVideoBinSize( geometryIndex, syntax.getVideoBitstream( geometryIndex ).size() );
      } else {
        TRACE_BITSTREAM( "%s \n", "Geometry" );
        bitstream.readVideoStream( syntax.createVideoBitstream( VIDEO_GEOMETRY ), V3CPayloadSize );
        bistreamStat.setVideoBinSize( VIDEO_GEOMETRY, syntax.getVideoBitstream( VIDEO_GEOMETRY ).size() );
      }
    }
  } else if ( V3CUnitType == V3C_AVD ) {
    auto &vuh = syntax.getV3CUnitHeader( static_cast<size_t>( V3C_AVD ) - 1 );
    auto &vps = syntax.getVps();
    if ( vps.getAttributeInformation( atlasIndex ).getAttributeCount() > 0 ) {
      int  attributeIndex     = vuh.getAttributeIndex();
      int  attrPartitionIndex = vuh.getAttributeDimensionIndex();
      auto attrTypeId         = vps.getAttributeInformation( atlasIndex ).getAttributeTypeId( attributeIndex );
      if ( vuh.getAuxiliaryVideoFlag() ) {
        auto        videoType     = getAttributeTypeRaw( attrTypeId, attrPartitionIndex );
        std::string attributeName = getAttributeName( attrTypeId );
        TRACE_BITSTREAM( "Attribute %s RAW, PARTITION: %d\n", attributeName.c_str(), vuh.getAttributeDimensionIndex() );
        bitstream.readVideoStream( syntax.createVideoBitstream( videoType ), V3CPayloadSize );
        bistreamStat.setVideoBinSize( videoType, syntax.getVideoBitstream( attributeIndex ).size() );
      } else {
        if ( vps.getMapCountMinus1( atlasIndex ) > 0 && vps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
          auto attributeIndex = static_cast<PCCVideoType>(
              VIDEO_ATTRIBUTE_T0 + vuh.getMapIndex() * MAX_NUM_ATTR_PARTITIONS + vuh.getAttributeDimensionIndex() );
          TRACE_BITSTREAM( "Attribute MAP: %d, PARTITION: %d\n", vuh.getMapIndex(), vuh.getAttributeDimensionIndex() );
          bitstream.readVideoStream( syntax.createVideoBitstream( attributeIndex ), V3CPayloadSize );
          bistreamStat.setVideoBinSize( attributeIndex, syntax.getVideoBitstream( attributeIndex ).size() );
        } else {
          PCCVideoType videoType     = getAttributeType( attrTypeId, attrPartitionIndex );
          std::string  attributeName = getAttributeName( attrTypeId );
          TRACE_BITSTREAM( "Attribute %s PARTITION: %d\n", attributeName.c_str(), vuh.getAttributeDimensionIndex() );
          bitstream.readVideoStream( syntax.createVideoBitstream( videoType ), V3CPayloadSize );
          bistreamStat.setVideoBinSize( videoType, syntax.getVideoBitstream( videoType ).size() );
        }
      }
    }
  } else if ( V3CUnitType == V3C_PVD ) {
    // Packed video data
    auto &vuh            = syntax.getV3CUnitHeader( static_cast<size_t>( V3C_PVD ) - 1 );
    auto  packedMapIndex = vuh.getPackedMapIndex();  // For V3C_PVD, use packedMapIndex
    auto  videoType      = static_cast<PCCVideoType>( VIDEO_PACKED + packedMapIndex );
    TRACE_BITSTREAM( "Packed Video MAP: %d\n", packedMapIndex );
    bitstream.readVideoStream( syntax.createVideoBitstream( videoType ), V3CPayloadSize );
    bistreamStat.setVideoBinSize( videoType, syntax.getVideoBitstream( videoType ).size() );
  }
}

// 8.3.3 Byte alignment syntax
void PCCBitstreamReader::byteAlignment( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.read( 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.read( 1 );  // f(1): equal to 0
  }
}

// 8.3.4 V3C parameter set syntax
// 8.3.4.1 General V3C parameter set syntax
void PCCBitstreamReader::v3cParameterSet( V3CParameterSet &sps, PCCHighLevelSyntax &syntax, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  printf( "    v3cParameterSet \n" );
  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  sps.setV3CParameterSetId( bitstream.read( 4 ) );  // u(4)
  bitstream.read( 8 );                              // u(8)
  sps.setAtlasCountMinus1( bitstream.read( 6 ) );   // u(6)
  sps.allocateAtlas();
  syntax.allocateAtlasHLS( sps.getAtlasCountMinus1() + 1 );
  for ( uint32_t j = 0; j < sps.getAtlasCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "Atlas = %zu \n", j );
    sps.setAtlasId( j, bitstream.read( 6 ) );         // u(6)
    sps.setFrameWidth( j, bitstream.readUvlc() );     // ue(v)
    sps.setFrameHeight( j, bitstream.readUvlc() );    // ue(v)
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
    sps.setExtensionCount( bitstream.read( 8 ) );  // u(8)
  }
  if ( sps.getExtensionCount() ) {
    auto extensionsLengthMinus1 = bitstream.readUvlc();  // ue(v)

    // Read the VPS extensions
    for ( uint8_t i = 0; i < sps.getExtensionCount(); ++i ) {
      auto extensionType   = bitstream.read( 8 );   // u(8)
      auto extensionLength = bitstream.read( 16 );  // u(16)
      sps.setExtensionType( i, extensionType );

      vpsExtension( extensionType, extensionLength, sps, bitstream );
    }
  }
  byteAlignment( bitstream );
}

// 8.3.4.2 Profile, tier, and level syntax
void PCCBitstreamReader::profileTierLevel( ProfileTierLevel &ptl, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  uint32_t value;
  READ_CODE_DESC( value, 1, "TierFlag" )
  ptl.setTierFlag( value != 0U );  // u(1)
  READ_CODE_DESC( value, 7, "ProfileCodecGroupIdc" )
  ptl.setProfileCodecGroupIdc( value );  // u(7)
  READ_CODE_DESC( value, 8, "ProfileToolsetIdc" )
  ptl.setProfileToolsetIdc( value );  // u(8)
  READ_CODE_DESC( value, 8, "ProfileReconstructionIdc" )
  ptl.setProfileReconstructionIdc( value );  // u(8)
  READ_CODE_DESC( value, 16, "ptl_reserved_zero_16bits " )
  READ_CODE_DESC( value, 4, "ptl_max_decodes_idc " )
  READ_CODE_DESC( value, 12, "ptl_reserved_0xfff_12bits " )  // u(16)
  READ_CODE_DESC( value, 8, "LevelIdc" )
  ptl.setLevelIdc( value );  // u(8)
  READ_CODE_DESC( value, 6, "NumSubProfiles" )
  ptl.setNumSubProfiles( value );  // u(6)
  READ_CODE_DESC( value, 1, "ExtendedSubProfileFlag" )
  ptl.setExtendedSubProfileFlag( value );  // u(1)
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

// 8.3.4.3 Occupancy parameter set syntax
void PCCBitstreamReader::occupancyInformation( OccupancyInformation &oi, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  oi.setOccupancyCodecId( bitstream.read( 8 ) );                    // u(8)
  oi.setLossyOccupancyCompressionThreshold( bitstream.read( 8 ) );  // u(8)
  oi.setOccupancy2DBitdepthMinus1( bitstream.read( 5 ) );           // u(5)
  oi.setOccupancyMSBAlignFlag( bitstream.read( 1 ) != 0U );         // u(1)
  TRACE_BITSTREAM( "  OccupancyLossyThreshold = %d  \n", oi.getLossyOccupancyCompressionThreshold() );
}

// 8.3.4.4 Geometry parameter set syntax
void PCCBitstreamReader::geometryInformation( GeometryInformation &gi, V3CParameterSet &sps, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  gi.setGeometryCodecId( bitstream.read( 8 ) );                      // u(8)
  gi.setGeometry2dBitdepthMinus1( bitstream.read( 5 ) );             // u(5)
  gi.setGeometryMSBAlignFlag( bitstream.read( 1 ) != 0U );           // u(1)
  gi.setGeometry3dCoordinatesBitdepthMinus1( bitstream.read( 5 ) );  // u(5)
  if ( sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
    gi.setAuxiliaryGeometryCodecId( bitstream.read( 8 ) );  // u(8)
  }
}

// 8.3.4.5 Attribute information
void PCCBitstreamReader::attributeInformation( AttributeInformation &ai,
                                               V3CParameterSet      &sps,
                                               PCCBitstream         &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  ai.setAttributeCount( bitstream.read( 7 ) );  // u(7)
  TRACE_BITSTREAM( "AttributeCount = %u  \n", ai.getAttributeCount() );
  ai.allocate();
  for ( uint32_t i = 0; i < ai.getAttributeCount(); i++ ) {
    ai.setAttributeTypeId( i, static_cast<AttributeTypeId>( bitstream.read( 4 ) ) );  // u(4)
    ai.setAttributeCodecId( i, bitstream.read( 8 ) );                                 // u(8)
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
    ai.setAttribute2dBitdepthMinus1( i, bitstream.read( 5 ) );    // u(5)
    ai.setAttributeMSBAlignFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
  }
}

// 8.3.4.6 Profile toolset constraints information syntax
void PCCBitstreamReader::profileToolsetConstraintsInformation( ProfileToolsetConstraintsInformation &ptci,
                                                               PCCBitstream                         &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ptci.setOneFrameOnlyFlag( bitstream.read( 1 ) );                         // u(1)
  ptci.setEOMContraintFlag( bitstream.read( 1 ) );                         // u(1)
  ptci.setMaxMapCountMinus1( bitstream.read( 4 ) );                        // u(4)
  ptci.setMaxAtlasCountMinus1( bitstream.read( 4 ) );                      // u(4)
  ptci.setMultipleMapStreamsConstraintFlag( bitstream.read( 1 ) );         // u(1)
  ptci.setPLRConstraintFlag( bitstream.read( 1 ) );                        // u(1)
  ptci.setAttributeMaxDimensionMinus1( bitstream.read( 6 ) );              // u(6)
  ptci.setAttributeMaxDimensionPartitionsMinus1( bitstream.read( 6 ) );    // u(6)
  ptci.setNoEightOrientationsConstraintFlag( bitstream.read( 1 ) );        // u(1)
  ptci.setNo45DegreeProjectionPatchConstraintFlag( bitstream.read( 1 ) );  // u(1)
  bitstream.read( 6 );                                                     // u(6)
  ptci.setNumReservedConstraintBytes( bitstream.read( 8 ) );               // u(8)
  ptci.allocate();
  for ( size_t i = 0; i < ptci.getNumReservedConstraintBytes(); i++ ) {
    ptci.setReservedConstraintByte( i, bitstream.read( 8 ) );  // u(8)
  }
}

// 8.3.4.7 Packing information syntax
void PCCBitstreamReader::packingInformation( PackingInformation &pin,
                                             uint8_t             j,
                                             uint8_t             version,
                                             V3CParameterSet    &vps,
                                             PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  const auto &pve = vps.getVpsPackedVideoExtension();

  uint32_t value;

  READ_CODE_DESC( value, 8, "pin_codec_id" )  // u(8)
  pin.setCodecId( value );
  READ_CODE_DESC( value, 1, "pin_occupancy_present_flag" )  // u(1)
  pin.setOccupancyPresentFlag( value );
  READ_CODE_DESC( value, 1, "pin_geometry_present_flag" )  // u(1)
  pin.setGeometryPresentFlag( value );
  READ_CODE_DESC( value, 1, "pin_attribute_present_flag" )  // u(1)
  pin.setAttributePresentFlag( value );

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
    READ_CODE_DESC( value, 5, "pin_occupancy_2d_bit_depth_minus1" )  // u(5)
    oi.setOccupancy2DBitdepthMinus1( value );
    READ_CODE_DESC( value, 1, "pin_occupancy_msb_align_flag" )  // u(1)
    oi.setOccupancyMSBAlignFlag( value );
    READ_CODE_DESC( value, 8, "pin_lossy_occupancy_compression_threshold" )  // u(8)
    oi.setLossyOccupancyCompressionThreshold( value );
  }
  if ( pin.getGeometryPresentFlag() ) {
    auto &gi = vps.getGeometryInformation( j );
    READ_CODE_DESC( value, 5, "pin_geometry_2d_bit_depth_minus1" )  // u(5)
    gi.setGeometry2dBitdepthMinus1( value );
    READ_CODE_DESC( value, 1, "pin_geometry_msb_align_flag" )  // u(1)
    gi.setGeometryMSBAlignFlag( value );
    READ_CODE_DESC( value, 5, "pin_geometry_3d_coordinates_bit_depth_minus1" )  // u(5)
    gi.setGeometry3dCoordinatesBitdepthMinus1( value );
  }

  auto nextAttributePartitionIndex = std::vector<uint8_t>{};

  if ( pin.getAttributePresentFlag() ) {
    auto &ai = vps.getAttributeInformation( j );
    READ_CODE_DESC( value, 7, "pin_attribute_count" )  // u(7)
    ai.setAttributeCount( value );
    nextAttributePartitionIndex.assign( ai.getAttributeCount(), 0 );
    ai.allocate();
    for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
      READ_CODE_DESC( value, 4, "pin_attribute_type_id" )  // u(4)
      ai.setAttributeTypeId( i, static_cast<AttributeTypeId>( value ) );
      READ_CODE_DESC( value, 5, "pin_attribute_2d_bit_depth_minus1" )  // u(5)
      ai.setAttribute2dBitdepthMinus1( i, value );
      READ_CODE_DESC( value, 1, "pin_attribute_msb_align_flag" )  // u(1)
      ai.setAttributeMSBAlignFlag( i, value );
      READ_CODE_DESC( value, 1, "pin_attribute_map_absolute_coding_persistence_flag" )  // u(1)
      ai.setAttributeMapAbsoluteCodingPersistenceFlag( i, value );
      uint8_t d;
      READ_CODE_DESC( d, 6, "pin_attribute_dimension_minus1" )  // u(6)
      ai.setAttributeDimensionMinus1( i, d );
      uint8_t m;
      if ( d == 0 ) {
        // pin_attribute_dimension_partitions_minus1[ j ][ i ] = 0
        m = 0;
      } else {
        READ_CODE_DESC( m, 6, "pin_attribute_dimension_partitions_minus1" )  // u(6)
        ai.setAttributeDimensionPartitionsMinus1( i, m );
      }
      for ( uint8_t k = 0; k < m; k++ ) {
        uint8_t n;
        if ( k + d == m ) {
          // pin_attribute_partition_channels_minus1[ j ][ i ][ k ] = 0
          n = 0;
        } else {
          READ_UVLC_DESC( n, "pin_attribute_partition_channels_minus1" )  // ue(v)
          ai.setAttributePartitionChannelsMinus1( i, k, n );
        }
        d -= n + 1;
      }
      // pin_attribute_partition_channels_minus1[ j ][ i ][ m ] = d
    }
  }
  uint32_t regionsCountMinus1;
  READ_UVLC_DESC( regionsCountMinus1, "pin_regions_count_minus1" )  // ue(v)

  if ( pve.getMultiplePackedMapsEnabledFlag() ) {
    READ_CODE_DESC( value, ceilLog2( regionsCountMinus1 + 1 ), "pin_packed_maps_count_minus1" )  // u(v)
    pin.setPackedMapCountMinus1( value );
  }
  if ( pve.getTemporalPackingEnabledFlag() ) {
    for ( uint8_t k = 0; k <= pin.getPackedMapCountMinus1(); ++k ) {
      READ_CODE_DESC( value, ceilLog2( regionsCountMinus1 + 1 ), "pin_constituent_frame_count_minus1" );  // u(v)
      pin.setConstituentFrameCountMinus1( k, value );
    }
  }
  if ( pve.getImplicitPackingMode() == ImplicitPackingMode::implicit_mode ) {
    for ( uint8_t k = 0; k <= pin.getPackedMapCountMinus1(); ++k ) {
      READ_UVLC_DESC( value, "pin_packed_map_grid_columns_minus1" );  // ue(v)
      pin.setPackedMapGridColumnsMinus1( k, value );
      READ_UVLC_DESC( value, "pin_packed_map_grid_rows_minus1" );  // ue(v)
      pin.setPackedMapGridRowsMinus1( k, value );
    }
  }
  if ( version >= 2 ) {
    READ_CODE_DESC( value, 1, "pin_lod_enabled_flag" )  // u(1)
    pin.setLodEnabledFlag( value );
    if ( pin.getLodEnabledFlag() ) {
      READ_CODE_DESC( value, 2, "pin_lod_level_count_minus2" )  // u(2)
      pin.setLodLevelCountMinus2( value );
    }
  }

  for ( size_t i = 0; i <= regionsCountMinus1; i++ ) {
    packedRegion( pin.addRegion(), pin, j, vps, nextAttributePartitionIndex, bitstream );
  }
}

void PCCBitstreamReader::packedRegion( PackedRegion             &region,
                                       const PackingInformation &pin,
                                       uint8_t                   j,
                                       V3CParameterSet          &vps,
                                       std::vector<uint8_t>     &nextAttributePartitionIndex,
                                       PCCBitstream             &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  const auto &pve = vps.getVpsPackedVideoExtension();
  uint32_t    value;

  if ( pin.getLodEnabledFlag() ) {
    uint8_t maskBits = pin.getLodLevelCountMinus2() + 2;
    READ_CODE_DESC( value, maskBits, "pin_region_lod_mask" )  // u(v)
    region.setRegionLodMask( value );
  }

  if ( pin.getPackedMapCountMinus1() > 0 ) {
    READ_CODE_DESC( value, ceilLog2( pin.getPackedMapCountMinus1() + 1 ), "pin_region_packed_map_index" )  // u(v)
    if ( value > pin.getPackedMapCountMinus1() ) {
      std::cerr << "pin_region_packed_map_index value " << value << " exceeds pin_packed_map_count_minus1 "
                << static_cast<uint32_t>( pin.getPackedMapCountMinus1() ) << ".\n";
      exit( -1 );
    }
    region.setPackedMapIndex( value );
  }
  const auto k = region.getPackedMapIndex();
  if ( pin.getConstituentFrameCountMinus1( k ) > 0 ) {
    READ_CODE_DESC( value, ceilLog2( pin.getConstituentFrameCountMinus1( k ) + 1 ),
                    "pin_region_constituent_frame_index" )  // u(v)
    region.setConstituentFrameIndex( value );
  }
  READ_CODE_DESC( value, 8, "pin_region_tile_id" )  // u(8)
  region.setTileId( value );
  READ_CODE_DESC( value, 2, "pin_region_type_id_minus2" )  // u(2)
  region.setTypeIdMinus2( value );

  if ( pve.getImplicitPackingMode() == ImplicitPackingMode::explicit_mode ) {
    READ_CODE_DESC( value, 16, "pin_region_top_left_x" )  // u(16)
    region.setTopLeftX( value );
    READ_CODE_DESC( value, 16, "pin_region_top_left_y" )  // u(16)
    region.setTopLeftY( value );
    READ_CODE_DESC( value, 16, "pin_region_width_minus1" )  // u(16)
    region.setWidthMinus1( value );
    READ_CODE_DESC( value, 16, "pin_region_height_minus1" )  // u(16)
    region.setHeightMinus1( value );
    READ_CODE_DESC( value, 16, "pin_region_unpack_top_left_x" )  // u(16)
    region.setUnpackTopLeftX( value );
    READ_CODE_DESC( value, 16, "pin_region_unpack_top_left_y" )  // u(16)
    region.setUnpackTopLeftY( value );
    READ_CODE_DESC( value, 1, "pin_region_rotation_flag" )  // u(1)
    region.setRotationFlag( value );
  }

  const auto typeId = region.getTypeIdMinus2() + 2;

  if ( typeId == V3C_AVD || typeId == V3C_GVD ) {
    READ_CODE_DESC( value, 4, "pin_region_map_index" )  // u(4)
    region.setMapIndex( value );
    READ_CODE_DESC( value, 1, "pin_region_auxiliary_data_flag" )  // u(1)
    region.setAuxiliaryDataFlag( value );
  }
  if ( typeId == V3C_AVD ) {
    READ_CODE_DESC( value, 7, "pin_region_attr_index" )  // u(7)
    region.setAttrIndex( value );
    size_t k = region.getAttrIndex();

    auto &ai = vps.getAttributeInformation( j );
    if ( ai.getAttributeDimensionMinus1( k ) > 0 ) {
      if ( pve.getImplicitPackingMode() == ImplicitPackingMode::explicit_mode )
        READ_CODE_DESC( value, 5, "pin_region_attr_partition_index" )  // u(5)
      else { value = nextAttributePartitionIndex[k]++; }
      region.setAttrPartitionIndex( value );
    }
  }
}

// 8.3.4.8 VPS extension syntax
void PCCBitstreamReader::vpsExtension( uint8_t          extensionType,
                                       uint16_t         extensionLength,
                                       V3CParameterSet &vps,
                                       PCCBitstream    &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  const auto bitOffset = bitstream.getPosition().bits_;

  if ( extensionType == VPS_EXT_PACKED ) {
    vpsPackedVideoExtension( vps.getVpsPackedVideoExtension(), 1, vps, bitstream );
  } else if ( extensionType == VPS_EXT_PACKED_2 ) {
    vpsPackedVideoExtension( vps.getVpsPackedVideoExtension(), 2, vps, bitstream );
  } else {
    std::clog << "WARNING: Skipping unsupported VPS extension type " << static_cast<int>( extensionType ) << std::endl;
    for ( uint16_t j = 0; j < extensionLength; j++ ) const auto vps_extension_data_byte = bitstream.read( 8 );  // u(8)
  }

  lengthAlignment( bitOffset, bitstream );
}

// 8.3.4.9 VPS packed video extension syntax
void PCCBitstreamReader::vpsPackedVideoExtension( VpsPackedVideoExtension &pve,
                                                  uint8_t                  version,
                                                  V3CParameterSet         &vps,
                                                  PCCBitstream            &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  uint32_t value;

  if ( version >= 2 ) {
    READ_CODE_DESC( value, 1, "vps_temporal_packing_enabled_flag" )  // u(1)
    pve.setTemporalPackingEnabledFlag( value );

    if ( pve.getTemporalPackingEnabledFlag() ) {
      READ_UVLC_DESC( value, "vps_temporal_packing_mode" );  // ue(v)
      pve.setTemporalPackingMode( static_cast<TemporalPackingMode>( value ) );
    }
    READ_CODE_DESC( value, 1, "vps_multiple_packed_maps_enabled_flag" );  // u(1)
    pve.setMultiplePackedMapsEnabledFlag( value );
    READ_UVLC_DESC( value, "vps_implicit_packing_mode" );  // ue(v)
    pve.setImplicitPackingMode( static_cast<ImplicitPackingMode>( value ) );
  }
  for ( uint8_t k = 0; k <= vps.getAtlasCountMinus1(); ++k ) {
    const auto j = vps.getAtlasId( k );
    READ_CODE_DESC( value, 1, "vps_packed_video_present_flag" )  // u(1)
    pve.setPackedVideoPresentFlag( j, value );

    if ( pve.getPackedVideoPresentFlag( j ) ) {
      packingInformation( pve.getPackingInformation( k ), j, version, vps, bitstream );
    }
  }
}

// 8.3.4.10 Length alignment syntax
void PCCBitstreamReader::lengthAlignment( uint8_t bitOffset, PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  uint32_t value;

  while ( bitstream.getPosition().bits_ != bitOffset ) {
    READ_CODE_DESC( value, 1, "length_alignment_bit_equal_to_zero" )  // f(1)
  }
}

// 8.3.5 NAL unit syntax
// 8.3.5.1 General NAL unit syntax
void PCCBitstreamReader::nalUnit( PCCBitstream &bitstream, NalUnit &nalUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  nalUnitHeader( bitstream, nalUnit );
  for ( size_t i = 2; i < nalUnit.getSize(); i++ ) {
    nalUnit.setData( i, bitstream.read( 8 ) );  // b(8)
  }
}

// 8.3.5.2 NAL unit header syntax
void PCCBitstreamReader::nalUnitHeader( PCCBitstream &bitstream, NalUnit &nalUnit ) {
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

// 8.3.6	Raw byte sequence payloads, trailing bits, and byte alignment syntax
// 8.3.6.1 Atlas sequence parameter set Rbsp
// 8.3.6.1.1 General Atlas sequence parameter set Rbsp
void PCCBitstreamReader::atlasSequenceParameterSetRbsp( AtlasSequenceParameterSetRbsp &asps,
                                                        PCCHighLevelSyntax            &syntax,
                                                        PCCBitstream                  &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  asps.setAtlasSequenceParameterSetId( bitstream.readUvlc() );         // ue(v)
  asps.setFrameWidth( bitstream.readUvlc() );                          // ue(v)
  asps.setFrameHeight( bitstream.readUvlc() );                         // ue(v)
  asps.setGeometry3dBitdepthMinus1( bitstream.read( 5 ) );             // u(5)
  asps.setGeometry2dBitdepthMinus1( bitstream.read( 5 ) );             // u(5)
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
  asps.setPixelDeinterleavingFlag( bitstream.read( 1 ) );        // u(1)
  if ( asps.getPixelDeinterleavingFlag() ) {
    asps.allocatePixelDeinterleavingMapFlag();
    for ( size_t i = 0; i < asps.getMapCountMinus1() + 1; i++ ) {
      asps.setPixelDeinterleavingMapFlag( i, bitstream.read( 1 ) );  // u(1)
    }
  }
  asps.setRawPatchEnabledFlag( bitstream.read( 1 ) );  // u(1)
  asps.setEomPatchEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getEomPatchEnabledFlag() && asps.getMapCountMinus1() == 0 ) {
    asps.setEomFixBitCountMinus1( bitstream.read( 4 ) );  // u(4)
  }
  if ( asps.getRawPatchEnabledFlag() || asps.getEomPatchEnabledFlag() ) {
    asps.setAuxiliaryVideoEnabledFlag( bitstream.read( 1 ) );  // u(1)
    TRACE_BITSTREAM( "getAuxiliaryVideoEnabledFlag = %u \n", asps.getAuxiliaryVideoEnabledFlag() );
  }
  asps.setPLREnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getPLREnabledFlag() ) { plrInformation( asps, syntax, bitstream ); }
  asps.setVuiParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getVuiParametersPresentFlag() ) { vuiParameters( bitstream, asps.getVuiParameters() ); }

  asps.setExtensionFlag( bitstream.read( 1 ) );  // u(1)
  if ( asps.getExtensionFlag() ) {
    asps.setVpccExtensionFlag( bitstream.read( 1 ) );  // u(1)
    asps.setExtension7Bits( bitstream.read( 7 ) );     // u(7)
  }
  if ( asps.getVpccExtensionFlag() ) { aspsVpccExtension( bitstream, asps, asps.getAspsVpccExtension() ); }
  if ( asps.getExtension7Bits() ) {
    while ( moreRbspData( bitstream ) ) {
      bitstream.read( 1 );  // u(1)
    }
  }
  rbspTrailingBits( bitstream );
}

// 8.3.6.1.2 Point local reconstruction information syntax
void PCCBitstreamReader::plrInformation( AtlasSequenceParameterSetRbsp &asps,
                                         PCCHighLevelSyntax            &syntax,
                                         PCCBitstream                  &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "  MapCountMinus1() = %u \n", asps.getMapCountMinus1() );
  asps.allocatePLRInformation();
  for ( size_t j = 0; j < asps.getMapCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "  PLR info map %zu \n", j );
    auto &plri = asps.getPLRInformation( j );
    plri.setMapEnabledFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( plri.getMapEnabledFlag() ) {
      plri.setNumberOfModesMinus1( bitstream.read( 4 ) );  // u(4)
      TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plri.getNumberOfModesMinus1() );
      plri.allocate();
      for ( size_t i = 0; i < plri.getNumberOfModesMinus1() + 1; i++ ) {
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

// 8.3.6.2 Atlas frame parameter set Rbsp syntax
// 8.3.6.2.1 General atlas frame parameter set Rbsp syntax
void PCCBitstreamReader::atlasFrameParameterSetRbsp( AtlasFrameParameterSetRbsp &afps,
                                                     PCCHighLevelSyntax         &syntax,
                                                     PCCBitstream               &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  afps.setAtlasFrameParameterSetId( bitstream.readUvlc() );     // ue(v)
  afps.setAtlasSequenceParameterSetId( bitstream.readUvlc() );  // ue(v)
  auto &asps = syntax.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  atlasFrameTileInformationRbsp( afps.getAtlasFrameTileInformationRbsp(), asps, bitstream );
  afps.setOutputFlagPresentFlag( bitstream.read( 1 ) );                // u(1)
  afps.setNumRefIdxDefaultActiveMinus1( bitstream.readUvlc() );        // ue(v)
  afps.setAdditionalLtAfocLsbLen( bitstream.readUvlc() );              // ue(v)
  afps.setLodModeEnableFlag( bitstream.read( 1 ) );                    // u(1)
  afps.setRaw3dOffsetBitCountExplicitModeFlag( bitstream.read( 1 ) );  // u(1)
  afps.setExtensionFlag( bitstream.read( 1 ) );                        // u(1)
  if ( afps.getExtensionFlag() ) {
    afps.setExtension8Bits( bitstream.read( 8 ) );  // u(8)
  }
  if ( afps.getExtension8Bits() ) {
    while ( moreRbspData( bitstream ) ) { bitstream.read( 0, 1 ); }  // u(1)
  }
  rbspTrailingBits( bitstream );
}

// 8.3.6.2.2 Atlas frame tile information syntax
void PCCBitstreamReader::atlasFrameTileInformationRbsp( AtlasFrameTileInformationRbsp &afti,
                                                        AtlasSequenceParameterSetRbsp &asps,
                                                        PCCBitstream                  &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  afti.setSingleTileInAtlasFrameFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !afti.getSingleTileInAtlasFrameFlag() ) {
    afti.setUniformPartitionSpacingFlag( bitstream.read( 1 ) != 0U );  // u(1)
    TRACE_BITSTREAM( "afti: uniformPartition :%zu !singleTile\n", afti.getUniformPartitionSpacingFlag() );
    if ( afti.getUniformPartitionSpacingFlag() ) {
      afti.setPartitionColumnWidthMinus1( 0, bitstream.readUvlc() );  //  ue(v)
      afti.setPartitionRowHeightMinus1( 0, bitstream.readUvlc() );    //  ue(v)
      afti.setNumPartitionColumnsMinus1(
          ceil( asps.getFrameWidth() / ( ( afti.getPartitionColumnWidthMinus1( 0 ) + 1 ) * 64.0 ) ) - 1 );
      afti.setNumPartitionRowsMinus1(
          ceil( asps.getFrameHeight() / ( ( afti.getPartitionRowHeightMinus1( 0 ) + 1 ) * 64.0 ) ) - 1 );
      TRACE_BITSTREAM( "afti: aspsWidth :%zu, partitionWidth: %zu, Number of Partitions Hor: %zu\n",
                       asps.getFrameWidth(), afti.getPartitionColumnWidthMinus1( 0 ) + 1,
                       afti.getNumPartitionColumnsMinus1() + 1 );
      TRACE_BITSTREAM( "afti: aspsHeight :%zu, partitionHeight: %zu, Number of Partitions Ver: %zu\n",
                       asps.getFrameHeight(), afti.getPartitionRowHeightMinus1( 0 ) + 1,
                       afti.getNumPartitionRowsMinus1() + 1 );

    } else {
      afti.setNumPartitionColumnsMinus1( bitstream.readUvlc() );  //  ue(v)
      afti.setNumPartitionRowsMinus1( bitstream.readUvlc() );     //  ue(v)
      for ( size_t i = 0; i < afti.getNumPartitionColumnsMinus1(); i++ ) {
        afti.setPartitionColumnWidthMinus1( i, bitstream.readUvlc() );  //  ue(v)
      }
      for ( size_t i = 0; i < afti.getNumPartitionRowsMinus1(); i++ ) {
        afti.setPartitionRowHeightMinus1( i, bitstream.readUvlc() );  //  ue(v)
      }

      TRACE_BITSTREAM( "afti: aspsWidth :%zu, partitionWidth: ", asps.getFrameWidth() );
      for ( size_t i = 0; i < afti.getNumPartitionColumnsMinus1(); i++ ) {
        TRACE_BITSTREAM( "%d, ", afti.getPartitionColumnWidthMinus1( i ) );
      }
      TRACE_BITSTREAM( "Number of Partitions Hor: %zu\n", afti.getNumPartitionColumnsMinus1() + 1 );

      TRACE_BITSTREAM( "afti: aspsHeight :%zu, partitionWidth: ", asps.getFrameHeight() );
      for ( size_t i = 0; i < afti.getNumPartitionRowsMinus1(); i++ ) {
        TRACE_BITSTREAM( "%d, ", afti.getPartitionRowHeightMinus1( i ) );
      }
      TRACE_BITSTREAM( "Number of Partitions Ver: %zu\n", afti.getNumPartitionRowsMinus1() + 1 );
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
          ( afti.getNumPartitionColumnsMinus1() + 1 ) * ( afti.getNumPartitionRowsMinus1() + 1 ) - 1 );
      for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
        afti.setTopLeftPartitionIdx( i, i );
        afti.setBottomRightPartitionColumnOffset( i, 0 );
        afti.setBottomRightPartitionRowOffset( i, 0 );
      }
      TRACE_BITSTREAM( "afti: singlePartitionPerTileFlag = true : %zu\n", afti.getNumTilesInAtlasFrameMinus1() );
      for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
        TRACE_BITSTREAM( "afti: blocks:%zu,%zu,%zu\n", afti.getTopLeftPartitionIdx( i ),
                         afti.getBottomRightPartitionColumnOffset( i ), afti.getBottomRightPartitionRowOffset( i ) );
      }
    }
  } else {
    afti.setNumTilesInAtlasFrameMinus1( 0 );
  }
  TRACE_BITSTREAM( "afti: singleTile :%zu\n", afti.getSingleTileInAtlasFrameFlag() );
  TRACE_BITSTREAM( "afti: uniformPartition :%zu\n", afti.getUniformPartitionSpacingFlag() );
  TRACE_BITSTREAM( "afti: numTilesInAtlasFrameMinus1 :%zu\n", afti.getNumTilesInAtlasFrameMinus1() );
  TRACE_BITSTREAM( "asps: getAuxiliaryVideoEnabledFlag :%zu\n", asps.getAuxiliaryVideoEnabledFlag() );
  if ( asps.getAuxiliaryVideoEnabledFlag() ) {
    afti.setAuxiliaryVideoTileRowWidthMinus1( bitstream.readUvlc() );  // ue(v)
    for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
      afti.setAuxiliaryVideoTileRowHeight( i, bitstream.readUvlc() );  // ue(v)
    }
  }
  afti.setSignalledTileIdFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( afti.getSignalledTileIdFlag() ) {
    afti.setSignalledTileIdLengthMinus1( bitstream.readUvlc() );  // ue(v)
    for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
      uint8_t bitCount = afti.getSignalledTileIdLengthMinus1() + 1;
      afti.setTileId( i, bitstream.read( bitCount ) );  // u(v)
    }
  } else {
    for ( size_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) { afti.setTileId( i, i ); }
  }
}

// 8.3.6.3 Atlas adaptation parameter set RBSP syntax
void PCCBitstreamReader::atlasAdaptationParameterSetRbsp( AtlasAdaptationParameterSetRbsp &aaps,
                                                          PCCBitstream                    &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  aaps.setAtlasAdaptationParameterSetId( bitstream.readUvlc() );  // ue(v)
  aaps.setExtensionFlag( bitstream.read( 1 ) );                   // u(1)
  if ( aaps.getExtensionFlag() ) {
    aaps.setVpccExtensionFlag( bitstream.read( 1 ) );    // u(1)
    aaps.setVpccGsExtensionFlag( bitstream.read( 1 ) );  // u(1)
    aaps.setExtension6Bits( bitstream.read( 6 ) );       // u(7)
  }
  if ( aaps.getVpccExtensionFlag() ) { aapsVpccExtension( bitstream, aaps.getAapsVpccExtension() ); }
  if ( aaps.getVpccGsExtensionFlag() ) { aapsVpccGsExtension( bitstream, aaps.getAapsVpccGsExtension() ); }
  if ( aaps.getExtension6Bits() ) {
    while ( moreRbspData( bitstream ) ) { bitstream.read( 0, 1 ); }  // u(1)
  }
  rbspTrailingBits( bitstream );
}

// 8.3.6.4  Supplemental enhancement information Rbsp
void PCCBitstreamReader::seiRbsp( PCCHighLevelSyntax &syntax,
                                  PCCBitstream       &bitstream,
                                  NalUnitType         nalUnitType,
                                  PCCSEI             &sei ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  do { seiMessage( bitstream, syntax, nalUnitType, sei ); } while ( moreRbspData( bitstream ) );
  rbspTrailingBits( bitstream );
}

// 8.3.6.5 Access unit delimiter RBSP syntax
void PCCBitstreamReader::accessUnitDelimiterRbsp( AccessUnitDelimiterRbsp &aud,
                                                  PCCHighLevelSyntax      &syntax,
                                                  PCCBitstream            &bitstream ) {
  aud.setAframeType( bitstream.read( 3 ) );  //	u(3)
  rbspTrailingBits( bitstream );
}
// 8.3.6.6 End of sequence RBSP syntax
void PCCBitstreamReader::endOfSequenceRbsp( EndOfSequenceRbsp  &eosbsp,
                                            PCCHighLevelSyntax &syntax,
                                            PCCBitstream       &bitstream ) {}

// 8.3.6.7 End of bitstream RBSP syntax
void PCCBitstreamReader::endOfAtlasSubBitstreamRbsp( EndOfAtlasSubBitstreamRbsp &eoasb,
                                                     PCCHighLevelSyntax         &syntax,
                                                     PCCBitstream               &bitstream ) {}

// 8.3.6.8 Filler data RBSP syntax
void PCCBitstreamReader::fillerDataRbsp( FillerDataRbsp &fdrbsp, PCCHighLevelSyntax &syntax, PCCBitstream &bitstream ) {
  // while ( next_bits( 8 ) == 0xFF ) bitstream.read( 8 );  // f(8) // JR TODO
  rbspTrailingBits( bitstream );
}

// 8.3.6.9  Atlas tile group layer Rbsp syntax
void PCCBitstreamReader::atlasTileLayerRbsp( AtlasTileLayerRbsp &atgl,
                                             PCCHighLevelSyntax &syntax,
                                             NalUnitType         nalUnitType,
                                             PCCBitstream       &bitstream ) {
  // setFrameIndex
  TRACE_BITSTREAM( "%s \n", __func__ );
  atlasTileHeader( atgl.getHeader(), syntax, nalUnitType, bitstream );
  atlasTileDataUnit( atgl.getDataUnit(), atgl.getHeader(), syntax, bitstream );
  rbspTrailingBits( bitstream );
}

// 8.3.6.10 RBSP trailing bit syntax
void PCCBitstreamReader::rbspTrailingBits( PCCBitstream &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.read( 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.read( 1 );  // f(1): equal to 0
  }
}

// 8.3.6.11  Atlas tile group header syntax
void PCCBitstreamReader::atlasTileHeader( AtlasTileHeader    &ath,
                                          PCCHighLevelSyntax &syntax,
                                          NalUnitType         nalUnitType,
                                          PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( nalUnitType >= NAL_BLA_W_LP && nalUnitType <= NAL_RSV_IRAP_ACL_29 ) {
    ath.setNoOutputOfPriorAtlasFramesFlag( bitstream.read( 1 ) );  // u(1)
  }
  if ( nalUnitType == NAL_TRAIL_R ) { ath.setTileNaluTypeInfo( 1 ); }
  if ( nalUnitType == NAL_TRAIL_N ) { ath.setTileNaluTypeInfo( 2 ); }
  ath.setAtlasFrameParameterSetId( bitstream.readUvlc() );       // ue(v)
  ath.setAtlasAdaptationParameterSetId( bitstream.readUvlc() );  // ue(v)
  size_t                         afpsId = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp    &afps   = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp &asps   = syntax.getAtlasSequenceParameterSet( aspsId );
  AtlasFrameTileInformationRbsp &afti   = afps.getAtlasFrameTileInformationRbsp();
  if ( afti.getSignalledTileIdFlag() ) {
    ath.setId( bitstream.read( afti.getSignalledTileIdLengthMinus1() + 1 ) );  // u(v)
  } else {
    TRACE_BITSTREAM( " getNumTilesInAtlasFrameMinus1: %zu\n", afti.getNumTilesInAtlasFrameMinus1() );
    if ( afti.getNumTilesInAtlasFrameMinus1() != 0 ) {
      ath.setId( bitstream.read( ceilLog2( afti.getNumTilesInAtlasFrameMinus1() + 1 ) ) );  // u(v)
    } else {
      ath.setId( 0 );
    }
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
  auto   &refList = ath.getRefAtlasFrameListSpsFlag() ? asps.getRefListStruct( rlsIdx ) : ath.getRefListStruct();
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
      ath.setPosMinDQuantizer( bitstream.read( 5 ) );  // u(5)
      TRACE_BITSTREAM( " AtghPosMinDQuantizer = %zu \n", ath.getPosMinDQuantizer() );
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        ath.setPosDeltaMaxDQuantizer( bitstream.read( 5 ) );  // u(5)
      }
    }
    if ( asps.getPatchSizeQuantizerPresentFlag() ) {
      ath.setPatchSizeXinfoQuantizer( bitstream.read( 3 ) );  // u(3)
      ath.setPatchSizeYinfoQuantizer( bitstream.read( 3 ) );  // u(3)
    }

    TRACE_BITSTREAM( "Raw3dOffsetBitCountExplicitModeFlag    = %d \n", afps.getRaw3dOffsetBitCountExplicitModeFlag() );
    if ( afps.getRaw3dOffsetBitCountExplicitModeFlag() ) {
      size_t bitCount = floorLog2( asps.getGeometry3dBitdepthMinus1() + 1 );
      TRACE_BITSTREAM( "bitCount(floorLog2( asps.getGeometry3dBitdepthMinus1() + 1 ) = %zu \n", bitCount );
      ath.setRaw3dOffsetAxisBitCountMinus1( bitstream.read( bitCount ) );  // u(v)
    } else {
      TRACE_BITSTREAM( "Geometry3dBitdepthMinus1 = %zu \n", asps.getGeometry3dBitdepthMinus1() );
      TRACE_BITSTREAM( "Geometry2dBitdepthMinus1 = %zu \n", asps.getGeometry2dBitdepthMinus1() );
      ath.setRaw3dOffsetAxisBitCountMinus1(
          std::max( 0, asps.getGeometry3dBitdepthMinus1() - asps.getGeometry2dBitdepthMinus1() ) - 1 );
    }
    if ( ath.getType() == P_TILE && refList.getNumRefEntries() > 1 ) {
      ath.setNumRefIdxActiveOverrideFlag( bitstream.read( 1 ) != 0U );  // u(1)
      if ( ath.getNumRefIdxActiveOverrideFlag() ) {
        ath.setNumRefIdxActiveMinus1( bitstream.readUvlc() );  // ue(v)
      }
    }
    TRACE_BITSTREAM( "==> Raw3dOffsetAxisBitCountMinus1  = %zu \n", ath.getRaw3dOffsetAxisBitCountMinus1() );
    TRACE_BITSTREAM( "==> NumRefIdxActiveOverrideFlag = %zu \n", ath.getNumRefIdxActiveOverrideFlag() );
    TRACE_BITSTREAM( "==> NumRefIdxActiveMinus1       = %zu \n", ath.getNumRefIdxActiveMinus1() );
  }
  byteAlignment( bitstream );
}

// 8.3.6.12  Reference list structure syntax
void PCCBitstreamReader::refListStruct( RefListStruct                 &rls,
                                        AtlasSequenceParameterSetRbsp &asps,
                                        PCCBitstream                  &bitstream ) {
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

// 8.3.7.1  General atlas tile group data unit syntax =patchTileDataUnit
void PCCBitstreamReader::atlasTileDataUnit( AtlasTileDataUnit  &atdu,
                                            AtlasTileHeader    &ath,
                                            PCCHighLevelSyntax &syntax,
                                            PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "Type = %zu \n", ath.getType() );
  if ( ath.getType() == SKIP_TILE ) {
    // for( p = 0; p < RefAtduTotalNumberOfPatches[ tileId ]  + 1 ; p ++ )
    skipPatchDataUnit( bitstream );
  } else {
    atdu.init();
    size_t patchIndex = 0;
    TRACE_BITSTREAM( "patch %zu : \n", patchIndex );
    prevPatchSizeU_   = 0;
    prevPatchSizeV_   = 0;
    predPatchIndex_   = 0;
    uint8_t patchMode = bitstream.readUvlc();  // ue(v)
    TRACE_BITSTREAM( "patchMode = %zu \n", patchMode );
    while ( ( patchMode != I_END ) && ( patchMode != P_END ) ) {
      auto &pid = atdu.addPatchInformationData( patchMode );
      pid.setTileOrder( atdu.getTileOrder() );
      pid.setPatchIndex( patchIndex );
      patchIndex++;
      patchInformationData( pid, patchMode, ath, syntax, bitstream );
      TRACE_BITSTREAM( "patch %zu : \n", patchIndex );
      patchMode = bitstream.readUvlc();  // ue(v)
      TRACE_BITSTREAM( "patchMode = %zu \n", patchMode );
    }
    prevFrameIndex_ = atdu.getTileOrder();
#ifdef BITSTREAM_TRACE
    if ( ( patchMode == I_END ) || ( patchMode == P_END ) ) {
      TRACE_BITSTREAM( "patchInformationData: AtghType = %zu patchMode = %zu \n", ath.getType(), patchMode );
    }
#endif
    TRACE_BITSTREAM( "atdu.getPatchCount() including END = %zu \n", atdu.getPatchCount() + 1 );
  }
}

// 8.3.7.2  Patch information data syntax
void PCCBitstreamReader::patchInformationData( PatchInformationData &pid,
                                               size_t                patchMode,
                                               AtlasTileHeader      &ath,
                                               PCCHighLevelSyntax   &syntax,
                                               PCCBitstream         &bitstream ) {
  TRACE_BITSTREAM( "%s: AtghType = %zu patchMode = %zu \n", __func__, ath.getType(), patchMode );
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
}

// 8.3.7.3  Patch data unit syntax : AtlasTileHeader instead of
void PCCBitstreamReader::patchDataUnit( PatchDataUnit      &pdu,
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
  pdu.set2dPosX( bitstream.readUvlc() );             // ue(v)
  pdu.set2dPosY( bitstream.readUvlc() );             // ue(v)
  pdu.set2dSizeXMinus1( bitstream.readUvlc() );      // ue(v)
  pdu.set2dSizeYMinus1( bitstream.readUvlc() );      // ue(v)
  pdu.set3dOffsetU( bitstream.read( bitCountUV ) );  // u(v)
  pdu.set3dOffsetV( bitstream.read( bitCountUV ) );  // u(v)
  pdu.set3dOffsetD( bitstream.read( bitCountD ) );   // u(v)
  TRACE_BITSTREAM( " 2dPosXY: %zu,%zu\n", pdu.get2dPosX(), pdu.get2dPosX() );
  TRACE_BITSTREAM( " 2dSizeXY: %d,%d\n", int32_t( pdu.get2dSizeXMinus1() + 1 ), int32_t( pdu.get2dSizeYMinus1() + 1 ) );
  TRACE_BITSTREAM( " 3dPosXY: %zu,%zu\n", pdu.get3dOffsetU(), pdu.get3dOffsetV() );
  TRACE_BITSTREAM( " Pdu3dPosMinZ: %zu ( bitCountD = %u = %u - %u + %u ) \n", pdu.get3dOffsetD(), bitCountD,
                   asps.getGeometry3dBitdepthMinus1(), ath.getPosMinDQuantizer(), 2 );
  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    uint8_t bitCountForMaxDepth = std::min( asps.getGeometry2dBitdepthMinus1(), asps.getGeometry3dBitdepthMinus1() ) +
                                  1 - ath.getPosDeltaMaxDQuantizer();
    pdu.set3dRangeD( bitstream.read( bitCountForMaxDepth ) );  // u(v)
    TRACE_BITSTREAM( " Pdu3dPosDeltaMaxZ: %zu ( bitCountForMaxDepth = %u) \n", pdu.get3dRangeD(), bitCountForMaxDepth );
  } else {
    pdu.set3dRangeD( 0 );
  }
  pdu.setProjectionId( bitstream.read( ceilLog2( asps.getMaxNumberProjectionsMinus1() + 1 ) ) );  // u(5 or 3)

  TRACE_BITSTREAM( "PduProjectionId = %zu ( MaxNumberProjectionsMinus1 = %d ) \n", pdu.getProjectionId(),
                   asps.getMaxNumberProjectionsMinus1() );
  pdu.setOrientationIndex( bitstream.read( ( asps.getUseEightOrientationsFlag() ? 3 : 1 ) ) );  // u(3 or 1)
  if ( afps.getLodModeEnableFlag() ) {
    pdu.setLodEnableFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( pdu.getLodEnableFlag() ) {
      pdu.setLodScaleXMinus1( uint8_t( bitstream.readUvlc() ) );  // ue(v)
      pdu.setLodScaleYIdc( uint8_t( bitstream.readUvlc() ) );     // ue(v)
    }
  } else {
    pdu.setLodEnableFlag( false );
    pdu.setLodScaleXMinus1( 0 );
    pdu.setLodScaleYIdc( 0 );
  }
  TRACE_BITSTREAM( "PLREnabledFlag = %d \n", asps.getPLREnabledFlag() );
  if ( asps.getPLREnabledFlag() ) {
    auto &plrd = pdu.getPLRData();
    TRACE_BITSTREAM( "Size = %ld %ld\n", pdu.get2dSizeXMinus1() + 1, pdu.get2dSizeYMinus1() + 1 );
    plrd.allocate( pdu.get2dSizeXMinus1() + 1, pdu.get2dSizeYMinus1() + 1 );
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
void PCCBitstreamReader::skipPatchDataUnit( PCCBitstream &bitstream ) { TRACE_BITSTREAM( "%s \n", __func__ ); }

// 8.3.7.5  Merge patch data unit syntax
void PCCBitstreamReader::mergePatchDataUnit( MergePatchDataUnit &mpdu,
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
  if ( numRefIdxActive > 1 ) {
    mpdu.setRefIndex( bitstream.readUvlc() );  // ue(v)
  } else {
    mpdu.setRefIndex( 0 );
  }
  mpdu.setOverride2dParamsFlag( bitstream.read( 1 ) );  // u(1)
  if ( mpdu.getOverride2dParamsFlag() ) {
    mpdu.set2dPosX( bitstream.readSvlc() );        // se(v)
    mpdu.set2dPosY( bitstream.readSvlc() );        // se(v)
    mpdu.set2dDeltaSizeX( bitstream.readSvlc() );  // se(v)
    mpdu.set2dDeltaSizeY( bitstream.readSvlc() );  // se(v)
    if ( asps.getPLREnabledFlag() ) { overridePlrFlag = true; }
  } else {
    mpdu.setOverride3dParamsFlag( bitstream.read( 1 ) );  // u(1)
    if ( mpdu.getOverride3dParamsFlag() ) {
      mpdu.set3dOffsetU( bitstream.readSvlc() );  // se(v)
      mpdu.set3dOffsetV( bitstream.readSvlc() );  // se(v)
      mpdu.set3dOffsetD( bitstream.readSvlc() );  // se(v)
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
        mpdu.set3dRangeD( bitstream.readSvlc() );  // se(v)
      } else
        mpdu.set3dRangeD( 0 );

      if ( asps.getPLREnabledFlag() ) {
        overridePlrFlag = ( bitstream.read( 1 ) );  // u(1)
        mpdu.setOverridePlrFlag( static_cast<int64_t>( overridePlrFlag ) );
      }
    }
  }
  if ( overridePlrFlag && asps.getPLREnabledFlag() ) {
    auto &plrd = mpdu.getPLRData();
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n", prevPatchSizeU_, prevPatchSizeV_,
                     mpdu.get2dDeltaSizeX(), mpdu.get2dDeltaSizeY(), prevPatchSizeU_ + mpdu.get2dDeltaSizeX(),
                     prevPatchSizeV_ + mpdu.get2dDeltaSizeY() );
    plrd.allocate( prevPatchSizeU_ + mpdu.get2dDeltaSizeX(), prevPatchSizeV_ + mpdu.get2dDeltaSizeY() );
    plrData( plrd, syntax, asps, bitstream );
    prevPatchSizeU_ += mpdu.get2dDeltaSizeX();
    prevPatchSizeV_ += mpdu.get2dDeltaSizeY();
  }
  TRACE_BITSTREAM(
      "%zu frame %zu MergePatch => Ref Frame Idx = %d ShiftUV = %ld %ld "
      "DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      mpdu.getRefIndex(), mpdu.get2dPosX(), mpdu.get2dPosY(), mpdu.get2dDeltaSizeX(), mpdu.get2dDeltaSizeY(),
      mpdu.get3dOffsetU(), mpdu.get3dOffsetV(), mpdu.get3dOffsetD(), mpdu.get3dRangeD() );
}

// 8.3.7.6  Inter patch data unit syntax
void PCCBitstreamReader::interPatchDataUnit( InterPatchDataUnit &ipdu,
                                             AtlasTileHeader    &ath,
                                             PCCHighLevelSyntax &syntax,
                                             PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t                         afpsId          = ath.getAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp    &afps            = syntax.getAtlasFrameParameterSet( afpsId );
  size_t                         aspsId          = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRbsp &asps            = syntax.getAtlasSequenceParameterSet( aspsId );
  size_t                         numRefIdxActive = syntax.getNumRefIdxActive( ath );
  if ( numRefIdxActive > 1 ) {
    ipdu.setRefIndex( bitstream.readUvlc() );  // ue(v)
  } else {
    ipdu.setRefIndex( 0 );
  }
  TRACE_BITSTREAM( "%zu frame: numRefIdxActive = %zu reference = frame%zu\n", ipdu.getFrameIndex(), numRefIdxActive,
                   ipdu.getRefIndex() );
  ipdu.setRefPatchIndex( bitstream.readSvlc() );
  ipdu.set2dPosX( bitstream.readSvlc() );        // se(v)
  ipdu.set2dPosY( bitstream.readSvlc() );        // se(v)
  ipdu.set2dDeltaSizeX( bitstream.readSvlc() );  // se(v)
  ipdu.set2dDeltaSizeY( bitstream.readSvlc() );  // se(v)
  ipdu.set3dOffsetU( bitstream.readSvlc() );     // se(v)
  ipdu.set3dOffsetV( bitstream.readSvlc() );     // se(v)
  ipdu.set3dOffsetD( bitstream.readSvlc() );     // se(v)
  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    ipdu.set3dRangeD( bitstream.readSvlc() );  // se(v)
  } else {
    ipdu.set3dRangeD( 0 );
  }
  TRACE_BITSTREAM(
      "%zu frame: numRefIdxActive = %zu reference = frame%zu patch%d 2Dpos = "
      "%ld %ld 2DdeltaSize = %ld %ld 3Dpos = %ld "
      "%ld %ld DeltaMaxZ = %ld\n",
      ipdu.getFrameIndex(), numRefIdxActive, ipdu.getRefIndex(), ipdu.getRefPatchIndex(), ipdu.get2dPosX(),
      ipdu.get2dPosY(), ipdu.get2dDeltaSizeX(), ipdu.get2dDeltaSizeY(), ipdu.get3dOffsetU(), ipdu.get3dOffsetV(),
      ipdu.get3dOffsetD(), ipdu.get3dRangeD() );
  if ( asps.getPLREnabledFlag() ) {
    auto   &atglPrev      = syntax.getAtlasTileLayer( prevFrameIndex_ );
    auto   &atghPrev      = atglPrev.getHeader();
    auto   &atgdPrev      = atglPrev.getDataUnit();
    auto   &pidPrev       = atgdPrev.getPatchInformationData( ipdu.getRefPatchIndex() + predPatchIndex_ );
    auto    patchModePrev = pidPrev.getPatchMode();
    int32_t sizeU         = ipdu.get2dDeltaSizeX();
    int32_t sizeV         = ipdu.get2dDeltaSizeY();
    if ( atghPrev.getType() == P_TILE ) {
      if ( patchModePrev == P_MERGE ) {
        auto &plrdPrev = pidPrev.getMergePatchDataUnit().getPLRData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      } else if ( patchModePrev == P_INTER ) {
        auto &plrdPrev = pidPrev.getInterPatchDataUnit().getPLRData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      } else if ( patchModePrev == P_INTRA ) {
        auto &plrdPrev = pidPrev.getPatchDataUnit().getPLRData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      }
    } else if ( atghPrev.getType() == I_TILE ) {
      if ( patchModePrev == I_INTRA ) {
        auto &plrdPrev = pidPrev.getPatchDataUnit().getPLRData();
        sizeU += plrdPrev.getBlockToPatchMapWidth();
        sizeV += plrdPrev.getBlockToPatchMapHeight();
      }
    }
    TRACE_BITSTREAM( "Prev Size = %d %d Delta Size = %ld %ld => %ld %ld \n", sizeU - ipdu.get2dDeltaSizeX(),
                     sizeV - ipdu.get2dDeltaSizeY(), ipdu.get2dDeltaSizeX(), ipdu.get2dDeltaSizeY(), sizeU, sizeV );
    auto &plrd = ipdu.getPLRData();
    plrd.allocate( sizeU, sizeV );
    plrData( plrd, syntax, asps, bitstream );
    prevPatchSizeU_ = sizeU;
    prevPatchSizeV_ = sizeV;
    predPatchIndex_ += ipdu.getRefPatchIndex() + 1;
  }
}

// 8.3.7.7  Raw patch data unit syntax
void PCCBitstreamReader::rawPatchDataUnit( RawPatchDataUnit   &rpdu,
                                           AtlasTileHeader    &ath,
                                           PCCHighLevelSyntax &syntax,
                                           PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  int32_t bitCount = ath.getRaw3dOffsetAxisBitCountMinus1() + 1;
  TRACE_BITSTREAM( " AtghRaw3dOffsetAxisBitCountMinus1 = %zu => bitcount = %d \n",
                   ath.getRaw3dOffsetAxisBitCountMinus1(), bitCount );
  auto &afti = syntax.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() ).getAtlasFrameTileInformationRbsp();
  auto  ath_id        = ath.getId();
  auto  tileIdToIndex = afti.getTileId( ath_id );
  uint32_t value;
  if ( afti.getAuxiliaryVideoTileRowHeight( tileIdToIndex ) ) {
    READ_CODE_DESC( value, 1, "patchInAuxiliaryVideoFlag" )
    rpdu.setPatchInAuxiliaryVideoFlag( value != 0U );  // u(1)
  } else {
    rpdu.setPatchInAuxiliaryVideoFlag( 0 );
  }

  READ_UVLC_DESC( value, "2dPosX" )
  rpdu.set2dPosX( value );  // ue(v)
  READ_UVLC_DESC( value, "2dPosY" )
  rpdu.set2dPosY( value );  // ue(v)
  READ_UVLC_DESC( value, "sizeXMinus1" )
  rpdu.set2dSizeXMinus1( value );  // ue(v)
  READ_UVLC_DESC( value, "sizeYMinus1" )
  rpdu.set2dSizeYMinus1( value );  // ue(v)
  READ_CODE_DESC( value, bitCount, "set3dOffsetU" )
  rpdu.set3dOffsetU( value );  // u(v)
  READ_CODE_DESC( value, bitCount, "set3dOffsetV" )
  rpdu.set3dOffsetV( value );  // u(v)
  READ_CODE_DESC( value, bitCount, "set3dOffsetD" )
  rpdu.set3dOffsetD( value );  // u(v)
  READ_UVLC_DESC( value, "rawPointsMinus1" )
  rpdu.setRawPointsMinus1( value );  // ue(v)
  auto &aaps = syntax.getAtlasAdaptationParameterSet( ath.getAtlasAdaptationParameterSetId() );
  if ( aaps.getVpccGsExtensionFlag() ) {
    auto &aapsGsExt = aaps.getAapsVpccGsExtension();
    rpduVpccGsExtension( bitstream, aapsGsExt, rpdu );
  }
  TRACE_BITSTREAM(
      "Raw Patch => UV %4zu %4zu  S=%4ld %4ld  UVD1=%4ld %4ld %4ld "
      "NumPcmPoints=%zu PatchInRawVideoFlag=%d \n",
      rpdu.get2dPosX(), rpdu.get2dPosY(), rpdu.get2dSizeXMinus1() + 1, rpdu.get2dSizeYMinus1() + 1, rpdu.get3dOffsetU(),
      rpdu.get3dOffsetV(), rpdu.get3dOffsetD(), (size_t)rpdu.getRawPointsMinus1() + 1,
      rpdu.getPatchInAuxiliaryVideoFlag() );
}

// 8.3.7.8 EOM patch data unit syntax
void PCCBitstreamReader::eomPatchDataUnit( EOMPatchDataUnit   &epdu,
                                           AtlasTileHeader    &ath,
                                           PCCHighLevelSyntax &syntax,
                                           PCCBitstream       &bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &afti = syntax.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() ).getAtlasFrameTileInformationRbsp();
  auto  ath_id        = ath.getId();
  auto  tileIdToIndex = afti.getTileId( ath_id );
  if ( afti.getAuxiliaryVideoTileRowHeight( tileIdToIndex ) ) {
    epdu.setPatchInAuxiliaryVideoFlag( bitstream.read( 1 ) != 0U );  // u(1)
  } else {
    epdu.setPatchInAuxiliaryVideoFlag( 0 );
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

// 8.3.7.9 Point local reconstruction data syntax
void PCCBitstreamReader::plrData( PLRData                       &plrd,
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
        plrd.setLevelFlag( bitstream.read( 1 ) != 0U );  // u(1)
      } else {
        plrd.setLevelFlag( true );
      }
      TRACE_BITSTREAM( "  LevelFlag = %u \n", plrd.getLevelFlag() );
      if ( !plrd.getLevelFlag() ) {
        for ( size_t i = 0; i < blockCount; i++ ) {
          plrd.setBlockPresentFlag( i, bitstream.read( 1 ) != 0U );  // u(1)
          if ( plrd.getBlockPresentFlag( i ) ) {
            plrd.setBlockModeMinus1( i, bitstream.read( bitCountMode ) );  // u(v)
          }
          TRACE_BITSTREAM( "  Mode[ %4zu / %4zu ]: Present = %d ModeMinus1 = %d \n", i, blockCount,
                           plrd.getBlockPresentFlag( i ),
                           plrd.getBlockPresentFlag( i ) ? plrd.getBlockModeMinus1( i ) : -1 );
        }
      } else {
        plrd.setPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
        if ( plrd.getPresentFlag() ) {
          plrd.setModeMinus1( bitstream.read( bitCountMode ) );  // u(v)
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
void PCCBitstreamReader::seiMessage( PCCBitstream       &bitstream,
                                     PCCHighLevelSyntax &syntax,
                                     NalUnitType         nalUnitType,
                                     PCCSEI             &sei ) {
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
  seiPayload( bitstream, syntax, nalUnitType, static_cast<SeiPayloadType>( payloadType ), payloadSize, sei );
  // if ( static_cast<SeiPayloadType>( payloadType ) == DECODED_ATLAS_INFORMATION_HASH ) {
  //   syntax.getSeiHash().push_back(
  //       static_cast<SEIDecodedAtlasInformationHash&>( *( sei.getSeiSuffix().back().get() ) ) );
  // }
}

// C.2 Sample stream V3C unit syntax and semantics
// C.2.1 Sample stream V3C header syntax
void PCCBitstreamReader::sampleStreamV3CHeader( PCCBitstream &bitstream, SampleStreamV3CUnit &ssvu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ssvu.setSsvhUnitSizePrecisionBytesMinus1( bitstream.read( 3 ) );  // u(3)
  bitstream.read( 5 );                                              // u(5)
}

// C.2.2 Sample stream NAL unit syntax
void PCCBitstreamReader::sampleStreamV3CUnit( PCCBitstream &bitstream, SampleStreamV3CUnit &ssvu, V3CUnit &v3cUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  v3cUnit.setSize( bitstream.read( 8 * ( ssvu.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) ) );  // u(v)
  auto pos = bitstream.getPosition();
  v3cUnit.getBitstream().copyFrom( bitstream, (size_t)pos.bytes_, v3cUnit.getSize() );
  uint8_t v3cUnitType8 = v3cUnit.getBitstream().buffer()[0];
  auto    v3cUnitType  = static_cast<V3CUnitType>( v3cUnitType8 >>= 3 );
  v3cUnit.setType( v3cUnitType );
  TRACE_BITSTREAM( "V3CUnitType: %hhu V3CUnitSize: %zu\n", v3cUnitType, v3cUnit.getSize() );
  printf( "  v3cUnit: size = %8zu type = %2hhu %s \n", v3cUnit.getSize(), v3cUnitType,
          toString( V3CUnitType( v3cUnitType ) ).c_str() );
}

// D.2 Sample stream NAL unit syntax and semantics
// D.2.1 Sample stream NAL header syntax
void PCCBitstreamReader::sampleStreamNalHeader( PCCBitstream &bitstream, SampleStreamNalUnit &ssnu ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ssnu.setSizePrecisionBytesMinus1( bitstream.read( 3 ) );  // u(3)
  bitstream.read( 5 );                                      // u(5)
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 = %lu \n", ssnu.getSizePrecisionBytesMinus1() );
}

// D.2.2 Sample stream NAL unit syntax
void PCCBitstreamReader::sampleStreamNalUnit( PCCHighLevelSyntax  &syntax,
                                              PCCBitstream        &bitstream,
                                              SampleStreamNalUnit &ssnu,
                                              size_t               index,
                                              PCCSEI              &prefixSEI ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &nalu = ssnu.getNalUnit( index );
  TRACE_BITSTREAM( "UnitSizePrecisionBytesMinus1 = %lu \n", ssnu.getSizePrecisionBytesMinus1() );
  nalu.setSize( bitstream.read( 8 * ( ssnu.getSizePrecisionBytesMinus1() + 1 ) ) );  // u(v)
  nalu.allocate();
  PCCBitstream ssnuBitstream;
#if defined( CONFORMANCE_TRACE ) || defined( BITSTREAM_TRACE )
  ssnuBitstream.setTrace( true );
  ssnuBitstream.setLogger( *logger_ );
#endif
  bitstream.copyTo( ssnuBitstream, nalu.getSize() );
  nalUnitHeader( ssnuBitstream, nalu );
  switch ( nalu.getType() ) {
    case NAL_ASPS: atlasSequenceParameterSetRbsp( syntax.addAtlasSequenceParameterSet(), syntax, ssnuBitstream ); break;
    case NAL_AFPS: atlasFrameParameterSetRbsp( syntax.addAtlasFrameParameterSet(), syntax, ssnuBitstream ); break;
    case NAL_AAPS: atlasAdaptationParameterSetRbsp( syntax.addAtlasAdaptationParameterSet(), ssnuBitstream ); break;
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
      atlasTileLayerRbsp( syntax.addAtlasTileLayer(), syntax, nalu.getType(), ssnuBitstream );
      syntax.getAtlasTileLayerList().back().getSEI().getSeiPrefix() = prefixSEI.getSeiPrefix();
      prefixSEI.getSeiPrefix().clear();
      break;
    case NAL_PREFIX_ESEI:
    case NAL_PREFIX_NSEI: seiRbsp( syntax, ssnuBitstream, nalu.getType(), prefixSEI ); break;
    case NAL_SUFFIX_ESEI:
    case NAL_SUFFIX_NSEI:
      seiRbsp( syntax, ssnuBitstream, nalu.getType(), syntax.getAtlasTileLayerList().back().getSEI() );
      break;
    default: fprintf( stderr, "sampleStreamNalUnit type = %d not supported\n", static_cast<int32_t>( nalu.getType() ) );
  }
}

// F.2  SEI payload syntax
// F.2.1  General SEI message syntax
void PCCBitstreamReader::seiPayload( PCCBitstream       &bitstream,
                                     PCCHighLevelSyntax &syntax,
                                     NalUnitType         nalUnitType,
                                     SeiPayloadType      payloadType,
                                     size_t              payloadSize,
                                     PCCSEI             &seiList ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  VpccSeiPayloadType vpccPayloadType;
  if ( payloadType == VPCC_REGISTERED ) {
    int32_t vpccPayloadTypeVal = 0;
    int32_t headerSize         = 0;
    int32_t byte               = 0;
    do {
      READ_CODE( byte, 8 );  // u(8)
      vpccPayloadTypeVal += byte;
      headerSize++;
    } while ( byte == 0xff );
    vpccPayloadType = static_cast<VpccSeiPayloadType>( vpccPayloadTypeVal );
    payloadSize -= headerSize;
  }
  SEI &sei = payloadType == VPCC_REGISTERED ? seiList.addSei( nalUnitType, vpccPayloadType )
                                            : seiList.addSei( nalUnitType, payloadType );
  printf( "        seiMessage: type = %d %s payloadSize = %zu \n", payloadType, toString( payloadType ).c_str(),
          payloadSize );
  fflush( stdout );
  if ( nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ) {
    if ( payloadType == BUFFERING_PERIOD ) {  // 0
      bufferingPeriod( bitstream, sei );
    } else if ( payloadType == ATLAS_FRAME_TIMING ) {  // 1
      assert( seiList.seiIsPresent( NAL_PREFIX_NSEI, BUFFERING_PERIOD ) );
      auto &bpsei = *seiList.getLastSei( NAL_PREFIX_NSEI, BUFFERING_PERIOD );
      atlasFrameTiming( bitstream, sei, bpsei, false );
    } else if ( payloadType == FILLER_PAYLOAD ) {  // 2
      fillerPayload( bitstream, sei, payloadSize );
    } else if ( payloadType == USER_DATAREGISTERED_ITUTT35 ) {  // 3
      userDataRegisteredItuTT35( bitstream, sei, payloadSize );
    } else if ( payloadType == USER_DATA_UNREGISTERED ) {  // 4
      userDataUnregistered( bitstream, sei, payloadSize );
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
      vpccRegisteredSEI( bitstream, vpccPayloadType, payloadSize, sei );
    } else {
      reservedSeiMessage( bitstream, sei, payloadSize );
    }
  } else {
    if ( payloadType == FILLER_PAYLOAD ) {  // 2
      fillerPayload( bitstream, sei, payloadSize );
    } else if ( payloadType == USER_DATAREGISTERED_ITUTT35 ) {  // 3
      userDataRegisteredItuTT35( bitstream, sei, payloadSize );
    } else if ( payloadType == USER_DATA_UNREGISTERED ) {  // 4
      userDataUnregistered( bitstream, sei, payloadSize );
    } else if ( payloadType == DECODED_ATLAS_INFORMATION_HASH ) {  // 21
      decodedAtlasInformationHash( bitstream, sei );
    } else {
      reservedSeiMessage( bitstream, sei, payloadSize );
    }
  }
  if ( moreDataInPayload( bitstream ) ) {
    if ( payloadExtensionPresent( bitstream ) ) {
      bitstream.read( 1 );  // u(v)
    }
    byteAlignment( bitstream );
  }
}

// Specified in ISO/IEC 23002-7 -> Filler payload SEI message syntax
void PCCBitstreamReader::fillerPayload( PCCBitstream &bitstream, SEI &sei, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  for ( size_t k = 0; k < payloadSize; k++ ) {
    bitstream.read( 8 );  // f(8) equal to 0xFF
  }
}

// Specified in ISO/IEC 23002-7 ->  User data registered by Recommendation ITU-T T.35 SEI message syntax
void PCCBitstreamReader::userDataRegisteredItuTT35( PCCBitstream &bitstream, SEI &seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIUserDataRegisteredItuTT35 &>( seiAbstract );
  sei.setCountryCode( bitstream.read( 8 ) );  // b(8)
  payloadSize--;
  if ( sei.getCountryCode() == 0xFF ) {
    sei.setCountryCodeExtensionByte( bitstream.read( 8 ) );  // b(8)
    payloadSize--;
  }
  auto &payload = sei.getPayloadByte();
  payload.resize( payloadSize );
  for ( auto &element : payload ) {
    element = bitstream.read( 8 );  // b(8)
  }
}

// Specified in ISO/IEC 23002-7 ->  User data unregistered SEI message syntax
void PCCBitstreamReader::userDataUnregistered( PCCBitstream &bitstream, SEI &seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIUserDataUnregistered &>( seiAbstract );
  for ( size_t i = 0; i < 16; i++ ) {
    sei.setUuidIsoIec11578( i, bitstream.read( 8 ) );  // u(128) <=> 16 * u(8)
  }
  payloadSize -= 16;
  sei.getUserDataPayloadByte().resize( payloadSize );
  for ( size_t i = 0; i < payloadSize; i++ ) {
    sei.setUserDataPayloadByte( i, bitstream.read( 8 ) );  // b(8)
  }
}

// F.2.2  Recovery point SEI message syntax
void PCCBitstreamReader::recoveryPoint( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIRecoveryPoint &>( seiAbstract );
  sei.setRecoveryAfocCnt( bitstream.readSvlc() );  // se(v)
  sei.setExactMatchFlag( bitstream.read( 1 ) );    // u(1)
  sei.setBrokenLinkFlag( bitstream.read( 1 ) );    // u(1)
}

// F.2.3  No reconstruction SEI message syntax
void PCCBitstreamReader::noReconstruction( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
}

// Specified in ISO/IEC 23002-7 ->  Reserved SEI message syntax
void PCCBitstreamReader::reservedSeiMessage( PCCBitstream &bitstream, SEI &seiAbstract, size_t payloadSize ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIReservedSeiMessage &>( seiAbstract );
  sei.getPayloadByte().resize( payloadSize );
  for ( size_t i = 0; i < payloadSize; i++ ) {
    sei.setPayloadByte( i, bitstream.read( 8 ) );  // b(8)
  }
}

// F.2.4  SEI manifest SEI message syntax
void PCCBitstreamReader::seiManifest( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIManifest &>( seiAbstract );
  sei.setNumSeiMsgTypes( bitstream.read( 16 ) );  // u(16)
  sei.allocate();
  for ( size_t i = 0; i < sei.getNumSeiMsgTypes(); i++ ) {
    sei.setSeiPayloadType( i, bitstream.read( 16 ) );  // u(16)
    sei.setSeiDescription( i, bitstream.read( 8 ) );   // u(8)
  }
}

// F.2.5  SEI prefix indication SEI message syntax
void PCCBitstreamReader::seiPrefixIndication( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIPrefixIndication &>( seiAbstract );
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

// F.2.6  Active substreams SEI message syntax
void PCCBitstreamReader::activeSubBitstreams( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIActiveSubBitstreams &>( seiAbstract );
  sei.setActiveSubBitstreamsCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !sei.getActiveSubBitstreamsCancelFlag() ) {
    sei.setActiveAttributesChangesFlag( bitstream.read( 1 ) != 0U );    // u(1)
    sei.setActiveMapsChangesFlag( bitstream.read( 1 ) != 0U );          // u(1)
    sei.setAuxiliarySubstreamsActiveFlag( bitstream.read( 1 ) != 0U );  // u(1)
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
}

// F.2.7  Component codec mapping SEI message syntax
void PCCBitstreamReader::componentCodecMapping( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIComponentCodecMapping &>( seiAbstract );
  sei.setComponentCodecCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !sei.getComponentCodecCancelFlag() ) {
    sei.setCodecMappingsCountMinus1( bitstream.read( 8 ) );  // u(8)
    sei.allocate();
    for ( size_t i = 0; i <= sei.getCodecMappingsCountMinus1(); i++ ) {
      sei.setCodecId( i, bitstream.read( 8 ) );                        // u(8)
      sei.setCodec4cc( sei.getCodecId( i ), bitstream.readString() );  // st(v)
    }
  }
}

// F.2.8  Volumetric Tiling SEI message syntax
// F.2.8.1 Scene object information SEI message syntax
void PCCBitstreamReader::sceneObjectInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEISceneObjectInformation &>( seiAbstract );
  sei.setPersistenceFlag( bool( bitstream.read( 1 ) ) );  // u(1)
  sei.setResetFlag( bool( bitstream.read( 1 ) ) );        // u(1)
  sei.setNumObjectUpdates( bitstream.readUvlc() );        // ue(v)
  sei.allocateObjectIdx();
  if ( sei.getNumObjectUpdates() > 0 ) {
    sei.setSimpleObjectsFlag( bool( bitstream.read( 1 ) ) );  // u(1)
    if ( static_cast<int>( sei.getSimpleObjectsFlag() ) == 0 ) {
      sei.setObjectLabelPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );       // u(1)
      sei.setPriorityPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );          // u(1)
      sei.setObjectHiddenPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );      // u(1)
      sei.setObjectDependencyPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );  // u(1)
      sei.setVisibilityConesPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );   // u(1)
      sei.set3dBoundingBoxPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );     // u(1)
      sei.setCollisionShapePresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );    // u(1)
      sei.setPointStylePresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );        // u(1)
      sei.setMaterialIdPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );        // u(1)
      sei.setExtensionPresentFlag( static_cast<bool>( bitstream.read( 1 ) ) );         // u(1)
    } else {
      sei.setObjectLabelPresentFlag( false );
      sei.setPriorityPresentFlag( false );
      sei.setObjectHiddenPresentFlag( false );
      sei.setObjectDependencyPresentFlag( false );
      sei.setVisibilityConesPresentFlag( false );
      sei.set3dBoundingBoxPresentFlag( false );
      sei.setCollisionShapePresentFlag( false );
      sei.setPointStylePresentFlag( false );
      sei.setMaterialIdPresentFlag( false );
      sei.setExtensionPresentFlag( false );
    }
    if ( sei.get3dBoundingBoxPresentFlag() ) {
      sei.set3dBoundingBoxScaleLog2( bitstream.read( 5 ) );        // u(5)
      sei.set3dBoundingBoxPrecisionMinus8( bitstream.read( 5 ) );  // u(5)
    }
    sei.setLog2MaxObjectIdxUpdated( bitstream.read( 5 ) );  // u(5)
    if ( sei.getObjectDependencyPresentFlag() ) {
      sei.setLog2MaxObjectDependencyIdx( bitstream.read( 5 ) );  // u(5)
    }
    for ( size_t i = 0; i <= sei.getNumObjectUpdates(); i++ ) {
      assert( sei.getObjectIdx().size() >= sei.getNumObjectUpdates() );
      sei.setObjectIdx( i, bitstream.read( sei.getLog2MaxObjectIdxUpdated() ) );  // u(v)
      size_t k = sei.getObjectIdx( i );
      sei.allocate( k + 1 );
      sei.setObjectCancelFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
      if ( sei.getObjectCancelFlag( k ) ) {
        if ( sei.getObjectLabelPresentFlag() ) {
          sei.setObjectLabelUpdateFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
          if ( sei.getObjectLabelUpdateFlag( k ) ) {
            sei.setObjectLabelIdx( k, bitstream.readUvlc() );  // ue(v)
          }
        }
        if ( sei.getPriorityPresentFlag() ) {
          sei.setPriorityUpdateFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
          if ( sei.getPriorityUpdateFlag( k ) ) {
            sei.setPriorityValue( k, bitstream.read( 4 ) );  // u(4)
          }
        }
        if ( sei.getObjectHiddenPresentFlag() ) {
          sei.setObjectHiddenFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
        }
        if ( sei.getObjectDependencyPresentFlag() ) {
          sei.setObjectDependencyUpdateFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
          if ( sei.getObjectDependencyUpdateFlag( k ) ) {
            sei.setObjectNumDependencies( k, bitstream.read( 4 ) );  // u(4)
            sei.allocateObjectNumDependencies( k, sei.getObjectNumDependencies( k ) );
            size_t bitCount = ceil( log2( sei.getObjectNumDependencies( k ) ) + 0.5 );
            for ( size_t j = 0; j < sei.getObjectNumDependencies( k ); j++ ) {
              sei.setObjectDependencyIdx( k, j, bitstream.read( bitCount ) );  // u(v)
            }
          }
        }
        if ( sei.getVisibilityConesPresentFlag() ) {
          sei.setVisibilityConesUpdateFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
          if ( sei.getVisibilityConesUpdateFlag( k ) ) {
            sei.setDirectionX( k, bitstream.read( 16 ) );  // u(16)
            sei.setDirectionY( k, bitstream.read( 16 ) );  // u(16)
            sei.setDirectionZ( k, bitstream.read( 16 ) );  // u(16)
            sei.setAngle( k, bitstream.read( 16 ) );       // u(16)
          }
        }  // cones

        if ( sei.get3dBoundingBoxPresentFlag() ) {
          sei.set3dBoundingBoxUpdateFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
          if ( sei.get3dBoundingBoxUpdateFlag( k ) ) {
            sei.set3dBoundingBoxX( k, bitstream.readUvlc() );       // ue(v)
            sei.set3dBoundingBoxY( k, bitstream.readUvlc() );       // ue(v)
            sei.set3dBoundingBoxZ( k, bitstream.readUvlc() );       // ue(v)
            sei.set3dBoundingBoxDeltaX( k, bitstream.readUvlc() );  // ue(v)
            sei.set3dBoundingBoxDeltaY( k, bitstream.readUvlc() );  // ue(v)
            sei.set3dBoundingBoxDeltaZ( k, bitstream.readUvlc() );  // ue(v)
          }
        }  // 3dBB

        if ( sei.getCollisionShapePresentFlag() ) {
          sei.setCollisionShapeUpdateFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
          if ( sei.getCollisionShapeUpdateFlag( k ) ) {
            sei.setCollisionShapeId( k, bitstream.read( 16 ) );  // u(16)
          }
        }  // collision
        if ( sei.getPointStylePresentFlag() ) {
          sei.setPointStyleUpdateFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
          if ( sei.getPointStyleUpdateFlag( k ) ) {
            sei.setPointShapeId( k, bitstream.read( 8 ) );  // u(8)
            sei.setPointSize( k, bitstream.read( 16 ) );    // u(16)
          }
        }  // pointstyle
        if ( sei.getMaterialIdPresentFlag() ) {
          sei.setMaterialIdUpdateFlag( k, bitstream.read( 1 ) != 0U );  // u(1)
          if ( sei.getMaterialIdUpdateFlag( k ) ) {
            sei.setMaterialId( k, bitstream.read( 16 ) );  // u(16)
          }
        }  // materialid
      }  // sei.getObjectCancelFlag(k)
    }  // for(size_t i=0; i<=sei.getNumObjectUpdates(); i++)
  }  // if( sei.getNumObjectUpdates() > 0 )
}

// F.2.8.2 Object label information SEI message syntax
void PCCBitstreamReader::objectLabelInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIObjectLabelInformation &>( seiAbstract );
  sei.setCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !sei.getCancelFlag() ) {
    sei.setLabelLanguagePresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
    if ( sei.getLabelLanguagePresentFlag() ) {
      while ( !bitstream.byteAligned() ) {
        bitstream.read( 1 );  // u(1)
      }
      sei.setLabelLanguage( bitstream.readString() );  // st(v)
    }
    sei.setNumLabelUpdates( bitstream.readUvlc() );  // ue(v)
    sei.allocate();
    for ( size_t i = 0; i < sei.getNumLabelUpdates(); i++ ) {
      sei.setLabelIdx( i, bitstream.readUvlc() );           // ue(v)
      sei.setLabelCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
      if ( !sei.getLabelCancelFlag() ) {
        while ( !bitstream.byteAligned() ) {
          bitstream.read( 1 );  // u(1)
        }
        sei.setLabel( sei.getLabelIdx( i ), bitstream.readString() );  // st(v)
      }
    }
    sei.setPersistenceFlag( bitstream.read( 1 ) );  // u(1)
  }
};

// F.2.8.3 Patch information SEI message syntax
void PCCBitstreamReader::patchInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIPatchInformation &>( seiAbstract );
  sei.setPersistenceFlag( bitstream.read( 1 ) != 0U );  // u(1)
  sei.setResetFlag( bitstream.read( 1 ) != 0U );        // u(1)
  sei.setNumTileUpdates( bitstream.readUvlc() );        // ue(v)
  if ( sei.getNumTileUpdates() > 0 ) {
    sei.setLog2MaxObjectIdxTracked( bitstream.read( 5 ) );  // u(5)
    sei.setLog2MaxPatchIdxUpdated( bitstream.read( 4 ) );   // u(4)
  }
  for ( size_t i = 0; i < sei.getNumTileUpdates(); i++ ) {
    sei.setTileId( i, bitstream.readUvlc() );  // ue(v)
    size_t j = sei.getTileId( i );
    sei.setTileCancelFlag( j, bitstream.read( 1 ) != 0U );  // u(1)
    sei.setNumPatchUpdates( j, bitstream.readUvlc() );      // ue(v)
    for ( size_t k = 0; k < sei.getNumPatchUpdates( j ); k++ ) {
      sei.setPatchIdx( j, k, bitstream.read( sei.getLog2MaxPatchIdxUpdated() ) );  // u(v)
      auto p = sei.getPatchIdx( j, k );
      sei.setPatchCancelFlag( j, p, bitstream.read( 1 ) != 0U );  // u(1)
      if ( !sei.getPatchCancelFlag( j, p ) ) {
        sei.setPatchNumberOfObjectsMinus1( j, p, bitstream.readUvlc() );  // ue(v)
        for ( size_t n = 0; n < sei.getPatchNumberOfObjectsMinus1( j, p ) + 1; n++ ) {
          sei.setPatchObjectIdx( j, p, n, bitstream.read( sei.getLog2MaxObjectIdxTracked() ) );  // u(v)
        }
      }
    }
  }
};

// F.2.8.4 Volumetric rectangle information SEI message syntax
void PCCBitstreamReader::volumetricRectangleInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIVolumetricRectangleInformation &>( seiAbstract );
  sei.setPersistenceFlag( bitstream.read( 1 ) != 0U );  // u(1)
  sei.setResetFlag( bitstream.read( 1 ) != 0U );        // u(1)
  sei.setNumRectanglesUpdates( bitstream.readUvlc() );  // ue(v)
  if ( sei.getNumRectanglesUpdates() > 0 ) {
    sei.setLog2MaxObjectIdxTracked( bitstream.read( 5 ) );     // u(5)
    sei.setLog2MaxRectangleIdxUpdated( bitstream.read( 4 ) );  // u(4)
  }
  for ( size_t k = 0; k < sei.getNumRectanglesUpdates(); k++ ) {
    sei.setRectangleIdx( k, bitstream.read( sei.getLog2MaxRectangleIdxUpdated() ) );  // u(v)
    auto p = sei.getRectangleIdx( k );
    sei.setRectangleCancelFlag( p, bitstream.read( 1 ) != 0U );  // u(1)
    if ( !sei.getRectangleCancelFlag( p ) ) {
      sei.allocate( p + 1 );
      sei.setBoundingBoxUpdateFlag( p, bitstream.read( 1 ) != 0U );  // u(1)
      if ( sei.getBoundingBoxUpdateFlag( p ) ) {
        sei.setBoundingBoxTop( p, ( bitstream.readUvlc() ) );     // ue(v)
        sei.setBoundingBoxLeft( p, ( bitstream.readUvlc() ) );    // ue(v)
        sei.setBoundingBoxWidth( p, ( bitstream.readUvlc() ) );   // ue(v)
        sei.setBoundingBoxHeight( p, ( bitstream.readUvlc() ) );  // ue(v)
      }
      sei.setRectangleNumberOfObjectsMinus1( p, bitstream.readUvlc() );  // ue(v)
      sei.allocateRectangleObjectIdx( p, sei.getRectangleNumberOfObjectsMinus1( p ) + 1 );
      for ( size_t n = 0; n < sei.getRectangleNumberOfObjectsMinus1( p ) + 1; n++ ) {
        sei.setRectangleObjectIdx( p, n, bitstream.read( sei.getLog2MaxObjectIdxTracked() ) );  // u(v)
      }
    }
  }
};

// F.2.8.5 Atlas object information  SEI message syntax
void PCCBitstreamReader::atlasObjectInformation( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIAtlasInformation &>( seiAbstract );
  sei.setPersistenceFlag( bitstream.read( 1 ) );   // u(1)
  sei.setResetFlag( bitstream.read( 1 ) );         // u(1)
  sei.setNumAtlasesMinus1( bitstream.read( 6 ) );  // u(6)
  sei.setNumUpdates( bitstream.readUvlc() );       // ue(v)
  sei.allocateAltasId();
  sei.allocateObjectIdx();
  if ( sei.getNumUpdates() > 0 ) {
    sei.setLog2MaxObjectIdxTracked( bitstream.read( 5 ) );  // u(5)
    for ( size_t i = 0; i < sei.getNumAtlasesMinus1() + 1; i++ ) {
      sei.setAtlasId( i, bitstream.read( 5 ) );  // u(6)
    }
    for ( size_t i = 0; i < sei.getNumUpdates() + 1; i++ ) {
      sei.setObjectIdx( i, bitstream.read( sei.getLog2MaxObjectIdxTracked() ) );  // u(v)
      for ( size_t j = 0; j < sei.getNumAtlasesMinus1() + 1; j++ ) {
        sei.setObjectInAtlasPresentFlag( i, j, bitstream.read( 1 ) );  // u(1)
      }
    }
  }
}

// F.2.9  Buffering period SEI message syntax
void PCCBitstreamReader::bufferingPeriod( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIBufferingPeriod &>( seiAbstract );
  sei.setNalHrdParamsPresentFlag( bitstream.read( 1 ) );             // u(1)
  sei.setAclHrdParamsPresentFlag( bitstream.read( 1 ) );             // u(1)
  sei.setInitialCabRemovalDelayLengthMinus1( bitstream.read( 5 ) );  // u(5)
  sei.setAuCabRemovalDelayLengthMinus1( bitstream.read( 5 ) );       // u(5)
  sei.setDabOutputDelayLengthMinus1( bitstream.read( 5 ) );          // u(5)
  sei.setIrapCabParamsPresentFlag( bitstream.read( 1 ) );            // u(1)
  if ( sei.getIrapCabParamsPresentFlag() ) {
    sei.setCabDelayOffset( bitstream.read( sei.getAuCabRemovalDelayLengthMinus1() + 1 ) );  // u(v)
    sei.setDabDelayOffset( bitstream.read( sei.getDabOutputDelayLengthMinus1() + 1 ) );     // u(v)
  }
  sei.setConcatenationFlag( bitstream.read( 1 ) != 0U );                                                   // u(1)
  sei.setAtlasCabRemovalDelayDeltaMinus1( bitstream.read( sei.getAuCabRemovalDelayLengthMinus1() + 1 ) );  // u(v)
  sei.setMaxSubLayersMinus1( bitstream.read( 3 ) );                                                        // u(3)
  sei.allocate();
  int32_t bitCount = sei.getInitialCabRemovalDelayLengthMinus1() + 1;
  for ( size_t i = 0; i <= sei.getMaxSubLayersMinus1(); i++ ) {
    sei.setHrdCabCntMinus1( i, bitstream.read( 3 ) );  // u(3)
    if ( sei.getNalHrdParamsPresentFlag() ) {
      for ( size_t j = 0; j < sei.getHrdCabCntMinus1( i ) + 1; j++ ) {
        sei.setNalInitialCabRemovalDelay( i, j, bitstream.read( bitCount ) );   // u(v)
        sei.setNalInitialCabRemovalOffset( i, j, bitstream.read( bitCount ) );  // u(v)
        if ( sei.getIrapCabParamsPresentFlag() ) {
          sei.setNalInitialAltCabRemovalDelay( i, j, bitstream.read( bitCount ) );   // u(v)
          sei.setNalInitialAltCabRemovalOffset( i, j, bitstream.read( bitCount ) );  // u(v)
        }
      }
    }
    if ( sei.getAclHrdParamsPresentFlag() ) {
      for ( size_t j = 0; j < sei.getHrdCabCntMinus1( i ) + 1; j++ ) {
        sei.setAclInitialCabRemovalDelay( i, j, bitstream.read( bitCount ) );   // u(v)
        sei.setAclInitialCabRemovalOffset( i, j, bitstream.read( bitCount ) );  // u(v)
        if ( sei.getIrapCabParamsPresentFlag() ) {
          sei.setAclInitialAltCabRemovalDelay( i, j, bitstream.read( bitCount ) );   // u(v)
          sei.setAclInitialAltCabRemovalOffset( i, j, bitstream.read( bitCount ) );  // u(v)
        }
      }
    }
  }
}

// F.2.10  Atlas frame timing SEI message syntax
void PCCBitstreamReader::atlasFrameTiming( PCCBitstream &bitstream,
                                           SEI          &seiAbstract,
                                           SEI          &seiBufferingPeriodAbstract,
                                           bool          cabDabDelaysPresentFlag ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei   = static_cast<SEIAtlasFrameTiming &>( seiAbstract );
  auto &bpsei = static_cast<SEIBufferingPeriod &>( seiBufferingPeriodAbstract );
  if ( cabDabDelaysPresentFlag ) {
    for ( uint32_t i = 0; i <= bpsei.getMaxSubLayersMinus1(); i++ ) {
      sei.setAftCabRemovalDelayMinus1( i, bitstream.read( bpsei.getAuCabRemovalDelayLengthMinus1() + 1 ) );  // u(v)
      sei.setAftDabOutputDelay( i, bitstream.read( bpsei.getDabOutputDelayLengthMinus1() + 1 ) );            // u(v)
    }
  }
}

// F.2.11	Viewport SEI messages family syntax
// F.2.11.1 Viewport camera parameters SEI messages syntax
void PCCBitstreamReader::viewportCameraParameters( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIViewportCameraParameters &>( seiAbstract );
  sei.setCameraId( bitstream.read( 10 ) );   // u(10)
  sei.setCancelFlag( bitstream.read( 1 ) );  // u(1)
  if ( sei.getCameraId() > 0 && !sei.getCancelFlag() ) {
    sei.setPersistenceFlag( bitstream.read( 1 ) );              // u(1)
    sei.setCameraType( bitstream.read( 3 ) );                   // u(3)
    if ( sei.getCameraType() == 0 ) {                           // equirectangular
      sei.setErpHorizontalFov( bitstream.read( 32 ) );          // u(32)
      sei.setErpVerticalFov( bitstream.read( 32 ) );            // u(32)
    } else if ( sei.getCameraType() == 1 ) {                    // perspective
      sei.setPerspectiveAspectRatio( bitstream.readFloat() );   // fl(32)
      sei.setPerspectiveHorizontalFov( bitstream.read( 32 ) );  // u(32)
    } else if ( sei.getCameraType() == 2 ) {                    /* orthographic */
      sei.setOrthoAspectRatio( bitstream.readFloat() );         // fl(32)
      sei.setOrthoHorizontalSize( bitstream.readFloat() );      // fl(32)
    }
    sei.setClippingNearPlane( bitstream.readFloat() );  // fl(32)
    sei.setClippingFarPlane( bitstream.readFloat() );   // fl(32)
  }
}

// F.2.11.2 Viewport position SEI messages syntax
void PCCBitstreamReader::viewportPosition( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIViewportPosition &>( seiAbstract );
  sei.setViewportId( bitstream.readUvlc() );                  // ue(v
  sei.setCameraParametersPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( sei.getCameraParametersPresentFlag() ) {
    sei.setViewportId( bitstream.read( 10 ) );  // u(10)
  }
  sei.setCancelFlag( bitstream.read( 1 ) );  // u(1)
  if ( !sei.getCancelFlag() ) {
    sei.setPersistenceFlag( bitstream.read( 1 ) );  // u(1)
    for ( size_t d = 0; d < 3; d++ ) {
      sei.setPosition( d, bitstream.readFloat() );  // fl(32)
    }
    sei.setRotationQX( bitstream.read( 16 ) );     // i(16)
    sei.setRotationQY( bitstream.read( 16 ) );     // i(16)
    sei.setRotationQZ( bitstream.read( 16 ) );     // i(16)
    sei.setCenterViewFlag( bitstream.read( 1 ) );  //  u(1)
    if ( !sei.getCenterViewFlag() ) {
      sei.setLeftViewFlag( bitstream.read( 1 ) );  // u(1)
    }
  }
}

// F.2.12	Decoded Atlas Information Hash SEI message syntax
// F.2.12.1 Decoded Atlas Information Hash syntax
void PCCBitstreamReader::decodedAtlasInformationHash( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIDecodedAtlasInformationHash &>( seiAbstract );
  sei.setCancelFlag( bitstream.read( 1 ) );  // u(1)
  if ( !sei.getCancelFlag() ) {
    sei.setPersistenceFlag( bitstream.read( 1 ) );                      // u(1)
    sei.setHashType( bitstream.read( 8 ) );                             // u(8)
    sei.setDecodedHighLevelHashPresentFlag( bitstream.read( 1 ) );      // u(1)
    sei.setDecodedAtlasHashPresentFlag( bitstream.read( 1 ) );          // u(1)
    sei.setDecodedAtlasB2pHashPresentFlag( bitstream.read( 1 ) );       // u(1)
    sei.setDecodedAtlasTilesHashPresentFlag( bitstream.read( 1 ) );     // u(1)
    sei.setDecodedAtlasTilesB2pHashPresentFlag( bitstream.read( 1 ) );  // u(1)
    bitstream.read( 1 );                                                // u(1)
    if ( sei.getDecodedHighLevelHashPresentFlag() ) { decodedHighLevelHash( bitstream, sei ); }
    if ( sei.getDecodedAtlasHashPresentFlag() ) { decodedAtlasHash( bitstream, sei ); }
    if ( sei.getDecodedAtlasB2pHashPresentFlag() ) { decodedAtlasB2pHash( bitstream, sei ); }
    if ( sei.getDecodedAtlasTilesHashPresentFlag() || sei.getDecodedAtlasTilesB2pHashPresentFlag() ) {
      sei.setNumTilesMinus1( bitstream.readUvlc() );   // ue(v)
      sei.setTileIdLenMinus1( bitstream.readUvlc() );  // ue(v)
      sei.allocateAtlasTilesHash( sei.getNumTilesMinus1() + 1 );
      for ( size_t t = 0; t <= sei.getNumTilesMinus1(); t++ ) {
        sei.setTileId( t, bitstream.read( sei.getTileIdLenMinus1() + 1 ) );  // u(v)
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

// F.2.12.2 Decoded high-level hash unit syntax
void PCCBitstreamReader::decodedHighLevelHash( PCCBitstream &bitstream, SEI &seiAbs ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      sei.setHighLevelMd5( i, bitstream.read( 8 ) );  // b(8)
    }
  } else if ( hType == 1 ) {
    sei.setHighLevelCrc( bitstream.read( 16 ) );  // u(16)
  } else if ( hType == 2 ) {
    sei.setHighLevelCheckSum( bitstream.read( 32 ) );  // u(32)
  }
}

// F.2.12.3 Decoded atlas hash unit syntax
void PCCBitstreamReader::decodedAtlasHash( PCCBitstream &bitstream, SEI &seiAbs ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      sei.setAtlasMd5( i, bitstream.read( 8 ) );  // b(8)
    }
  } else if ( hType == 1 ) {
    sei.setAtlasCrc( bitstream.read( 16 ) );  // u(16)
  } else if ( hType == 2 ) {
    sei.setAtlasCheckSum( bitstream.read( 32 ) );  // u(32)
  }
}

// F.2.12.4 Decoded atlas b2p hash unit syntax
void PCCBitstreamReader::decodedAtlasB2pHash( PCCBitstream &bitstream, SEI &seiAbs ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      sei.setAtlasB2pMd5( i, bitstream.read( 8 ) );  // b(8)
    }
  } else if ( hType == 1 ) {
    sei.setAtlasB2pCrc( bitstream.read( 16 ) );  // u(16)
  } else if ( hType == 2 ) {
    sei.setAtlasB2pCheckSum( bitstream.read( 32 ) );  // u(32)
  }
}

// F.2.12.5 Decoded atlas tiles hash unit syntax
void PCCBitstreamReader::decodedAtlasTilesHash( PCCBitstream &bitstream, SEI &seiAbs, size_t id ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      sei.setAtlasTilesMd5( id, i, bitstream.read( 8 ) );  // b(8)
    }
  } else if ( hType == 1 ) {
    sei.setAtlasTilesCrc( id, bitstream.read( 16 ) );  // u(16)
  } else if ( hType == 2 ) {
    sei.setAtlasTilesCheckSum( id, bitstream.read( 32 ) );  // u(32)
  }
}

// F.2.12.6 Decoded atlas tile b2p hash unit syntax
void PCCBitstreamReader::decodedAtlasTilesB2pHash( PCCBitstream &bitstream, SEI &seiAbs, size_t id ) {
  auto   &sei   = static_cast<SEIDecodedAtlasInformationHash &>( seiAbs );
  uint8_t hType = sei.getHashType();
  if ( hType == 0 ) {
    for ( size_t i = 0; i < 16; i++ ) {
      sei.setAtlasTilesB2pMd5( id, i, bitstream.read( 8 ) );  // b(8)
    }
  } else if ( hType == 1 ) {
    sei.setAtlasTilesB2pCrc( id, bitstream.read( 16 ) );  // u(16)
  } else if ( hType == 2 ) {
    sei.setAtlasTilesB2pCheckSum( id, bitstream.read( 32 ) );  // u(32)
  }
}

// F.2.13 Time code SEI message syntax
void PCCBitstreamReader::timeCode( PCCBitstream &bitstream, SEI &seiAbstract ) {
  auto &sei = static_cast<SEITimeCode &>( seiAbstract );
  sei.setNumUnitsInTick( bitstream.read( 32 ) );    // u(32)
  sei.setTimeScale( bitstream.read( 32 ) );         // u(32)
  sei.setCountingType( bitstream.read( 5 ) );       // u(5)
  sei.setFullTimestampFlag( bitstream.read( 1 ) );  // u(1)
  sei.setDiscontinuityFlag( bitstream.read( 1 ) );  // u(1)
  sei.setCntDroppedFlag( bitstream.read( 1 ) );     // u(1)
  sei.setNFrames( bitstream.read( 9 ) );            // u(9)
  if ( sei.getFullTimestampFlag() ) {
    sei.setSecondsValue( bitstream.read( 6 ) );  // u(6)
    sei.setMinutesValue( bitstream.read( 6 ) );  // u(6)
    sei.setHoursValue( bitstream.read( 5 ) );    // u(5)
  } else {
    sei.setSecondFlag( bitstream.read( 1 ) );  // u(1)
    if ( sei.getSecondFlag() ) {
      sei.setSecondsValue( bitstream.read( 6 ) );  // u(6)
      sei.setMinutesFlag( bitstream.read( 1 ) );   // u(1)
      if ( sei.getMinutesFlag() ) {
        sei.setMinutesValue( bitstream.read( 6 ) );  // u(6)
        sei.setHoursFlag( bitstream.read( 1 ) );     // u(1)
        if ( sei.getHoursFlag() ) {
          sei.setHoursValue( bitstream.read( 5 ) );  // u(5)
        }
      }
    }
  }
  sei.setTimeOffsetLength( bitstream.read( 5 ) );  // u(5)
  if ( sei.getTimeOffsetLength() > 0 ) {
    sei.setTimeOffsetValue( bitstream.readS( sei.getTimeOffsetLength() ) );  // i(v)
  }
}

// F.2.14	Packed independent regions SEI payload syntax

// F.2.15	Custom attribute information SEI payload syntax

// G.2  VUI syntax
// G.2.1  VUI parameters syntax
void PCCBitstreamReader::vuiParameters( PCCBitstream &bitstream, VUIParameters &vp ) {
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
  vp.setTileRestrictionsPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( vp.getTileRestrictionsPresentFlag() ) {
    vp.setFixedAtlasTileStructureFlag( bitstream.read( 1 ) );              // u(1)
    vp.setFixedVideoTileStructureFlag( bitstream.read( 1 ) );              //	u(1)
    vp.setConstrainedTilesAcrossV3cComponentsIdc( bitstream.readUvlc() );  // ue(v)
    vp.setMaxNumTilesPerAtlasMinus1( bitstream.readUvlc() );               // ue(v)
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
        vp.setAnchorPoint( d, bitstream.readUvlc() );  // u(v)
      }
    }
  }
}
// G.2.2  HRD parameters syntax
void PCCBitstreamReader::hrdParameters( PCCBitstream &bitstream, HrdParameters &hp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  hp.setNalParametersPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  hp.setAclParametersPresentFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( hp.getNalParametersPresentFlag() || hp.getAclParametersPresentFlag() ) {
    hp.setBitRateScale( bitstream.read( 4 ) );  // u(4)
    hp.setCabSizeScale( bitstream.read( 4 ) );  // u(4)
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
      hrdSubLayerParameters( bitstream, hp.getHdrSubLayerParameters( 0, i ), hp.getCabCntMinus1( i ) );
    }
    if ( hp.getAclParametersPresentFlag() ) {
      hrdSubLayerParameters( bitstream, hp.getHdrSubLayerParameters( 1, i ), hp.getCabCntMinus1( i ) );
    }
  }
}

// G.2.3  Sub-layer HRD parameters syntax
void PCCBitstreamReader::hrdSubLayerParameters( PCCBitstream &bitstream, HrdSubLayerParameters &hlsp, size_t cabCnt ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  hlsp.allocate( cabCnt + 1 );
  for ( size_t i = 0; i <= cabCnt; i++ ) {
    hlsp.setBitRateValueMinus1( i, bitstream.readUvlc() );  // ue(v)
    hlsp.setCabSizeValueMinus1( i, bitstream.readUvlc() );  // ue(v)
    hlsp.setCbrFlag( i, bitstream.read( 1 ) != 0U );        // u(1)
  }
}

// G.2.4 Maximum coded video resolution syntax
void PCCBitstreamReader::maxCodedVideoResolution( PCCBitstream &bitstream, MaxCodedVideoResolution &mcvr ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "%s \n", __func__ );
  mcvr.setOccupancyResolutionPresentFlag( bitstream.read( 1 ) );  // u(1)
  mcvr.setGeometryResolutionPresentFlag( bitstream.read( 1 ) );   // u(1)
  mcvr.setAttributeResolutionPresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( mcvr.getOccupancyResolutionPresentFlag() ) {
    mcvr.setOccupancyWidth( bitstream.readUvlc() );   // ue(v)
    mcvr.setOccupancyHeight( bitstream.readUvlc() );  // ue(v)
  }
  if ( mcvr.getGeometryResolutionPresentFlag() ) {
    mcvr.setGeometryWidth( bitstream.readUvlc() );   // ue(v)
    mcvr.setGeometryHeight( bitstream.readUvlc() );  // ue(v)
  }
  if ( mcvr.getAttributeResolutionPresentFlag() ) {
    mcvr.setAttributeWidth( bitstream.readUvlc() );   // ue(v)
    mcvr.setAttributeHeight( bitstream.readUvlc() );  // ue(v)
  }
}

// G.2.5 Coordinate system parameters syntax
void PCCBitstreamReader::coordinateSystemParameters( PCCBitstream &bitstream, CoordinateSystemParameters &csp ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  csp.setForwardAxis( bitstream.read( 2 ) );    // u(2)
  csp.setDeltaLeftAxis( bitstream.read( 1 ) );  // u(1)
  csp.setForwardSign( bitstream.read( 1 ) );    // u(1)
  csp.setLeftSign( bitstream.read( 1 ) );       // u(1)
  csp.setUpSign( bitstream.read( 1 ) );         // u(1)
}

// H.8	Syntax and Semantics
// H.8.3.6.1.3	ASPS V-PCC extension syntax
void PCCBitstreamReader::aspsVpccExtension( PCCBitstream                  &bitstream,
                                            AtlasSequenceParameterSetRbsp &asps,
                                            AspsVpccExtension             &ext ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ext.setRemoveDuplicatePointEnableFlag( bitstream.read( 1 ) );
  if ( asps.getPixelDeinterleavingFlag() || asps.getPLREnabledFlag() ) {
    ext.setSurfaceThicknessMinus1( bitstream.readUvlc() );  // ue(v)
  }
}

// H.8.3.6.3.2	AAPS V-PCC extension syntax
void PCCBitstreamReader::aapsVpccExtension( PCCBitstream &bitstream, AapsVpccExtension &ext ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  ext.setCameraParametersPresentFlag( bitstream.read( 1 ) );  // u(1);
  if ( ext.getCameraParametersPresentFlag() ) { atlasCameraParameters( bitstream, ext.getAtlasCameraParameters() ); }
}

// H.8.3.6.3.3	Atlas camera parameters syntax
void PCCBitstreamReader::atlasCameraParameters( PCCBitstream &bitstream, AtlasCameraParameters &acp ) {
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

// H.8.3.6.3.4	AAPS V-PCC GS extension syntax
void PCCBitstreamReader::aapsVpccGsExtension( PCCBitstream &bitstream, AapsVpccGsExtension &ext ) {
  TRACE_BITSTREAM_IN( "%s \n", __func__ );
  int value;
  READ_CODE_DESC( value, 1, "aaps_vpcc_gs_dequantization_present_flag" );
  ext.setDequantizationPresentFlag( value );  // u(1)
  READ_CODE_DESC( value, 1, "aaps_vpcc_gs_adaptive_quantization_enable_flag" );
  ext.setAdaptiveQuantizationEnableFlag( value );  // u(1)
  READ_CODE_DESC( value, 1, "aaps_vpcc_gs_sh_property_present_flag" );
  ext.setShPropertyPresentFlag( value );  // u(1)
  READ_CODE_DESC( value, 1, "aaps_vpcc_gs_pred_enable_flag" );
  ext.setPredEnableFlag( value );  // u(1)
  READ_CODE_DESC( value, 1, "aaps_vpcc_gs_2dsh_transform_present_flag" );
  ext.setTransform2DPresentFlag( value );  // u(1)
  READ_CODE_DESC( value, 1, "aaps_vpcc_gs_scale_transform_present_flag" );
  ext.setScaleTransformPresentFlag( value );  // u(1)
  READ_CODE_DESC( value, 1, "placeholder_for_aaps_vpcc_gs_raw_patch_data_arrangement_flag" );
  if ( value != 0 ) {
    TRACE_BITSTREAM(
        "Error: placeholder_for_aaps_vpcc_gs_raw_patch_data_arrangement_flag is set to 1, but should be 0\n" );
    exit( EXIT_FAILURE );
  }
  READ_CODE_DESC( value, 1, "aaps_vpcc_gs_opacity_property_present_flag" );
  ext.setOpacityPropertyPresentFlag( value );  // u(1)
  READ_CODE_DESC( value, 8, "aaps_vpcc_gs_reserved_zero_8bits" );
  ext.setReservedZero8bits( value );  // u(8);

  if ( ext.getDequantizationPresentFlag() ) { gsDequantization( bitstream, ext.getGsDequantization() ); }
  if ( ext.getShPropertyPresentFlag() ) { gsShProperty( bitstream, ext.getGsShProperty() ); }
  if ( ext.getPredEnableFlag() ) { gsPrediction( bitstream, ext.getGsPrediction() ); }
  if ( ext.getTransform2DPresentFlag() ) { gs2DTransform( bitstream, ext.getGs2DTransform() ); }
  if ( ext.getScaleTransformPresentFlag() ) { gsScaleTransform( bitstream, ext.getGsScaleTransform() ); }
  if ( ext.getOpacityPropertyPresentFlag() ) { gsOpacityProperty( bitstream, ext.getGsOpacityProperty() ); }

  if ( ext.getReservedZero8bits() != 0 ) {
    while ( moreRbspData( bitstream ) ) {
      bitstream.read( 1 );  // aaps_vpcc_gs_extension_data_bit
    }
  }
  byteAlignment( bitstream );
  TRACE_BITSTREAM_OUT( "%s \n", __func__ );
}

// H.8.3.6.3.5	Gaussian splat dequantization syntax
void PCCBitstreamReader::gsDequantization( PCCBitstream &bitstream, GsDequantization &gsd ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  int   value;
  float valueFlt;
  READ_UVLC_DESC( value, "DequantizationType" );
  gsd.setDequantizationType( value );
  if ( gsd.getDequantizationType() == 1 ) {
    for ( int i = 0; i < 3; i++ ) {
      READ_FLOAT_DESC( valueFlt, "MinPosition" );
      gsd.setMinPosition( i, valueFlt );
      READ_FLOAT_DESC( valueFlt, "MaxPosition" );
      gsd.setMaxPosition( i, valueFlt );
    }
    READ_CODE_DESC( value, 1, "RotationPresentFlag" );
    gsd.setRotationPresentFlag( value );
    if ( gsd.getRotationPresentFlag() ) {
      READ_CODE_DESC( value, 1, "RotationSameMinMaxFlag" );
      gsd.setRotSameMinMaxFlag( value );
      READ_CODE_DESC( value, 1, "placeholder_for_gsd_rot_quaternion_representation_type_flag" );
      if ( value != 0 ) {
        TRACE_BITSTREAM(
            "Error: placeholder_for_gsd_rot_quaternion_representation_type_flag is set to 1, but should be 0\n" );
        exit( EXIT_FAILURE );
      }
      for ( int i = 0; i < 4; i++ ) {
        if ( !gsd.getRotSameMinMaxFlag() || i == 0 ) {
          READ_FLOAT_DESC( valueFlt, "MinRotation" );
          gsd.setMinRotation( i, valueFlt );
          READ_FLOAT_DESC( valueFlt, "MaxRotation" );
          gsd.setMaxRotation( i, valueFlt );
        } else {
          gsd.setMinRotation( i, gsd.getMinRotation( 0 ) );
          gsd.setMaxRotation( i, gsd.getMaxRotation( 0 ) );
        }
      }
    }
    READ_CODE_DESC( value, 1, "ScalePresentFlag" );
    gsd.setScalePresentFlag( value );
    if ( gsd.getScalePresentFlag() ) {
      READ_CODE( value, 1 );
      gsd.setScaleSameNumQuantRegionMinus1_( value );
      READ_CODE( value, 1 );
      gsd.setScaleSameNumBitsForTransitionPoint_( value );
      if ( gsd.getScaleSameNumQuantRegionMinus1_() ) {
        if ( gsd.getScaleSameNumBitsForTransitionPoint_() ) {
          READ_CODE( value, 1 );
          gsd.setScaleSameQuantRegionTransitionPoint_( value );
        } else {
          gsd.setScaleSameQuantRegionTransitionPoint_( false );
        }
        READ_CODE( value, 1 );
        gsd.setScaleSameRegionStepSize_( value );
      } else {
        gsd.setScaleSameQuantRegionTransitionPoint_( false );
        gsd.setScaleSameRegionStepSize_( false );
      }
      READ_CODE_DESC( value, 1, "ScaleSameMinMaxFlag" );
      gsd.setScaleSameMinMaxFlag( value );
      uint32_t numBitsForTransitionPoint{};
      uint32_t numRegion{};
      for ( int i = 0; i < 3; i++ ) {
        if ( !gsd.getScaleSameMinMaxFlag() || i == 0 ) {
          READ_FLOAT_DESC( valueFlt, "MinScale" );
          gsd.setMinScale( i, valueFlt );
          READ_FLOAT_DESC( valueFlt, "MaxScale" );
          gsd.setMaxScale( i, valueFlt );
        } else {
          gsd.setMinScale( i, gsd.getMinScale( 0 ) );
          gsd.setMaxScale( i, gsd.getMaxScale( 0 ) );
        }
        if ( ( i == 0 ) || !gsd.getScaleSameNumQuantRegionMinus1_() ) READ_UVLC( numRegion );
        gsd.setScaleNumQuantRegionsMinus1( i, numRegion );
        if ( gsd.getScaleNumQuantRegionsMinus1( i ) ) {
          if ( ( i == 0 ) || !gsd.getScaleSameNumBitsForTransitionPoint_() ) READ_CODE( numBitsForTransitionPoint, 4 );
          gsd.setScaleNumBitsForTransitionPoint( i, numBitsForTransitionPoint );
          if ( ( i == 0 ) || !gsd.getScaleSameQuantRegionTransitionPoint_() ) {
            for ( int j = 0; j < gsd.getScaleNumQuantRegionsMinus1( i ); j++ ) {
              uint16_t transitionPoints;
              READ_CODE( transitionPoints, numBitsForTransitionPoint );
              gsd.setScaleQuantRegionTransitionPoint( i, j, transitionPoints );
            }
          } else {
            for ( int j = 0; j < gsd.getScaleNumQuantRegionsMinus1( i ); j++ ) {
              uint16_t transitionPoints = gsd.getScaleQuantRegionTransitionPoint( 0, j );
              gsd.setScaleQuantRegionTransitionPoint( i, j, transitionPoints );
            }
          }
          if ( ( i == 0 ) || !gsd.getScaleSameRegionStepSize_() ) {
            for ( int j = 0; j < gsd.getScaleNumQuantRegionsMinus1( i ) + 1; j++ ) {
              float stepSize;
              READ_FLOAT( stepSize );
              gsd.setScaleRegionStepSize( i, j, stepSize );
            }
          } else {
            for ( int j = 0; j < gsd.getScaleNumQuantRegionsMinus1( i ) + 1; j++ ) {
              float stepSize = gsd.getScaleRegionStepSize( 0, j );
              gsd.setScaleRegionStepSize( i, j, stepSize );
            }
          }
        }
      }
    }
    READ_CODE_DESC( value, 1, "OpacityPresentFlag" );
    gsd.setOpacityPresentFlag( value );
    if ( gsd.getOpacityPresentFlag() ) {
      READ_FLOAT_DESC( valueFlt, "MinOpacity" );
      gsd.setMinOpacity( valueFlt );
      READ_FLOAT_DESC( valueFlt, "MaxOpacity" );
      gsd.setMaxOpacity( valueFlt );
      READ_UVLC_DESC( value, "OpacityNumQuantRegionsMinus1" );
      gsd.setOpacityNumQuantRegionsMinus1( value );
      if ( gsd.getOpacityNumQuantRegionsMinus1() > 0 ) {
        READ_CODE_DESC( value, 4, "OpacityNumBitsForTransitionPoint" );
        gsd.setOpacityNumBitsForTransitionPoint( value );
        for ( int m = 0; m < gsd.getOpacityNumQuantRegionsMinus1(); m++ ) {
          READ_CODE_DESC( value, gsd.getOpacityNumBitsForTransitionPoint(), "OpacityQuantRegionTransitionPoint" );
          gsd.setOpacityQuantRegionTransitionPoint( m, value );
        }
        for ( int m = 0; m < gsd.getOpacityNumQuantRegionsMinus1() + 1; m++ ) {
          READ_FLOAT_DESC( valueFlt, "OpacityRegionStepSize" );
          gsd.setOpacityRegionStepSize( m, valueFlt );
        }
      }
    }
    READ_CODE_DESC( value, 1, "ShPresentFlag" );
    gsd.setShPresentFlag( value );
    if ( gsd.getShPresentFlag() ) {
      READ_CODE_DESC( value, 1, "ShMultipleRegionsEnableFlag" );
      gsd.setShMultipleRegionsEnableFlag( value );
      if ( gsd.getShMultipleRegionsEnableFlag() ) {
        READ_CODE_DESC( value, 1, "ShSameNumQuantRegionsMinus1Flag" );
        gsd.setShSameNumRegionMius1Flag( value );
        READ_CODE_DESC( value, 1, "placeholder_for_gsd_sh_same_num_bits_for_transition_point_flag" );
        if ( value != 0 ) {
          TRACE_BITSTREAM(
              "Error: placeholder_for_gsd_sh_same_num_bits_for_transition_point_flag is set to 1, but should be 0\n" );
          exit( EXIT_FAILURE );
        }
        READ_CODE_DESC( value, 1, "ShSameQuantRegionTransitionPointFlag" );
        gsd.setShSameQuantRegionTransitionPointFlag( value );
        READ_CODE_DESC( value, 1, "ShSameRegionStepSizeFlag" );
        gsd.setShSameRegionStepSizeFlag( value );
      }
      READ_CODE_DESC( value, 1, "ShSameMinMaxFlag" );
      gsd.setShSameMinMaxFlag( value );
      READ_UVLC_DESC( value, "ShNumLvl" );
      gsd.setShNumLvl( value );
      for ( int i = 0; i < gsd.getShNumLvl(); i++ ) {
        READ_UVLC_DESC( value, "ShNumChan" );
        gsd.setShNumChan( i, value );
        for ( int c = 0; c < gsd.getShNumChan( i ); c++ ) {
          if ( !gsd.getShSameMinMaxFlag() || ( ( i == 0 ) && ( c == 0 ) ) ) {
            READ_FLOAT_DESC( valueFlt, "MinSh" );
            gsd.setMinSh( i, c, valueFlt );
            READ_FLOAT_DESC( valueFlt, "MaxSh" );
            gsd.setMaxSh( i, c, valueFlt );
          } else {
            gsd.setMinSh( i, c, gsd.getMinSh( 0, 0 ) );
            gsd.setMaxSh( i, c, gsd.getMaxSh( 0, 0 ) );
          }
          if ( gsd.getShMultipleRegionsEnableFlag() ) {
            if ( !gsd.getShSameNumRegionMius1Flag() || ( ( i == 0 ) && ( c == 0 ) ) ) {
              READ_UVLC_DESC( value, "NumQuantRegionsMinus1" );
              gsd.setNumQuantRegionsMinus1( i, c, value );
            } else {
              gsd.setNumQuantRegionsMinus1( i, c, gsd.getNumQuantRegionsMinus1( 0, 0 ) );
            }
            if ( gsd.getNumQuantRegionsMinus1( i, c ) > 0 ) {
              for ( int m = 0; m < gsd.getNumQuantRegionsMinus1( i, c ); m++ ) {
                if ( !gsd.getShSameQuantRegionTransitionPointFlag() || ( ( i == 0 ) && ( c == 0 ) ) ) {
                  READ_UVLC_DESC( value, "TransitionPoint" );
                  gsd.setQuantRegionTransitionPointDelta( i, c, m, value );

                  if ( m == 0 )
                    gsd.setQuantRegionTransitionPoint( i, c, m, value );
                  else
                    gsd.setQuantRegionTransitionPoint( i, c, m,
                                                       gsd.getQuantRegionTransitionPoint( i, c, m - 1 ) + value );

                } else {
                  gsd.setQuantRegionTransitionPoint( i, c, m, gsd.getQuantRegionTransitionPoint( 0, 0, m ) );
                }
              }
              for ( int m = 0; m < gsd.getNumQuantRegionsMinus1( i, c ) + 1; m++ ) {
                if ( !gsd.getShSameRegionStepSizeFlag() || ( ( i == 0 ) && ( c == 0 ) ) ) {
                  READ_FLOAT_DESC( valueFlt, "RegionStepSize" );
                  gsd.setRegionStepSize( i, c, m, valueFlt );
                } else {
                  gsd.setRegionStepSize( i, c, m, gsd.getRegionStepSize( 0, 0, m ) );
                }
              }
            }
          }
        }
      }
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.6	Gaussian splat spherical harmonics property syntax
void PCCBitstreamReader::gsShProperty( PCCBitstream &bitstream, GsSphericalHarmonicsProperty &gshp ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  uint32_t value;
  READ_UVLC_DESC( value, "ShMappingMethod" );
  gshp.setShMappingMethod( value );
  if ( gshp.getShMappingMethod() == 2 ) {
    READ_CODE_DESC( value, 6, "ShNumBitsMappingTableEntry" );
    gshp.setShNumBitsMappingTableEntry( value );
    uint32_t shNumACCoeff = 45;
    gshp.resizeShMappingTable( shNumACCoeff );
    for ( int m = 0; m < shNumACCoeff; m++ ) {
      READ_CODE_DESC( value, gshp.getShNumBitsMappingTableEntry(), "ShMappingTable" );
      gshp.setShMappingTable( m, value );
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.7	Gaussian splat spherical harmonics transform syntax
void PCCBitstreamReader::gs2DTransform( PCCBitstream &bitstream, Gs2DTransform &gs2DTransform ) {
  int   value;
  float valueFlt;
  TRACE_BITSTREAM_IN( "%s", __func__ );
  READ_CODE_DESC( value, 4, "placeholder_for_gsht_num_sh_minus1" );
  if ( value != 15 ) {
    TRACE_BITSTREAM( "Error: placeholder_for_gsht_num_sh_minus1 should be 15 \n" );
    exit( EXIT_FAILURE );
  }
  READ_CODE_DESC( value, 4, "TrRowNumGroups" );
  gs2DTransform.setTrRowNumGroups( value );
  READ_CODE_DESC( value, 1, "DimensionReductionEnableFlag" );
  gs2DTransform.setDimensionReductionEnableFlag( value != 0 );
  for ( int i = 0; i < gs2DTransform.getTrRowNumGroups(); i++ ) {
    if ( gs2DTransform.getDimensionReductionEnableFlag() ) {
      READ_CODE_DESC( value, 4, "NumOfRetainComponentsMinus1" );
      gs2DTransform.setNumOfRetainComponentsMinus1( i, value );
    }
    READ_CODE_DESC( value, 4, "RowTrSizeMinus1" );
    gs2DTransform.setRowTrSizeMinus1( i, value );
    for ( int j = 0; j < ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); j++ ) {
      READ_CODE_DESC( value, 4, "TrRowGroupIdx" );
      gs2DTransform.setTrRowGroupIdx( i, j, value );
    }
    READ_CODE_DESC( value, 1, "KltFlag" );
    gs2DTransform.setKltFlag( i, value );
    if ( gs2DTransform.getKltFlag( i ) ) {
      for ( int m = 0; m < 3 * ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); m++ ) {
        for ( int n = 0; n < 3 * ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); n++ ) {
          READ_FLOAT_DESC( valueFlt, "KltCoeff" );
          gs2DTransform.setKltCoeff( i, m, n, valueFlt );
        }
      }
      bool isDcGroup = ( gs2DTransform.getRowTrSizeMinus1( i ) == 0 && gs2DTransform.getTrRowGroupIdx( i, 0 ) == 0 );
      for ( int m = 0; m < 3 * ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); m++ ) {
        if ( !isDcGroup ) {
          READ_FLOAT_DESC( valueFlt, "KltStdDev" );
          gs2DTransform.setKltStdDev( i, m, valueFlt );
        } else {
          gs2DTransform.setKltStdDev( i, m, 1.0f );
        }
        READ_FLOAT_DESC( valueFlt, "KltMean" );
        gs2DTransform.setKltMean( i, m, valueFlt );
      }
    } else {
      READ_CODE_DESC( value, 3, "TrRowType" );
      gs2DTransform.setTrRowType( i, value );
    }
  }
  std::vector<uint8_t> colTr;
  colTr.resize( 16, 1 );
  for ( int i = 0; i < gs2DTransform.getTrRowNumGroups(); i++ ) {
    if ( gs2DTransform.getKltFlag( i ) ) {
      for ( int j = 0; j < ( gs2DTransform.getRowTrSizeMinus1( i ) + 1 ); j++ ) {
        colTr[gs2DTransform.getTrRowGroupIdx( i, j )] = 0;
      }
    }
  }
  for ( int i = 0; i < 16; i++ ) {
    if ( colTr[i] == 1 ) {
      READ_CODE_DESC( value, 3, "TrColType" );
      gs2DTransform.setTrColType( i, value );
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.11 Gaussian splat opacity property syntax
void PCCBitstreamReader::gsOpacityProperty( PCCBitstream &bitstream, GsOpacityProperty &gsOpacityProperty ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  int value;
  READ_CODE_DESC( value, 1, "LinearOpacityFlag" );
  gsOpacityProperty.setLinearOpacityFlag( value );
}
// H.8.3.6.3.7  Gaussian splat spherical harmonics prediction syntax
void PCCBitstreamReader::gsPrediction( PCCBitstream &bitstream, GsPrediction &pred ) {
  int   value;
  float valueFlt;
  TRACE_BITSTREAM_IN( "%s", __func__ );
  READ_UVLC_DESC( value, "gsp_sh_num_components" );
  pred.setShNumComponents( value );
  for ( int i = 0; i < pred.getShNumComponents(); i++ ) {
    READ_CODE_DESC( value, 1, "gsp_use_prediction_flag" );
    pred.setUsePredictionFlag( i, value != 0 );
    if ( pred.getUsePredictionFlag( i ) ) {
      READ_CODE_DESC( value, 4, "gsp_num_pred_src_minus1" );
      pred.setNumPredSrcMinus1( i, value );
      for ( int j = 0; j <= pred.getNumPredSrcMinus1( i ); j++ ) {
        READ_CODE_DESC( value, 6, "gsp_pred_src_idx" );
        pred.setPredSrcIdx( i, j, value );
        READ_FLOAT_DESC( valueFlt, "gsp_pred_coeff" );
        pred.setPredCoeff( i, j, valueFlt );
      }
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.6.3.9  Gaussian splat scale transform syntax
void PCCBitstreamReader::gsScaleTransform( PCCBitstream &bitstream, GsScaleTransform &gsst ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  float valueFlt;
  int   valueInt;
  READ_CODE_DESC( valueInt, 1, "SameScalingFlag" );
  gsst.setSameScalingFlag( valueInt != 0 );
  READ_CODE_DESC( valueInt, 1, "SamePowerFlag" );
  gsst.setSamePowerTransformFlag( valueInt != 0 );
  for ( int i = 0; i < 3; i++ ) {
    if ( i == 0 || !gsst.getSameScalingFlag() ) {
      READ_FLOAT_DESC( valueFlt, "ScaleScaling" );
      gsst.setScaleScaling( i, valueFlt );
    } else {
      gsst.setScaleScaling( i, gsst.getScaleScaling( 0 ) );
    }
    if ( i == 0 || !gsst.getSamePowerTransformFlag() ) {
      READ_FLOAT_DESC( valueFlt, "ScalePower" );
      gsst.setScalePower( i, valueFlt );
    } else {
      gsst.setScalePower( i, gsst.getScalePower( 0 ) );
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.8.3.9	Raw patch data unit V-PCC Gaussian Splat extension syntax
void PCCBitstreamReader::rpduVpccGsExtension( PCCBitstream        &bitstream,
                                              AapsVpccGsExtension &ext,
                                              RawPatchDataUnit    &rpdu ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  int value;
  if ( ext.getAdaptiveQuantizationEnableFlag() ) {
    READ_UVLC_DESC( value, "" );  // ue(v)
    rpdu.setShBitdepthDelta( value );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.20.2	SEI payload syntax
// H.20.2.13	Attribute transformation parameters SEI message syntax
void PCCBitstreamReader::attributeTransformationParams( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIAttributeTransformationParams &>( seiAbstract );
  sei.setCancelFlag( bitstream.read( 1 ) != 0U );  // u(1)
  if ( !sei.getCancelFlag() ) {
    sei.setNumAttributeUpdates( bitstream.readUvlc() );  // ue(v)
    sei.allocate();
    for ( size_t j = 0; j < sei.getNumAttributeUpdates(); j++ ) {
      sei.setAttributeIdx( j, bitstream.read( 8 ) );  // u(8)
      size_t index = sei.getAttributeIdx( j );
      sei.setDimensionMinus1( index, bitstream.read( 8 ) );  // u(8)
      sei.allocate( index );
      for ( size_t i = 0; i < sei.getDimensionMinus1( index ) + 1; i++ ) {
        sei.setScaleParamsEnabledFlag( index, i, bitstream.read( 1 ) != 0U );   // u(1)
        sei.setOffsetParamsEnabledFlag( index, i, bitstream.read( 1 ) != 0U );  // u(1)
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

// H.20.2.14 Occupancy synthesis SEI message syntax
void PCCBitstreamReader::occupancySynthesis( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIOccupancySynthesis &>( seiAbstract );
  sei.setPersistenceFlag( bitstream.read( 1 ) );   // u(1)
  sei.setResetFlag( bitstream.read( 1 ) );         // u(1)
  sei.setInstancesUpdated( bitstream.read( 8 ) );  // u(8)
  sei.allocate();
  for ( size_t i = 0; i < sei.getInstancesUpdated(); i++ ) {
    sei.setInstanceIndex( i, bitstream.read( 8 ) );  // u(8)
    size_t k = sei.getInstanceIndex( i );
    sei.setInstanceCancelFlag( k, bitstream.read( 1 ) );  // u(1)
    if ( !sei.getInstanceCancelFlag( k ) ) {
      sei.setMethodType( k, bitstream.readUvlc() );  // ue(v)
      if ( sei.getMethodType( k ) == 1 ) {
        sei.setPbfLog2ThresholdMinus1( k, bitstream.read( 2 ) );  // u(2)
        sei.setPbfPassesCountMinus1( k, bitstream.read( 2 ) );    // u(2)
        sei.setPbfFilterSizeMinus1( k, bitstream.read( 3 ) );     // u(3)
      }
    }
  }
}

// H.20.2.15 Geometry smoothing SEI message syntax
void PCCBitstreamReader::geometrySmoothing( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIGeometrySmoothing &>( seiAbstract );
  sei.setPersistenceFlag( bitstream.read( 1 ) );   // u(1)
  sei.setResetFlag( bitstream.read( 1 ) );         // u(1)
  sei.setInstancesUpdated( bitstream.read( 8 ) );  // u(8)
  sei.allocate();
  for ( size_t i = 0; i < sei.getInstancesUpdated(); i++ ) {
    sei.setInstanceIndex( i, bitstream.read( 8 ) );  // u(8)
    size_t k = sei.getInstanceIndex( i );
    sei.setInstanceCancelFlag( k, bitstream.read( 1 ) );  // u(1)
    if ( !sei.getInstanceCancelFlag( k ) ) {
      sei.setMethodType( k, bitstream.readUvlc() );  // ue(v)
      if ( sei.getMethodType( k ) == 1 ) {
        sei.setFilterEomPointsFlag( k, bitstream.read( 1 ) );  // u(1)
        sei.setGridSizeMinus2( k, bitstream.read( 7 ) );       // u(7)
        sei.setThreshold( k, bitstream.read( 8 ) );            // u(8)
      }
    }
  }
}

// H.20.2.16 Attribute smoothing SEI message syntax
void PCCBitstreamReader::attributeSmoothing( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto &sei = static_cast<SEIAttributeSmoothing &>( seiAbstract );
  sei.setPersistenceFlag( bitstream.read( 1 ) );        // u(1)
  sei.setResetFlag( bitstream.read( 1 ) );              // u(1)
  sei.setNumAttributesUpdated( bitstream.readUvlc() );  // ue(v)
  sei.allocate();
  for ( size_t j = 0; j < sei.getNumAttributesUpdated(); j++ ) {
    sei.setAttributeIdx( j, bitstream.read( 7 ) );  // u(7)
    size_t k = sei.getAttributeIdx( j );
    sei.setAttributeSmoothingCancelFlag( k, bitstream.read( 1 ) );  // u(1)
    sei.setInstancesUpdated( k, bitstream.read( 8 ) );              // u(8)
    for ( size_t i = 0; i < sei.getInstancesUpdated( k ); i++ ) {
      size_t m = bitstream.read( 8 );  // u(8)
      sei.allocate( k + 1, m + 1 );
      sei.setInstanceIndex( k, i, m );
      sei.setInstanceCancelFlag( k, m, bitstream.read( 1 ) );  // u(1)
      if ( sei.getInstanceCancelFlag( k, m ) != 1 ) {
        sei.setMethodType( k, m, bitstream.readUvlc() );  // ue(v)
        if ( sei.getMethodType( k, m ) ) {
          sei.setFilterEomPointsFlag( k, m, bitstream.read( 1 ) );  // u(1)
          sei.setGridSizeMinus2( k, m, bitstream.read( 5 ) );       // u(5)
          sei.setThreshold( k, m, bitstream.read( 8 ) );            // u(8)
          sei.setThresholdVariation( k, m, bitstream.read( 8 ) );   // u(8)
          sei.setThresholdDifference( k, m, bitstream.read( 8 ) );  // u(8)
        }
      }
    }
  }
}

// H.20.2.17	V-PCC registered SEI message syntax
void PCCBitstreamReader::vpccRegisteredSEI( PCCBitstream      &bitstream,
                                            VpccSeiPayloadType vpccPayloadType,
                                            int32_t            payloadSize,
                                            SEI               &seiAbstract ) {
  vpccRegisteredSeiPayload( bitstream, static_cast<VpccSeiPayloadType>( vpccPayloadType ), payloadSize, seiAbstract );
}

// H.20.2.18	V-PCC registered SEI payload syntax
void PCCBitstreamReader::vpccRegisteredSeiPayload( PCCBitstream      &bitstream,
                                                   VpccSeiPayloadType vpccPayloadType,
                                                   int32_t            payLoadSize,
                                                   SEI               &seiAbstract ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  if ( vpccPayloadType == GS_CAMERA_POSE_INFO ) {  // 0
    gsCameraModelInfo( bitstream, seiAbstract );
  } else {
    reservedSeiMessage( bitstream, seiAbstract, payLoadSize );
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}

// H.20.2.19	GS camera pose information SEI message syntax
void PCCBitstreamReader::gsCameraModelInfo( PCCBitstream &bitstream, SEI &seiAbstract ) {
  TRACE_BITSTREAM_IN( "%s", __func__ );
  auto    &sei = static_cast<SEIGsCameraModelInfo &>( seiAbstract );
  uint32_t value;
  float    valueFlt;
  READ_UVLC_DESC( value, "CameraNum" );  // ue(v)
  sei.setCameraNum( value );
  READ_CODE_DESC( value, 1, "SameIntrinsicsFlag" );  // u(1)
  sei.setSameIntrinsicsFlag( value == 1 );
  sei.allocate();
  for ( size_t i = 0; i < sei.getCameraNum(); i++ ) {
    READ_UVLC_DESC( value, "CameraId" );  // ue(v)
    sei.setCameraId( i, value );
    READ_CODE_DESC( value, 1, "CancelFlag" );  // u(1)
    sei.setCancelFlag( i, value );
    if ( !sei.getCancelFlag( i ) ) {
      READ_CODE_DESC( value, 1, "PersistenceFlag" );  // u(1)
      sei.setPersistenceFlag( i, value );
      if ( i == 0 || ( !sei.getSameIntrinsicsFlag() ) ) {
        READ_CODE_DESC( value, 16, "ImageWidth" );  // u(16)
        sei.setImageWidth( i, value );
        READ_CODE_DESC( value, 16, "ImageHeight" );  // u(16)
        sei.setImageHeight( i, value );
        READ_FLOAT_DESC( valueFlt, "HorFocalLength" );  // fl(32)
        sei.setHorFocalLength( i, valueFlt );
        READ_FLOAT_DESC( valueFlt, "VerFocalLength" );  // fl(32)
        sei.setVerFocalLength( i, valueFlt );
        READ_FLOAT_DESC( valueFlt, "HorPrincipalPoint" );  // fl(32)
        sei.setHorPrincipalPoint( i, valueFlt );
        READ_FLOAT_DESC( valueFlt, "VerPrincipalPoint" );  // fl(32)
        sei.setVerPrincipalPoint( i, valueFlt );
      }
      for ( size_t j = 0; j < 4; j++ ) {
        READ_FLOAT_DESC( valueFlt, "Quaternion" );  // fl(32)
        sei.setQuaternion( i, j, valueFlt );
      }
      for ( size_t j = 0; j < 3; j++ ) {
        READ_FLOAT_DESC( valueFlt, "Translation" );  // fl(32)
        sei.setTranslation( i, j, valueFlt );
      }
    }
  }
  TRACE_BITSTREAM_OUT( "%s", __func__ );
}
