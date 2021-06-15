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
#include "PCCVideoBitstream.h"

using namespace pcc;

std::string PCCVideoBitstream::getExtension() {
  size_t typeIndex = (size_t)type_;
  if ( typeIndex == (size_t)VIDEO_OCCUPANCY ) {
    return std::string( "occupancy" );
  } else if ( typeIndex == (size_t)VIDEO_GEOMETRY ) {
    return std::string( "geometry" );
  } else if ( typeIndex >= (size_t)VIDEO_GEOMETRY_D0 && typeIndex <= (size_t)VIDEO_GEOMETRY_D15 ) {
    return std::string( "geometryD" ) + std::to_string( typeIndex - (size_t)VIDEO_GEOMETRY_D0 );
  } else if ( typeIndex == (size_t)VIDEO_GEOMETRY_RAW ) {
    return std::string( "geomteryRaw" );
  } else if ( typeIndex == (size_t)VIDEO_ATTRIBUTE ) {
    return std::string( "attribute" );
  } else if ( typeIndex >= (size_t)VIDEO_ATTRIBUTE_T0 && typeIndex <= (size_t)VIDEO_ATTRIBUTE_T15 ) {
    return std::string( "attributeT" ) + std::to_string( typeIndex - ( size_t )( VIDEO_ATTRIBUTE_T0 ) );
  } else if ( typeIndex == (size_t)VIDEO_ATTRIBUTE_RAW ) {
    return std::string( "attributeRaw" );
  } else {
    return std::string( "unknown" );
  }
}

bool PCCVideoBitstream::write( const std::string& filename ) {
  std::ofstream file( filename, std::ios::binary );
  if ( !file.good() ) { return false; }
  file.write( reinterpret_cast<char*>( data_.data() ), data_.size() );
  file.close();
  return true;
}

bool PCCVideoBitstream::read( const std::string& filename ) {
  std::ifstream file( filename, std::ios::binary | std::ios::ate );
  if ( !file.good() ) { return false; }
  const uint64_t fileSize = file.tellg();
  resize( (size_t)fileSize );
  file.clear();
  file.seekg( 0 );
  file.read( reinterpret_cast<char*>( data_.data() ), data_.size() );
  file.close();
  return true;
}

#if defined( WIN32 )
bool PCCVideoBitstream::_read( const std::string& filename ) { return read( filename ); }
bool PCCVideoBitstream::_write( const std::string& filename ) { return write( filename ); }
#endif

void PCCVideoBitstream::byteStreamToSampleStream( size_t precision, bool emulationPreventionBytes ) {
  size_t               startIndex = 0, endIndex = 0;
  std::vector<uint8_t> data;
  do {
    size_t sizeStartCode = data_[startIndex + 2] == 0x00 ? 4 : 3;
    endIndex             = getEndOfNaluPosition( startIndex + sizeStartCode );
    size_t headerIndex   = data.size();
    for ( size_t i = 0; i < precision; i++ ) { data.push_back( 0 ); }  // reserve nalu size
    if ( emulationPreventionBytes ) {
      for ( size_t i = startIndex + sizeStartCode, zeroCount = 0; i < endIndex; i++ ) {
        if ( ( zeroCount == 3 ) && ( data_[i] <= 3 ) ) {
          zeroCount = 0;
        } else {
          zeroCount = ( data_[i] == 0 ) ? zeroCount + 1 : 0;
          data.push_back( data_[i] );
        }
      }
    } else {
      for ( size_t i = startIndex + sizeStartCode; i < endIndex; i++ ) { data.push_back( data_[i] ); }
    }
    size_t naluSize = data.size() - ( headerIndex + precision );
    for ( size_t i = 0; i < precision; i++ ) {
      data[headerIndex + i] = ( naluSize >> ( 8 * ( precision - ( i + 1 ) ) ) ) & 0xff;
    }
    startIndex = endIndex;
  } while ( endIndex < data_.size() );
  data_.swap( data );
}

void PCCVideoBitstream::sampleStreamToByteStream( bool   isAvc,
                                                  bool   isVvc,
                                                  size_t precision,
                                                  bool   emulationPreventionBytes,
                                                  bool   changeStartCodeSize ) {
  size_t               sizeStartCode = 4, startIndex = 0, endIndex = 0;
  std::vector<uint8_t> data;
  bool                 newFrame = true;
  printf( "isAvc = %d isVvc = %d \n", isAvc, isVvc );
  do {
    int32_t naluSize = 0;
    for ( size_t i = 0; i < precision; i++ ) { naluSize = ( naluSize << 8 ) + data_[startIndex + i]; }
    endIndex = startIndex + precision + naluSize;
    for ( size_t i = 0; i < sizeStartCode - 1; i++ ) { data.push_back( 0 ); }
    data.push_back( 1 );
    if ( emulationPreventionBytes ) {
      for ( size_t i = startIndex + precision, zeroCount = 0; i < endIndex; i++ ) {
        if ( zeroCount == 3 && data_[i] <= 0x03 ) {
          data.push_back( 0x03 );
          zeroCount = 0;
        }
        zeroCount = ( data_[i] == 0x00 ) ? zeroCount + 1 : 0;
        data.push_back( data_[i] );
      }
    } else {
      for ( size_t i = startIndex + precision; i < endIndex; i++ ) { data.push_back( data_[i] ); }
    }
    startIndex = endIndex;
    if ( ( startIndex + precision ) < data_.size() ) {
      int  naluType         = 0;
      bool useLongStartCode = false;
      newFrame              = false;
      // HEVC
      //   Bool forbidden_zero_bit = bs.read(1);           // forbidden_zero_bit
      //   nalu.m_nalUnitType = (NalUnitType) bs.read(6);  // nal_unit_type
      //   nalu.m_nuhLayerId = bs.read(6);                 // nuh_layer_id
      //   nalu.m_temporalId = bs.read(3) - 1;             // nuh_temporal_id_plus1
      // VVC
      //   nalu.m_forbiddenZeroBit   = bs.read(1);                 // forbidden zero bit
      //   nalu.m_nuhReservedZeroBit = bs.read(1);                 // nuh_reserved_zero_bit
      //   nalu.m_nuhLayerId         = bs.read(6);                 // nuh_layer_id
      //   nalu.m_nalUnitType        = (NalUnitType) bs.read(5);   // nal_unit_type
      //   nalu.m_temporalId         = bs.read(3) - 1;             // nuh_temporal_id_plus1
      if ( isAvc ) {
        useLongStartCode = true;
      } else if ( isVvc ) {
        naluType         = ( ( ( data_[startIndex + precision + 1] ) & 248 ) >> 3 );
        useLongStartCode = newFrame || ( naluType >= 12 && naluType < 20 );
        if ( naluType < 12 ) { newFrame = true; }
      } else {
        naluType         = ( ( ( data_[startIndex + precision] ) & 126 ) >> 1 );
        useLongStartCode = newFrame || ( naluType >= 32 && naluType < 41 );
        if ( naluType < 12 ) { newFrame = true; }
      }
      sizeStartCode = useLongStartCode ? 4 : 3;
    }
  } while ( endIndex < data_.size() );
  data_.swap( data );
}

size_t PCCVideoBitstream::getEndOfNaluPosition( size_t startIndex ) {
  const size_t size = data_.size();
  if ( size < startIndex + 4 ) { return size; }
  for ( size_t i = startIndex; i < size - 4; i++ ) {
    if ( ( data_[i + 0] == 0x00 ) && ( data_[i + 1] == 0x00 ) &&
         ( ( data_[i + 2] == 0x01 ) || ( ( data_[i + 2] == 0x00 ) && ( data_[i + 3] == 0x01 ) ) ) ) {
      return i;
    }
  }
  return size;
}
