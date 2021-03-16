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

#ifndef PCC_BITSTREAM_VIDEOBITSTREAM_H
#define PCC_BITSTREAM_VIDEOBITSTREAM_H

#include "PCCBitstreamCommon.h"
namespace pcc {

class PCCVideoBitstream {
 public:
  PCCVideoBitstream( PCCVideoType type ) : type_( type ) { data_.clear(); }
  ~PCCVideoBitstream() { data_.clear(); }

  PCCVideoBitstream&    operator=( const PCCVideoBitstream& ) = default;
  void                  resize( size_t size ) { data_.resize( size ); }
  std::vector<uint8_t>& vector() { return data_; }
  uint8_t*              buffer() { return data_.data(); }
  size_t                size() { return data_.size(); }
  PCCVideoType          type() { return type_; }

  void trace() { std::cout << toString( type_ ) << " ->" << size() << " B " << std::endl; }

  std::string getExtension() {
    size_t typeIndex = (size_t)type_;
    if ( typeIndex == (size_t)VIDEO_OCCUPANCY ) {
      return std::string( "occupancy" );
    } else if ( typeIndex == (size_t)VIDEO_GEOMETRY ) {
      return std::string( "geometry" );
    } else if ( typeIndex >= (size_t)VIDEO_GEOMETRY_D0 && typeIndex <= (size_t)VIDEO_GEOMETRY_D15 ) {
      return std::string( "geometryD" ) + std::to_string( typeIndex - (size_t)VIDEO_GEOMETRY_D0 );
    } else if ( typeIndex == (size_t)VIDEO_GEOMETRY_RAW ) {
      return std::string( "geomteryRaw" );
    } else if ( typeIndex == (size_t)VIDEO_TEXTURE ) {
      return std::string( "texture" );
    } else if ( typeIndex >= (size_t)VIDEO_TEXTURE_T0 && typeIndex <= (size_t)VIDEO_TEXTURE_T15 ) {
      return std::string( "textureT" ) + std::to_string( typeIndex - ( size_t )( VIDEO_TEXTURE_T0 ) );
    } else if ( typeIndex == (size_t)VIDEO_TEXTURE_RAW ) {
      return std::string( "textureRaw" );
    } else {
      return std::string( "unknown" );
    }
  }
  bool write( const std::string& filename ) {
    std::ofstream file( filename, std::ios::binary );
    if ( !file.good() ) { return false; }
    file.write( reinterpret_cast<char*>( data_.data() ), data_.size() );
    file.close();
    return true;
  }
  bool read( const std::string& filename ) {
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

  bool write_JM( const std::string& filename ) {
    std::ofstream file( filename, std::ios::binary );
    if ( !file.good() ) { return false; }
    file.write( reinterpret_cast<char*>( data_.data() ), data_.size() );
    file.close();
    return true;
  }
  bool read_JM( const std::string& filename ) {
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

  void byteStreamToSampleStream( size_t precision = 4, bool emulationPreventionBytes = false ) {
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

  void sampleStreamToByteStream( size_t precision                = 4,
                                 bool   emulationPreventionBytes = false,
                                 bool   changeStartCodeSize      = true ) {
    size_t               sizeStartCode = 4, startIndex = 0, endIndex = 0;
    std::vector<uint8_t> data;
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
        // the first NALU of a frame, NAL_UNIT_VPS, NAL_UNIT_SPS, NAL_UNIT_PPS : size=4
        int  naluType   = ( ( data_[startIndex + precision] ) & 126 ) >> 1;
        bool isNewFrame = false;  // isNewFrameNalu(POC calculation);
        if ( isNewFrame == true || ( changeStartCodeSize && ( naluType == 32 || naluType == 33 || naluType == 34 ) ) ) {
          sizeStartCode = 4;
        } else {
          sizeStartCode = 3;
        }
      }
    } while ( endIndex < data_.size() );
    data_.swap( data );
  }

 private:
  size_t getEndOfNaluPosition( size_t startIndex ) {
    const size_t size = data_.size();
    if ( size < startIndex + 4 ) { return size; }
    for ( size_t i = startIndex; i < size - 4 ; i++ ) {
      if ( ( data_[i + 0] == 0x00 ) && 
           ( data_[i + 1] == 0x00 ) &&
           ( ( data_[i + 2] == 0x01 ) || ( ( data_[i + 2] == 0x00 ) && ( data_[i + 3] == 0x01 ) ) ) ) {
        return i;
      }
    }
    return size;
  }
  std::vector<uint8_t> data_;
  PCCVideoType         type_;
};

}  // namespace pcc

#endif /* PCC_BITSTREAM_VIDEOBITSTREAM_H */
