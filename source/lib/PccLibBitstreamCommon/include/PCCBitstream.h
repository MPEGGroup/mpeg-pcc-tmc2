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

#ifndef PCC_BITSTREAM_BITSTREAM_H
#define PCC_BITSTREAM_BITSTREAM_H

#include "PCCBitstreamCommon.h"

namespace pcc {

struct PCCBistreamPosition {
  uint64_t bytes_;
  uint8_t  bits_;
};

class PCCVideoBitstream;

class PCCBitstreamGofStat {
 public:
  PCCBitstreamGofStat() {
    v3cUnitSize_.resize( NUM_V3C_UNIT_TYPE, 0 );
    videoBinSize_.resize( NUM_VIDEO_TYPE, 0 );
  }
  ~PCCBitstreamGofStat() {
    v3cUnitSize_.clear();
    videoBinSize_.clear();
  }
  void                 overwriteV3CUnitSize( V3CUnitType type, size_t size ) { v3cUnitSize_[type] = size; }
  void                 setV3CUnitSize( V3CUnitType type, size_t size ) { v3cUnitSize_[type] += size; }
  void                 setVideoBinSize( PCCVideoType type, size_t size ) { videoBinSize_[type] += size; }
  size_t               getV3CUnitSize( V3CUnitType type ) { return v3cUnitSize_[type]; }
  size_t               getVideoBinSize( PCCVideoType type ) { return videoBinSize_[type]; }
  PCCBitstreamGofStat& operator+=( const PCCBitstreamGofStat& other ) {
    for ( size_t i = 0; i < v3cUnitSize_.size(); i++ ) { v3cUnitSize_[i] += other.v3cUnitSize_[i]; }
    for ( size_t i = 0; i < videoBinSize_.size(); i++ ) { videoBinSize_[i] += other.videoBinSize_[i]; }
    return *this;
  }
  size_t getTotal() {
    return v3cUnitSize_[V3C_VPS] + v3cUnitSize_[V3C_AD] + v3cUnitSize_[V3C_OVD] + v3cUnitSize_[V3C_GVD] +
           v3cUnitSize_[V3C_AVD];
  }

  size_t getTotalGeometry() {
    size_t retVal = 0;
    for ( int i = VIDEO_GEOMETRY; i <= VIDEO_GEOMETRY_RAW; i++ ) retVal += videoBinSize_[i];
    return retVal;
  }
  size_t getTotalAttribute() {
    size_t retVal = 0;
    for ( int i = VIDEO_ATTRIBUTE; i < VIDEO_ATTRIBUTE_RAW + MAX_NUM_ATTR_PARTITIONS; i++ ) retVal += videoBinSize_[i];
    return retVal;
  }
  size_t getTotalMetadata() { return getTotal() - getTotalGeometry() - getTotalAttribute(); }

  void trace() {
    printf( "    V3CUnitSize[ V3C_VPS ]: %9zu B %9zu b\n", v3cUnitSize_[V3C_VPS], v3cUnitSize_[V3C_VPS] * 8 );
    printf( "    V3CUnitSize[ V3C_AD  ]: %9zu B %9zu b\n", v3cUnitSize_[V3C_AD], v3cUnitSize_[V3C_AD] * 8 );
    printf( "    V3CUnitSize[ V3C_OVD ]: %9zu B %9zu b ( Ocm video = %9zu B )\n", v3cUnitSize_[V3C_OVD],
            v3cUnitSize_[V3C_OVD] * 8, videoBinSize_[VIDEO_OCCUPANCY] );
    printf(
        "    V3CUnitSize[ V3C_GVD ]: %9zu B %9zu b ( Geo video = %9zu B + "
        "%9zu B + %9zu B + %9zu B )\n",
        v3cUnitSize_[V3C_GVD], v3cUnitSize_[V3C_GVD] * 8, videoBinSize_[VIDEO_GEOMETRY],
        videoBinSize_[VIDEO_GEOMETRY_D0], videoBinSize_[VIDEO_GEOMETRY_D1], videoBinSize_[VIDEO_GEOMETRY_RAW] );
    printf(
        "    V3CUnitSize[ V3C_AVD ]: %9zu B %9zu b ( Tex video = %9zu B + "
        "(%9zu B + %9zu B) + %9zu B )\n",
        v3cUnitSize_[V3C_AVD], v3cUnitSize_[V3C_AVD] * 8, videoBinSize_[VIDEO_ATTRIBUTE],
        videoBinSize_[VIDEO_ATTRIBUTE_T0], videoBinSize_[VIDEO_ATTRIBUTE_T1], videoBinSize_[VIDEO_ATTRIBUTE_RAW] );
  }

 private:
  std::vector<size_t> v3cUnitSize_;
  std::vector<size_t> videoBinSize_;
};

class PCCBitstreamStat {
 public:
  PCCBitstreamStat() { header_ = 0; }
  ~PCCBitstreamStat() { bitstreamGofStat_.clear(); }
  void newGOF() {
    PCCBitstreamGofStat element;
    bitstreamGofStat_.push_back( element );
  }
  void setHeader( size_t size ) { header_ = size; }
  void incrHeader( size_t size ) { header_ += size; }
  void overwriteV3CUnitSize( V3CUnitType type, size_t size ) {
    bitstreamGofStat_.back().overwriteV3CUnitSize( type, size );
  }
  void   setV3CUnitSize( V3CUnitType type, size_t size ) { bitstreamGofStat_.back().setV3CUnitSize( type, size ); }
  void   setVideoBinSize( PCCVideoType type, size_t size ) { bitstreamGofStat_.back().setVideoBinSize( type, size ); }
  size_t getV3CUnitSize( V3CUnitType type ) { return bitstreamGofStat_.back().getV3CUnitSize( type ); }
  size_t getVideoBinSize( PCCVideoType type ) { return bitstreamGofStat_.back().getVideoBinSize( type ); }
  size_t getTotalGeometry() { return bitstreamGofStat_.back().getTotalGeometry(); }
  size_t getTotalAttribute() { return bitstreamGofStat_.back().getTotalAttribute(); }
  size_t getTotalMetadata() { return bitstreamGofStat_.back().getTotalMetadata(); }

  void trace( bool byGOF = false ) {
    printf( "Bitstream stat: \n" );
    printf( "  Header:                     %9zu B %9zu b\n", header_, header_ * 8 );
    if ( byGOF ) {
      for ( size_t i = 0; i < bitstreamGofStat_.size(); i++ ) {
        printf( "  GOF %2zu: \n", i );
        bitstreamGofStat_[i].trace();
      }
      printf( "  Total: \n" );
    }
    PCCBitstreamGofStat totalBitstreamStat;
    for ( auto& element : bitstreamGofStat_ ) { totalBitstreamStat += element; }
    totalBitstreamStat.trace();
    size_t totalMetadata  = totalBitstreamStat.getTotalMetadata() + header_;
    size_t totalGeometry  = totalBitstreamStat.getTotalGeometry();
    size_t totalAttribute = totalBitstreamStat.getTotalAttribute();
    size_t total          = totalMetadata + totalGeometry + totalAttribute;
    printf( "  TotalMetadata:              %9zu B %9zu b \n", totalMetadata, totalMetadata * 8 );
    printf( "  TotalGeometry:              %9zu B %9zu b \n", totalGeometry, totalGeometry * 8 );
    printf( "  TotalAttribute:               %9zu B %9zu b \n", totalAttribute, totalAttribute * 8 );
    printf( "  Total:                      %9zu B %9zu b \n", total, total * 8 );
  }

 private:
  size_t                           header_;
  std::vector<PCCBitstreamGofStat> bitstreamGofStat_;
};

class PCCBitstream {
 public:
  PCCBitstream();
  ~PCCBitstream();

  bool initialize( std::vector<uint8_t>& data );
  bool initialize( const PCCBitstream& bitstream );
  bool initialize( const std::string& compressedStreamPath );
  void initialize( uint64_t capacity ) { data_.resize( capacity, 0 ); }
  void clear() {
    data_.clear();
    position_.bits_  = 0;
    position_.bytes_ = 0;
  }
  void beginning() {
    position_.bits_  = 0;
    position_.bytes_ = 0;
  }
  bool                  write( const std::string& compressedStreamPath );
  uint8_t*              buffer() { return data_.data(); }
  std::vector<uint8_t>& vector() { return data_; }
  uint64_t&             size() { return position_.bytes_; }
  uint64_t              capacity() { return data_.size(); }
  PCCBistreamPosition   getPosition() { return position_; }
  void                  setPosition( PCCBistreamPosition& val ) { position_ = val; }
  PCCBitstream&         operator+=( const uint64_t size ) {
    position_.bytes_ += size;
    return *this;
  }
  void copyFrom( PCCBitstream& dataBitstream, const uint64_t startByte, const uint64_t bitstreamSize );
  void copyTo( PCCBitstream& dataBitstream, uint64_t startByte, uint64_t outputSize );
  void writeVideoStream( PCCVideoBitstream& videoBitstream );
  void readVideoStream( PCCVideoBitstream& videoBitstream, size_t videoStreamSize );
  bool byteAligned() { return ( position_.bits_ == 0 ); }
  bool moreData() { return position_.bytes_ < data_.size(); }
  void computeMD5();

  inline std::string readString() {
    while ( !byteAligned() ) { read( 1 ); }
    std::string str;
    char        element = read( 8 );
    while ( element != 0x00 ) {
      str.push_back( element );
      element = read( 8 );
    }
    return str;
  }

  inline void writeString( std::string str ) {
    while ( !byteAligned() ) { write( 0, 1 ); }
    for ( auto& element : str ) { write( element, 8 ); }
    write( 0, 8 );
  }

  inline uint32_t peekByteAt( uint64_t peekPos ) { return data_[peekPos]; }
  inline uint32_t read( uint8_t bits, bool bFullStream = false ) {
    uint32_t code = read( bits, position_ );
#ifdef BITSTREAM_TRACE
    if ( bFullStream == true )
      trace( "FullStream: CodU[%2u]: %4zu \n", bits, code );
    else
      trace( "  CodU[%2u]: %4zu \n", bits, code );
#endif
    return code;
  }
  template <typename T>
  void write( T value, uint8_t bits, bool bFullStream = false ) {
    write( static_cast<uint32_t>( value ), bits, position_ );
#ifdef BITSTREAM_TRACE
    if ( bFullStream == true )
      trace( "FullStream: CodU[%2u]: %4zu \n", bits, static_cast<uint32_t>( value ) );
    else
      trace( "  CodU[%2u]: %4zu \n", bits, static_cast<uint32_t>( value ) );
#endif
  }

  inline void writeS( int32_t value, uint8_t bits ) {
    assert( bits > 0 );
    uint32_t code;
    if ( value >= 0 ) {
      code = (uint32_t)value;
    } else {
      code = ( uint32_t )( value & ( ( 1 << ( bits - 1 ) ) - 1 ) );
      code |= ( 1 << ( bits - 1 ) );
    }
    write( code, bits );
#ifdef BITSTREAM_TRACE
    trace( "  CodS[%2u]: %4zu \n", bits, value );
#endif
  }

  inline int32_t readS( uint8_t bits ) {
    assert( bits > 0 );
    uint32_t code = read( bits );
    int32_t  value;
    uint32_t midPoint = ( 1 << ( bits - 1 ) );
    if ( code < midPoint ) {
      value = (int32_t)code;
    } else {
      value = (int32_t)code | ~( midPoint - 1 );
    }
#ifdef BITSTREAM_TRACE
    trace( "  CodS[%2u]: %4zu \n", bits, value );
#endif
    return value;
  }

  template <typename T>
  inline void writeUvlc( T value ) {
    uint32_t code = static_cast<uint32_t>( value );
#ifdef BITSTREAM_TRACE
    uint32_t orgCode            = code;
    bool     traceStartingValue = trace_;
    trace_                      = false;
#endif
    uint32_t length = 1, temp = ++code;
    while ( 1 != temp ) {
      temp >>= 1;
      length += 2;
    }
    write( 0, length >> 1 );
    write( code, ( length + 1 ) >> 1 );
#ifdef BITSTREAM_TRACE
    trace_ = traceStartingValue;
    trace( "  CodeUvlc: %4zu \n", orgCode );
#endif
  }

  inline uint32_t readUvlc() {
#ifdef BITSTREAM_TRACE
    bool traceStartingValue = trace_;
    trace_                  = false;
#endif
    uint32_t value = 0, code = 0, length = 0;
    code = read( 1 );
    if ( 0 == code ) {
      length = 0;
      while ( !( code & 1 ) ) {
        code = read( 1 );
        length++;
      }
      value = read( length );
      value += ( 1 << length ) - 1;
    }
#ifdef BITSTREAM_TRACE
    trace_ = traceStartingValue;
    trace( "  CodeUvlc: %4zu \n", value );
#endif
    return value;
  }

  template <typename T>
  inline void writeSvlc( T value ) {
    int32_t code = static_cast<int32_t>( value );
    writeUvlc( ( uint32_t )( code <= 0 ? -code << 1 : ( code << 1 ) - 1 ) );
#ifdef BITSTREAM_TRACE
    trace( "  CodeSvlc: %4d \n", code );
#endif
  }

  inline int32_t readSvlc() {
    uint32_t bits = readUvlc();
#ifdef BITSTREAM_TRACE
    trace( "  CodeSvlc: %4d \n", ( bits & 1 ) ? ( int32_t )( bits >> 1 ) + 1 : -( int32_t )( bits >> 1 ) );
#endif
    return ( bits & 1 ) ? ( int32_t )( bits >> 1 ) + 1 : -( int32_t )( bits >> 1 );
  }

  inline void writeFloat( float value ) {
    uint32_t code = 0;
    memcpy( &code, &value, sizeof( float ) );
    write( code, 32 );
  }

  inline float readFloat() {
    uint32_t code  = read( 32 );
    float    value = 0;
    memcpy( &value, &code, sizeof( float ) );
    return value;
  }

#ifdef BITSTREAM_TRACE
  template <typename... Args>
  void trace( const char* pFormat, Args... eArgs ) {
    if ( trace_ ) {
      logger_->traceStream( "[%6llu - %2u]: ", position_.bytes_, position_.bits_ );
      logger_->traceStream( pFormat, eArgs... );
    }
  }
  void setTrace( bool trace ) { trace_ = trace; }
  void setLogger( PCCLogger& logger ) { logger_ = &logger; }
#endif
 private:
  inline void realloc( const size_t size = 4096 ) { data_.resize( data_.size() + ( ( ( size / 4096 ) + 1 ) * 4096 ) ); }
  inline uint32_t read( uint8_t bits, PCCBistreamPosition& pos ) {
    uint32_t value = 0;
    for ( size_t i = 0; i < bits; i++ ) {
      value |= ( ( data_[pos.bytes_] >> ( 7 - pos.bits_ ) ) & 1 ) << ( bits - 1 - i );
      if ( pos.bits_ == 7 ) {
        pos.bytes_++;
        pos.bits_ = 0;
      } else {
        pos.bits_++;
      }
    }
    return value;
  }

  inline void write( uint32_t value, uint8_t bits, PCCBistreamPosition& pos ) {
    if ( pos.bytes_ + bits + 16 >= data_.size() ) { realloc(); }
    for ( size_t i = 0; i < bits; i++ ) {
      data_[pos.bytes_] |= ( ( value >> ( bits - 1 - i ) ) & 1 ) << ( 7 - pos.bits_ );
      if ( pos.bits_ == 7 ) {
        pos.bytes_++;
        pos.bits_ = 0;
      } else {
        pos.bits_++;
      }
    }
  }

  std::vector<uint8_t> data_;
  PCCBistreamPosition  position_;

#ifdef BITSTREAM_TRACE
  bool       trace_;
  PCCLogger* logger_ = nullptr;
#endif
};

}  // namespace pcc

#endif /* PCC_BITSTREAM_BITSTREAM_H */
