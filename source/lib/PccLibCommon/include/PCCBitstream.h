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

#ifndef PCCBitstream_h
#define PCCBitstream_h

#include "PCCCommon.h"

namespace pcc {

struct PCCBistreamPosition {
  uint64_t bytes;
  uint8_t  bits;
};
class PCCVideoBitstream;

#ifdef BITSTREAM_TRACE
#define TRACE_BITSTREAM( fmt, ... ) bitstream.trace( fmt, ##__VA_ARGS__ );
#else
#define TRACE_BITSTREAM( fmt, ... )
#endif

class PCCBitstreamGofStat {
 public:
  PCCBitstreamGofStat() { 
    vpccUnitSize_.resize( 5, 0 );
    videoBinSize_.resize( NUM_VIDEO_TYPE, 0 );
  }
  ~PCCBitstreamGofStat() {
    vpccUnitSize_.clear();
    videoBinSize_.clear();
  }
  void                 setVpccUnitSize( VPCCUnitType type, size_t size ) { vpccUnitSize_[type] = size; }
  void                 setVideoBinSize( PCCVideoType type, size_t size ) { videoBinSize_[type] = size; }
  size_t               getVpccUnitSize( VPCCUnitType type ) { return vpccUnitSize_[type]; }
  size_t               getVideoBinSize( PCCVideoType type ) { return videoBinSize_[type]; }
  PCCBitstreamGofStat& operator+=( const PCCBitstreamGofStat& other ) {
    for ( size_t i = 0; i < vpccUnitSize_.size(); i++ ) { vpccUnitSize_[i] += other.vpccUnitSize_[i]; }
    for ( size_t i = 0; i < videoBinSize_.size(); i++ ) { videoBinSize_[i] += other.videoBinSize_[i]; }
    return *this;
  }
  size_t getTotal() {
    return vpccUnitSize_[VPCC_VPS] + vpccUnitSize_[VPCC_AD] + vpccUnitSize_[VPCC_OVD] + vpccUnitSize_[VPCC_GVD] +
           vpccUnitSize_[VPCC_AVD];
  }

  size_t getTotalGeometry() {
    return videoBinSize_[VIDEO_GEOMETRY] + videoBinSize_[VIDEO_GEOMETRY_D0] + videoBinSize_[VIDEO_GEOMETRY_D1] +
           videoBinSize_[VIDEO_GEOMETRY_RAW];
  }
  size_t getTotalTexture() {
    return videoBinSize_[VIDEO_TEXTURE] + videoBinSize_[VIDEO_TEXTURE_T0] + videoBinSize_[VIDEO_TEXTURE_T1] +
           videoBinSize_[VIDEO_TEXTURE_RAW];
  }
  size_t getTotalMetadata() { return getTotal() - getTotalGeometry() - getTotalTexture(); }

  void trace() {
    printf( "    vpccUnitSize[ VPCC_VPS ]: %9lu B %9lu b\n", vpccUnitSize_[VPCC_VPS], vpccUnitSize_[VPCC_VPS] * 8 );
    printf( "    vpccUnitSize[ VPCC_AD  ]: %9lu B %9lu b\n", vpccUnitSize_[VPCC_AD], vpccUnitSize_[VPCC_AD] * 8 );
    printf( "    vpccUnitSize[ VPCC_OVD ]: %9lu B %9lu b ( Ocm video = %9lu B )\n", vpccUnitSize_[VPCC_OVD],
            vpccUnitSize_[VPCC_OVD] * 8, videoBinSize_[VIDEO_OCCUPANCY] );
    printf( "    vpccUnitSize[ VPCC_GVD ]: %9lu B %9lu b ( Geo video = %9lu B + %9lu B + %9lu B + %9lu B )\n",
            vpccUnitSize_[VPCC_GVD], vpccUnitSize_[VPCC_GVD] * 8, videoBinSize_[VIDEO_GEOMETRY],
            videoBinSize_[VIDEO_GEOMETRY_D0], videoBinSize_[VIDEO_GEOMETRY_D1], videoBinSize_[VIDEO_GEOMETRY_RAW] );
    printf( "    vpccUnitSize[ VPCC_AVD ]: %9lu B %9lu b ( Tex video = %9lu B + (%9lu B + %9lu B) + %9lu B )\n",
            vpccUnitSize_[VPCC_AVD], vpccUnitSize_[VPCC_AVD] * 8, videoBinSize_[VIDEO_TEXTURE],
            videoBinSize_[VIDEO_TEXTURE_T0], videoBinSize_[VIDEO_TEXTURE_T1], videoBinSize_[VIDEO_TEXTURE_RAW] );
  }
 private:
  std::vector<size_t> vpccUnitSize_;
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
  void   setHeader( size_t size ) { header_ = size; }
  void   incrHeader( size_t size ) { header_ += size; }
  void   setVpccUnitSize( VPCCUnitType type, size_t size ) { bitstreamGofStat_.back().setVpccUnitSize( type, size ); }
  void   setVideoBinSize( PCCVideoType type, size_t size ) { bitstreamGofStat_.back().setVideoBinSize( type, size ); }
  size_t getVpccUnitSize( VPCCUnitType type ) { return bitstreamGofStat_.back().getVpccUnitSize( type ); }
  size_t getVideoBinSize( PCCVideoType type ) { return bitstreamGofStat_.back().getVideoBinSize( type ); }

  size_t getTotalGeometry() { return bitstreamGofStat_.back().getTotalGeometry(); }
  size_t getTotalTexture() { return bitstreamGofStat_.back().getTotalTexture(); }
  size_t getTotalMetadata() { return bitstreamGofStat_.back().getTotalMetadata(); }

  void trace( bool byGOF = false ) {
    printf( "Bitstream stat: \n" );
    printf( "  Header:                     %9lu B %9lu b\n", header_, header_ * 8 );
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
    size_t totalMetadata = totalBitstreamStat.getTotalMetadata() + header_;
    size_t totalGeometry = totalBitstreamStat.getTotalGeometry();
    size_t totalTexture  = totalBitstreamStat.getTotalTexture();
    size_t total         = totalMetadata + totalGeometry + totalTexture;
    printf( "  TotalMetadata:              %9lu B %9lu b \n", totalMetadata, totalMetadata * 8 );
    printf( "  TotalGeometry:              %9lu B %9lu b \n", totalGeometry, totalGeometry * 8 );
    printf( "  TotalTexture:               %9lu B %9lu b \n", totalTexture, totalTexture * 8 );
    printf( "  Total:                      %9lu B %9lu b \n", total, total * 8 );
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
  bool initialize( std::string compressedStreamPath );
  void initialize( uint64_t capacity ) { data_.resize( capacity, 0 ); }
  void clear() {
    data_.clear();
    position_.bits  = 0;
    position_.bytes = 0;
  }
  void beginning() {
    position_.bits  = 0;
    position_.bytes = 0;
  }
  bool                write( std::string compressedStreamPath );
  uint8_t*            buffer() { return data_.data(); }
  uint64_t&           size() { return position_.bytes; }
  uint64_t            capacity() { return data_.size(); }
  PCCBistreamPosition getPosition() { return position_; }
  void                setPosition( PCCBistreamPosition& val ) { position_ = val; }
  PCCBitstream&       operator+=( const uint64_t size ) {
    position_.bytes += size;
    return *this;
  }
  void writeHeader();
  void writeBuffer( const uint8_t* data, const size_t size );
  void copyFrom( PCCBitstream& dataBitstream, const uint64_t startByte, const uint64_t bitstreamSize );
  void copyTo( PCCBitstream& dataBitstream, uint64_t startByte, uint64_t outputSize );
  void write( PCCVideoBitstream& videoBitstream );
  bool readHeader();
  void read( PCCVideoBitstream& videoBitstream );
  bool byteAligned() { return ( position_.bits == 0 ); }
  bool moreData() { return position_.bytes < data_.size(); }

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
    while ( !byteAligned() ) { write( 0 ); }
    for ( auto& element : str ) { write( element, 8 ); }
  }

  inline uint32_t read( uint8_t bits, bool bFullStream = false ) {
    uint32_t code = read( bits, position_ );
#ifdef BITSTREAM_TRACE
    if ( bFullStream == true )
      trace( "FullStream: CodU[%2u]: %4lu \n", bits, code );
    else
      trace( "  CodU[%2u]: %4lu \n", bits, code );
#endif
    return code;
  }
  void write( uint32_t value, uint8_t bits, bool bFullStream = false ) {
    write( value, bits, position_ );
#ifdef BITSTREAM_TRACE
    if ( bFullStream == true )
      trace( "FullStream: CodU[%2u]: %4lu \n", bits, value );
    else
      trace( "  CodU[%2u]: %4lu \n", bits, value );
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
    trace( "  CodS[%2u]: %4lu \n", bits, value );
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
    trace( "  CodS[%2u]: %4lu \n", bits, value );
#endif
    return value;
  }

  inline void writeUvlc( uint32_t code ) {
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
    trace( "  CodeUvlc: %4lu \n", orgCode );
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
    trace( "  CodeUvlc: %4lu \n", value );
#endif
    return value;
  }

  inline void writeSvlc( int32_t code ) {
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

#ifdef BITSTREAM_TRACE
  template <typename... Args>
  void trace( const char* pFormat, Args... eArgs ) {
    if ( trace_ ) {
      FILE* output = traceFile_ ? traceFile_ : stdout;
      fprintf( output, "[%6lu - %2u]: ", position_.bytes, position_.bits );
      fflush( output );
      fprintf( output, pFormat, eArgs... );
      fflush( output );
    }
  }
  void  setTrace( bool trace ) { trace_ = trace; }
  void  setTraceFile( FILE* traceFile ) { traceFile_ = traceFile; }
  FILE* getTraceFile() { return traceFile_; }

  bool getTrace() { return trace_; }
  bool openTrace( std::string file ) {
    if ( traceFile_ ) {
      fclose( traceFile_ );
      traceFile_ = NULL;
    }
    if ( ( traceFile_ = fopen( file.c_str(), "w+" ) ) == NULL ) { return false; }
    return true;
  }
  void closeTrace() {
    if ( traceFile_ ) {
      fclose( traceFile_ );
      traceFile_ = NULL;
    }
  }
#endif
 private:
  inline void realloc( const size_t size = 4096 ) { data_.resize( data_.size() + ( ( ( size / 4096 ) + 1 ) * 4096 ) ); }
  inline uint32_t read( uint8_t bits, PCCBistreamPosition& pos ) {
    uint32_t value = 0;
    for ( size_t i = 0; i < bits; i++ ) {
      value |= ( ( data_[pos.bytes] >> ( 7 - pos.bits ) ) & 1 ) << ( bits - 1 - i );
      if ( pos.bits == 7 ) {
        pos.bytes++;
        pos.bits = 0;
      } else {
        pos.bits++;
      }
    }
    return value;
  }

  inline void write( uint32_t value, uint8_t bits, PCCBistreamPosition& pos ) {
    if ( pos.bytes + bits + 16 >= data_.size() ) { realloc(); }
    for ( size_t i = 0; i < bits; i++ ) {
      data_[pos.bytes] |= ( ( value >> ( bits - 1 - i ) ) & 1 ) << ( 7 - pos.bits );
      if ( pos.bits == 7 ) {
        pos.bytes++;
        pos.bits = 0;
      } else {
        pos.bits++;
      }
    }
  }

  std::vector<uint8_t> data_;
  PCCBistreamPosition  position_;
  PCCBistreamPosition  totalSizeIterator_;

#ifdef BITSTREAM_TRACE
  bool  trace_;
  FILE* traceFile_;
#endif
};

}  // namespace pcc

#endif /* PCCBitstream_h */
