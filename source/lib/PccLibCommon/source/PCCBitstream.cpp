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
#include "PCCCommon.h"
#include "PCCBitstream.h"
#include "PCCVideoBitstream.h"

using namespace pcc;

PCCBitstream::PCCBitstream() {
  position_.bytes = 0;
  position_.bits  = 0;
  data_.clear();
#ifdef BITSTREAM_TRACE
  trace_     = false;
  traceFile_ = NULL;
#endif
}

PCCBitstream::~PCCBitstream() { data_.clear(); }


bool PCCBitstream::initialize( const PCCBitstream& bitstream ) {
  position_.bytes = 0;
  position_.bits  = 0;
  data_.resize( bitstream.data_.size(), 0 );
  memcpy( data_.data(), bitstream.data_.data(), bitstream.data_.size() ); 
  return true;
}


bool PCCBitstream::initialize( std::string compressedStreamPath ) {
  std::ifstream fin( compressedStreamPath, std::ios::binary );
  if ( !fin.is_open() ) { return false; }
  fin.seekg( 0, std::ios::end );
  uint64_t bitStreamSize = fin.tellg();
  fin.seekg( 0, std::ios::beg );
  initialize( bitStreamSize );
  fin.read( reinterpret_cast<char*>( data_.data() ), bitStreamSize );
  if ( !fin ) { return false; }
  fin.close();
  return true;
}

bool PCCBitstream::write( std::string compressedStreamPath ) {
  assert( size() <= capacity() );
  write<uint64_t>( size(), totalSizeIterator_ );
  std::ofstream fout( compressedStreamPath, std::ios::binary );
  if ( !fout.is_open() ) { return false; }
  fout.write( reinterpret_cast<const char*>( data_.data() ), size() );
  fout.close();
  return true;
}

// Must be moved in PccEncoderBitstream and PccDeccoderBitstream.
bool PCCBitstream::readHeader( PCCMetadataEnabledFlags& gofLevelMetadataEnabledFlags ) {
  printf("readHeader \n"); fflush(stdout);
#ifdef BITSTREAM_TRACE
  trace("Code: header \n"); 
#endif
  uint64_t totalSize            = 0;
  uint32_t containerMagicNumber = read<uint32_t>();
  if ( containerMagicNumber != PCCTMC2ContainerMagicNumber ) { return false; }
  uint32_t containerVersion = read<uint32_t>();
  if ( containerVersion != PCCTMC2ContainerVersion ) { return false; }
  totalSize = read<uint64_t>();
  assert( data_.size() == totalSize );
  gofLevelMetadataEnabledFlags.getMetadataEnabled() = read<uint8_t>();
  if ( gofLevelMetadataEnabledFlags.getMetadataEnabled() ) {
    gofLevelMetadataEnabledFlags.getScaleEnabled()      = read<uint8_t>();
    gofLevelMetadataEnabledFlags.getOffsetEnabled()     = read<uint8_t>();
    gofLevelMetadataEnabledFlags.getRotationEnabled()   = read<uint8_t>();
    gofLevelMetadataEnabledFlags.getPointSizeEnabled()  = read<uint8_t>();
    gofLevelMetadataEnabledFlags.getPointShapeEnabled() = read<uint8_t>();
#ifdef CE210_MAXDEPTH_EVALUATION
    gofLevelMetadataEnabledFlags.setMaxDepthEnabled( read<uint8_t>() );
#endif
  }
  return true;
}

void PCCBitstream::writeHeader( const PCCMetadataEnabledFlags& gofLevelMetadataEnabledFlags ) {
#ifdef BITSTREAM_TRACE
  trace("Code: header \n"); 
#endif
  write<uint32_t>( PCCTMC2ContainerMagicNumber );
  write<uint32_t>( PCCTMC2ContainerVersion );
  totalSizeIterator_ = getPosition();
  write<uint64_t>( 0 );  // reserved
  write<uint8_t>( gofLevelMetadataEnabledFlags.getMetadataEnabled() );
  if ( gofLevelMetadataEnabledFlags.getMetadataEnabled() ) {
    write<uint8_t>( gofLevelMetadataEnabledFlags.getScaleEnabled() );
    write<uint8_t>( gofLevelMetadataEnabledFlags.getOffsetEnabled() );
    write<uint8_t>( gofLevelMetadataEnabledFlags.getRotationEnabled() );
    write<uint8_t>( gofLevelMetadataEnabledFlags.getPointSizeEnabled() );
    write<uint8_t>( gofLevelMetadataEnabledFlags.getPointShapeEnabled() );
#ifdef CE210_MAXDEPTH_EVALUATION
    write<uint8_t>( gofLevelMetadataEnabledFlags.getMaxDepthEnabled() );
#endif
  }
}
//~Must be moved in PccEncoderBitstream and PccDeccoderBitstream.

void PCCBitstream::writeUvlc( uint32_t code ) {
#ifdef BITSTREAM_TRACE
  uint32_t orgCode = code;
  bool traceStartingValue = trace_;
  trace_ = false; 
#endif
  uint32_t length = 1, temp = ++code;
  while( 1 != temp ){ temp >>= 1; length += 2; }
  write( 0,     length    >> 1 );
  write( code, (length+1) >> 1 );
#ifdef BITSTREAM_TRACE
  trace_ = traceStartingValue;
  trace("Code: Uvlc : %4lu \n",orgCode) ; 
#endif
}

uint32_t PCCBitstream::readUvlc() {
#ifdef BITSTREAM_TRACE
  bool traceStartingValue = trace_;
  trace_ = false; 
#endif
  uint32_t value = 0, code = 0, length = 0;
  code = read( 1 );
  if( 0 == code )  {
    length = 0;
    while( ! ( code & 1 )) {
      code = read( 1 );
      length++;
    }
    value = read( length );
    value += (1 << length)-1;
  }
#ifdef BITSTREAM_TRACE
  trace_ = traceStartingValue;
  trace("Code: Uvlc : %4lu \n", value );
#endif
  return value;
}

void PCCBitstream::writeSvlc( int32_t code ) {
  writeUvlc( ( uint32_t )( code <= 0 ) ? -code << 1 : ( code << 1 ) - 1 );
#ifdef BITSTREAM_TRACE
  trace("Code: Svlc : %4d \n",code) ; 
#endif
}
int32_t PCCBitstream::readSvlc() {
  uint32_t bits = readUvlc();
#ifdef BITSTREAM_TRACE
  if( trace_ ) {
     fprintf(traceFile_?traceFile_:stdout,"Code: Svlc : %4d \n", ( bits & 1 ) ? -( int32_t )( bits >> 1 ) : ( int32_t )( bits >> 1 )) ;
     fflush(stdout); 
  }
#endif
  return ( bits & 1 ) ? -( int32_t )( bits >> 1 ) : ( int32_t )( bits >> 1 );
}
void PCCBitstream::read( PCCVideoBitstream& videoBitstream ) {
#ifdef BITSTREAM_TRACE
  trace("Code: PCCVideoBitstream \n") ; 
#endif
  uint32_t size = read<uint32_t>();
#ifdef BITSTREAM_TRACE
  trace("Code: size = %lu \n",size) ; 
#endif
  videoBitstream.resize( size );
  memcpy( videoBitstream.buffer(), data_.data() + position_.bytes, size );
  videoBitstream.trace();
  position_.bytes += size;
#ifdef BITSTREAM_TRACE
  trace("Code: data \n") ; 
#endif
#ifdef BUG_FIX_BITDEPTH
  // videoBitstream.setWidth( read<uint16_t>() ); 
  // videoBitstream.setHeight( read<uint16_t>() ); 
  videoBitstream.setBitdepth( read<uint8_t>() ); 
#ifdef BITSTREAM_TRACE
  trace("Code: depth = %lu \n",videoBitstream.getBitdepth() ) ; 
#endif
  printf("BITSTREAM READ BITDEPTH = %u \n",videoBitstream.getBitdepth());
#endif
#ifdef BITSTREAM_TRACE
  trace("Code: video : %4lu \n",size) ; 
#endif
  printf("read PCCVideoBitstream done size = %u \n",size); fflush(stdout);
}

void PCCBitstream::write( PCCVideoBitstream& videoBitstream ) {
#ifdef BITSTREAM_TRACE
  trace("Code: PCCVideoBitstream \n") ; 
#endif
  writeBuffer( videoBitstream.buffer(), videoBitstream.size() );
#ifdef BITSTREAM_TRACE
  trace("Code: data \n") ; 
#endif
#ifdef BUG_FIX_BITDEPTH
  printf("BITSTREAM WRITE BITDEPTH = %u \n",videoBitstream.getBitdepth());
  // write<uint16_t>( videoBitstream.getWidth() );
  // write<uint16_t>( videoBitstream.getHeight() );
  write<uint8_t>( videoBitstream.getBitdepth() );
#ifdef BITSTREAM_TRACE
  trace("Code: depth = %lu \n",videoBitstream.getBitdepth() ) ; 
#endif
#endif 
  videoBitstream.trace();
#ifdef BITSTREAM_TRACE
  trace("Code: video : %4lu \n",videoBitstream.size() ) ; 
#endif
}

void PCCBitstream::writeBuffer( const uint8_t* data, const size_t size ) {
  realloc( size );
  write<uint32_t>( size );
#ifdef BITSTREAM_TRACE
  trace("Code: size = %lu \n",size) ; 
#endif
  memcpy( data_.data() + position_.bytes, data, size );
  position_.bytes += size;
}

uint32_t PCCBitstream::read( uint8_t bits ) { 
  uint32_t code = read( bits, position_ ); 
#ifdef BITSTREAM_TRACE
  trace("Code: %5lu : %4lu \n", bits, code ) ; 
#endif
  return code;   
}
void     PCCBitstream::write( uint32_t value, uint8_t bits ) { 
  write( value, bits, position_ ); 
#ifdef BITSTREAM_TRACE
  trace("Code: %5lu : %4lu \n",bits, value ) ; 
#endif
}

void PCCBitstream::align() { align( position_ ); }
void PCCBitstream::align( PCCBistreamPosition& pos ) {
  if ( pos.bits != 0 ) { pos.bits = 0, pos.bytes++; }
}

uint32_t PCCBitstream::read( uint8_t bits, PCCBistreamPosition& pos ) {
  assert( bits >= 32 );
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
  
#ifdef BITSTREAM_TRACE
  trace("      %5lu : %4lu \n",bits, value ) ; 
#endif
  return value;
}

inline void PCCBitstream::write( uint32_t value, uint8_t bits, PCCBistreamPosition& pos ) {
  assert( bits >= 32 );
  if ( pos.bytes + 16 >= data_.size() ) { realloc(); }
  for ( size_t i = 0; i < bits; i++ ) {
    assert( pos.bytes >= data_.size() );
    data_[pos.bytes] |= ( ( value >> ( bits - 1 - i ) ) & 1 ) << ( 7 - pos.bits );
    if ( pos.bits == 7 ) {
      pos.bytes++;
      pos.bits = 0;
    } else {
      pos.bits++;
    }
  }
#ifdef BITSTREAM_TRACE
  trace("      %5lu : %4lu  \n", bits, value ) ; 
#endif
}
