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
  write( (int32_t) size(), 64, totalSizeIterator_ );
  std::ofstream fout( compressedStreamPath, std::ios::binary );
  if ( !fout.is_open() ) { return false; }
  fout.write( reinterpret_cast<const char*>( data_.data() ), size() );
  fout.close();
  return true;
}

bool PCCBitstream::readHeader() {
#ifdef BITSTREAM_TRACE
  trace( "Code: header \n" );
#endif
  uint64_t totalSize            = 0;
  uint32_t containerMagicNumber = read( 32 );
  if ( containerMagicNumber != PCCTMC2ContainerMagicNumber ) { return false; }
  uint32_t containerVersion = read( 32 );
  if ( containerVersion != PCCTMC2ContainerVersion ) { return false; }
  totalSize = read( 64 );
  bitstreamStat_.setHeader( size() );
  return true;
}

void PCCBitstream::writeHeader() {
#ifdef BITSTREAM_TRACE
  trace( "Code: header \n" );
#endif
  write( PCCTMC2ContainerMagicNumber, 32 );
  write( PCCTMC2ContainerVersion, 32 );
  totalSizeIterator_ = getPosition();
  write( 0, 64 );  // reserved
  bitstreamStat_.setHeader( size() );
}

void PCCBitstream::read( PCCVideoBitstream& videoBitstream ) {
#ifdef BITSTREAM_TRACE
  trace( "Code: PCCVideoBitstream \n" );
#endif
  uint32_t size = read( 32 );
#ifdef BITSTREAM_TRACE
  trace( "Code: size = %lu \n", size );
#endif
  videoBitstream.resize( size );
  memcpy( videoBitstream.buffer(), data_.data() + position_.bytes, size );
  videoBitstream.trace();
  position_.bytes += size;
#ifdef BITSTREAM_TRACE
  trace( "Code: video : %4lu \n", size );
#endif
  bitstreamStat_.setVideoBinSize( videoBitstream.type(), videoBitstream.size() );
}

void PCCBitstream::write( PCCVideoBitstream& videoBitstream ) {
#ifdef BITSTREAM_TRACE
  trace( "Code: PCCVideoBitstream \n" );
#endif
  writeBuffer( videoBitstream.buffer(), videoBitstream.size() );
  videoBitstream.trace();
  bitstreamStat_.setVideoBinSize( videoBitstream.type(), videoBitstream.size() );
#ifdef BITSTREAM_TRACE
  trace( "Code: video : %4lu \n", videoBitstream.size() );
#endif
}

void PCCBitstream::writeBuffer( const uint8_t* data, const size_t size ) {
  realloc( size );
  write( (int32_t) size, 32 );
#ifdef BITSTREAM_TRACE
  trace( "Code: size = %lu \n", size );
#endif
  memcpy( data_.data() + position_.bytes, data, size );
  position_.bytes += size;
}
