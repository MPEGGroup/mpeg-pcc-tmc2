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

PCCBitstream::PCCBitstream(){
  position_.bytes = 0;
  position_.bits = 0;
  data_.clear();
}
PCCBitstream::~PCCBitstream(){
  data_.clear();
}

bool PCCBitstream::initialize( std::string compressedStreamPath ) {
  std::ifstream fin( compressedStreamPath, std::ios::binary );
  if( !fin.is_open() ) {
    return false;
  }
  fin.seekg( 0, std::ios::end );
  uint64_t bitStreamSize = fin.tellg();
  fin.seekg( 0, std::ios::beg );
  initialize( bitStreamSize );
  fin.read( reinterpret_cast<char *>( data_.data() ), bitStreamSize );
  if( !fin ) {
    return false;
  }
  fin.close();
  return true;
}

bool PCCBitstream::write( std::string compressedStreamPath ) {
  assert(size() <= capacity());
  write<uint64_t>( size(), totalSizeIterator_ );
  std::ofstream fout( compressedStreamPath, std::ios::binary );
  if (!fout.is_open()) {
    return false;
  }
  fout.write(reinterpret_cast<const char *>(data_.data()), size());
  fout.close();
  return true;
}

bool PCCBitstream::readHeader(PCCMetadataEnabledFlags &gofLevelMetadataEnabledFlags) {
  uint32_t containerMagicNumber = 0;
  uint32_t containerVersion = 0;
  uint64_t totalSize = 0;
  read<uint32_t>( containerMagicNumber );
  if (containerMagicNumber != PCCTMC2ContainerMagicNumber) {
    return false;
  }
  read<uint32_t>( containerVersion );
  if (containerVersion != PCCTMC2ContainerVersion) {
    return false;
  }
  read<uint64_t>( totalSize );
  assert( data_.size() == totalSize );
  uint8_t tmp;
  read<uint8_t>(tmp);
  gofLevelMetadataEnabledFlags.getMetadataEnabled() = tmp;
  if (gofLevelMetadataEnabledFlags.getMetadataEnabled()) {
    read<uint8_t>(tmp);
    gofLevelMetadataEnabledFlags.getScaleEnabled() = tmp;
    read<uint8_t>(tmp);
    gofLevelMetadataEnabledFlags.getOffsetEnabled() = tmp;
    read<uint8_t>(tmp);
    gofLevelMetadataEnabledFlags.getRotationEnabled() = tmp;
    read<uint8_t>(tmp);
    gofLevelMetadataEnabledFlags.getPointSizeEnabled() = tmp;
    read<uint8_t>(tmp);
    gofLevelMetadataEnabledFlags.getPointShapeEnabled() = tmp;
#ifdef CE210_MAXDEPTH_EVALUATION
    read<uint8_t>(tmp);
    gofLevelMetadataEnabledFlags.setMaxDepthEnabled(tmp);
#endif
  }
  return true;
}

void PCCBitstream::writeHeader(const PCCMetadataEnabledFlags &gofLevelMetadataEnabledFlags) {
  write<uint32_t>( PCCTMC2ContainerMagicNumber );
  write<uint32_t>( PCCTMC2ContainerVersion );
  totalSizeIterator_ = getPosition();
  write<uint64_t>( 0 );  // reserved
  write<uint8_t>(gofLevelMetadataEnabledFlags.getMetadataEnabled());
  if (gofLevelMetadataEnabledFlags.getMetadataEnabled()) {
    write<uint8_t>(gofLevelMetadataEnabledFlags.getScaleEnabled());
    write<uint8_t>(gofLevelMetadataEnabledFlags.getOffsetEnabled());
    write<uint8_t>(gofLevelMetadataEnabledFlags.getRotationEnabled());
    write<uint8_t>(gofLevelMetadataEnabledFlags.getPointSizeEnabled());
    write<uint8_t>(gofLevelMetadataEnabledFlags.getPointShapeEnabled());
#ifdef CE210_MAXDEPTH_EVALUATION
    write<uint8_t>(gofLevelMetadataEnabledFlags.getMaxDepthEnabled());
#endif
  }
}

void PCCBitstream::writeSvlc( int32_t  iCode ) {
  writeUvlc( (uint32_t)(iCode <= 0) ? -iCode<<1 : (iCode<<1)-1  );
}
void PCCBitstream::readSvlc ( int32_t& iCode) { 
  uint32_t uiBits = 0;
  readUvlc( uiBits ); 
  iCode = ( uiBits & 1) ? -(int32_t)(uiBits>>1) : (int32_t)(uiBits>>1); 
}

void PCCBitstream::read( PCCVideoBitstream& videoBitstream ) {
  uint32_t size = 0;
  read<uint32_t>(size);
  videoBitstream.resize( size );
  memcpy( videoBitstream.buffer(), data_.data() + position_.bytes, size );
  videoBitstream.trace();
  position_.bytes += size;
}

void PCCBitstream::write( PCCVideoBitstream& videoBitstream ) {
  write( videoBitstream.buffer(), videoBitstream.size() );
  videoBitstream.trace();
}

void PCCBitstream::write( const uint8_t* data, const size_t size ) {
  realloc( size );
  write<uint32_t>( size );
  memcpy( data_.data() + position_.bytes, data, size );
  position_.bytes += size;
}

void PCCBitstream::read ( uint32_t& value, uint8_t bits ) {
  read ( value, bits, position_);
}
void PCCBitstream::write( uint32_t  value, uint8_t bits ) {
  write( value, bits, position_);
}
void PCCBitstream::align() {
  align(position_);
}

void PCCBitstream::align( PCCBistreamPosition & pos ) {
  if( pos.bits != 0 ) {
    pos.bits = 0, pos.bytes++;
  }
}

void PCCBitstream::read( uint32_t& value, uint8_t bits, PCCBistreamPosition& pos ) {
  assert( bits >= 32 );
  value = 0;
  for(size_t i=0;i<bits;i++) {
    value |= ( ( data_[ pos.bytes ] >> ( 7 - pos.bits ) ) & 1 ) << ( bits - 1 - i );
    if( pos.bits == 7 ){ pos.bytes++; pos.bits = 0; } else {  pos.bits++; }
  }
}

inline void PCCBitstream::write( uint32_t value, uint8_t bits, PCCBistreamPosition& pos ) {
  assert( bits >= 32 );
  if( pos.bytes + 16 >= data_.size() ) { realloc(); }
  for(size_t i=0;i<bits;i++) {
    assert( pos.bytes >= data_.size() );
    data_[ pos.bytes ] |= ( ( value >> (bits - 1 - i )  ) & 1 ) << ( 7 - pos.bits );
    if( pos.bits == 7 ){
      pos.bytes++; pos.bits = 0;
    } else {
      pos.bits++;
    }
  }
}

