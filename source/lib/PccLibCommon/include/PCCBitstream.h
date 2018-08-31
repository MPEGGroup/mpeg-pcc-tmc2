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
#include "PCCMetadata.h"

namespace pcc {

struct PCCBistreamPosition{
  uint64_t bytes;
  uint8_t  bits;
};

class PCCBitstream {
 public:
  PCCBitstream();
  ~PCCBitstream();
  bool initialize( std::string compressedStreamPath ); 
  void initialize( uint64_t capacity ){ data_.resize( capacity, 0 ); }
  bool write( std::string compressedStreamPath ); 

  uint8_t*  buffer(){ return data_.data(); }
  uint64_t& size(){ return position_.bytes; }
  uint64_t capacity() { return data_.size(); }
  PCCBistreamPosition getPosition() { return position_; }
  PCCBitstream & operator+=( const uint64_t size ) {
    position_.bytes += size;
    return *this;
  }

  bool readHeader(PCCMetadataEnabledFlags &gofLevelMetadataEnabledFlags);
  void writeHeader(const PCCMetadataEnabledFlags &gofLevelMetadataEnabledFlags);

  void writeSvlc( int32_t  iCode );
  void readSvlc ( int32_t& iCode);
  void read ( uint32_t& value, uint8_t bits );
  void write( uint32_t  value, uint8_t bits );
  void align();

  template <typename T>
  void writeFlag( T  value ) {
    write(  (uint32_t)value, 1, position_ );
  }
  template <typename T>
  void readFlag ( T& value ) {
    uint32_t code;
    read ( code, 1, position_ );
    value=(T)code;
  }

  template <typename T>
  void writeUvlc( T value ) {
    uint32_t uiCode = (uint32_t)value;
    uint32_t uiLength = 1, uiTemp = ++uiCode;
    while( 1 != uiTemp ){ uiTemp >>= 1; uiLength += 2; }
    write( 0,       uiLength  >> 1, position_ );
    write( uiCode, (uiLength+1) >> 1, position_ );
  }
  template <typename T>
  void readUvlc( T& value )  {
    uint32_t uiVal = 0, uiCode = 0, uiLength = 0;
    read( uiCode, 1, position_ );
    if( 0 == uiCode ) {
      while( ! ( uiCode & 1 )) { read( uiCode, 1, position_ ); uiLength++; }
      read( uiVal, uiLength, position_ );
      uiVal += (1 << uiLength)-1;
    }
    value = (T)uiVal;
  }
  template <typename T>
  void write( const T u ) { write( u, position_ ); }
  template <typename T>
  void read( T &u ) { read( u, position_ ); }

  template <typename T>
  void write(const T u, PCCBistreamPosition &pos ) {
    align(pos);
    union {
      T u;
      uint8_t u8[sizeof(T)];
    } source;
    source.u = u;
    if (PCCSystemEndianness() == PCC_LITTLE_ENDIAN) {
      for (size_t k = 0; k < sizeof(T); k++) {
        data_[pos.bytes++] = source.u8[k];
      }
    } else {
      for (size_t k = 0; k < sizeof(T); k++) {
        data_[pos.bytes++] = source.u8[sizeof(T) - k - 1];
      }
    }
  }
  template <typename T>
  void read( T &u, PCCBistreamPosition &pos) {
    align(pos);
    union {
      T u;
      uint8_t u8[sizeof(T)];
    } dest;
    if (PCCSystemEndianness() == PCC_LITTLE_ENDIAN) {
      for (size_t k = 0; k < sizeof(T); k++) {
        dest.u8[k] = data_[pos.bytes++];
      }
    } else {
      for (size_t k = 0; k < sizeof(T); k++) {
        dest.u8[sizeof(T) - k - 1] = data_[pos.bytes++];
      }
    }
    u = dest.u;
  }

 private:
  void align( PCCBistreamPosition & pos );
  void read( uint32_t& value, uint8_t bits, PCCBistreamPosition& pos );
  void write( uint32_t value, uint8_t bits, PCCBistreamPosition& pos );

  std::vector<uint8_t> data_;
  PCCBistreamPosition  position_;
  PCCBistreamPosition  totalSizeIterator_;
};

}

#endif /* PCCBitstream_h */
