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

#include "PCCVideo.h"

using namespace pcc;

template <typename T, size_t N>
bool PCCVideo<T, N>::read( const std::string    fileName,
                           const size_t         sizeU0,
                           const size_t         sizeV0,
                           const PCCCOLORFORMAT format,
                           const size_t         nbyte ) {
  std::ifstream infile( fileName, std::ios::binary );         
  auto          ret = read( infile, sizeU0, sizeV0, format, nbyte );
  infile.close();
  return ret;
}

template <typename T, size_t N>
bool PCCVideo<T, N>::read( std::ifstream&       infile,
                           const size_t         sizeU0,
                           const size_t         sizeV0,
                           const PCCCOLORFORMAT format,
                           const size_t         nbyte ) {
  while ( !infile.eof() ) {
    PCCImage<T, N> frame;
    if ( frame.read( infile, sizeU0, sizeV0, format, nbyte ) ) { frames_.push_back( frame ); }
  }
  return !frames_.empty();
}

template <typename T, size_t N>
bool PCCVideo<T, N>::write( const std::string fileName, const size_t nbyte ) {
  std::ofstream outfile( fileName, std::ios::binary );
  if ( write( outfile, nbyte ) ) {
    outfile.close();
    return true;
  }
  return false;
}

template <typename T, size_t N>
bool PCCVideo<T, N>::write( std::ofstream& outfile, const size_t nbyte ) {
  for ( auto& frame : frames_ ) {
    if ( !frame.write( outfile, nbyte ) ) { return false; }
  }
  return true;
}

#if defined( WIN32 )
template <typename T, size_t N>
bool PCCVideo<T, N>::_read( const std::string    fileName,
                            const size_t         sizeU0,
                            const size_t         sizeV0,
                            const PCCCOLORFORMAT format,
                            const size_t         nbyte ) {
  return read( fileName, sizeU0, sizeV0, format, nbyte );
}

template <typename T, size_t N>
bool PCCVideo<T, N>::_write( const std::string fileName, const size_t nbyte ) {
  return write( fileName, nbyte );
}
#endif

template <typename T, size_t N>
void PCCVideo<T, N>::convertBitdepth( uint8_t bitdepthInput, uint8_t bitdepthOutput, bool msbAlignFlag ) {
  for ( auto& frame : frames_ ) { frame.convertBitdepth( bitdepthInput, bitdepthOutput, msbAlignFlag ); }
}

template <typename T, size_t N>
void PCCVideo<T, N>::convertYUV420ToYUV444() {
  for ( auto& frame : frames_ ) { frame.convertYUV420ToYUV444(); }
}

template <typename T, size_t N>
void PCCVideo<T, N>::convertYUV444ToYUV420() {
  for ( auto& frame : frames_ ) { frame.convertYUV444ToYUV420(); }
}

template <typename T, size_t N>
bool PCCVideo<T, N>::allPixelsEqualToZero() {
  for ( auto& frame : frames_ ) {
    if ( !frame.allPixelsEqualToZero() ) { return false; }
  }
  return true;
}

template <typename T, size_t N>
void PCCVideo<T, N>::upsample( size_t rate ) {
  for ( auto& frame : frames_ ) { frame.upsample( rate ); }
}

template class pcc::PCCVideo<uint8_t, 3>;
template class pcc::PCCVideo<uint16_t, 3>;